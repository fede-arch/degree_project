#include "NavigationPlugin.hh"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <sstream>
#include <vector>
#include <utility>

namespace tethys {

  // --- NavigationPrivateData ---
  class NavigationPrivateData {

    // GAZEBO TRANSPORT
    public: gz::transport::Node node;
    public: gz::transport::Node::Publisher thrustPub;
    public: gz::transport::Node::Publisher vertFinPub;
    public: gz::transport::Node::Publisher horizFinPub;
    public: gz::transport::Node::Publisher resultPub;
    public: std::mutex mtx;

    // POSE VEICOLO
    public: double posX = 0, posY = 0, posZ = 0;
    public: double yaw = 0, pitch = 0, roll = 0;

    // GOAL
    public: double goalX = 0, goalY = 0, goalZ = 0;

    // STATO EPISODIO
    public: enum class EpisodeState { RUNNING, ARRIVED, COLLISION, TIMEOUT };
    public: EpisodeState episodeState   = EpisodeState::RUNNING;
    public: bool         resultPublished  = false;
    public: int64_t      episodeStartIter = -1;
    public: int64_t      maxIterations    = 300000;

    // PARAMETRI CONTROLLO
    public: double gainSteer     = 0.5;
    public: double gainPitch     = 0.5;
    public: double maxFinAngle   = 0.15;
    public: double radiusArrived = 2.0;

    // PARAMETRI LIDAR
    public: float r_min = 2.5f;
    public: float r_max = 60.0f;

    // PARAMETRI GRIGLIA 3D (r x phi x theta)
    public: int   n_r        = 60;
    public: float delta_r    = 1.0f;
    public: float A          = 15.0f;
    public: float B          = 0.25f;    // A / r_max
    public: float C_MAX      = 14.0625f;
    public: float r_active   = 50.0f;
    public: int   n_r_active = 50;
    public: std::vector<float> grid_c;  // dimensione: n_r * n_phi * n_theta
    public: std::vector<float> c_star;  // dimensione: n_r_active * n_phi * n_theta
    public: float gridDecay  = 0.97f;

    // PARAMETRI ISTOGRAMMA POLARE (phi x theta)
    public: int   n_phi       = 36;
    public: int   n_theta     = 36;
    public: float delta_phi   = M_PI / 36;   // 5 deg/bin
    public: float delta_theta = M_PI / 36;   // 5 deg/bin
    public: float phi_min     = -M_PI / 2;
    public: float phi_max     = +M_PI / 2;
    public: float theta_min   = -M_PI / 2;
    public: float theta_max   = +M_PI / 2;
    public: int   L           = 5;           // semi-finestra smoothing
    public: float VALLEY_THRESHOLD = 50.0f;
    public: double measurements[36][36] = {};
    public: float  h[36][36]        = {};
    public: float  h_smooth[36][36] = {};

    // VFH - DENSITÀ PHI E RISULTATO
    public: float phi_density[36] = {};
    public: float phi_smooth[36]  = {};
    public: int k_targ       = 18;   // settore phi del goal
    public: int k_targ_theta = 18;   // settore theta del goal
    public: int best_phi     = 18;   // phi scelto dal VFH
    public: int best_theta   = 18;   // theta scelto dal VFH
    public: int safetyWindow = 3;

    // ACCESSOR INLINE per grid_c e c_star
    public: inline float& gc(int r, int p, int t) {
        return grid_c[r * n_phi * n_theta + p * n_theta + t];
    }
    public: inline float& cs(int r, int p, int t) {
        return c_star[r * n_phi * n_theta + p * n_theta + t];
    }

    // FASE 1 -> CALLBACK POINTCLOUD
    public: void OnPointCloud(const gz::msgs::PointCloudPacked &_msg) {
      std::lock_guard<std::mutex> lock(this->mtx);

      if (episodeState != EpisodeState::RUNNING) return;

      std::memset(measurements, 0, sizeof(measurements));
      int step         = _msg.point_step();
      int total_points = _msg.width() * _msg.height();
      const char *data = _msg.data().data();

      for (int i = 0; i < total_points; i++) {
        const char *base = data + i * step;

        float x, y, z;
        std::memcpy(&x, base + 0, 4);
        std::memcpy(&y, base + 4, 4);
        std::memcpy(&z, base + 8, 4);

        if (std::isinf(x) || std::isinf(y) || std::isinf(z)) continue;

        // Conversione da cartesiane a polari
        double r     = std::sqrt(x*x + y*y + z*z);
        double phi   = std::atan2(y, x);
        double theta = std::atan2(z, std::sqrt(x*x + y*y));

        if (r < r_min || r > r_max) continue;

        // Discretizzazione angoli → indici
        int phi_idx   = (int)((phi_max - phi)     / delta_phi);
        int theta_idx = (int)((theta - theta_min) / delta_theta);

        if (phi_idx   < 0 || phi_idx   >= n_phi)  continue;
        if (theta_idx < 0 || theta_idx >= n_theta) continue;

        measurements[phi_idx][theta_idx] = (float)r;
      }

      updateGlobalGrid();
      extract_active_region();
      build_polar_histogram();
      smooth_histogram();
      findBestDirection();
    }

    // FASE 2 -> AGGIORNAMENTO GRIGLIA
    public: void updateGlobalGrid() {

        // Decay: tutte le celle perdono gridDecay per frame
        for (int ri = 0; ri < n_r; ri++)
            for (int p = 0; p < n_phi; p++)
                for (int t = 0; t < n_theta; t++)
                    gc(ri, p, t) *= gridDecay;

        // Aggiorna con nuove misure
        for (int phi_idx = 0; phi_idx < n_phi; phi_idx++) {
            for (int theta_idx = 0; theta_idx < n_theta; theta_idx++) {

                float meas = measurements[phi_idx][theta_idx];
                if (meas <= 0.0f || meas < r_min) continue;

                int r_idx = (int)(meas / delta_r);
                if (r_idx < 0 || r_idx >= n_r) continue;

                float magnitude = A - B * meas;
                if (magnitude < 0.0f) continue;

                gc(r_idx, phi_idx, theta_idx) += magnitude;
                if (gc(r_idx, phi_idx, theta_idx) > C_MAX)
                    gc(r_idx, phi_idx, theta_idx) = C_MAX;
            }
        }
    }

    // FASE 3 -> ESTRAZIONE REGIONE REATTIVA
    public: void extract_active_region() {
        std::fill(c_star.begin(), c_star.end(), 0.0f);

        for (int r_idx = 0; r_idx < n_r_active; r_idx++)
            for (int phi_idx = 0; phi_idx < n_phi; phi_idx++)
                for (int theta_idx = 0; theta_idx < n_theta; theta_idx++)
                    cs(r_idx, phi_idx, theta_idx) = gc(r_idx, phi_idx, theta_idx);
    }

    // FASE 4 -> COSTRUZIONE ISTOGRAMMA POLARE
    public: void build_polar_histogram() {
        std::memset(h, 0, sizeof(h));

        for (int phi_idx = 0; phi_idx < n_phi; phi_idx++)
            for (int theta_idx = 0; theta_idx < n_theta; theta_idx++)
                for (int r_idx = 0; r_idx < n_r_active; r_idx++)
                    h[phi_idx][theta_idx] += cs(r_idx, phi_idx, theta_idx);
    }

    // FASE 5 -> SMOOTH ISTOGRAMMA
    public: void smooth_histogram() {
        std::memset(h_smooth, 0, sizeof(h_smooth));

        for (int phi_idx = 0; phi_idx < n_phi; phi_idx++) {
            for (int theta_idx = 0; theta_idx < n_theta; theta_idx++) {

                float sum  = 0.0f;
                int   count = 0;

                for (int dphi = -L; dphi <= L; dphi++) {
                    for (int dtheta = -L; dtheta <= L; dtheta++) {

                        int p = phi_idx   + dphi;
                        int t = theta_idx + dtheta;

                        if (p < 0 || p >= n_phi)  continue;
                        if (t < 0 || t >= n_theta) continue;

                        int weight = (dphi   == -L || dphi   == L) ? 1 : 2;
                        weight    *= (dtheta == -L || dtheta == L) ? 1 : 2;

                        sum   += weight * h[p][t];
                        count += weight;
                    }
                }

                h_smooth[phi_idx][theta_idx] = (count > 0) ? sum / count : 0.0f;
            }
        }
    }


    // FASE 6 -> TROVA DIREZIONE MIGLIORE
    public: void findBestDirection() {
        compute_goal_sectors();
        find_best_direction_2d();

        // Controlla ARRIVED
        double dx = goalX - posX, dy = goalY - posY, dz = goalZ - posZ;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < radiusArrived && episodeState == EpisodeState::RUNNING) {
            episodeState = EpisodeState::ARRIVED;
            std::cout << "[NAV] ARRIVED! dist=" << dist << "m" << std::endl;
        }

        if (episodeState != EpisodeState::RUNNING) {
            publish_result_and_stop();
            return;
        }

        command_fins();
    }

    // A1: Collassa h_smooth → phi_density (max su theta)
    public: void compute_phi_density() {
        std::memset(phi_density, 0, sizeof(phi_density));
        for (int p = 0; p < n_phi; p++)
            for (int t = 0; t < n_theta; t++)
                phi_density[p] = std::max(phi_density[p], h_smooth[p][t]);
    }

    // A2: Smooth 1D su phi
    public: void smooth_phi_density() {
        std::memset(phi_smooth, 0, sizeof(phi_smooth));
        for (int k = 0; k < n_phi; k++) {
            float sum = 0.0f; int cnt = 0;
            for (int dl = -L; dl <= L; dl++) {
                int kk = k + dl;
                if (kk < 0 || kk >= n_phi) continue;
                int w = (std::abs(dl) == L) ? 1 : 2;
                sum += w * phi_density[kk];
                cnt += w;
            }
            phi_smooth[k] = (cnt > 0) ? sum / cnt : 0.0f;
        }
    }

    // A3: Calcola settori target da goal
    public: void compute_goal_sectors() {
        double dx = goalX - posX;
        double dy = goalY - posY;

        double phi_goal = std::atan2(dy, dx) - yaw - M_PI;
        while (phi_goal >  M_PI) phi_goal -= 2.0 * M_PI;
        while (phi_goal < -M_PI) phi_goal += 2.0 * M_PI;
        phi_goal = std::clamp(phi_goal, (double)phi_min, (double)phi_max);
        k_targ   = std::clamp((int)((phi_max - phi_goal) / delta_phi), 0, n_phi - 1);

        double dxy        = std::sqrt(dx*dx + dy*dy);
        double dz         = goalZ - posZ;
        double theta_goal = std::atan2(dz, dxy);
        while (theta_goal >  M_PI) theta_goal -= 2.0 * M_PI;
        while (theta_goal < -M_PI) theta_goal += 2.0 * M_PI;
        theta_goal   = std::clamp(theta_goal, (double)theta_min, (double)theta_max);
        k_targ_theta = std::clamp((int)((theta_goal - theta_min) / delta_theta), 0, n_theta - 1);
    }

    // B: Trova direzione libera più vicina al goal
    public: void find_best_direction_2d() {
        best_phi   = k_targ;
        best_theta = k_targ_theta;
        float best_cost = std::numeric_limits<float>::max();

        int W = safetyWindow;

        for (int p = 0; p < n_phi; p++) {
            for (int t = 0; t < n_theta; t++) {

                bool all_free = true;
                for (int dp = -W; dp <= W && all_free; dp++) {
                    for (int dt = -W; dt <= W && all_free; dt++) {
                        int pp = p + dp, tt = t + dt;
                        if (pp < 0 || pp >= n_phi)  continue;
                        if (tt < 0 || tt >= n_theta) continue;
                        if (h_smooth[pp][tt] >= VALLEY_THRESHOLD)
                            all_free = false;
                    }
                }

                if (all_free) {
                    float dphi   = (float)(p - k_targ);
                    float dtheta = (float)(t - k_targ_theta);
                    float cost   = std::sqrt(dphi*dphi + dtheta*dtheta * 4.0f);
                    if (cost < best_cost) {
                        best_cost  = cost;
                        best_phi   = p;
                        best_theta = t;
                    }
                }
            }
        }
    }

    // C: Converte best_phi, best_theta in comandi pinne
    public: void command_fins() {
        double phi_best   = phi_max   - best_phi   * delta_phi;
        double theta_best = theta_min + best_theta * delta_theta;

        double steer_h = std::clamp(phi_best   * gainSteer, -maxFinAngle, maxFinAngle);
        double steer_v = std::clamp(theta_best * gainPitch, -maxFinAngle, maxFinAngle);
        steer_v = -steer_v;

        gz::msgs::Double msg_h, msg_v;
        msg_h.set_data(steer_h);
        msg_v.set_data(steer_v);
        this->vertFinPub.Publish(msg_h);
        this->horizFinPub.Publish(msg_v);
    }

    // PUBBLICA RISULTATO E FERMA MOTORI
    public: void publish_result_and_stop() {
        if (resultPublished) return;
        resultPublished = true;

        std::string result;
        switch (episodeState) {
            case EpisodeState::ARRIVED:   result = "arrived";   break;
            case EpisodeState::COLLISION: result = "collision"; break;
            case EpisodeState::TIMEOUT:   result = "timeout";   break;
            default: return;
        }

        gz::msgs::StringMsg msg;
        msg.set_data(result);
        resultPub.Publish(msg);
        std::cout << "[NAV] Episode END: " << result << std::endl;

        gz::msgs::Double zero;
        zero.set_data(0.0);
        thrustPub.Publish(zero);
        vertFinPub.Publish(zero);
        horizFinPub.Publish(zero);
    }

    // CALLBACK POSE
    public: void OnPose(const gz::msgs::Pose_V &_msg) {
        std::lock_guard<std::mutex> lock(this->mtx);
        for (int i = 0; i < _msg.pose_size(); i++) {
            if (_msg.pose(i).name() != "tethys_0") continue;
            auto &pos = _msg.pose(i).position();
            auto &ori = _msg.pose(i).orientation();
            posX = pos.x(); posY = pos.y(); posZ = pos.z();
            double ox = ori.x(), oy = ori.y(), oz = ori.z(), ow = ori.w();
            yaw   = std::atan2(2*(ow*oz + ox*oy), 1 - 2*(oy*oy + oz*oz));
            pitch = std::asin(std::clamp(2*(ow*oy - oz*ox), -1.0, 1.0));
            roll  = std::atan2(2*(ow*ox + oy*oz), 1 - 2*(ox*ox + oy*oy));
        }
    }

    // CALLBACK CONTACT
    public: void OnContact(const gz::msgs::Contacts &_msg) {
        std::lock_guard<std::mutex> lock(this->mtx);
        if (_msg.contact_size() > 0 && episodeState == EpisodeState::RUNNING) {
            episodeState = EpisodeState::COLLISION;
            std::cout << "[NAV] COLLISION detected!" << std::endl;
            publish_result_and_stop();
        }
    }
  };

  // --- NavigationPlugin ---
  NavigationPlugin::NavigationPlugin() : dataPtr(std::make_unique<NavigationPrivateData>()) {}

  NavigationPlugin::~NavigationPlugin() = default;

  void NavigationPlugin::Configure(const gz::sim::Entity &,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &,
                                   gz::sim::EventManager &) {
      auto* d = this->dataPtr.get();

      // SUBSCRIPTIONS
      d->node.Subscribe("/tethys_0/lidar/points",
          &NavigationPrivateData::OnPointCloud, d);
      d->node.Subscribe("/world/empty_environment/dynamic_pose/info",
          &NavigationPrivateData::OnPose, d);
      d->node.Subscribe(
          "/world/empty_environment/model/tethys_0/link/base_link/sensor/contact_sensor/contact",
          &NavigationPrivateData::OnContact, d);

      // PUBLISHERS
      d->thrustPub   = d->node.Advertise<gz::msgs::Double>("/model/tethys_0/joint/propeller_joint/cmd_thrust");
      d->vertFinPub  = d->node.Advertise<gz::msgs::Double>("/model/tethys_0/joint/vertical_fins_joint/0/cmd_pos");
      d->horizFinPub = d->node.Advertise<gz::msgs::Double>("/model/tethys_0/joint/horizontal_fins_joint/0/cmd_pos");
      d->resultPub   = d->node.Advertise<gz::msgs::StringMsg>("/es/episode_result");

      // PARAMETRI DA SDF
      d->goalX         = _sdf->Get<double>("goal_x",          0.0).first;
      d->goalY         = _sdf->Get<double>("goal_y",          0.0).first;
      d->goalZ         = _sdf->Get<double>("goal_z",          0.0).first;

      d->gainSteer     = _sdf->Get<double>("gain_steer",      0.5).first;
      d->gainPitch     = _sdf->Get<double>("gain_pitch",      0.5).first;
      d->maxFinAngle   = _sdf->Get<double>("max_fin_angle",   0.15).first;
      d->radiusArrived = _sdf->Get<double>("radius_arrived",  2.0).first;
      d->maxIterations = _sdf->Get<int>   ("max_iterations",  300000).first;

      d->r_max            = _sdf->Get<double>("r_max",            60.0).first;
      d->r_min            = _sdf->Get<double>("r_min",             2.5).first;
      d->r_active         = _sdf->Get<double>("r_active",         50.0).first;
      d->VALLEY_THRESHOLD = _sdf->Get<double>("valley_threshold", 50.0).first;

      d->gridDecay    = _sdf->Get<double>("grid_decay",    0.97).first;
      d->A            = _sdf->Get<double>("magnitude_a",   15.0).first;
      d->L            = _sdf->Get<int>   ("smooth_l",      5).first;
      d->safetyWindow = _sdf->Get<int>   ("safety_window", 3).first;

      // Derivati
      d->n_r        = (int)(d->r_max    / d->delta_r);
      d->n_r_active = (int)(d->r_active / d->delta_r);
      d->B          = d->A / d->r_max;

      // Alloca array dinamicamente
      d->grid_c.assign(d->n_r        * d->n_phi * d->n_theta, 0.0f);
      d->c_star.assign(d->n_r_active * d->n_phi * d->n_theta, 0.0f);

      std::cout << "[CONFIGURE] goal=(" << d->goalX << "," << d->goalY << "," << d->goalZ << ")"
                << " r_max=" << d->r_max << " n_r=" << d->n_r
                << " r_active=" << d->r_active << " n_r_active=" << d->n_r_active
                << " threshold=" << d->VALLEY_THRESHOLD << std::endl;
  }

  void NavigationPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                   gz::sim::EntityComponentManager &) {
      if (_info.paused) return;

      {
          std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

          if (this->dataPtr->episodeStartIter < 0)
              this->dataPtr->episodeStartIter = _info.iterations;

          if (this->dataPtr->episodeState == NavigationPrivateData::EpisodeState::RUNNING) {
              int64_t elapsed = _info.iterations - this->dataPtr->episodeStartIter;
              if (elapsed > this->dataPtr->maxIterations) {
                  this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::TIMEOUT;
                  std::cout << "[NAV] TIMEOUT!" << std::endl;
                  this->dataPtr->publish_result_and_stop();
              }
          }

          if (this->dataPtr->episodeState != NavigationPrivateData::EpisodeState::RUNNING)
              return;
      }

      if (_info.iterations % 100 == 0) {
          gz::msgs::Double thrustMsg;
          thrustMsg.set_data(-31.0);
          this->dataPtr->thrustPub.Publish(thrustMsg);
      }
  }

} // namespace tethys

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)