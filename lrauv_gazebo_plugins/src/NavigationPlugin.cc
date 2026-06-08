#include "NavigationPlugin.hh"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <algorithm>

namespace tethys {

  // --- NavigationPrivateData ---
  class NavigationPrivateData {

    // --- Gazebo transport ---
    public: gz::transport::Node node;
    public: gz::transport::Node::Publisher thrustPub;
    public: gz::transport::Node::Publisher vertFinPub;
    public: gz::transport::Node::Publisher horizFinPub;
    public: std::mutex mtx;

    // --- Dati lidar ---
    public: std::vector<float> ranges;
    public: int horizCount = 0;
    public: int vertCount = 0;

    // --- Dati pose ---
    public: double posX = 0, posY = 0, posZ = 0;
    public: double oriX = 0, oriY = 0, oriZ = 0, oriW = 1;
    public: double yaw = 0, pitch = 0, roll = 0;
    public: double vertFinAngle = 0;
    public: double horizFinAngle = 0;

    // --- VFH parametri ---
    static constexpr int NUM_AZ = 360;
    static constexpr int NUM_EL = 4;

    const double EL_ANGLES[NUM_EL] = {-0.2618, -0.0873, 0.0873, 0.2618};

    public: double threshold      = 0.001;
    public: int    sMax           = 20;
    public: int    smoothL        = 5;
    public: double gainSteer      = 0.5;
    public: double gainPitch      = 0.5;
    public: double radiusArrived  = 5.0;
    public: double radiusSlowdown = 15.0;

    public: int lastBestDir = -1;
    public: int lastBestEl  = 1;

    // --- VFH dati ---
    public: std::vector<double> histogram = std::vector<double>(NUM_AZ * NUM_EL, 0.0);
    public: double goalX = 0, goalY = 0, goalZ = 0;

    // --- Stato episodio ---
    public: enum class EpisodeState { RUNNING, ARRIVED, COLLISION, TIMEOUT };
    public: EpisodeState episodeState = EpisodeState::RUNNING;
    public: bool resultPublished = false;
    public: int poseCount = 0;  
    public: int64_t maxIterations = 300000;
    public: gz::transport::Node::Publisher resultPub;

    // --- Anti-stuck ---
    public: double lastPosX = 0, lastPosY = 0, lastPosZ = 0;
    public: int64_t lastMoveIteration = 0;
    public: int64_t stuckTimeout = 10000; // 10s 

    // CALLBACK LIDAR
    public: void OnLidar(const gz::msgs::LaserScan &_msg) {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->ranges.clear();
      this->horizCount = _msg.count();
      this->vertCount  = _msg.vertical_count();
      for (int i = 0; i < _msg.ranges_size(); i++)
        this->ranges.push_back(_msg.ranges(i));
    }

    // CALLBACK POSE
    public: void OnPose(const gz::msgs::Pose_V &_msg) {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->poseCount++;
      for (int i = 0; i < _msg.pose_size(); i++) {
        const auto &name = _msg.pose(i).name();

        if (name == "tethys") {
          auto &pos = _msg.pose(i).position();
          auto &ori = _msg.pose(i).orientation();
          this->posX = pos.x();
          this->posY = pos.y();
          this->posZ = pos.z();
          this->oriX = ori.x();
          this->oriY = ori.y();
          this->oriZ = ori.z();
          this->oriW = ori.w();
          this->yaw   = std::atan2(2*(oriW*oriZ + oriX*oriY), 1 - 2*(oriY*oriY + oriZ*oriZ));
          this->pitch = std::asin(2*(oriW*oriY - oriZ*oriX));
          this->roll  = std::atan2(2*(oriW*oriX + oriY*oriZ), 1 - 2*(oriX*oriX + oriY*oriY));
        }

        if (name == "vertical_fins") {
          auto &ori = _msg.pose(i).orientation();
          double w = ori.w(), x = ori.x(), y = ori.y(), z = ori.z();
          this->vertFinAngle = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
        }

        if (name == "horizontal_fins") {
          auto &ori = _msg.pose(i).orientation();
          double w = ori.w(), x = ori.x(), y = ori.y(), z = ori.z();
          this->horizFinAngle = std::asin(2*(w*y - z*x));
        }
      }
    }

    // CALLBACK CONTACT
    public: void OnContact(const gz::msgs::Contacts &_msg) {
      std::lock_guard<std::mutex> lock(this->mtx);
      if (_msg.contact_size() > 0 &&
          this->episodeState == EpisodeState::RUNNING) {
        this->episodeState = EpisodeState::COLLISION;
        std::cout << "[NAV] COLLISION detected!" << std::endl;
      }
    }

    // CALLBACK PARAMS
    public: void OnParams(const gz::msgs::StringMsg &_msg) {
      std::lock_guard<std::mutex> lock(this->mtx);
      auto data = _msg.data();
      std::cout << "[NAV] RAW JSON: " << data << std::endl;

      auto getDouble = [&](const std::string &key) -> double {
        auto pos = data.find("\"" + key + "\":");
        if (pos == std::string::npos) return -1.0;
        pos += key.size() + 3;
        return std::stod(data.substr(pos));
      };
      auto getInt = [&](const std::string &key) -> int {
        return (int)std::round(getDouble(key));
      };

      this->threshold      = getDouble("threshold");
      this->sMax           = getInt("s_max");
      this->smoothL        = getInt("smooth_l");
      this->gainSteer      = getDouble("gain_steer");
      this->gainPitch      = getDouble("gain_pitch");
      this->radiusArrived  = getDouble("radius_arrived");
      this->radiusSlowdown = getDouble("radius_slowdown");

      this->episodeState    = EpisodeState::RUNNING;
      this->resultPublished = false;
      this->lastBestDir     = -1;
      this->lastBestEl      = 1;
      this->lastMoveIteration = 0;
      this->lastPosX = 0;
      this->lastPosY = 0;
      this->lastPosZ = 0;
      this->poseCount = 0;  // resetta, aspetta 3 pose fresche

      gz::msgs::Double stop, fin, horiz;
      stop.set_data(0.0);
      fin.set_data(0.0);
      horiz.set_data(0.0);
      this->thrustPub.Publish(stop);
      this->vertFinPub.Publish(fin);
      this->horizFinPub.Publish(horiz);

      std::cout << "[NAV] Params updated: threshold=" << this->threshold
                << " gainSteer=" << this->gainSteer
                << " gainPitch=" << this->gainPitch
                << " sMax=" << this->sMax << std::endl;
      std::cout << "[NAV] OnParams - pos=(" << this->posX << "," << this->posY << "," << this->posZ 
          << ") yaw=" << this->yaw * 180.0/M_PI << "°" << std::endl;
      std::cout << "[NAV] OnParams - goalAngle=" << GoalAngle() * 180.0/M_PI << "°" << std::endl;
      std::cout << "[NAV] Episode reset" << std::endl;
    }

    // BUILD HISTOGRAM
    public: void BuildHistogram() {
      histogram.assign(NUM_AZ * NUM_EL, 0.0);
      for (int j = 0; j < NUM_EL; j++) {
        for (int i = 0; i < NUM_AZ; i++) {
          int idx = i + j * NUM_AZ;
          float d = ranges[idx];
          histogram[idx] = (std::isinf(d) || d <= 0.0f) ? 0.0 : 1.0 / (d * d);
        }
      }
    }

    // SMOOTH HISTOGRAM
    public: void SmoothHistogram() {
      std::vector<double> smoothed(NUM_AZ * NUM_EL, 0.0);
      for (int j = 0; j < NUM_EL; j++) {
        for (int k = 0; k < NUM_AZ; k++) {
          double sum = 0.0;
          for (int m = -this->smoothL; m <= this->smoothL; m++)
            sum += histogram[((k + m + NUM_AZ) % NUM_AZ) + j * NUM_AZ];
          smoothed[k + j * NUM_AZ] = sum / (2*this->smoothL + 1);
        }
      }
      histogram = smoothed;
    }

    // GOAL ANGLE
    public: double GoalAngle() {
      double dx = goalX - posX;
      double dy = goalY - posY;
      double angleWorld = std::atan2(dy, dx);
      double angleVehicle = angleWorld - yaw - M_PI;
      while (angleVehicle < 0)       angleVehicle += 2*M_PI;
      while (angleVehicle >= 2*M_PI) angleVehicle -= 2*M_PI;
      return angleVehicle;
    }

    // FIND BEST DIRECTION 3D
    public: std::pair<int,int> FindBestDirection() {
      double goalAngle = GoalAngle();
      int goalSector = (int)(goalAngle * 180.0 / M_PI) % NUM_AZ;

      double dx = goalX - posX;
      double dy = goalY - posY;
      double dz = goalZ - posZ;
      double distXY = std::sqrt(dx*dx + dy*dy);
      double goalElev = std::atan2(dz, distXY);

      int goalElLayer = 1;
      double minElDiff = 1e9;
      for (int j = 0; j < NUM_EL; j++) {
        double diff = std::abs(EL_ANGLES[j] - goalElev);
        if (diff < minElDiff) {
          minElDiff = diff;
          goalElLayer = j;
        }
      }

      int idx = goalSector + goalElLayer * NUM_AZ;
      if (histogram[idx] < this->threshold)
        return {goalSector, goalElLayer};

      int bestAz = -1, bestEl = 1;
      double bestCost = 1e9;

      auto angDist = [&](int a, int b) {
        int d = std::abs(a - b);
        if (d > NUM_AZ/2) d = NUM_AZ - d;
        return d;
      };

      for (int j = 0; j < NUM_EL; j++) {
        std::vector<std::pair<int,int>> valleys;
        int valleyStart = -1;
        for (int k = 0; k < NUM_AZ; k++) {
          if (histogram[k + j * NUM_AZ] < this->threshold) {
            if (valleyStart == -1) valleyStart = k;
          } else {
            if (valleyStart != -1) {
              valleys.push_back({valleyStart, k-1});
              valleyStart = -1;
            }
          }
        }
        if (valleyStart != -1)
          valleys.push_back({valleyStart, NUM_AZ-1});

        for (auto &v : valleys) {
          int k_n = v.first;
          int k_f = v.second;
          int width = k_f - k_n;

          if (angDist(k_f, goalSector) < angDist(k_n, goalSector))
            std::swap(k_n, k_f);

          int theta;
          if (width <= this->sMax) {
            theta = (k_n + k_f) / 2;
          } else {
            theta = (k_n < k_f) ? k_n + this->sMax/2 : k_n - this->sMax/2;
            theta = (theta + NUM_AZ) % NUM_AZ;
          }

          double costAz = (double)angDist(theta, goalSector);
          double costEl = std::abs(j - goalElLayer) * 20.0;
          double cost = costAz + costEl;

          if (cost < bestCost) {
            bestCost = cost;
            bestAz = theta;
            bestEl = j;
          }
        }
      }

      lastBestDir = bestAz;
      lastBestEl  = bestEl;
      return {bestAz, bestEl};
    }
  };

  // --- NavigationPlugin ---
  NavigationPlugin::NavigationPlugin() : dataPtr(std::make_unique<NavigationPrivateData>()) {}

  NavigationPlugin::~NavigationPlugin() = default;

  void NavigationPlugin::Configure(const gz::sim::Entity &,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &,
                                   gz::sim::EventManager &) {
    this->dataPtr->node.Subscribe("/tethys/lidar",
      &NavigationPrivateData::OnLidar, this->dataPtr.get());

    this->dataPtr->node.Subscribe("/world/empty_environment/dynamic_pose/info",
      &NavigationPrivateData::OnPose, this->dataPtr.get());

    this->dataPtr->node.Subscribe("/world/empty_environment/model/tethys/link/base_link/sensor/contact_sensor/contact",
      &NavigationPrivateData::OnContact, this->dataPtr.get());

    this->dataPtr->node.Subscribe("/es/vfh_params",
      &NavigationPrivateData::OnParams, this->dataPtr.get());

    this->dataPtr->thrustPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
      "/model/tethys/joint/propeller_joint/cmd_thrust");

    this->dataPtr->vertFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
      "/model/tethys/joint/vertical_fins_joint/0/cmd_pos");

    this->dataPtr->horizFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
      "/model/tethys/joint/horizontal_fins_joint/0/cmd_pos");

    this->dataPtr->resultPub = this->dataPtr->node.Advertise<gz::msgs::StringMsg>(
      "/es/episode_result"); 

    this->dataPtr->goalX = _sdf->Get<double>("goal_x", 0.0).first;
    this->dataPtr->goalY = _sdf->Get<double>("goal_y", 0.0).first;
    this->dataPtr->goalZ = _sdf->Get<double>("goal_z", 0.0).first;

    this->dataPtr->poseCount = 0;

    this->dataPtr->threshold      = _sdf->Get<double>("threshold",       0.001).first;
    this->dataPtr->sMax           = _sdf->Get<int>   ("s_max",           20).first;
    this->dataPtr->smoothL        = _sdf->Get<int>   ("smooth_l",        5).first;
    this->dataPtr->gainSteer      = _sdf->Get<double>("gain_steer",      0.5).first;
    this->dataPtr->gainPitch      = _sdf->Get<double>("gain_pitch",      0.5).first;
    this->dataPtr->radiusArrived  = _sdf->Get<double>("radius_arrived",  5.0).first;
    this->dataPtr->radiusSlowdown = _sdf->Get<double>("radius_slowdown", 15.0).first;
    this->dataPtr->maxIterations  = _sdf->Get<int>   ("max_iterations",  300000).first;

    std::cout << "[NAV] goal=("
              << this->dataPtr->goalX << ", "
              << this->dataPtr->goalY << ", "
              << this->dataPtr->goalZ << ")" << std::endl;
    std::cout << "[NAV] threshold=" << this->dataPtr->threshold
              << " sMax=" << this->dataPtr->sMax
              << " smoothL=" << this->dataPtr->smoothL
              << " gainSteer=" << this->dataPtr->gainSteer
              << " gainPitch=" << this->dataPtr->gainPitch
              << " radiusArrived=" << this->dataPtr->radiusArrived
              << " radiusSlowdown=" << this->dataPtr->radiusSlowdown
              << " maxIterations=" << this->dataPtr->maxIterations << std::endl;
  }
 
  void NavigationPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                   gz::sim::EntityComponentManager &) {
    if (_info.paused) return;
    if (this->dataPtr->poseCount < 3) return;

    // CONTROLLO - ogni 100ms
    if (_info.iterations % 100 == 0) {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      // FINE EPISODIO
      if (this->dataPtr->episodeState != NavigationPrivateData::EpisodeState::RUNNING) {
        if (!this->dataPtr->resultPublished) {
          gz::msgs::StringMsg msg;
          if (this->dataPtr->episodeState == NavigationPrivateData::EpisodeState::ARRIVED)
            msg.set_data("ARRIVED");
          else if (this->dataPtr->episodeState == NavigationPrivateData::EpisodeState::COLLISION)
            msg.set_data("COLLISION");
          else
            msg.set_data("TIMEOUT");
          this->dataPtr->resultPub.Publish(msg);
          this->dataPtr->resultPublished = true;
          std::cout << "[NAV] Episode ended: " << msg.data() << std::endl;
        }
        return;
      }

      // TIMEOUT
      if ((int64_t)_info.iterations > this->dataPtr->maxIterations) {
        this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::TIMEOUT;
        return;
      }

      // STUCK CHECK
      double moveDist = std::sqrt(
          std::pow(this->dataPtr->posX - this->dataPtr->lastPosX, 2) +
          std::pow(this->dataPtr->posY - this->dataPtr->lastPosY, 2) +
          std::pow(this->dataPtr->posZ - this->dataPtr->lastPosZ, 2));

      if (moveDist > 0.5) {
          this->dataPtr->lastPosX = this->dataPtr->posX;
          this->dataPtr->lastPosY = this->dataPtr->posY;
          this->dataPtr->lastPosZ = this->dataPtr->posZ;
          this->dataPtr->lastMoveIteration = _info.iterations;
      }

      if (_info.iterations - this->dataPtr->lastMoveIteration > this->dataPtr->stuckTimeout) {
          this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::TIMEOUT;
          std::cout << "[NAV] STUCK detected! Episode ended as TIMEOUT" << std::endl;
          return;
      }

      double dx = this->dataPtr->goalX - this->dataPtr->posX;
      double dy = this->dataPtr->goalY - this->dataPtr->posY;
      double dz = this->dataPtr->goalZ - this->dataPtr->posZ;
      double distGoal = std::sqrt(dx*dx + dy*dy + dz*dz);

      if (distGoal < this->dataPtr->radiusArrived) {
        this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::ARRIVED;
        gz::msgs::Double stop, fin, horiz;
        stop.set_data(0.0);
        fin.set_data(0.0);
        horiz.set_data(0.0);
        this->dataPtr->thrustPub.Publish(stop);
        this->dataPtr->vertFinPub.Publish(fin);
        this->dataPtr->horizFinPub.Publish(horiz);
        return;
      }

      if (!this->dataPtr->ranges.empty()) {
        this->dataPtr->BuildHistogram();
        this->dataPtr->SmoothHistogram();
      }

      auto [bestDir, bestElLayer] = this->dataPtr->FindBestDirection();

      if (bestDir >= 0) {
        double steerError = bestDir * M_PI / 180.0;
        if (steerError > M_PI) steerError -= 2*M_PI;
        double finCmd = std::clamp(steerError * this->dataPtr->gainSteer, -0.261799, 0.261799);

        double targetElev = this->dataPtr->EL_ANGLES[bestElLayer];
        double horizFinCmd = std::clamp(-targetElev * this->dataPtr->gainPitch, -0.261799, 0.261799);

        
        double thrust = -31.0;
        if (distGoal < this->dataPtr->radiusSlowdown)
          thrust = -31.0 * (distGoal / this->dataPtr->radiusSlowdown);

        gz::msgs::Double thrustMsg, finMsg, horizMsg;
        thrustMsg.set_data(thrust);
        finMsg.set_data(finCmd);
        horizMsg.set_data(horizFinCmd);
        std::cout << "[THRUST] thrust=" << thrust 
          << " finCmd=" << finCmd 
          << " horizFinCmd=" << horizFinCmd << std::endl;
        this->dataPtr->thrustPub.Publish(thrustMsg);
        this->dataPtr->vertFinPub.Publish(finMsg);
        this->dataPtr->horizFinPub.Publish(horizMsg);
      }
    }

    // STAMPA - ogni 1s
    if (_info.iterations % 1000 == 0) {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      double dx = this->dataPtr->goalX - this->dataPtr->posX;
      double dy = this->dataPtr->goalY - this->dataPtr->posY;
      double dz = this->dataPtr->goalZ - this->dataPtr->posZ;
      double distGoal = std::sqrt(dx*dx + dy*dy + dz*dz);

      auto [bestDir, bestElLayer] = this->dataPtr->FindBestDirection();
      double steerError = bestDir * M_PI / 180.0;
      if (steerError > M_PI) steerError -= 2*M_PI;

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\nt=" << _info.iterations/1000 << "s " << std::endl;
      std::cout << "[POSE]  pos=(" << this->dataPtr->posX << ", "
                << this->dataPtr->posY << ", " << this->dataPtr->posZ
                << ") yaw=" << this->dataPtr->yaw * 180.0/M_PI << "°" << std::endl;
      std::cout << "[GOAL]  dist=" << distGoal << "m"
                << " angle=" << this->dataPtr->GoalAngle() * 180.0/M_PI << "°" << std::endl;
    }
  }
}

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)