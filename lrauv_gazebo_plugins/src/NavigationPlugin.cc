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

    // --- Gazebo transport ---
    public: gz::transport::Node node;
    public: gz::transport::Node::Publisher thrustPub;
    public: gz::transport::Node::Publisher vertFinPub;
    public: gz::transport::Node::Publisher horizFinPub;
    public: gz::transport::Node::Publisher resultPub;
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

    // I 4 layer verticali del lidar: DEVONO coincidere con vertical_min/max_angle
    // e con vertical_samples del sensore nell'SDF (-15, -5, +5, +15 gradi).
    const double EL_ANGLES[NUM_EL] = {-0.2618, -0.0873, 0.0873, 0.2618};

    public: double threshold      = 0.001;   // soglia di occupazione su 1/d^2 (post-smoothing)
    public: int    sMax           = 20;       // larghezza max valle "stretta" (settori = gradi)
    public: int    smoothL        = 5;        // semi-finestra di smoothing in azimuth
    public: double gainSteer      = 0.5;
    public: double gainPitch      = 0.5;
    public: double radiusArrived  = 2.0;
    public: double radiusSlowdown = 15.0;
    public: double elevCost       = 25.0;     // penalita' per cambio layer di elevazione

    // --- NUOVO: percorribilita' + limiti attuatori ---
    public: double robotRadius    = 0.6;      // raggio "efficace" del veicolo
    public: double safetyMargin   = 0.6;      // margine di sicurezza
    // Limite di deflessione delle pinne: SOTTO alpha_stall (0.17 rad nell'SDF)
    // per restare nel regime lineare e non perdere autorita' di controllo.
    public: double maxFinAngle    = 0.15;
    public: double maxThrust      = 31.0;

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

    // --- NUOVO: clock relativo all'episodio + metriche per la fitness ---
    // episodeStartIter < 0 => da (ri)inizializzare al prossimo tick di controllo.
    // Rende timeout/stuck corretti sia che tu rilanci il sim per ogni valutazione
    // (le iterazioni ripartono da 0), sia che tu resetti live via /es/vfh_params
    // (le iterazioni continuano a crescere).
    public: int64_t episodeStartIter = -1;
    public: double pathLength = 0.0;
    public: double pathPrevX = 0, pathPrevY = 0, pathPrevZ = 0;
    public: double minClearance = std::numeric_limits<double>::infinity();

    // --- Anti-stuck ---
    public: double lastPosX = 0, lastPosY = 0, lastPosZ = 0;
    public: int64_t lastMoveIteration = 0;
    public: int64_t stuckTimeout = 10000; // 10s

    // ---- helper attuatori ----
    public: void StopActuators() {
      gz::msgs::Double z; z.set_data(0.0);
      this->thrustPub.Publish(z);
      this->vertFinPub.Publish(z);
      this->horizFinPub.Publish(z);
    }

    // ---- helper: distanza grezza in un settore/layer (inf se libero) ----
    public: double RawRange(int sec, int layer) {
      int idx = sec + layer * NUM_AZ;
      if (idx < 0 || idx >= (int)this->ranges.size())
        return std::numeric_limits<double>::infinity();
      float d = this->ranges[idx];
      if (std::isinf(d) || d <= 0.0f)
        return std::numeric_limits<double>::infinity();
      return (double)d;
    }

    // ---- helper: stringa risultato ricca per la fitness dell'ES ----
    // Formato:  <TAG>;t=<iter>;dist=<m>;path=<m>;clr=<m>
    //   t    = durata episodio in iterazioni (1000 iter = 1s con dt=1ms)
    //   dist = distanza finale dal goal
    //   path = lunghezza del percorso effettivamente compiuto
    //   clr  = distanza minima da un ostacolo nell'episodio (999 = mai visto ostacoli)
    public: std::string MakeResult(const std::string &tag, int64_t elapsedIter) {
      double dx = this->goalX - this->posX;
      double dy = this->goalY - this->posY;
      double dz = this->goalZ - this->posZ;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      double clr  = std::isinf(this->minClearance) ? 999.0 : this->minClearance;
      std::ostringstream os;
      os << tag
         << ";t="    << elapsedIter
         << ";dist=" << dist
         << ";path=" << this->pathLength
         << ";clr="  << clr;
      return os.str();
    }

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
          this->pitch = std::asin(std::clamp(2*(oriW*oriY - oriZ*oriX), -1.0, 1.0));
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
          this->horizFinAngle = std::asin(std::clamp(2*(w*y - z*x), -1.0, 1.0));
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
      const std::string &data = _msg.data();
      std::cout << "[NAV] RAW JSON: " << data << std::endl;

      // Aggiorna SOLO le chiavi effettivamente presenti nel JSON.
      // (La vecchia getDouble ritornava -1 per le chiavi mancanti e
      //  sovrascriveva i parametri: bug subdolo per l'ES.)
      auto setD = [&](const std::string &key, double &target) {
        auto p = data.find("\"" + key + "\":");
        if (p == std::string::npos) return;
        p += key.size() + 3;
        try { target = std::stod(data.substr(p)); } catch (...) {}
      };
      auto setI = [&](const std::string &key, int &target) {
        double tmp = (double)target;
        setD(key, tmp);
        target = (int)std::round(tmp);
      };

      setD("threshold",       this->threshold);
      setI("s_max",           this->sMax);
      setI("smooth_l",        this->smoothL);
      setD("gain_steer",      this->gainSteer);
      setD("gain_pitch",      this->gainPitch);
      setD("radius_slowdown", this->radiusSlowdown);
      setD("radius_arrived",  this->radiusArrived);
      setD("elev_cost",       this->elevCost);
      setD("robot_radius",    this->robotRadius);
      setD("safety_margin",   this->safetyMargin);
      setD("max_fin_angle",   this->maxFinAngle);
      setD("max_thrust",      this->maxThrust);

      // reset episodio
      this->episodeState     = EpisodeState::RUNNING;
      this->resultPublished  = false;
      this->lastBestDir      = -1;
      this->lastBestEl       = 1;
      this->episodeStartIter = -1;   // reinizializzato al prossimo tick (allinea il clock)
      this->poseCount        = 0;    // aspetta pose fresche prima di agire

      this->StopActuators();

      std::cout << "[NAV] Params: thr=" << this->threshold
                << " sMax=" << this->sMax << " smoothL=" << this->smoothL
                << " gSteer=" << this->gainSteer << " gPitch=" << this->gainPitch
                << " rSlow=" << this->radiusSlowdown
                << " R=" << this->robotRadius << " margin=" << this->safetyMargin
                << " maxFin=" << this->maxFinAngle << std::endl;
      std::cout << "[NAV] Episode reset" << std::endl;
    }

    // BUILD HISTOGRAM  (magnitudine 1/d^2 + traccia la clearance minima)
    public: void BuildHistogram() {
      histogram.assign(NUM_AZ * NUM_EL, 0.0);
      const int need = NUM_AZ * NUM_EL;
      if ((int)ranges.size() < need) return;   // SAFETY: frame lidar incompleto -> niente OOB
      for (int j = 0; j < NUM_EL; j++) {
        for (int i = 0; i < NUM_AZ; i++) {
          int idx = i + j * NUM_AZ;
          float d = ranges[idx];
          if (std::isinf(d) || d <= 0.0f) {
            histogram[idx] = 0.0;
          } else {
            histogram[idx] = 1.0 / ((double)d * (double)d);
            if ((double)d < this->minClearance) this->minClearance = (double)d;
          }
        }
      }
    }

    // SMOOTH HISTOGRAM  (media mobile in azimuth, con wrap-around)
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

    // GOAL DIR BODY  - direzione del goal nel frame del veicolo via quaternione COMPLETO
    // (yaw+pitch+roll). Ritorna {settore_azimuth [0..359], elevazione_rad}.
    // Settore 0 = muso (-X), cresce CCW intorno a +Z: coerente con l'indice del lidar
    // (scan -pi..+pi -> indice 0 = -X). A pitch/roll piccoli coincide IDENTICAMENTE con
    // la vecchia GoalAngle, ma a quota variabile corregge il disallineamento di frame
    // (prima l'elevazione del goal era in frame mondo, i layer del lidar in frame corpo).
    public: std::pair<int,double> GoalDirBody() {
      double gx = goalX - posX, gy = goalY - posY, gz = goalZ - posZ;
      double w = oriW, x = oriX, y = oriY, z = oriZ;
      // mondo -> corpo  (R^T, con R = corpo->mondo)
      double bx = (1-2*(y*y+z*z))*gx + (2*(x*y+w*z))*gy + (2*(x*z-w*y))*gz;
      double by = (2*(x*y-w*z))*gx + (1-2*(x*x+z*z))*gy + (2*(y*z+w*x))*gz;
      double bz = (2*(x*z+w*y))*gx + (2*(y*z-w*x))*gy + (1-2*(x*x+y*y))*gz;
      double az = std::atan2(by, bx);                       // 0 = +X (coda), +-pi = -X (muso)
      int sector = ((int)std::lround(az * 180.0 / M_PI) + 180) % NUM_AZ;
      if (sector < 0) sector += NUM_AZ;
      double elev = std::atan2(bz, std::sqrt(bx*bx + by*by)); // + = verso body +Z (in alto)
      return {sector, elev};
    }

    // FIND BEST DIRECTION 3D
    public: std::pair<int,int> FindBestDirection() {
      auto [goalSector, goalElev] = GoalDirBody();

      int goalElLayer = 1;
      double minElDiff = 1e9;
      for (int j = 0; j < NUM_EL; j++) {
        double diff = std::abs(EL_ANGLES[j] - goalElev);
        if (diff < minElDiff) { minElDiff = diff; goalElLayer = j; }
      }

      // scorciatoia: se la direzione del goal e' libera, vai dritto
      int gidx = goalSector + goalElLayer * NUM_AZ;
      if (histogram[gidx] < this->threshold) {
        lastBestDir = goalSector; lastBestEl = goalElLayer;
        return {goalSector, goalElLayer};
      }

      {
          int layerOrder[NUM_EL];
          int n = 0;
          for (int j = goalElLayer; j >= 0; j--)        
              layerOrder[n++] = j;
          for (int j = goalElLayer + 1; j < NUM_EL; j++)
              layerOrder[n++] = j;

          for (int li = 0; li < NUM_EL; li++) {
              int j = layerOrder[li];
              int idx = goalSector + j * NUM_AZ;
              if (histogram[idx] < this->threshold) {
                  lastBestDir = goalSector; lastBestEl = j;
                  return {goalSector, j};
              }
          }
      }

      auto angDist = [&](int a, int b) {
        int d = std::abs(a - b);
        if (d > NUM_AZ/2) d = NUM_AZ - d;
        return d;
      };

      // varco lineare minimo richiesto per far passare il veicolo
      const double clearanceNeeded = 2.0 * (this->robotRadius + this->safetyMargin);

      int bestAz = -1, bestEl = 1;
      double bestCost = 1e9;

      for (int j = 0; j < NUM_EL; j++) {
        // 1) trova le valli (run di settori liberi) nel layer j
        std::vector<std::pair<int,int>> valleys;
        int valleyStart = -1;
        for (int k = 0; k < NUM_AZ; k++) {
          bool free = histogram[k + j * NUM_AZ] < this->threshold;
          if (free) {
            if (valleyStart == -1) valleyStart = k;
          } else if (valleyStart != -1) {
            valleys.push_back({valleyStart, k - 1});
            valleyStart = -1;
          }
        }
        if (valleyStart != -1) {
          // la valle in coda si fonde con la prima se questa parte da 0 (wrap-around)
          if (!valleys.empty() && valleys.front().first == 0)
            valleys.front().first = valleyStart;
          else
            valleys.push_back({valleyStart, NUM_AZ - 1});
        }

        // 2) valuta ogni valle
        for (auto &v : valleys) {
          int s0 = v.first;   // primo settore libero (la valle si estende in +mod)
          int s1 = v.second;  // ultimo settore libero
          // larghezza in settori (=gradi), wrap-aware
          int width = (s1 >= s0) ? (s1 - s0 + 1) : (NUM_AZ - s0 + s1 + 1);

          // --- gate di percorribilita' (VFH+): il varco deve far passare il veicolo ---
          int leftObs  = (s0 - 1 + NUM_AZ) % NUM_AZ;  // ostacolo che delimita un lato
          int rightObs = (s1 + 1) % NUM_AZ;           // ostacolo che delimita l'altro lato
          double dObs = std::min(RawRange(leftObs, j), RawRange(rightObs, j));
          if (std::isfinite(dObs)) {
            double widthRad  = width * M_PI / 180.0;
            double linearGap = 2.0 * dObs * std::sin(widthRad * 0.5);
            if (linearGap < clearanceNeeded) continue; // troppo stretto: scarta la valle
          }

          // --- direzione candidata DENTRO la valle (wrap-safe) ---
          int theta;
          if (width <= this->sMax) {
            // centro della valle stretta: (s0 + width/2) mod NUM_AZ funziona anche col wrap
            theta = (s0 + width / 2) % NUM_AZ;
          } else {
            // valle larga: entra di sMax/2 dal bordo piu' vicino al goal
            int dA = angDist(s0, goalSector);
            int dB = angDist(s1, goalSector);
            if (dA <= dB) theta = (s0 + this->sMax/2) % NUM_AZ;             // da s0 in avanti
            else          theta = (s1 - this->sMax/2 + NUM_AZ) % NUM_AZ;    // da s1 indietro
          }

          double costAz = (double)angDist(theta, goalSector);
          double costEl = std::abs(j - goalElLayer) * this->elevCost;
          double cost   = costAz + costEl;
          if (cost < bestCost) { bestCost = cost; bestAz = theta; bestEl = j; }
        }
      }

      // Fallback: se il gate di percorribilita' ha scartato tutto (es. ostacolo a
      // distanza <= lidar_min_range), invece di fermarsi il veicolo punta verso il
      // settore con la magnitudine piu' bassa (la direzione "meno bloccata").
      // E' un comportamento di fuga approssimativo ma impedisce il freeze.
      if (bestAz < 0) {
        double minH = 1e9;
        for (int j = 0; j < NUM_EL; j++) {
          for (int k = 0; k < NUM_AZ; k++) {
            double h = histogram[k + j * NUM_AZ];
            if (h < minH) { minH = h; bestAz = k; bestEl = j; }
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

    // Il topic del contact sensor dipende dalla versione di gz-sim:
    // - il sistema Sensors onora il <topic> dell'SDF (/tethys/contact)
    // - il sistema Contact potrebbe usare il path scopato invece.
    // Ci iscriviamo a ENTRAMBI per sicurezza: uno dei due funzionera'.
    // Diagnosi: gz topic -l | grep contact
    this->dataPtr->node.Subscribe("/tethys/contact",
      &NavigationPrivateData::OnContact, this->dataPtr.get());
    this->dataPtr->node.Subscribe(
      "/world/empty_environment/model/tethys/link/base_link/sensor/contact_sensor/contact",
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

    this->dataPtr->threshold      = _sdf->Get<double>("threshold",       0.001).first;
    this->dataPtr->sMax           = _sdf->Get<int>   ("s_max",           20).first;
    this->dataPtr->smoothL        = _sdf->Get<int>   ("smooth_l",        5).first;
    this->dataPtr->gainSteer      = _sdf->Get<double>("gain_steer",      0.5).first;
    this->dataPtr->gainPitch      = _sdf->Get<double>("gain_pitch",      0.5).first;
    this->dataPtr->radiusArrived  = _sdf->Get<double>("radius_arrived",  2.0).first;
    this->dataPtr->radiusSlowdown = _sdf->Get<double>("radius_slowdown", 15.0).first;
    this->dataPtr->maxIterations  = _sdf->Get<int>   ("max_iterations",  300000).first;

    // --- NUOVO: parametri esponibili anche all'ES ---
    this->dataPtr->elevCost     = _sdf->Get<double>("elev_cost",     10.0).first;
    this->dataPtr->robotRadius  = _sdf->Get<double>("robot_radius",  0.6).first;
    this->dataPtr->safetyMargin = _sdf->Get<double>("safety_margin", 0.6).first;
    this->dataPtr->maxFinAngle  = _sdf->Get<double>("max_fin_angle", 0.15).first;
    this->dataPtr->maxThrust    = _sdf->Get<double>("max_thrust",    31.0).first;

    this->dataPtr->poseCount        = 0;
    this->dataPtr->episodeStartIter = -1;

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
              << " robotRadius=" << this->dataPtr->robotRadius
              << " safetyMargin=" << this->dataPtr->safetyMargin
              << " maxFinAngle=" << this->dataPtr->maxFinAngle
              << " maxIterations=" << this->dataPtr->maxIterations << std::endl;
  }

  void NavigationPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                   gz::sim::EntityComponentManager &) {
    if (_info.paused) return;
    if (this->dataPtr->poseCount < 3) return;

    // CONTROLLO - ogni 100ms
    if (_info.iterations % 100 == 0) {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      // (ri)inizializza il clock relativo all'episodio e le metriche
      if (this->dataPtr->episodeStartIter < 0) {
        this->dataPtr->episodeStartIter  = (int64_t)_info.iterations;
        this->dataPtr->lastMoveIteration = (int64_t)_info.iterations;
        this->dataPtr->pathLength        = 0.0;
        this->dataPtr->minClearance      = std::numeric_limits<double>::infinity();
        this->dataPtr->lastPosX  = this->dataPtr->posX;
        this->dataPtr->lastPosY  = this->dataPtr->posY;
        this->dataPtr->lastPosZ  = this->dataPtr->posZ;
        this->dataPtr->pathPrevX = this->dataPtr->posX;
        this->dataPtr->pathPrevY = this->dataPtr->posY;
        this->dataPtr->pathPrevZ = this->dataPtr->posZ;
      }

      int64_t elapsed = (int64_t)_info.iterations - this->dataPtr->episodeStartIter;

      // FINE EPISODIO
      if (this->dataPtr->episodeState != NavigationPrivateData::EpisodeState::RUNNING) {
        if (!this->dataPtr->resultPublished) {
          std::string tag =
            (this->dataPtr->episodeState == NavigationPrivateData::EpisodeState::ARRIVED)   ? "ARRIVED"   :
            (this->dataPtr->episodeState == NavigationPrivateData::EpisodeState::COLLISION) ? "COLLISION" :
                                                                                              "TIMEOUT";
          gz::msgs::StringMsg msg;
          msg.set_data(this->dataPtr->MakeResult(tag, elapsed));
          this->dataPtr->resultPub.Publish(msg);
          this->dataPtr->resultPublished = true;
          std::cout << "[NAV] Episode ended: " << msg.data() << std::endl;
        }
        return;
      }

      // PATH LENGTH (integra lo spostamento ad ogni tick di controllo)
      {
        double pdx = this->dataPtr->posX - this->dataPtr->pathPrevX;
        double pdy = this->dataPtr->posY - this->dataPtr->pathPrevY;
        double pdz = this->dataPtr->posZ - this->dataPtr->pathPrevZ;
        this->dataPtr->pathLength += std::sqrt(pdx*pdx + pdy*pdy + pdz*pdz);
        this->dataPtr->pathPrevX = this->dataPtr->posX;
        this->dataPtr->pathPrevY = this->dataPtr->posY;
        this->dataPtr->pathPrevZ = this->dataPtr->posZ;
      }

      // TIMEOUT (relativo all'episodio)
      if (elapsed > this->dataPtr->maxIterations) {
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
        this->dataPtr->lastMoveIteration = (int64_t)_info.iterations;
      }

      if ((int64_t)_info.iterations - this->dataPtr->lastMoveIteration > this->dataPtr->stuckTimeout) {
        this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::TIMEOUT;
        std::cout << "[NAV] STUCK detected! Episode ended as TIMEOUT" << std::endl;
        return;
      }

      // DISTANZA DAL GOAL
      double dx = this->dataPtr->goalX - this->dataPtr->posX;
      double dy = this->dataPtr->goalY - this->dataPtr->posY;
      double dz = this->dataPtr->goalZ - this->dataPtr->posZ;
      double distGoal = std::sqrt(dx*dx + dy*dy + dz*dz);

      if (distGoal < this->dataPtr->radiusArrived) {
        this->dataPtr->episodeState = NavigationPrivateData::EpisodeState::ARRIVED;
        this->dataPtr->StopActuators();
        return;
      }

      if (!this->dataPtr->ranges.empty()) {
        this->dataPtr->BuildHistogram();
        this->dataPtr->SmoothHistogram();
      }

      auto [bestDir, bestElLayer] = this->dataPtr->FindBestDirection();

      if (bestDir < 0) {
        // completamente circondato: fermati (lo stuck-timeout chiudera' l'episodio)
        this->dataPtr->StopActuators();
      } else {
        double steerError = bestDir * M_PI / 180.0;
        if (steerError > M_PI) steerError -= 2*M_PI;
        double finCmd = std::clamp(steerError * this->dataPtr->gainSteer,
                                   -this->dataPtr->maxFinAngle, this->dataPtr->maxFinAngle);

        auto [_, goalElev] = this->dataPtr->GoalDirBody();
        double horizFinCmd = std::clamp(-goalElev * this->dataPtr->gainPitch,
                                        -this->dataPtr->maxFinAngle, this->dataPtr->maxFinAngle);

        double thrust = -this->dataPtr->maxThrust;

        if (distGoal < this->dataPtr->radiusSlowdown) {
            thrust *= (distGoal / this->dataPtr->radiusSlowdown);
            horizFinCmd *= (distGoal / this->dataPtr->radiusSlowdown);
        }
        gz::msgs::Double thrustMsg, finMsg, horizMsg;
        thrustMsg.set_data(thrust);
        finMsg.set_data(finCmd);
        horizMsg.set_data(horizFinCmd);

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
      double clr = std::isinf(this->dataPtr->minClearance) ? 999.0 : this->dataPtr->minClearance;
      auto [gSec, gElev] = this->dataPtr->GoalDirBody();
      double headErr = (gSec <= 180) ? (double)gSec : (double)gSec - 360.0;

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\nt=" << _info.iterations/1000 << "s"
                << " dist=" << distGoal << "m"
                << " head=" << headErr << "°"
                << " elev=" << gElev * 180.0/M_PI << "°"
                << " path=" << this->dataPtr->pathLength << "m"
                << " clr="  << clr << "m" << std::endl;
    }
  }
}

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)