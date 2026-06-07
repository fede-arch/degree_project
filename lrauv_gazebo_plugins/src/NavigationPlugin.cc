#include "NavigationPlugin.hh"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <algorithm>

namespace tethys {

  class NavigationPrivateData {

    // --- Gazebo transport ---
    public: gz::transport::Node node;
    public: gz::transport::Node::Publisher thrustPub;
    public: gz::transport::Node::Publisher vertFinPub;
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
    static constexpr int    NUM_AZ    = 360;
    static constexpr int    NUM_EL    = 4;
    static constexpr double THRESHOLD = 0.001;

    // --- VFH dati ---
    public: std::vector<double> histogram = std::vector<double>(NUM_AZ * NUM_EL, 0.0);
    public: double goalX = 0, goalY = 0, goalZ = 0;

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
      std::vector<double> smoothed(NUM_AZ, 0.0);
      int l = 5;
      for (int k = 0; k < NUM_AZ; k++) {
        double sum = 0.0;
        for (int m = -l; m <= l; m++)
          sum += histogram[(k + m + NUM_AZ) % NUM_AZ];
        smoothed[k] = sum / (2*l + 1);
      }
      for (int k = 0; k < NUM_AZ; k++)
        histogram[k] = smoothed[k];
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

    // FIND BEST DIRECTION
    public: int FindBestDirection() {
      double goalAngle = GoalAngle();
      int goalSector = (int)(goalAngle * 180.0 / M_PI) % NUM_AZ;

      if (histogram[goalSector] < THRESHOLD)
        return goalSector;

      static constexpr int S_MAX = 20;

      std::vector<std::pair<int,int>> valleys;
      int valleyStart = -1;
      for (int k = 0; k < NUM_AZ; k++) {
        if (histogram[k] < THRESHOLD) {
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

      if (valleys.empty()) return -1;

      auto angDist = [&](int a, int b) {
        int d = std::abs(a - b);
        if (d > NUM_AZ/2) d = NUM_AZ - d;
        return d;
      };

      int bestSector = -1;
      double bestCost = 1e9;

      for (auto &v : valleys) {
        int k_n = v.first;
        int k_f = v.second;
        int width = k_f - k_n;

        if (angDist(k_f, goalSector) < angDist(k_n, goalSector))
          std::swap(k_n, k_f);

        int theta;
        if (width <= S_MAX) {
          theta = (k_n + k_f) / 2;
        } else {
          theta = (k_n < k_f) ? k_n + S_MAX/2 : k_n - S_MAX/2;
          theta = (theta + NUM_AZ) % NUM_AZ;
        }

        double cost = (double)angDist(theta, goalSector);
        if (theta > NUM_AZ/2) cost += 5.0;

        if (cost < bestCost) {
          bestCost = cost;
          bestSector = theta;
        }
      }
      return bestSector;
    }
  };

  NavigationPlugin::NavigationPlugin()
    : dataPtr(std::make_unique<NavigationPrivateData>()) {}

  NavigationPlugin::~NavigationPlugin() = default;

  void NavigationPlugin::Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
  {
    this->dataPtr->node.Subscribe("/tethys/lidar",
      &NavigationPrivateData::OnLidar, this->dataPtr.get());

    this->dataPtr->node.Subscribe(
      "/world/empty_environment/dynamic_pose/info",
      &NavigationPrivateData::OnPose, this->dataPtr.get());

    this->dataPtr->thrustPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
      "/model/tethys/joint/propeller_joint/cmd_thrust");

    this->dataPtr->vertFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
      "/model/tethys/joint/vertical_fins_joint/0/cmd_pos");

    this->dataPtr->goalX = _sdf->Get<double>("goal_x", 0.0).first;
    this->dataPtr->goalY = _sdf->Get<double>("goal_y", 0.0).first;
    this->dataPtr->goalZ = _sdf->Get<double>("goal_z", 0.0).first;

    std::cout << "[NAV] goal=("
              << this->dataPtr->goalX << ", "
              << this->dataPtr->goalY << ", "
              << this->dataPtr->goalZ << ")" << std::endl;
  }

  void NavigationPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &)
  {
    if (_info.paused) return;

    // CONTROLLO - ogni 100ms
    if (_info.iterations % 100 == 0) {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      double dx = this->dataPtr->goalX - this->dataPtr->posX;
      double dy = this->dataPtr->goalY - this->dataPtr->posY;
      if (std::sqrt(dx*dx + dy*dy) < 2.0) {
        gz::msgs::Double stop;
        stop.set_data(0.0);
        this->dataPtr->thrustPub.Publish(stop);
        this->dataPtr->vertFinPub.Publish(stop);
        return;
      }

      if (!this->dataPtr->ranges.empty()) {
        this->dataPtr->BuildHistogram();
        this->dataPtr->SmoothHistogram();
      }

      int bestDir = this->dataPtr->FindBestDirection();
      if (bestDir >= 0) {
        double steerError = bestDir * M_PI / 180.0;
        if (steerError > M_PI) steerError -= 2*M_PI;
        double finCmd = std::clamp(steerError * 0.5, -0.261799, 0.261799);

        gz::msgs::Double thrustMsg, finMsg;
        thrustMsg.set_data(-31.0);
        finMsg.set_data(finCmd);
        this->dataPtr->thrustPub.Publish(thrustMsg);
        this->dataPtr->vertFinPub.Publish(finMsg);
      }
    }

    // STAMPA - ogni 1s
    if (_info.iterations % 1000 == 0) {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      double goalAng = this->dataPtr->GoalAngle() * 180.0/M_PI;
      int bestDir = this->dataPtr->FindBestDirection();
      double steerError = bestDir * M_PI / 180.0;
      if (steerError > M_PI) steerError -= 2*M_PI;
      double finCmd = std::clamp(steerError * 0.5, -0.261799, 0.261799);

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\n=== t=" << _info.iterations/1000 << "s ===" << std::endl;
      std::cout << "[POSE]  pos=(" << this->dataPtr->posX << ", "
                << this->dataPtr->posY << ", " << this->dataPtr->posZ
                << ") yaw=" << this->dataPtr->yaw * 180.0/M_PI << "°" << std::endl;
      std::cout << "[GOAL]  angle=" << goalAng << "°" << std::endl;
      std::cout << "[VFH]   best_angle=" << bestDir << "°" << std::endl;
      std::cout << "[CTRL]  steerError=" << steerError * 180.0/M_PI
                << "° finCmd=" << finCmd << " rad" << std::endl;
      std::cout << "[FIN]   vert=" << this->dataPtr->vertFinAngle * 180.0/M_PI
                << "° horiz=" << this->dataPtr->horizFinAngle * 180.0/M_PI
                << "°" << std::endl;
    }
  }
}

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)