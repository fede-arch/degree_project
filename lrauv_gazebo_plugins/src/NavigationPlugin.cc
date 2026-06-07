#include "NavigationPlugin.hh"
#include <iostream>
#include <cmath>
#include <iomanip>

namespace tethys {

  // Classe ricevente i messaggi
    class NavigationPrivateData {

          // --- Gazebo transport ---
          public: gz::transport::Node node;
          public: std::mutex mtx;

          // --- Dati lidar ---
          public: std::vector<float> ranges;
          public: int horizCount = 0;    // 72 settori azimuth
          public: int vertCount = 0; // 4 settori elevation

          // --- Dati pose ---
          public: double posX = 0, posY = 0, posZ = 0;
          public: double oriX = 0, oriY = 0, oriZ = 0, oriW = 1;
          public: double yaw = 0, pitch = 0, roll = 0;

          // --- VFH parametri ---
          static constexpr int    NUM_AZ    = 72;
          static constexpr int    NUM_EL    = 4;
          static constexpr double THRESHOLD = 0.005;

          // --- VFH dati ---
          public: std::vector<double> histogram = std::vector<double>(NUM_AZ * NUM_EL, 0.0);
          public: double goalX = 0, goalY = 0, goalZ = 0;



          // CALLBACK LIDAR
          public: void OnLidar(const gz::msgs::LaserScan &_msg){

            std::lock_guard<std::mutex> lock(this->mtx);

            this->ranges.clear();
            this->horizCount = _msg.count();         // 72
            this->vertCount = _msg.vertical_count(); // 4

            for (int i = 0; i < _msg.ranges_size(); i++) // 288 valori (72x4)
              this->ranges.push_back(_msg.ranges(i));
              
          }

          // CALLBACK POSE
          public: void OnPose(const gz::msgs::Pose_V &_msg) {

            std::lock_guard<std::mutex> lock(this->mtx);
            
            for (int i = 0; i < _msg.pose_size(); i++) {
              if (_msg.pose(i).name() == "tethys"){

                auto &pos = _msg.pose(i).position();
                auto &ori = _msg.pose(i).orientation();
                this->posX = pos.x();
                this->posY = pos.y();
                this->posZ = pos.z();
                this->oriX = ori.x();
                this->oriY = ori.y();
                this->oriZ = ori.z();
                this->oriW = ori.w();

                // calcola yaw e pitch dal quaternione
                this->yaw   = std::atan2(2*(oriW*oriZ + oriX*oriY), 1 - 2*(oriY*oriY + oriZ*oriZ));

                this->pitch = std::asin(2*(oriW*oriY - oriZ*oriX));

                this->roll  = std::atan2(2*(oriW*oriX + oriY*oriZ), 1 - 2*(oriX*oriX + oriY*oriY));

                break;
              }
            }
          }

          // BUILD HISTOGRAM
          public: void BuildHistogram() {
            // azzero
            histogram.assign(NUM_AZ * NUM_EL, 0.0);

            for (int j = 0; j < NUM_EL; j++) {
              for (int i = 0; i < NUM_AZ; i++) {
                int   idx = i + j * NUM_AZ;
                float d   = ranges[idx];

                if (std::isinf(d) || d <= 0.0f)
                  histogram[idx] = 0.0;
                else
                  histogram[idx] = 1.0 / (d * d);
              }
            }
          }

          // SMOOTH HISTOGRAM 
          public: void SmoothHistogram() {
            std::vector<double> smoothed(NUM_AZ, 0.0);
            int l = 5; // finestra smoothing

            for (int k = 0; k < NUM_AZ; k++) {
              double sum = 0.0;
              for (int m = -l; m <= l; m++) {
                int idx = (k + m + NUM_AZ) % NUM_AZ; // wrapping 360°
                sum += histogram[idx];
              }
              smoothed[k] = sum / (2*l + 1);
            }

            for (int k = 0; k < NUM_AZ; k++)
              histogram[k] = smoothed[k];
          }

          // GOAL ANGLE 
          public: double GoalAngle() {
            double dx = goalX - posX;
            double dy = goalY - posY;
            double angleWorld = std::atan2(dy, dx); // angolo nel frame mondo

            // avanti Tethys = X- = π nel mondo
            double angleVehicle = angleWorld - yaw - M_PI;

            // normalizza in [0, 2π]
            while (angleVehicle < 0)       angleVehicle += 2*M_PI;
            while (angleVehicle >= 2*M_PI) angleVehicle -= 2*M_PI;

            return angleVehicle;
          }

          // FIND BEST DIRECTION 
          public: int FindBestDirection() {
            double goalAngle = GoalAngle();
            int goalSector = (int)(goalAngle * 180.0 / M_PI / 5.0) % NUM_AZ;

            int bestSector = -1;
            double bestCost = 1e9;

            for (int k = 0; k < NUM_AZ; k++) {
              if (histogram[k] < THRESHOLD) {
                // distanza angolare dal goal con wrapping
                int diff = std::abs(k - goalSector);
                if (diff > NUM_AZ/2) diff = NUM_AZ - diff;
                double cost = (double)diff;

                if (cost < bestCost) {
                  bestCost = cost;
                  bestSector = k;
                }
              }
            }
            return bestSector; // -1 se nessun settore libero
          }
    };

    NavigationPlugin::NavigationPlugin() : dataPtr(std::make_unique<NavigationPrivateData>()) {}

    NavigationPlugin::~NavigationPlugin() = default;

    void NavigationPlugin::Configure(const gz::sim::Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    gz::sim::EntityComponentManager &_ecm,
                                    gz::sim::EventManager &) {

      this->dataPtr->node.Subscribe( "/tethys/lidar", &NavigationPrivateData::OnLidar, this->dataPtr.get());

      this->dataPtr->node.Subscribe( "/world/empty_environment/dynamic_pose/info", &NavigationPrivateData::OnPose, this->dataPtr.get());

      this->dataPtr->goalX = _sdf->Get<double>("goal_x", 0.0).first;
      this->dataPtr->goalY = _sdf->Get<double>("goal_y", 0.0).first;
      this->dataPtr->goalZ = _sdf->Get<double>("goal_z", 0.0).first;
    }

    void NavigationPlugin::PreUpdate( const gz::sim::UpdateInfo &_info,
                                      gz::sim::EntityComponentManager &_ecm) {
                                        
      if (_info.paused) return;
      if (_info.iterations % 1000 != 0) return;

      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

      if (!this->dataPtr->ranges.empty()) {
          this->dataPtr->BuildHistogram();
          this->dataPtr->SmoothHistogram();
      }    

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\n=== NAV DEBUG (t=" << _info.iterations/1000 << "s) ===" << std::endl;

      std::cout << "[POSE] pos=("
            << this->dataPtr->posX << ", "
            << this->dataPtr->posY << ", "
            << this->dataPtr->posZ << ")"
            << " yaw=" << this->dataPtr->yaw * 180.0/M_PI << "°"
            << " pitch=" << this->dataPtr->pitch * 180.0/M_PI << "°"
            << " roll="  << this->dataPtr->roll  * 180.0/M_PI << "°"
            << std::endl;


      std::cout << "[HIST] ";
      for (int i = 0; i < this->dataPtr->NUM_AZ; i++)  {
        double h = this->dataPtr->histogram[i];
        if (h > 0.0)
          std::cout << i << "(" << i*5 << "°)=" << h << " ";
      }
      std::cout << std::endl;

      int bestDir = this->dataPtr->FindBestDirection();
      double goalAng = this->dataPtr->GoalAngle() * 180.0/M_PI;

      std::cout << "[VFH] goal=" << goalAng << "°"
                << " best_sector=" << bestDir
                << " best_angle=" << bestDir * 5 << "°"
                << std::endl;
    }
}

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)