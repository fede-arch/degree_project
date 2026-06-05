#include "NavigationPlugin.hh"
#include <iostream>
#include <cmath>
#include <iomanip>

namespace tethys
{

class NavigationPrivateData
{
  public: gz::transport::Node node;
  public: std::mutex mtx;

  public: std::vector<float> ranges;
  public: int rangeCount = 0;
  public: int verticalCount = 0;

  public: double posX = 0, posY = 0, posZ = 0;
  public: double oriX = 0, oriY = 0, oriZ = 0, oriW = 1;

  public: void OnLidar(const gz::msgs::LaserScan &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->rangeCount = _msg.count();
    this->verticalCount = _msg.vertical_count();
    this->ranges.clear();
    for (int i = 0; i < _msg.ranges_size(); i++)
      this->ranges.push_back(_msg.ranges(i));
  }

  public: void OnPose(const gz::msgs::Pose_V &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    for (int i = 0; i < _msg.pose_size(); i++)
    {
      if (_msg.pose(i).name() == "tethys")
      {
        auto &pos = _msg.pose(i).position();
        auto &ori = _msg.pose(i).orientation();
        this->posX = pos.x();
        this->posY = pos.y();
        this->posZ = pos.z();
        this->oriX = ori.x();
        this->oriY = ori.y();
        this->oriZ = ori.z();
        this->oriW = ori.w();
        break;
      }
    }
  }
};

NavigationPlugin::NavigationPlugin()
  : dataPtr(std::make_unique<NavigationPrivateData>())
{}

NavigationPlugin::~NavigationPlugin() = default;

void NavigationPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  this->dataPtr->node.Subscribe(
    "/tethys/lidar",
    &NavigationPrivateData::OnLidar,
    this->dataPtr.get());

  this->dataPtr->node.Subscribe(
    "/world/empty_environment/dynamic_pose/info",
    &NavigationPrivateData::OnPose,
    this->dataPtr.get());

  std::cout << "[NAV] Plugin configurato!" << std::endl;
}

void NavigationPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) return;
  if (_info.iterations % 1000 != 0) return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n=== NAV DEBUG (t=" << _info.iterations/1000 << "s) ===" << std::endl;

  std::cout << "[POSE] pos=("
            << this->dataPtr->posX << ", "
            << this->dataPtr->posY << ", "
            << this->dataPtr->posZ << ")"
            << " ori=("
            << this->dataPtr->oriX << ", "
            << this->dataPtr->oriY << ", "
            << this->dataPtr->oriZ << ", "
            << this->dataPtr->oriW << ")"
            << std::endl;

  if (!this->dataPtr->ranges.empty())
  {
    auto r = this->dataPtr->ranges;
    std::cout << "[LIDAR] raggi=" << r.size()
              << " (" << this->dataPtr->rangeCount
              << "x" << this->dataPtr->verticalCount << ")"
              << " | avanti=" << r[0]
              << " | destra=" << r[18]
              << " | dietro=" << r[36]
              << " | sinistra=" << r[54]
              << std::endl;
  }
  else
  {
    std::cout << "[LIDAR] nessun dato ancora" << std::endl;
  }
}

}

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)