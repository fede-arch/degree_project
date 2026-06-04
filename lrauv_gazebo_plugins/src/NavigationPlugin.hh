#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <mutex>
#include <string>

namespace tethys
{
  class NavigationPrivateData;

  /// \brief Navigation plugin for Tethys AUV
  /// Drives the vehicle towards a 3D target point
  /// using proportional controllers on thrust, pitch and yaw.
  class NavigationPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: NavigationPlugin();
    public: ~NavigationPlugin() = default;

    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/);

    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm);

    private: std::unique_ptr<NavigationPrivateData> dataPtr;
  };
}
