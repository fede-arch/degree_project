#ifndef TETHYS_NAVIGATIONPLUGIN_HH_
#define TETHYS_NAVIGATIONPLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <memory>
#include <mutex>
#include <vector>

namespace tethys {

  class NavigationPrivateData;  // classe dati privati

  class NavigationPlugin: public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {

        public: NavigationPlugin();  // costruttore
        public: ~NavigationPlugin(); // distruttore

        public: void Configure( const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &_eventMgr) override;

        public: void PreUpdate( const gz::sim::UpdateInfo &_info,
                                gz::sim::EntityComponentManager &_ecm) override;

        private: std::unique_ptr<NavigationPrivateData> dataPtr; // puntatore a classe dati privati
  };
}

#endif