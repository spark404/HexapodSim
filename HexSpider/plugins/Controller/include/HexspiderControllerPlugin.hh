#ifndef HEXSPIDERCONTROLLERPLUGIN_HH_
#define HEXSPIDERCONTROLLERPLUGIN_HH_

#include <gz/sim/System.hh>

namespace gz::sim::systems {

    class GZ_SIM_VISIBLE HexspiderControllerPlugin
            : public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemPostUpdate,
              public gz::sim::ISystemPreUpdate,
              public gz::sim::ISystemReset {
    public:

        HexspiderControllerPlugin();

        ~HexspiderControllerPlugin();

        void
        Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm,
                  EventManager &_eventMgr) override;

        void Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

        void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

        void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override;

    private:
        class Impl;
        std::unique_ptr<Impl> impl;

        class Leg;
    };

}

#endif /* HEXSPIDERCONTROLLERPLUGIN_HH_ */