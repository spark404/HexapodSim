#include "HexspiderControllerPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/physics/detail/FeatureList.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>


GZ_ADD_PLUGIN(gz::sim::systems::HexspiderControllerPlugin, gz::sim::System,
              gz::sim::systems::HexspiderControllerPlugin::ISystemConfigure,
              gz::sim::systems::HexspiderControllerPlugin::ISystemPostUpdate,
              gz::sim::systems::HexspiderControllerPlugin::ISystemReset,
              gz::sim::systems::HexspiderControllerPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::HexspiderControllerPlugin, "HexspiderControllerPlugin")

namespace gz::sim::systems {

    class HexspiderControllerPlugin::Impl {
    public:
        World world{kNullEntity};
    public:
        std::string worldName;
    public:
        Model parentModel{kNullEntity};
    public:
        std::string parentModelName;

        std::vector<HexspiderControllerPlugin::Leg> legs;
    };

    class HexspiderControllerPlugin::Leg {

    public:
        std::string leg_identifier;

        std::string chassis_joint_name;
        Entity chassis_joint{kNullEntity};

        std::string upper_joint_name;
        Entity upper_joint{kNullEntity};

        std::string lower_joint_name;
        Entity lower_joint{kNullEntity};
    };

    HexspiderControllerPlugin::HexspiderControllerPlugin() :
            impl(std::make_unique<HexspiderControllerPlugin::Impl>()) {}

    HexspiderControllerPlugin::~HexspiderControllerPlugin() = default;

    void HexspiderControllerPlugin::Configure(const gz::sim::Entity &_entity,
                                              const std::shared_ptr<const sdf::Element> &_sdf,
                                              gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) {
        gzlog << "[HexspiderControllerPlugin] Configure\n";

        this->impl->parentModel = Model(_entity);
        if (!this->impl->parentModel.Valid(_ecm)) {
            gzerr << "HexspiderControllerPlugin should be attached to a model. "
                     "Failed to initialize.\n";
            return;
        }
        this->impl->parentModelName = this->impl->parentModel.Name(_ecm);

        this->impl->world = World(_ecm.EntityByComponents(components::World()));
        if (!this->impl->world.Valid(_ecm)) {
            gzerr << "HexspiderControllerPlugin - world not found. "
                     "Failed to initialize.\n";
            return;
        }
        if (this->impl->world.Name(_ecm).has_value()) {
            this->impl->worldName = this->impl->world.Name(_ecm).value();
        }

        auto sdfLegElem = _sdf->FindElement("leg");
        while (sdfLegElem) {
            Leg leg;

            auto nameParam = sdfLegElem->GetAttribute("name");
            leg.leg_identifier = nameParam->GetAsString();

            auto sdfChassisTwistElem = sdfLegElem->FindElement("chassis_twist");
            if (sdfChassisTwistElem) {
                leg.chassis_joint_name = sdfChassisTwistElem->Get<std::string>();
            }

            auto sdfUpperElem = sdfLegElem->FindElement("upper");
            if (sdfUpperElem) {
                leg.upper_joint_name = sdfUpperElem->Get<std::string>();
            }

            auto sdfLowerElem = sdfLegElem->FindElement("lower");
            if (sdfLowerElem) {
                leg.lower_joint_name = sdfLowerElem->Get<std::string>();
            }

            gzlog << "[" << this->impl->parentModelName << "] "
                  << "Add leg " <<  leg.leg_identifier << " joints: "
                  << leg.chassis_joint_name <<  ", "
                  << leg.upper_joint_name << ", "
                  << leg.lower_joint_name << "\n";

            this->impl->legs.push_back(leg);
            sdfLegElem = sdfLegElem->GetNextElement("leg");
        }

        gzlog << "[" << this->impl->parentModelName << "] "
              << "Hexspider ready to go." << "\n";
    }

    void HexspiderControllerPlugin::Reset(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {

    }

    void
    HexspiderControllerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
        if (_info.dt < std::chrono::steady_clock::duration::zero())
        {
            gzwarn << "Detected jump back in time ["
                   << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                   << "s]. HexspiderControllerPlugin may not work properly." << std::endl;
        }

        for (auto &item: this->impl->legs) {
            if (item.chassis_joint == kNullEntity && !item.chassis_joint_name.empty()) {
                Entity joint = this->impl->parentModel.JointByName(_ecm, item.chassis_joint_name);
                if (joint != kNullEntity) {
                    item.chassis_joint = joint;
                }
                gzlog << "[" << this->impl->parentModelName << "] "
                      << item.leg_identifier << " chassis joint is entity " << joint << "\n";
            }

            if (item.upper_joint == kNullEntity && !item.upper_joint_name.empty()) {
                Entity joint = this->impl->parentModel.JointByName(_ecm, item.upper_joint_name);
                if (joint != kNullEntity) {
                    item.upper_joint = joint;
                    gzlog << "[" << this->impl->parentModelName << "] "
                          << item.leg_identifier << " upper joint is entity " << joint << "\n";
                }
            }

            if (item.lower_joint == kNullEntity && !item.lower_joint_name.empty()) {
                Entity joint = this->impl->parentModel.JointByName(_ecm, item.lower_joint_name);
                if (joint != kNullEntity) {
                    item.lower_joint = joint;
                    gzlog << "[" << this->impl->parentModelName << "] "
                          << item.leg_identifier << " lower joint is entity " << joint << "\n";
                }
            }
        }

        // Nothing to do
        if (_info.paused)
            return;


    }

    void HexspiderControllerPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                               const gz::sim::EntityComponentManager &_ecm) {

    }

}