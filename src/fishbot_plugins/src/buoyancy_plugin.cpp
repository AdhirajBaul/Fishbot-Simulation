#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{

class BuoyancyPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr)
    {
        model_ = _model;

        base_link_ = model_->GetLink("base_link");

        if (!base_link_)
        {
            gzerr << "Could not find base_link!\n";
            return;
        }

        world_ = model_->GetWorld();

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(&BuoyancyPlugin::OnUpdate, this));

        gzmsg << "Fishbot dynamic-COM buoyancy plugin loaded\n";
    }

private:
    void OnUpdate()
    {
        const auto links = model_->GetLinks();

        double total_mass = 0.0;
        ignition::math::Vector3d weighted_position =
            ignition::math::Vector3d::Zero;

        // Calculate instantaneous COM of the complete articulated fishbot
        for (const auto &link : links)
        {
            if (!link || !link->GetInertial())
                continue;

            const double mass = link->GetInertial()->Mass();

            // Position of this link's centre of gravity in world coordinates
            const ignition::math::Vector3d link_com_world =
                link->WorldCoGPose().Pos();

            weighted_position += mass * link_com_world;
            total_mass += mass;
        }

        if (total_mass <= 0.0)
            return;

        const ignition::math::Vector3d model_com_world =
            weighted_position / total_mass;

        // Read gravity from the Gazebo world instead of hard-coding 9.81
        const ignition::math::Vector3d gravity =
            world_->Gravity();

        // Equal and opposite to the model's total weight
        const ignition::math::Vector3d buoyancy_force =
            -total_mass * gravity;

        // Apply the resultant buoyancy at the instantaneous model COM
        base_link_->AddForceAtWorldPosition(
            buoyancy_force,
            model_com_world);
    }

private:
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)

}