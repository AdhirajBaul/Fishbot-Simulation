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

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(&BuoyancyPlugin::OnUpdate, this));

        gzmsg << "Fishbot buoyancy plugin loaded\n";
    }

private:

    void OnUpdate()
    {
        const double buoyancy_force = 42.16;

        ignition::math::Vector3d force(
            0.0,
            0.0,
            buoyancy_force);

        // Center of buoyancy relative to base_link frame
        ignition::math::Vector3d cob(
            0.0,  // Forward/backward
            0.0,   // Left/right
            0.0);  // Up/down

        base_link_->AddForceAtRelativePosition(force, cob);
    }

private:

    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)

}