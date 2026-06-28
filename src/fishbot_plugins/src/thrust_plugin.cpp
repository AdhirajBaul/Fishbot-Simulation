#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{

class ThrustPlugin : public ModelPlugin
{
public:

    void Load(physics::ModelPtr _model,
              sdf::ElementPtr /*_sdf*/) override
    {
        model_ = _model;

        base_link_ = model_->GetLink("base_link");

        tail_joint_ = model_->GetJoint("tail_joint");

        if (!base_link_)
        {
            gzerr << "Could not find base_link!\n";
            return;
        }

        if (!tail_joint_)
        {
            gzerr << "Could not find tail_joint!\n";
            return;
        }

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(&ThrustPlugin::OnUpdate, this));

        gzmsg << "Thrust Plugin Loaded\n";
    }

private:

    void OnUpdate()
    {
        double tail_velocity =
            std::abs(tail_joint_->GetVelocity(0));

        double thrust =
            thrust_coefficient_ * tail_velocity;

        ignition::math::Vector3d thrust_force(
            thrust,
            0,
            0);

        base_link_->AddForce(thrust_force);
    }

private:

    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    physics::JointPtr tail_joint_;

    event::ConnectionPtr update_connection_;

    double thrust_coefficient_ = 3.0;
};

GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin)

}