#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{

class DragPlugin : public ModelPlugin
{
public:

    void Load(physics::ModelPtr _model,
              sdf::ElementPtr /*_sdf*/) override
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
                std::bind(&DragPlugin::OnUpdate, this));

        gzmsg << "Drag Plugin Loaded\n";
    }

private:

    void OnUpdate()
    {
        ignition::math::Vector3d velocity =
            base_link_->WorldLinearVel();

        double speed = velocity.Length();

        ignition::math::Vector3d drag_force =
            -drag_coefficient_ * speed * velocity;

        base_link_->AddForce(drag_force);
    }

private:

    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    event::ConnectionPtr update_connection_;

    double drag_coefficient_ = 20.0;
};

GZ_REGISTER_MODEL_PLUGIN(DragPlugin)

}