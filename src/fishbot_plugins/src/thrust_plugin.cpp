#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>

#include <cmath>
#include <functional>

namespace gazebo
{

class ThrustPlugin : public ModelPlugin
{
public:
    void Load(
        physics::ModelPtr _model,
        sdf::ElementPtr /* _sdf */) override
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
                std::bind(
                    &ThrustPlugin::OnUpdate,
                    this));

        gzmsg << "Finite-difference thrust plugin loaded\n";
    }

private:
    ignition::math::Vector3d CalculateModelCOM() const
    {
        double total_mass = 0.0;

        ignition::math::Vector3d weighted_position =
            ignition::math::Vector3d::Zero;

        for (const auto &link : model_->GetLinks())
        {
            if (!link || !link->GetInertial())
                continue;

            const double mass =
                link->GetInertial()->Mass();

            const ignition::math::Vector3d link_com_world =
                link->WorldCoGPose().Pos();

            weighted_position +=
                mass * link_com_world;

            total_mass += mass;
        }

        if (total_mass <= 0.0)
            return base_link_->WorldCoGPose().Pos();

        return weighted_position / total_mass;
    }

    void OnUpdate()
    {
        const common::Time current_time =
            model_->GetWorld()->SimTime();

        const double current_position =
            tail_joint_->Position(0);

        /*
         * On the first update, there is no previous position or
         * time available, so initialize them and wait for the
         * next update.
         */
        if (!initialized_)
        {
            previous_position_ = current_position;
            previous_time_ = current_time;

            filtered_tail_velocity_ = 0.0;
            initialized_ = true;

            return;
        }

        const double dt =
            (current_time - previous_time_).Double();

        /*
         * This can happen if the simulation is reset or if two
         * updates have the same simulation timestamp.
         */
        if (dt <= 0.0)
        {
            previous_position_ = current_position;
            previous_time_ = current_time;
            filtered_tail_velocity_ = 0.0;

            return;
        }

        /*
         * Calculate angular velocity manually:
         *
         * angular velocity = change in angle / change in time
         */
        const double raw_tail_velocity =
            (current_position - previous_position_) / dt;

        previous_position_ = current_position;
        previous_time_ = current_time;

        /*
         * Low-pass filtering reduces numerical spikes caused by
         * position controllers and discrete simulation updates.
         */
        filtered_tail_velocity_ =
            velocity_filter_alpha_ * raw_tail_velocity +
            (1.0 - velocity_filter_alpha_) *
                filtered_tail_velocity_;

        /*
         * abs() means both halves of the tail stroke generate
         * forward thrust.
         */
        const double thrust =
            thrust_coefficient_ *
            std::abs(filtered_tail_velocity_);

        if (thrust < minimum_thrust_)
            return;

        /*
         * Define forward as +X in the fish's local frame.
         * Change 1.0 to -1.0 if your fish faces local -X.
         */
        const ignition::math::Vector3d local_forward(
            1.0,
            0.0,
            0.0);

        /*
         * Convert the local forward direction to a world-frame
         * direction so thrust follows the fish as it turns.
         */
        const ignition::math::Vector3d forward_world =
            base_link_->WorldPose().Rot() *
            local_forward;

        const ignition::math::Vector3d thrust_force_world =
            thrust * forward_world;

        /*
         * Apply thrust through the whole fishbot's instantaneous
         * COM to avoid introducing an artificial external torque.
         */
        const ignition::math::Vector3d model_com_world =
            CalculateModelCOM();

        base_link_->AddForceAtWorldPosition(
            thrust_force_world,
            model_com_world);

        /*
         * Print diagnostic values periodically.
         */
        if (++debug_counter_ >= 500)
        {
            debug_counter_ = 0;

            gzmsg
                << "Tail position: "
                << current_position
                << " rad | Raw velocity: "
                << raw_tail_velocity
                << " rad/s | Filtered velocity: "
                << filtered_tail_velocity_
                << " rad/s | Thrust: "
                << thrust
                << " N | Base velocity: "
                << base_link_->WorldLinearVel()
                << "\n";
        }
    }

private:
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    physics::JointPtr tail_joint_;

    event::ConnectionPtr update_connection_;

    /*
     * Variables required for finite-difference velocity.
     */
    bool initialized_ = false;

    double previous_position_ = 0.0;
    common::Time previous_time_;

    double filtered_tail_velocity_ = 0.0;

    /*
     * 1.0 means no filtering.
     * Smaller values produce smoother velocity estimates.
     */
    double velocity_filter_alpha_ = 0.9;

    /*
     * Start conservatively. A value of 100 can produce extremely
     * large forces once velocity calculation begins working.
     */
    double thrust_coefficient_ = 0.5;

    double minimum_thrust_ = 0.001;

    unsigned int debug_counter_ = 0;
};

GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin)

}