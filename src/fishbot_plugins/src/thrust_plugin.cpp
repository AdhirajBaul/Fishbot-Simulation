#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <cmath>
#include <functional>

namespace gazebo
{

class ThrustPlugin : public ModelPlugin
{
public:
    void Load(
        physics::ModelPtr _model,
        sdf::ElementPtr _sdf) override
    {
        model_ = _model;

        base_link_ =
            model_->GetLink("base_link");

        tail_link_ =
            model_->GetLink("tail_link");

        tail_joint_ =
            model_->GetJoint("tail_joint");

        if (!base_link_)
        {
            gzerr << "Could not find base_link!\n";
            return;
        }

        if (!tail_link_)
        {
            gzerr << "Could not find tail_link!\n";
            return;
        }

        if (!tail_joint_)
        {
            gzerr << "Could not find tail_joint!\n";
            return;
        }

        /*
         * Optional parameters supplied through the URDF/Xacro.
         */
        if (_sdf->HasElement("thrust_coefficient"))
        {
            thrust_coefficient_ =
                _sdf->Get<double>(
                    "thrust_coefficient");
        }

        if (_sdf->HasElement("steering_coefficient"))
        {
            steering_coefficient_ =
                _sdf->Get<double>(
                    "steering_coefficient");
        }

        if (_sdf->HasElement("velocity_filter_alpha"))
        {
            velocity_filter_alpha_ =
                _sdf->Get<double>(
                    "velocity_filter_alpha");
        }

        if (_sdf->HasElement("water_density"))
        {
            water_density_ =
                _sdf->Get<double>(
                    "water_density");
        }

        if (_sdf->HasElement("tail_area"))
        {
            tail_area_ =
                _sdf->Get<double>(
                    "tail_area");
        }

        if (_sdf->HasElement("tail_lever_length"))
        {
            tail_lever_length_ =
                _sdf->Get<double>(
                    "tail_lever_length");
        }

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(
                    &ThrustPlugin::OnUpdate,
                    this));

        gzmsg
            << "Tail thrust and steering plugin loaded\n"
            << "Forward thrust coefficient: "
            << thrust_coefficient_ << "\n"
            << "Steering coefficient: "
            << steering_coefficient_ << "\n";
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

            if (mass <= 0.0)
                continue;

            const ignition::math::Vector3d link_com_world =
                link->WorldCoGPose().Pos();

            weighted_position +=
                mass * link_com_world;

            total_mass += mass;
        }

        if (total_mass <= 0.0)
        {
            return base_link_->WorldCoGPose().Pos();
        }

        return weighted_position / total_mass;
    }

    void OnUpdate()
    {
        const common::Time current_time =
            model_->GetWorld()->SimTime();

        const double current_tail_position =
            tail_joint_->Position(0);

        /*
         * The first update only establishes the initial position
         * and simulation time.
         */
        if (!initialized_)
        {
            previous_position_ =
                current_tail_position;

            previous_time_ =
                current_time;

            filtered_tail_velocity_ = 0.0;

            initialized_ = true;

            return;
        }

        const double dt =
            (current_time - previous_time_).Double();

        /*
         * A non-positive timestep can occur when Gazebo resets.
         */
        if (dt <= 0.0)
        {
            previous_position_ =
                current_tail_position;

            previous_time_ =
                current_time;

            filtered_tail_velocity_ = 0.0;

            return;
        }

        /*
         * Estimate tail angular velocity from position:
         *
         * omega = change in angle / change in time
         */
        const double raw_tail_velocity =
            (
                current_tail_position -
                previous_position_
            ) / dt;

        previous_position_ =
            current_tail_position;

        previous_time_ =
            current_time;

        /*
         * Low-pass filter the finite-difference velocity.
         */
        filtered_tail_velocity_ =
            velocity_filter_alpha_ *
                raw_tail_velocity +
            (1.0 - velocity_filter_alpha_) *
                filtered_tail_velocity_;

        const ignition::math::Quaterniond body_rotation =
            base_link_->WorldPose().Rot();

        ApplyForwardThrust(
            body_rotation);

        ApplyTailSteering(
            body_rotation,
            current_tail_position);

        PrintDebugInformation(
            current_tail_position,
            raw_tail_velocity);
    }

    void ApplyForwardThrust(
        const ignition::math::Quaterniond &body_rotation)
    {
        /*
         * Both directions of tail motion generate positive
         * forward thrust.
         */
        const double thrust =
            thrust_coefficient_ *
            std::abs(filtered_tail_velocity_);

        last_forward_thrust_ = thrust;

        /*
         * Do not return from OnUpdate here because the steering
         * calculation still needs to execute.
         */
        if (thrust < minimum_thrust_)
            return;

        const ignition::math::Vector3d forward_body(
            1.0,
            0.0,
            0.0);

        /*
         * Convert fish-local +X into world coordinates.
         */
        const ignition::math::Vector3d forward_world =
            body_rotation.RotateVector(
                forward_body);

        const ignition::math::Vector3d thrust_force_world =
            thrust * forward_world;

        /*
         * Applying the forward thrust through the complete fish
         * COM prevents the forward force from adding an
         * artificial pitch or yaw torque.
         */
        const ignition::math::Vector3d model_com_world =
            CalculateModelCOM();

        base_link_->AddForceAtWorldPosition(
            thrust_force_world,
            model_com_world);
    }

    void ApplyTailSteering(
        const ignition::math::Quaterniond &body_rotation,
        const double tail_position)
    {
        /*
         * Convert the fish's world velocity into its body frame.
         */
        const ignition::math::Vector3d velocity_body =
            body_rotation.RotateVectorReverse(
                base_link_->WorldLinearVel());

        const double forward_speed =
            velocity_body.X();

        /*
         * Approximate the lateral speed of the tail caused by its
         * angular motion:
         *
         * v_tail = r * omega
         */
        const double tail_lateral_speed =
            tail_lever_length_ *
            filtered_tail_velocity_;

        /*
         * The effective water-flow speed includes both:
         *
         * 1. Forward motion of the complete fish
         * 2. Lateral motion of the oscillating tail
         */
        const double effective_flow_squared =
            forward_speed * forward_speed +
            tail_lateral_speed * tail_lateral_speed;

        if (effective_flow_squared <
            minimum_flow_squared_)
        {
            last_steering_force_ = 0.0;
            return;
        }

        /*
         * Simplified lateral tail force:
         *
         * Fy = 0.5 rho A C v² sin(theta)
         *
         * A symmetric tail oscillation has approximately zero
         * average lateral force.
         *
         * Adding a positive or negative tail bias produces a
         * nonzero average lateral force and therefore a turn.
         */
        const double steering_force_y =
            0.5 *
            water_density_ *
            tail_area_ *
            steering_coefficient_ *
            effective_flow_squared *
            std::sin(tail_position);

        last_steering_force_ =
            steering_force_y;

        const ignition::math::Vector3d steering_force_body(
            0.0,
            steering_force_y,
            0.0);

        /*
         * Convert the body-frame lateral force into world
         * coordinates.
         */
        const ignition::math::Vector3d steering_force_world =
            body_rotation.RotateVector(
                steering_force_body);

        /*
         * Apply lateral force at the tail COM.
         *
         * Since the tail is behind the complete fish COM,
         * this force creates a yaw moment.
         */
        tail_link_->AddForce(
            steering_force_world);
    }

    void PrintDebugInformation(
        const double tail_position,
        const double raw_tail_velocity)
    {
        if (++debug_counter_ < 500)
            return;

        debug_counter_ = 0;

        const double yaw =
            base_link_->WorldPose().Rot().Yaw();

        const double yaw_rate =
            base_link_->WorldAngularVel().Z();

        gzmsg
            << "Tail position: "
            << tail_position
            << " rad | Raw velocity: "
            << raw_tail_velocity
            << " rad/s | Filtered velocity: "
            << filtered_tail_velocity_
            << " rad/s\n"
            << "Forward thrust: "
            << last_forward_thrust_
            << " N | Lateral tail force: "
            << last_steering_force_
            << " N | Yaw: "
            << yaw
            << " rad | Yaw rate: "
            << yaw_rate
            << " rad/s\n";
    }

private:
    physics::ModelPtr model_;

    physics::LinkPtr base_link_;
    physics::LinkPtr tail_link_;

    physics::JointPtr tail_joint_;

    event::ConnectionPtr update_connection_;

    bool initialized_ = false;

    double previous_position_ = 0.0;
    common::Time previous_time_;

    double filtered_tail_velocity_ = 0.0;

    /*
     * 0.9 follows the measured tail velocity closely while
     * removing some numerical noise.
     */
    double velocity_filter_alpha_ = 0.9;

    /*
     * Forward force model:
     *
     * Fx = Kt * |tail angular velocity|
     */
    double thrust_coefficient_ = 0.5;

    /*
     * Water and tail parameters.
     */
    double water_density_ = 1000.0;

    // Tail surface: 0.10 m × 0.13 m
    double tail_area_ = 0.013;

    /*
     * Distance from the tail joint to its COM according to your
     * URDF inertial origin.
     */
    double tail_lever_length_ = 0.06;

    /*
     * Tune this to change turning strength.
     */
    double steering_coefficient_ = 0.5;

    double minimum_thrust_ = 0.001;
    double minimum_flow_squared_ = 0.0001;

    double last_forward_thrust_ = 0.0;
    double last_steering_force_ = 0.0;

    unsigned int debug_counter_ = 0;
};

GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin)

}