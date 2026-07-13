#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>

#include <cmath>
#include <functional>

namespace gazebo
{

class DragPlugin : public ModelPlugin
{
public:
    void Load(
        physics::ModelPtr _model,
        sdf::ElementPtr _sdf) override
    {
        model_ = _model;
        base_link_ = model_->GetLink("base_link");

        if (!base_link_)
        {
            gzerr << "Could not find base_link!\n";
            return;
        }

        // Optional values supplied from URDF/Xacro
        if (_sdf->HasElement("linear_drag_x"))
        {
            linear_drag_x_ =
                _sdf->Get<double>("linear_drag_x");
        }

        if (_sdf->HasElement("linear_drag_y"))
        {
            linear_drag_y_ =
                _sdf->Get<double>("linear_drag_y");
        }

        if (_sdf->HasElement("linear_drag_z"))
        {
            linear_drag_z_ =
                _sdf->Get<double>("linear_drag_z");
        }

        if (_sdf->HasElement("roll_drag"))
        {
            roll_drag_ =
                _sdf->Get<double>("roll_drag");
        }

        if (_sdf->HasElement("pitch_drag"))
        {
            pitch_drag_ =
                _sdf->Get<double>("pitch_drag");
        }

        if (_sdf->HasElement("yaw_drag"))
        {
            yaw_drag_ =
                _sdf->Get<double>("yaw_drag");
        }

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(
                    &DragPlugin::OnUpdate,
                    this));

        gzmsg
            << "3D hydrodynamic drag plugin loaded\n"
            << "Linear coefficients: "
            << linear_drag_x_ << ", "
            << linear_drag_y_ << ", "
            << linear_drag_z_ << "\n"
            << "Angular coefficients: "
            << roll_drag_ << ", "
            << pitch_drag_ << ", "
            << yaw_drag_ << "\n";
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
        const ignition::math::Quaterniond body_rotation =
            base_link_->WorldPose().Rot();

        ApplyLinearDrag(body_rotation);
        ApplyAngularDrag(body_rotation);

        PrintDebugInformation();
    }

    void ApplyLinearDrag(
        const ignition::math::Quaterniond &body_rotation)
    {
        /*
         * Gazebo gives velocity in world coordinates.
         */
        const ignition::math::Vector3d velocity_world =
            base_link_->WorldLinearVel();

        /*
         * Convert world velocity into the fish's body frame.
         *
         * Body X: forward/backward
         * Body Y: sideways
         * Body Z: vertical relative to the fish
         */
        const ignition::math::Vector3d velocity_body =
            body_rotation.RotateVectorReverse(
                velocity_world);

        /*
         * Quadratic drag:
         *
         * F = -C |v| v
         */
        const double drag_x =
            -linear_drag_x_ *
            std::abs(velocity_body.X()) *
            velocity_body.X();

        const double drag_y =
            -linear_drag_y_ *
            std::abs(velocity_body.Y()) *
            velocity_body.Y();

        const double drag_z =
            -linear_drag_z_ *
            std::abs(velocity_body.Z()) *
            velocity_body.Z();

        const ignition::math::Vector3d drag_force_body(
            drag_x,
            drag_y,
            drag_z);

        /*
         * AddForceAtWorldPosition() expects a world-frame force.
         */
        const ignition::math::Vector3d drag_force_world =
            body_rotation.RotateVector(
                drag_force_body);

        const ignition::math::Vector3d model_com_world =
            CalculateModelCOM();

        /*
         * Apply the resultant drag through the complete model COM
         * so linear drag does not create an artificial torque.
         */
        base_link_->AddForceAtWorldPosition(
            drag_force_world,
            model_com_world);

        last_velocity_body_ = velocity_body;
        last_drag_force_body_ = drag_force_body;
    }

    void ApplyAngularDrag(
        const ignition::math::Quaterniond &body_rotation)
    {
        /*
         * Gazebo supplies angular velocity in world coordinates.
         */
        const ignition::math::Vector3d angular_velocity_world =
            base_link_->WorldAngularVel();

        /*
         * Convert angular velocity into body coordinates:
         *
         * X: roll rate
         * Y: pitch rate
         * Z: yaw rate
         */
        const ignition::math::Vector3d angular_velocity_body =
            body_rotation.RotateVectorReverse(
                angular_velocity_world);

        const double roll_damping_torque =
            -roll_drag_ *
            std::abs(angular_velocity_body.X()) *
            angular_velocity_body.X();

        const double pitch_damping_torque =
            -pitch_drag_ *
            std::abs(angular_velocity_body.Y()) *
            angular_velocity_body.Y();

        const double yaw_damping_torque =
            -yaw_drag_ *
            std::abs(angular_velocity_body.Z()) *
            angular_velocity_body.Z();

        const ignition::math::Vector3d drag_torque_body(
            roll_damping_torque,
            pitch_damping_torque,
            yaw_damping_torque);

        /*
         * AddTorque() expects world-frame torque.
         */
        const ignition::math::Vector3d drag_torque_world =
            body_rotation.RotateVector(
                drag_torque_body);

        base_link_->AddTorque(
            drag_torque_world);

        last_angular_velocity_body_ =
            angular_velocity_body;

        last_drag_torque_body_ =
            drag_torque_body;
    }

    void PrintDebugInformation()
    {
        if (++debug_counter_ < 500)
            return;

        debug_counter_ = 0;

        const double yaw =
            base_link_->WorldPose().Rot().Yaw();

        gzmsg
            << "Body velocity: "
            << last_velocity_body_
            << " m/s | Linear drag: "
            << last_drag_force_body_
            << " N\n"
            << "Body angular velocity: "
            << last_angular_velocity_body_
            << " rad/s | Angular drag: "
            << last_drag_torque_body_
            << " Nm | Yaw: "
            << yaw
            << " rad\n";
    }

private:
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;

    event::ConnectionPtr update_connection_;

    /*
     * Linear drag coefficients.
     *
     * A streamlined fish should generally have less forward drag
     * than sideways or vertical drag.
     */
    double linear_drag_x_ = 20.0;
    double linear_drag_y_ = 40.0;
    double linear_drag_z_ = 30.0;

    /*
     * Angular drag coefficients.
     *
     * Pitch damping is deliberately moderate so the fins can still
     * control pitch during the next development step.
     */
    double roll_drag_ = 0.25;
    double pitch_drag_ = 0.50;
    double yaw_drag_ = 1.00;

    unsigned int debug_counter_ = 0;

    ignition::math::Vector3d last_velocity_body_ =
        ignition::math::Vector3d::Zero;

    ignition::math::Vector3d last_drag_force_body_ =
        ignition::math::Vector3d::Zero;

    ignition::math::Vector3d last_angular_velocity_body_ =
        ignition::math::Vector3d::Zero;

    ignition::math::Vector3d last_drag_torque_body_ =
        ignition::math::Vector3d::Zero;
};

GZ_REGISTER_MODEL_PLUGIN(DragPlugin)

}