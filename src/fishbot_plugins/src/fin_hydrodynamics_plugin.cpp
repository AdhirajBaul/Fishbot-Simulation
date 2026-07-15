#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>

#include <cmath>
#include <functional>

namespace gazebo
{

class FinHydrodynamicsPlugin : public ModelPlugin
{
public:
    void Load(
        physics::ModelPtr _model,
        sdf::ElementPtr /* _sdf */) override
    {
        model_ = _model;

        base_link_ =
            model_->GetLink("base_link");

        left_fin_link_ =
            model_->GetLink("left_fin_link");

        right_fin_link_ =
            model_->GetLink("right_fin_link");

        left_fin_joint_ =
            model_->GetJoint("left_fin_joint");

        right_fin_joint_ =
            model_->GetJoint("right_fin_joint");

        if (!base_link_ ||
            !left_fin_link_ ||
            !right_fin_link_ ||
            !left_fin_joint_ ||
            !right_fin_joint_)
        {
            gzerr
                << "Could not find pectoral fin "
                << "links or joints!\n";
            return;
        }

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(
                    &FinHydrodynamicsPlugin::OnUpdate,
                    this));

        gzmsg
            << "Pectoral fin hydrodynamics plugin loaded\n";
    }

private:
    void OnUpdate()
    {
        const ignition::math::Quaterniond body_rotation =
            base_link_->WorldPose().Rot();

        const ignition::math::Vector3d velocity_body =
            body_rotation.RotateVectorReverse(
                base_link_->WorldLinearVel());

        const double forward_speed =
            velocity_body.X();

        if (std::abs(forward_speed) <
            minimum_forward_speed_)
        {
            return;
        }

        ApplyFinLift(
            left_fin_link_,
            left_fin_joint_,
            body_rotation,
            forward_speed);

        ApplyFinLift(
            right_fin_link_,
            right_fin_joint_,
            body_rotation,
            forward_speed);
    }

    void ApplyFinLift(
        const physics::LinkPtr &fin_link,
        const physics::JointPtr &fin_joint,
        const ignition::math::Quaterniond &body_rotation,
        const double forward_speed)
    {
        const double fin_angle =
            fin_joint->Position(0);

        /*
         * Dynamic pressure:
         *
         * q = 0.5 rho v²
         */
        const double dynamic_pressure =
            0.5 *
            water_density_ *
            forward_speed *
            forward_speed;

        /*
         * Linear lift-coefficient approximation:
         *
         * Cl = lift_slope * angle
         */
        double lift_coefficient =
            lift_slope_ * fin_angle;

        if (lift_coefficient >
            maximum_lift_coefficient_)
        {
            lift_coefficient =
                maximum_lift_coefficient_;
        }

        if (lift_coefficient <
            -maximum_lift_coefficient_)
        {
            lift_coefficient =
                -maximum_lift_coefficient_;
        }

        const double lift_force_z =
            dynamic_pressure *
            fin_area_ *
            lift_coefficient;

        /*
         * Deflected fins also create additional drag.
         */
        const double drag_coefficient =
            base_drag_coefficient_ +
            angle_drag_coefficient_ *
            fin_angle *
            fin_angle;

        const double drag_force_x =
            -std::copysign(
                dynamic_pressure *
                fin_area_ *
                drag_coefficient,
                forward_speed);

        const ignition::math::Vector3d fin_force_body(
            drag_force_x,
            0.0,
            lift_force_z);

        const ignition::math::Vector3d fin_force_world =
            body_rotation.RotateVector(
                fin_force_body);

        /*
         * Force applied at each fin creates the appropriate
         * pitching moment around the fish COM.
         */
        fin_link->AddForce(fin_force_world);
    }

private:
    physics::ModelPtr model_;

    physics::LinkPtr base_link_;
    physics::LinkPtr left_fin_link_;
    physics::LinkPtr right_fin_link_;

    physics::JointPtr left_fin_joint_;
    physics::JointPtr right_fin_joint_;

    event::ConnectionPtr update_connection_;

    double water_density_ = 1000.0;

    // 0.07 × 0.06
    double fin_area_ = 0.0042;

    double lift_slope_ = 4.0;
    double maximum_lift_coefficient_ = 1.2;

    double base_drag_coefficient_ = 0.05;
    double angle_drag_coefficient_ = 4.0;

    double minimum_forward_speed_ = 0.05;
};

GZ_REGISTER_MODEL_PLUGIN(FinHydrodynamicsPlugin)

}