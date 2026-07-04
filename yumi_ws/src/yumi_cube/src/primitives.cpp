#include "yumi_cube/primitives.hpp"

#include <array>
#include <cmath>
#include <vector>

#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace yumi_cube
{
namespace
{

const char * primitive_arm_name(PrimitiveArm arm)
{
    return arm == PrimitiveArm::RIGHT ? "right" : "left";
}

void print_primitive(int value, const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger, "Primitive to execute: %d", value);
}

void send_rubik_keys(const PrimitiveContext & context, const char * keys)
{
    if (context.send_rubik_key) {
        context.send_rubik_key(keys);
    }
}

struct PrimitiveJointSnapshot {
    std::vector<double> right;
    std::vector<double> left;
};

bool capture_initial_joint_snapshot(
    int primitive_value,
    PrimitiveJointSnapshot & snapshot,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (!context.has_joint_state_for_arm(PrimitiveArm::RIGHT) ||
        !context.has_joint_state_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(
            logger,
            "Primitive %d waiting for both arm joint states before saving initial configuration.",
            primitive_value);
        return false;
    }

    snapshot.right = context.joint_state_for_arm(PrimitiveArm::RIGHT);
    snapshot.left = context.joint_state_for_arm(PrimitiveArm::LEFT);
    if (snapshot.right.size() < 7 || snapshot.left.size() < 7) {
        RCLCPP_WARN(
            logger,
            "Primitive %d received incomplete initial joint state: right=%zu left=%zu.",
            primitive_value,
            snapshot.right.size(),
            snapshot.left.size());
        return false;
    }

    snapshot.right.resize(7);
    snapshot.left.resize(7);
    return true;
}

const std::vector<double> & joints_for_arm(const PrimitiveJointSnapshot & snapshot, PrimitiveArm arm)
{
    return arm == PrimitiveArm::RIGHT ? snapshot.right : snapshot.left;
}

bool return_arm_to_initial_joints(
    int primitive_value,
    PrimitiveArm arm,
    const PrimitiveJointSnapshot & snapshot,
    int n_segments,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    const auto & joints = joints_for_arm(snapshot, arm);
    if (joints.size() < 7) {
        RCLCPP_WARN(
            logger,
            "Primitive %d cannot return %s arm: initial joint snapshot is incomplete.",
            primitive_value,
            primitive_arm_name(arm));
        return false;
    }

    RCLCPP_INFO(
        logger,
        "Primitive %d: returning %s arm to initial joint configuration.",
        primitive_value,
        primitive_arm_name(arm));
    context.start_joint_path(joints, n_segments, arm);
    return true;
}

tf2::Vector3 project_axis_onto_plane(const tf2::Vector3 & axis, const tf2::Vector3 & normal)
{
    return axis - normal * axis.dot(normal);
}

// Builds an orientation whose z axis points along the reference y axis
// (y_direction = +1) or against it (y_direction = -1), keeping the moving
// arm's x axis as continuous as possible.
geometry_msgs::msg::Quaternion align_moving_z_with_reference_y(
    const geometry_msgs::msg::Quaternion & moving_orientation,
    const geometry_msgs::msg::Quaternion & reference_orientation,
    double y_direction)
{
    tf2::Quaternion q_moving;
    tf2::fromMsg(moving_orientation, q_moving);

    tf2::Quaternion q_reference;
    tf2::fromMsg(reference_orientation, q_reference);

    tf2::Vector3 z = tf2::quatRotate(q_reference, tf2::Vector3(0, y_direction, 0));
    if (z.length2() < 1e-8) {
        z = tf2::Vector3(0, y_direction, 0);
    }
    z.normalize();

    tf2::Vector3 x = project_axis_onto_plane(tf2::quatRotate(q_moving, tf2::Vector3(1, 0, 0)), z);
    if (x.length2() < 1e-8) {
        x = project_axis_onto_plane(tf2::quatRotate(q_moving, tf2::Vector3(0, 1, 0)), z);
    }
    if (x.length2() < 1e-8) {
        x = project_axis_onto_plane(tf2::quatRotate(q_reference, tf2::Vector3(1, 0, 0)), z);
    }
    if (x.length2() < 1e-8) {
        x = tf2::Vector3(1, 0, 0);
    }
    x.normalize();

    tf2::Vector3 y = z.cross(x);
    y.normalize();

    tf2::Matrix3x3 R(
        x.x(), y.x(), z.x(),
        x.y(), y.y(), z.y(),
        x.z(), y.z(), z.z());

    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    return tf2::toMsg(q);
}

geometry_msgs::msg::Point point_in_reference_frame(
    const geometry_msgs::msg::Pose & reference_pose,
    double local_x,
    double local_y,
    double local_z)
{
    tf2::Quaternion q_reference;
    tf2::fromMsg(reference_pose.orientation, q_reference);

    const tf2::Vector3 offset = tf2::quatRotate(
        q_reference,
        tf2::Vector3(local_x, local_y, local_z));

    geometry_msgs::msg::Point point = reference_pose.position;
    point.x += offset.x();
    point.y += offset.y();
    point.z += offset.z();
    return point;
}

bool start_last_joint_rotation(
    int primitive_value,
    PrimitiveArm arm,
    double angle,
    int n_segments,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (!context.has_joint_state_for_arm(arm)) {
        RCLCPP_INFO(
            logger,
            "Primitive %d waiting for %s arm joint state before joint 7 rotation.",
            primitive_value,
            primitive_arm_name(arm));
        return false;
    }

    auto target_joints = context.joint_state_for_arm(arm);
    if (target_joints.size() < 7) {
        RCLCPP_WARN(
            logger,
            "Primitive %d received incomplete %s arm joint state: %zu positions.",
            primitive_value,
            primitive_arm_name(arm),
            target_joints.size());
        return false;
    }

    target_joints[6] += angle;
    RCLCPP_INFO(
        logger,
        "Primitive %d: rotating %s joint 7 by %.4f rad.",
        primitive_value,
        primitive_arm_name(arm),
        angle);
    context.start_joint_path(target_joints, n_segments, arm);
    return true;
}

// ---------------------------------------------------------------------------
// Handoff-rotation primitives (0-5): the moving arm re-grips the cube at the
// handoff pose and twists the gripped face with its last joint.

struct HandoffRotationSpec {
    int value;
    PrimitiveArm held_arm;    // arm currently holding the cube
    PrimitiveArm moving_arm;  // arm that re-grips and rotates the face
    double moving_yaw;        // joint-7 rotation applied by the moving arm
    const char * rubik_keys;  // simulator commands mirroring the physical turn
};

const std::array<HandoffRotationSpec, 6> kHandoffRotationSpecs = {{
    {0, PrimitiveArm::LEFT,  PrimitiveArm::RIGHT,  M_PI_2,        "f f f"},  // r
    {1, PrimitiveArm::LEFT,  PrimitiveArm::RIGHT, -M_PI_2,        "f"},      // r'
    {2, PrimitiveArm::RIGHT, PrimitiveArm::LEFT,   M_PI_2,        "b b b"},  // l
    {3, PrimitiveArm::RIGHT, PrimitiveArm::LEFT,  -M_PI_2,        "b"},      // l'
    {4, PrimitiveArm::LEFT,  PrimitiveArm::RIGHT,  M_PI - 0.0001, "f f"},    // r2
    {5, PrimitiveArm::RIGHT, PrimitiveArm::LEFT,  -M_PI + 0.0001, "b b"},    // l2
}};

PrimitiveStatus run_handoff_rotation(
    const HandoffRotationSpec & spec,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    if (!context.is_active_arm(spec.held_arm)) {
        phase = 0;
        RCLCPP_INFO(
            logger,
            "Primitive %d requires %s active arm. Requesting CHANGE_ARM.",
            spec.value,
            primitive_arm_name(spec.held_arm));
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(spec.moving_arm)) {
        RCLCPP_INFO(
            logger,
            "Primitive %d waiting for %s arm pose.",
            spec.value,
            primitive_arm_name(spec.moving_arm));
        return PrimitiveStatus::RUNNING;
    }

    if (context.waiting_for_gripper()) {
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0:
            if (!capture_initial_joint_snapshot(spec.value, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(spec.value, logger);
            RCLCPP_INFO(
                logger,
                "Primitive %d: opening %s gripper.",
                spec.value,
                primitive_arm_name(spec.moving_arm));
            context.send_gripper_command(false, spec.moving_arm);
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 1: {
            const auto held_pose = context.pose_for_arm(spec.held_arm);
            const auto moving_pose = context.pose_for_arm(spec.moving_arm);
            RCLCPP_INFO(
                logger,
                "Primitive %d: moving %s arm to handoff pose.",
                spec.value,
                primitive_arm_name(spec.moving_arm));
            ++phase;
            context.start_path(
                context.build_handoff_change_path(
                    moving_pose.position,
                    held_pose.position,
                    spec.moving_arm),
                context.opposite_handoff_orientation(held_pose.orientation, moving_pose.orientation),
                2,
                spec.moving_arm);
            return PrimitiveStatus::RUNNING;
        }

        case 2:
            if (!start_last_joint_rotation(spec.value, spec.moving_arm, spec.moving_yaw, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 3:
            ++phase;
            return_arm_to_initial_joints(spec.value, spec.moving_arm, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            context.restore_active_pose();
            send_rubik_keys(context, spec.rubik_keys);
            return PrimitiveStatus::COMPLETED;
    }
}

// ---------------------------------------------------------------------------
// Top-face primitives (6-9, 14, 15): the left arm holds the cube, tilts and
// shifts it, then the right arm approaches from the side and twists the face.

struct TopFaceTurnSpec {
    int value;
    double left_tilt;         // joint-7 pre-tilt of the left (holding) arm
    double left_lift;         // world-z shift of the left arm before the approach
    double approach_y_sign;   // approach side along the left-gripper y axis
    double right_yaw;         // joint-7 rotation applied by the right arm
    const char * rubik_keys;  // simulator commands mirroring the physical turn
};

const std::array<TopFaceTurnSpec, 6> kTopFaceTurnSpecs = {{
    {6,  -M_PI_4,        0.10, -1.0,  M_PI_2,        "u u u"},  // d'
    {7,  -M_PI_4,        0.10, -1.0, -M_PI_2,        "u"},      // d
    {8,  -M_PI_4 - 0.2, -0.05,  1.0,  M_PI_2 - 0.01, "d d d"},  // u
    {9,  -M_PI_4 - 0.2, -0.05,  1.0, -M_PI_2 - 0.01, "d"},      // u'
    {14, -M_PI_4,        0.10, -1.0,  M_PI,          "u u"},    // d2
    {15, -M_PI_4 - 0.2, -0.05,  1.0,  M_PI,          "d d"},    // u2
}};

PrimitiveStatus run_top_face_turn(
    const TopFaceTurnSpec & spec,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        RCLCPP_INFO(logger, "Primitive %d requires left active arm. Requesting CHANGE_ARM.", spec.value);
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive %d waiting for right arm pose.", spec.value);
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0:
            if (!capture_initial_joint_snapshot(spec.value, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(spec.value, logger);
            if (!start_last_joint_rotation(spec.value, PrimitiveArm::LEFT, spec.left_tilt, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z += spec.left_lift;
            RCLCPP_INFO(
                logger,
                "Primitive %d: shifting left arm holding cube by %.2f m in world z.",
                spec.value,
                spec.left_lift);
            ++phase;
            context.start_path(
                {left_pose.position, lifted_position},
                left_pose.orientation,
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            const auto right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            const auto near_approach_position =
                point_in_reference_frame(left_pose, 0.0, spec.approach_y_sign * 0.07, 0.05);
            const auto target_position =
                point_in_reference_frame(left_pose, 0.0, spec.approach_y_sign * 0.058, 0.05);
            RCLCPP_INFO(
                logger,
                "Primitive %d: moving right arm through sampled approach path and aligning right z axis with left y axis.",
                spec.value);
            ++phase;
            context.start_path(
                context.build_path({right_pose.position, near_approach_position, target_position}),
                align_moving_z_with_reference_y(
                    right_pose.orientation, left_pose.orientation, -spec.approach_y_sign),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            if (!start_last_joint_rotation(spec.value, PrimitiveArm::RIGHT, spec.right_yaw, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            send_rubik_keys(context, spec.rubik_keys);
            return_arm_to_initial_joints(spec.value, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(spec.value, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

// ---------------------------------------------------------------------------
// Side-face primitives (10-13, 16, 17): the right arm holds the cube and moves
// it forward, then the left arm approaches from the side and twists the face.

struct SideFaceTurnSpec {
    int value;
    double right_x_shift;     // world-x shift of the right (holding) arm
    bool flip_right_joint7;   // rotate right joint 7 by pi before the approach
    double approach_y_sign;   // approach side along the right-gripper y axis
    double left_yaw;          // joint-7 rotation applied by the left arm
    const char * rubik_keys;  // simulator commands mirroring the physical turn
};

const std::array<SideFaceTurnSpec, 6> kSideFaceTurnSpecs = {{
    {10, 0.10, true,  -1.0,  M_PI_2,        "l l l"},  // b'
    {11, 0.15, false,  1.0,  M_PI_2 - 0.01, "r r r"},  // f
    {12, 0.10, true,  -1.0, -M_PI_2 + 0.01, "l"},      // b
    {13, 0.15, false,  1.0, -M_PI_2 + 0.01, "r"},      // f'
    {16, 0.15, false,  1.0,  M_PI,          "r r"},    // f2
    {17, 0.15, false,  1.0,  M_PI,          "r r"},    // b2
}};

PrimitiveStatus run_side_face_turn(
    const SideFaceTurnSpec & spec,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        RCLCPP_INFO(logger, "Primitive %d requires right active arm. Requesting CHANGE_ARM.", spec.value);
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive %d waiting for left arm pose.", spec.value);
        return PrimitiveStatus::RUNNING;
    }

    // Primitives without the joint-7 flip skip that step of the shared sequence.
    const int step = (phase == 0 || spec.flip_right_joint7) ? phase : phase + 1;

    switch (step) {
        case 0: {
            print_primitive(spec.value, logger);
            if (!capture_initial_joint_snapshot(spec.value, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += spec.right_x_shift;

            RCLCPP_INFO(
                logger,
                "Primitive %d: moving right arm %+.2f on sampled world-x path.",
                spec.value,
                spec.right_x_shift);
            ++phase;
            context.start_path(
                context.build_path({right_desired_pose.position, target_position}),
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1:
            if (!start_last_joint_rotation(spec.value, PrimitiveArm::RIGHT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 2: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position =
                point_in_reference_frame(right_desired_pose, 0.0, spec.approach_y_sign * 0.158, 0.05);
            const auto near_approach_position =
                point_in_reference_frame(right_desired_pose, 0.0, spec.approach_y_sign * 0.108, 0.05);
            const auto final_position =
                point_in_reference_frame(right_desired_pose, 0.0, spec.approach_y_sign * 0.058, 0.05);

            RCLCPP_INFO(
                logger,
                "Primitive %d: moving left arm through sampled approach path to final aligned pose.",
                spec.value);
            ++phase;
            context.start_path(
                context.build_path({
                    left_desired_pose.position,
                    far_approach_position,
                    near_approach_position,
                    final_position}),
                align_moving_z_with_reference_y(
                    left_desired_pose.orientation,
                    right_desired_pose.orientation,
                    -spec.approach_y_sign),
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            if (!start_last_joint_rotation(spec.value, PrimitiveArm::LEFT, spec.left_yaw, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            send_rubik_keys(context, spec.rubik_keys);
            return_arm_to_initial_joints(spec.value, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(spec.value, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

template <typename Spec, std::size_t N>
const Spec * find_spec(const std::array<Spec, N> & specs, int value)
{
    for (const auto & spec : specs) {
        if (spec.value == value) {
            return &spec;
        }
    }
    return nullptr;
}

}  // namespace

PrimitiveStatus execute_primitive(
    int primitive_value,
    const geometry_msgs::msg::Pose & /*current_pose*/,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (const auto * spec = find_spec(kHandoffRotationSpecs, primitive_value)) {
        return run_handoff_rotation(*spec, logger, context);
    }
    if (const auto * spec = find_spec(kTopFaceTurnSpecs, primitive_value)) {
        return run_top_face_turn(*spec, logger, context);
    }
    if (const auto * spec = find_spec(kSideFaceTurnSpecs, primitive_value)) {
        return run_side_face_turn(*spec, logger, context);
    }

    RCLCPP_WARN(logger, "Ignoring unknown primitive value: %d", primitive_value);
    return PrimitiveStatus::COMPLETED;
}

}  // namespace yumi_cube
