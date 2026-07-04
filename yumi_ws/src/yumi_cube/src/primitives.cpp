#include "yumi_cube/primitives.hpp"

#include <algorithm>
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

using PrimitiveFunction = PrimitiveStatus (*)(
    const geometry_msgs::msg::Pose &,
    const rclcpp::Logger &,
    const PrimitiveContext &);

void print_primitive(int value, const geometry_msgs::msg::Pose & current_pose, const rclcpp::Logger & logger)
{
    (void)current_pose;
    RCLCPP_INFO(logger, "Primitive to execute: %d", value);
}

const char * primitive_arm_name(PrimitiveArm arm)
{
    return arm == PrimitiveArm::RIGHT ? "right" : "left";
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

geometry_msgs::msg::Quaternion align_moving_z_with_reference_y_direction(
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

geometry_msgs::msg::Quaternion align_moving_z_with_reference_y(
    const geometry_msgs::msg::Quaternion & moving_orientation,
    const geometry_msgs::msg::Quaternion & reference_orientation)
{
    return align_moving_z_with_reference_y_direction(moving_orientation, reference_orientation, 1.0);
}

geometry_msgs::msg::Quaternion align_moving_z_with_negative_reference_y(
    const geometry_msgs::msg::Quaternion & moving_orientation,
    const geometry_msgs::msg::Quaternion & reference_orientation)
{
    return align_moving_z_with_reference_y_direction(moving_orientation, reference_orientation, -1.0);
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

PrimitiveStatus execute_handoff_rotation_primitive(
    int primitive_value,
    PrimitiveArm held_arm,
    PrimitiveArm moving_arm,
    double yaw,
    int & phase,
    PrimitiveJointSnapshot & initial_joints,
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (!context.is_active_arm(held_arm)) {
        phase = 0;
        RCLCPP_INFO(
            logger,
            "Primitive %d requires %s active arm. Requesting CHANGE_ARM.",
            primitive_value,
            primitive_arm_name(held_arm));
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(moving_arm)) {
        RCLCPP_INFO(
            logger,
            "Primitive %d waiting for %s arm pose.",
            primitive_value,
            primitive_arm_name(moving_arm));
        return PrimitiveStatus::RUNNING;
    }

    if (context.waiting_for_gripper()) {
        return PrimitiveStatus::RUNNING;
    }

    auto held_pose = context.pose_for_arm(held_arm);
    const auto moving_pose = context.pose_for_arm(moving_arm);

    switch (phase) {
        case 0:
            if (!capture_initial_joint_snapshot(primitive_value, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(primitive_value, current_pose, logger);
            RCLCPP_INFO(
                logger,
                "Primitive %d: opening %s gripper.",
                primitive_value,
                primitive_arm_name(moving_arm));
            context.send_gripper_command(false, moving_arm);
            ++phase;
            return PrimitiveStatus::RUNNING;

        case 1:
            RCLCPP_INFO(
                logger,
                "Primitive %d: moving %s arm to handoff pose.",
                primitive_value,
                primitive_arm_name(moving_arm));
            ++phase;
            context.start_path(
                context.build_handoff_change_path(
                    moving_pose.position,
                    held_pose.position,
                    moving_arm),
                context.opposite_handoff_orientation(held_pose.orientation, moving_pose.orientation),
                2,
                moving_arm);
            return PrimitiveStatus::RUNNING;

        case 2: {
            if (!start_last_joint_rotation(primitive_value, moving_arm, yaw, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            ++phase;
            return_arm_to_initial_joints(primitive_value, moving_arm, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_0(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        0,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        M_PI_2,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);

    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("f f f");
    }

    return status;
}

PrimitiveStatus primitive_1(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        1,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        -M_PI_2,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);

    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("f");
    }

    return status;
}

PrimitiveStatus primitive_2(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        2,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        M_PI_2,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);

    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("b b b");
    }

    return status;

        
}

PrimitiveStatus primitive_3(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        3,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        -M_PI_2,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);

    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("b");
    }

    return status;
}

PrimitiveStatus primitive_4(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        4,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        M_PI-0.0001,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);


    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("f f");
    }

    return status;
    
}

PrimitiveStatus primitive_5(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;

    const auto status = execute_handoff_rotation_primitive(
        5,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        -M_PI + 0.0001,
        phase,
        initial_joints,
        current_pose,
        logger,
        context);


    if (status == PrimitiveStatus::COMPLETED && context.send_rubik_key) {
        context.send_rubik_key("b b");
    }

    return status;
}

PrimitiveStatus primitive_6(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 6 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 6 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {

        case 0: {
            if (!capture_initial_joint_snapshot(6, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(6, current_pose, logger);
            if (!start_last_joint_rotation(6, PrimitiveArm::LEFT, -M_PI_4, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z += 0.10;
            RCLCPP_INFO(logger, "Primitive 6: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, -0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, -0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 6: moving right arm through sampled approach path and aligning right z axis with left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(6, PrimitiveArm::RIGHT, M_PI_2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("u u u");
            return_arm_to_initial_joints(6, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(6, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus execute_right_reference_frame_rotation_primitive(
    int primitive_value,
    double local_y,
    double yaw,
    bool align_negative_y,
    int & phase,
    PrimitiveJointSnapshot & initial_joints,
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        RCLCPP_INFO(
            logger,
            "Primitive %d requires left active arm. Requesting CHANGE_ARM.",
            primitive_value);
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive %d waiting for right arm pose.", primitive_value);
        return PrimitiveStatus::RUNNING;
    }

    const double approach_step = local_y >= 0.0 ? 0.05 : -0.05;
    const double near_approach_y = local_y + approach_step;
    const double far_approach_y = local_y + 2.0 * approach_step;

    switch (phase) {
        case 0: {
            if (!capture_initial_joint_snapshot(primitive_value, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(primitive_value, current_pose, logger);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            const auto far_approach_position = point_in_reference_frame(left_pose, 0.0, far_approach_y, 0.05);
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, near_approach_y, 0.05);
            const auto final_position = point_in_reference_frame(left_pose, 0.0, local_y, 0.05);
            const auto final_orientation = align_negative_y
                ? align_moving_z_with_negative_reference_y(right_desired_pose.orientation, left_pose.orientation)
                : align_moving_z_with_reference_y(right_desired_pose.orientation, left_pose.orientation);

            RCLCPP_INFO(
                logger,
                "Primitive %d: moving right arm through approach waypoint to final aligned pose.",
                primitive_value);
            ++phase;
            context.start_path(
                context.build_path({
                    right_desired_pose.position,
                    far_approach_position,
                    near_approach_position,
                    final_position}),
                final_orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            if (!start_last_joint_rotation(primitive_value, PrimitiveArm::RIGHT, yaw, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 2:
            ++phase;
            return_arm_to_initial_joints(primitive_value, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_7(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 6 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 6 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {

        case 0: {
            if (!capture_initial_joint_snapshot(6, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(6, current_pose, logger);
            if (!start_last_joint_rotation(6, PrimitiveArm::LEFT, -M_PI_4, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z += 0.10;
            RCLCPP_INFO(logger, "Primitive 6: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, -0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, -0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 6: moving right arm through sampled approach path and aligning right z axis with left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(6, PrimitiveArm::RIGHT, -M_PI_2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("u");
            return_arm_to_initial_joints(6, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(6, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_8(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 8 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 8 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            if (!capture_initial_joint_snapshot(8, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(8, current_pose, logger);
            if (!start_last_joint_rotation(8, PrimitiveArm::LEFT, -M_PI_4-0.2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z -= 0.05;
            RCLCPP_INFO(logger, "Primitive 8: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, 0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, 0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 8: moving right arm through sampled approach path and aligning right z axis with negative left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_negative_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(8, PrimitiveArm::RIGHT, M_PI_2 - 0.01, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("d d d");
            return_arm_to_initial_joints(8, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(8, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_9(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 8 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 8 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            if (!capture_initial_joint_snapshot(8, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(8, current_pose, logger);
            if (!start_last_joint_rotation(8, PrimitiveArm::LEFT, -M_PI_4-0.2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z -= 0.05;
            RCLCPP_INFO(logger, "Primitive 8: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, 0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, 0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 8: moving right arm through sampled approach path and aligning right z axis with negative left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_negative_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(8, PrimitiveArm::RIGHT, -M_PI_2 - 0.01, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("d");
            return_arm_to_initial_joints(8, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(8, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_10(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 10 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 10 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(10, current_pose, logger);
            if (!capture_initial_joint_snapshot(10, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.1;

            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            RCLCPP_INFO(logger, "Primitive 10: moving right arm +0.15 on sampled world-x path.");
            ++phase;
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            if (!start_last_joint_rotation(12, PrimitiveArm::RIGHT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, -0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, -0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, -0.058, 0.05);
            const auto final_orientation = align_moving_z_with_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            RCLCPP_INFO(logger, "Primitive 10: moving left arm through sampled approach path to final aligned pose.");
            ++phase;
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(10, PrimitiveArm::LEFT, M_PI_2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("l l l");
            return_arm_to_initial_joints(10, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(10, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_11(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 11 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 11 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(11, current_pose, logger);
            if (!capture_initial_joint_snapshot(11, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.15;

            RCLCPP_INFO(logger, "Primitive 17: moving right arm -0.07 on world x.");
            ++phase;
            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, 0.058, 0.05);
            const auto final_orientation = align_moving_z_with_negative_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            RCLCPP_INFO(logger, "Primitive 11: moving left arm through approach waypoint to final aligned pose.");
            ++phase;
            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            if (!start_last_joint_rotation(11, PrimitiveArm::LEFT, M_PI_2 - 0.01, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            ++phase;
            context.send_rubik_key("r r r");
            return_arm_to_initial_joints(11, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            return_arm_to_initial_joints(11, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_12(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 12 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 12 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(12, current_pose, logger);
            if (!capture_initial_joint_snapshot(12, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.1;

            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            RCLCPP_INFO(logger, "Primitive 12: moving right arm +0.15 on sampled world-x path.");
            ++phase;
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            if (!start_last_joint_rotation(12, PrimitiveArm::RIGHT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, -0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, -0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, -0.058, 0.05);
            const auto final_orientation = align_moving_z_with_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            RCLCPP_INFO(logger, "Primitive 12: moving left arm through sampled approach path to final aligned pose.");
            ++phase;
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(12, PrimitiveArm::LEFT, -M_PI_2 + 0.01, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("l");
            return_arm_to_initial_joints(12, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(12, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_13(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 13 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 13 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(13, current_pose, logger);
            if (!capture_initial_joint_snapshot(13, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.15;

            RCLCPP_INFO(logger, "Primitive 13: moving right arm -0.07 on world x.");
            ++phase;
            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, 0.058, 0.05);
            const auto final_orientation = align_moving_z_with_negative_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            RCLCPP_INFO(logger, "Primitive 13: moving left arm through approach waypoint to final aligned pose.");
            ++phase;
            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            if (!start_last_joint_rotation(13, PrimitiveArm::LEFT, -M_PI_2 + 0.01, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            ++phase;
            context.send_rubik_key("r");
            return_arm_to_initial_joints(13, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            return_arm_to_initial_joints(13, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_14(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 14 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 14 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {

        case 0: {
            if (!capture_initial_joint_snapshot(14, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(14, current_pose, logger);
            if (!start_last_joint_rotation(14, PrimitiveArm::LEFT, -M_PI_4, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z += 0.10;
            RCLCPP_INFO(logger, "Primitive 14: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, -0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, -0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 14: moving right arm through sampled approach path and aligning right z axis with left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(6, PrimitiveArm::RIGHT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("u u");
            return_arm_to_initial_joints(6, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(6, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_15(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> forward_path;

    if (!context.is_active_arm(PrimitiveArm::LEFT)) {
        phase = 0;
        forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 15 requires left active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::RIGHT)) {
        RCLCPP_INFO(logger, "Primitive 15 waiting for right arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            if (!capture_initial_joint_snapshot(15, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            print_primitive(15, current_pose, logger);
            if (!start_last_joint_rotation(15, PrimitiveArm::LEFT, -M_PI_4-0.2, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            auto lifted_position = left_pose.position;
            lifted_position.z -= 0.05;
            RCLCPP_INFO(logger, "Primitive 15: lifting left arm holding cube by 0.10 m in world z.");
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
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, 0.07, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, 0.058, 0.05);
            forward_path = context.build_path({
                right_pose.position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 15: moving right arm through sampled approach path and aligning right z axis with negative left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_negative_reference_y(right_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            if (!start_last_joint_rotation(15, PrimitiveArm::RIGHT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 4:
            ++phase;
            context.send_rubik_key("d d");
            return_arm_to_initial_joints(15, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 5:
            ++phase;
            return_arm_to_initial_joints(15, PrimitiveArm::LEFT, initial_joints, 1, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_16(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 16 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 16 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(16, current_pose, logger);
            if (!capture_initial_joint_snapshot(16, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.15;

            RCLCPP_INFO(logger, "Primitive 16: moving right arm -0.07 on world x.");
            ++phase;
            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, 0.058, 0.05);
            const auto final_orientation = align_moving_z_with_negative_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            RCLCPP_INFO(logger, "Primitive 11: moving left arm through approach waypoint to final aligned pose.");
            ++phase;
            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            if (!start_last_joint_rotation(16, PrimitiveArm::LEFT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            ++phase;
            context.send_rubik_key("r r");
            return_arm_to_initial_joints(16, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            return_arm_to_initial_joints(16, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

PrimitiveStatus primitive_17(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static PrimitiveJointSnapshot initial_joints;
    static std::vector<geometry_msgs::msg::Point> right_forward_path;
    static std::vector<geometry_msgs::msg::Point> left_forward_path;

    if (!context.is_active_arm(PrimitiveArm::RIGHT)) {
        phase = 0;
        right_forward_path.clear();
        left_forward_path.clear();
        RCLCPP_INFO(logger, "Primitive 17 requires right active arm. Requesting CHANGE_ARM.");
        context.request_change_arm();
        return PrimitiveStatus::RUNNING;
    }

    if (!context.has_pose_for_arm(PrimitiveArm::LEFT)) {
        RCLCPP_INFO(logger, "Primitive 17 waiting for left arm pose.");
        return PrimitiveStatus::RUNNING;
    }

    switch (phase) {
        case 0: {
            print_primitive(17, current_pose, logger);
            if (!capture_initial_joint_snapshot(17, initial_joints, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.15;

            RCLCPP_INFO(logger, "Primitive 17: moving right arm -0.07 on world x.");
            ++phase;
            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto far_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.158, 0.05);
            const auto near_approach_position = point_in_reference_frame(right_desired_pose, 0.0, 0.108, 0.05);
            const auto final_position = point_in_reference_frame(right_desired_pose, 0.0, 0.058, 0.05);
            const auto final_orientation = align_moving_z_with_negative_reference_y(
                left_desired_pose.orientation,
                right_desired_pose.orientation);

            RCLCPP_INFO(logger, "Primitive 17: moving left arm through approach waypoint to final aligned pose.");
            ++phase;
            left_forward_path = context.build_path({
                left_desired_pose.position,
                far_approach_position,
                near_approach_position,
                final_position});
            context.start_path(
                left_forward_path,
                final_orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            if (!start_last_joint_rotation(17, PrimitiveArm::LEFT, M_PI, 1, logger, context)) {
                return PrimitiveStatus::RUNNING;
            }
            ++phase;
            return PrimitiveStatus::RUNNING;
        }

        case 3:
            ++phase;
            context.send_rubik_key("r r");
            return_arm_to_initial_joints(17, PrimitiveArm::LEFT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        case 4:
            ++phase;
            return_arm_to_initial_joints(17, PrimitiveArm::RIGHT, initial_joints, 2, logger, context);
            return PrimitiveStatus::RUNNING;

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

const std::array<PrimitiveFunction, 18> kPrimitiveFunctions = {
    primitive_0,
    primitive_1,
    primitive_2,
    primitive_3,
    primitive_4,
    primitive_5,
    primitive_6,
    primitive_7,
    primitive_8,
    primitive_9,
    primitive_10,
    primitive_11,
    primitive_12,
    primitive_13,
    primitive_14,
    primitive_15,
    primitive_16,
    primitive_17,
};

}  // namespace

PrimitiveStatus execute_primitive(
    int primitive_value,
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    if (primitive_value < 0 || primitive_value >= static_cast<int>(kPrimitiveFunctions.size())) {
        RCLCPP_WARN(logger, "Ignoring unknown primitive value: %d", primitive_value);
        return PrimitiveStatus::COMPLETED;
    }

    return kPrimitiveFunctions[primitive_value](current_pose, logger, context);
}

}  // namespace yumi_cube
