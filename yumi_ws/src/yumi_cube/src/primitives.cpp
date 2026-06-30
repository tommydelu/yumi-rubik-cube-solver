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

PrimitiveStatus execute_handoff_rotation_primitive(
    int primitive_value,
    PrimitiveArm held_arm,
    PrimitiveArm moving_arm,
    double yaw,
    int & phase,
    geometry_msgs::msg::Pose & initial_moving_pose,
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

    const auto held_pose = context.pose_for_arm(held_arm);
    const auto moving_pose = context.pose_for_arm(moving_arm);

    switch (phase) {
        case 0:
            initial_moving_pose = moving_pose;
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
            const auto moving_desired_pose = context.desired_pose_for_arm(moving_arm);
            RCLCPP_INFO(
                logger,
                "Primitive %d: rotating %s end-effector around local z.",
                primitive_value,
                primitive_arm_name(moving_arm));
            ++phase;
            context.start_path(
                context.hold_path(moving_desired_pose.position),
                context.rotated_orientation_from(moving_desired_pose.orientation, 0.0, 0.0, yaw),
                1,
                moving_arm);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            const auto moving_desired_pose = context.desired_pose_for_arm(moving_arm);
            RCLCPP_INFO(
                logger,
                "Primitive %d: returning %s arm to initial pose.",
                primitive_value,
                primitive_arm_name(moving_arm));
            ++phase;
            context.start_path(
                {moving_desired_pose.position, initial_moving_pose.position},
                initial_moving_pose.orientation,
                1,
                moving_arm);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        0,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        M_PI_2,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_1(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        1,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        -M_PI_2,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_2(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        2,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        M_PI_2,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_3(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        3,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        -M_PI_2,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_4(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        0,
        PrimitiveArm::LEFT,
        PrimitiveArm::RIGHT,
        M_PI-0.0001,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_5(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_moving_pose;
    return execute_handoff_rotation_primitive(
        3,
        PrimitiveArm::RIGHT,
        PrimitiveArm::LEFT,
        -M_PI + 0.0001,
        phase,
        initial_moving_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_6(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_right_pose;
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
            print_primitive(6, current_pose, logger);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            const auto far_approach_position = point_in_reference_frame(left_pose, 0.0, -0.118, 0.05);
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, -0.108, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, -0.058, 0.05);
            forward_path = context.build_path({
                right_desired_pose.position,
                far_approach_position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 6: moving right arm through sampled approach path and aligning right z axis with left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_reference_y(right_desired_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(logger, "Primitive 6: rotating right end-effector pi/2 around local z.");
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, M_PI_2-0.01),
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            auto return_path = forward_path;
            std::reverse(return_path.begin(), return_path.end());
            if (return_path.empty()) {
                return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 6: returning right arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    geometry_msgs::msg::Pose & initial_right_pose,
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
            print_primitive(primitive_value, current_pose, logger);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);
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
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(
                logger,
                "Primitive %d: rotating right end-effector halfway around local z.",
                primitive_value);
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, yaw / 2.0),
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(
                logger,
                "Primitive %d: finishing right end-effector local z rotation.",
                primitive_value);
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, yaw / 2.0),
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            const auto near_return_approach = point_in_reference_frame(left_pose, 0.0, near_approach_y, 0.05);
            const auto far_return_approach = point_in_reference_frame(left_pose, 0.0, far_approach_y, 0.05);
            RCLCPP_INFO(
                logger,
                "Primitive %d: returning right arm through approach waypoint to initial pose.",
                primitive_value);
            ++phase;
            context.start_path(
                context.build_path({
                    right_desired_pose.position,
                    near_return_approach,
                    far_return_approach,
                    initial_right_pose.position}),
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_right_pose;
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
            print_primitive(6, current_pose, logger);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            const auto left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            const auto far_approach_position = point_in_reference_frame(left_pose, 0.0, -0.118, 0.05);
            const auto near_approach_position = point_in_reference_frame(left_pose, 0.0, -0.108, 0.05);
            const auto target_position = point_in_reference_frame(left_pose, 0.0, -0.058, 0.05);
            forward_path = context.build_path({
                right_desired_pose.position,
                far_approach_position,
                near_approach_position,
                target_position});
            RCLCPP_INFO(
                logger,
                "Primitive 6: moving right arm through sampled approach path and aligning right z axis with left y axis.");
            ++phase;
            context.start_path(
                forward_path,
                align_moving_z_with_reference_y(right_desired_pose.orientation, left_pose.orientation),
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(logger, "Primitive 6: rotating right end-effector pi/2 around local z.");
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, -M_PI_2+0.01),
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 2: {
            auto return_path = forward_path;
            std::reverse(return_path.begin(), return_path.end());
            if (return_path.empty()) {
                return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 6: returning right arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_right_pose;
    return execute_right_reference_frame_rotation_primitive(
        8,
        0.058,
        M_PI_2 - 0.01,
        true,
        phase,
        initial_right_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_9(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_right_pose;
    return execute_right_reference_frame_rotation_primitive(
        9,
        0.058,
        M_PI_2 - 0.01,
        true,
        phase,
        initial_right_pose,
        current_pose,
        logger,
        context);
}

PrimitiveStatus primitive_10(
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context)
{
    static int phase = 0;
    static geometry_msgs::msg::Pose initial_left_pose;
    static geometry_msgs::msg::Pose initial_right_pose;
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
            initial_left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.23;

            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            RCLCPP_INFO(logger, "Primitive 10: moving right arm +0.17 on sampled world-x path.");
            ++phase;
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(logger, "Primitive 10: rotating right end-effector M_PI around local z.");
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, M_PI),
                1,
                PrimitiveArm::RIGHT);
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
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 10: rotating left end-effector halfway around local z.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (M_PI_2 - 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 4: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 10: finishing left end-effector local z rotation.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (M_PI_2 - 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 5: {
            auto left_return_path = left_forward_path;
            std::reverse(left_return_path.begin(), left_return_path.end());
            if (left_return_path.empty()) {
                left_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::LEFT).position,
                    initial_left_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 10: returning left arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                left_return_path,
                initial_left_pose.orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 6: {
            auto right_return_path = right_forward_path;
            std::reverse(right_return_path.begin(), right_return_path.end());
            if (right_return_path.empty()) {
                right_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 10: returning right arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                right_return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_left_pose;
    static geometry_msgs::msg::Pose initial_right_pose;
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
            initial_left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.23;

            RCLCPP_INFO(logger, "Primitive 11: moving right arm -0.07 on world x.");
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
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 11: rotating left end-effector halfway around local z.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (M_PI_2 - 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 11: finishing left end-effector local z rotation.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (M_PI_2 - 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 4: {
            RCLCPP_INFO(logger, "Primitive 11: returning left arm through approach waypoints to initial pose.");
            ++phase;
            auto left_return_path = left_forward_path;
            std::reverse(left_return_path.begin(), left_return_path.end());
            if (left_return_path.empty()) {
                left_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::LEFT).position,
                    initial_left_pose.position};
            }
            context.start_path(
                left_return_path,
                initial_left_pose.orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 5: {
            RCLCPP_INFO(logger, "Primitive 11: returning right arm through intermediate pose to initial pose.");
            ++phase;
            auto right_return_path = right_forward_path;
            std::reverse(right_return_path.begin(), right_return_path.end());
            if (right_return_path.empty()) {
                right_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            context.start_path(
                right_return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_left_pose;
    static geometry_msgs::msg::Pose initial_right_pose;
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
            initial_left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.23;

            right_forward_path = context.build_path({right_desired_pose.position, target_position});
            RCLCPP_INFO(logger, "Primitive 10: moving right arm +0.17 on sampled world-x path.");
            ++phase;
            context.start_path(
                right_forward_path,
                right_desired_pose.orientation,
                1,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        case 1: {
            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            RCLCPP_INFO(logger, "Primitive 10: rotating right end-effector M_PI around local z.");
            ++phase;
            context.start_path(
                context.hold_path(right_desired_pose.position),
                context.rotated_orientation_from(right_desired_pose.orientation, 0.0, 0.0, M_PI),
                1,
                PrimitiveArm::RIGHT);
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
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 10: rotating left end-effector halfway around local z.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (-M_PI_2 + 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 4: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 10: finishing left end-effector local z rotation.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (-M_PI_2 + 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 5: {
            auto left_return_path = left_forward_path;
            std::reverse(left_return_path.begin(), left_return_path.end());
            if (left_return_path.empty()) {
                left_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::LEFT).position,
                    initial_left_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 10: returning left arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                left_return_path,
                initial_left_pose.orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 6: {
            auto right_return_path = right_forward_path;
            std::reverse(right_return_path.begin(), right_return_path.end());
            if (right_return_path.empty()) {
                right_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            RCLCPP_INFO(logger, "Primitive 10: returning right arm along the exact reversed sampled path.");
            ++phase;
            context.start_path(
                right_return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

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
    static geometry_msgs::msg::Pose initial_left_pose;
    static geometry_msgs::msg::Pose initial_right_pose;
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
            initial_left_pose = context.pose_for_arm(PrimitiveArm::LEFT);
            initial_right_pose = context.pose_for_arm(PrimitiveArm::RIGHT);

            const auto right_desired_pose = context.desired_pose_for_arm(PrimitiveArm::RIGHT);
            auto target_position = right_desired_pose.position;
            target_position.x += 0.23;

            RCLCPP_INFO(logger, "Primitive 11: moving right arm -0.07 on world x.");
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
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 11: rotating left end-effector halfway around local z.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (-M_PI_2 + 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 3: {
            const auto left_desired_pose = context.desired_pose_for_arm(PrimitiveArm::LEFT);
            RCLCPP_INFO(logger, "Primitive 11: finishing left end-effector local z rotation.");
            ++phase;
            context.start_path(
                context.hold_path(left_desired_pose.position),
                context.rotated_orientation_from(left_desired_pose.orientation, 0.0, 0.0, (-M_PI_2 + 0.01) / 2.0),
                1,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 4: {
            RCLCPP_INFO(logger, "Primitive 11: returning left arm through approach waypoints to initial pose.");
            ++phase;
            auto left_return_path = left_forward_path;
            std::reverse(left_return_path.begin(), left_return_path.end());
            if (left_return_path.empty()) {
                left_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::LEFT).position,
                    initial_left_pose.position};
            }
            context.start_path(
                left_return_path,
                initial_left_pose.orientation,
                2,
                PrimitiveArm::LEFT);
            return PrimitiveStatus::RUNNING;
        }

        case 5: {
            RCLCPP_INFO(logger, "Primitive 11: returning right arm through intermediate pose to initial pose.");
            ++phase;
            auto right_return_path = right_forward_path;
            std::reverse(right_return_path.begin(), right_return_path.end());
            if (right_return_path.empty()) {
                right_return_path = {
                    context.desired_pose_for_arm(PrimitiveArm::RIGHT).position,
                    initial_right_pose.position};
            }
            context.start_path(
                right_return_path,
                initial_right_pose.orientation,
                2,
                PrimitiveArm::RIGHT);
            return PrimitiveStatus::RUNNING;
        }

        default:
            phase = 0;
            right_forward_path.clear();
            left_forward_path.clear();
            context.restore_active_pose();
            return PrimitiveStatus::COMPLETED;
    }
}

const std::array<PrimitiveFunction, 14> kPrimitiveFunctions = {
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
