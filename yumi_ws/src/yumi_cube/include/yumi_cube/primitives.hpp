#pragma once

#include <functional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/logger.hpp"

namespace yumi_cube
{

enum class PrimitiveArm {
    RIGHT,
    LEFT
};

enum class PrimitiveStatus {
    RUNNING,
    COMPLETED
};

struct PrimitiveContext {
    std::function<bool(PrimitiveArm)> is_active_arm;
    std::function<bool(PrimitiveArm)> has_pose_for_arm;
    std::function<bool()> waiting_for_gripper;
    std::function<void()> request_change_arm;
    std::function<void(bool, PrimitiveArm)> send_gripper_command;
    std::function<geometry_msgs::msg::Pose(PrimitiveArm)> pose_for_arm;
    std::function<geometry_msgs::msg::Pose(PrimitiveArm)> desired_pose_for_arm;
    std::function<std::vector<geometry_msgs::msg::Point>(
        const geometry_msgs::msg::Point &,
        const geometry_msgs::msg::Point &,
        PrimitiveArm)> build_handoff_change_path;
    std::function<geometry_msgs::msg::Quaternion(
        const geometry_msgs::msg::Quaternion &,
        const geometry_msgs::msg::Quaternion &)> opposite_handoff_orientation;
    std::function<std::vector<geometry_msgs::msg::Point>(const geometry_msgs::msg::Point &)> hold_path;
    std::function<geometry_msgs::msg::Quaternion(
        const geometry_msgs::msg::Quaternion &,
        double,
        double,
        double)> rotated_orientation_from;
    std::function<void(
        const std::vector<geometry_msgs::msg::Point> &,
        const geometry_msgs::msg::Quaternion &,
        int,
        PrimitiveArm)> start_path;
    std::function<std::vector<geometry_msgs::msg::Point>(
        const std::vector<geometry_msgs::msg::Point> &)> build_path;
    std::function<void()> restore_active_pose;
};

PrimitiveStatus execute_primitive(
    int primitive_value,
    const geometry_msgs::msg::Pose & current_pose,
    const rclcpp::Logger & logger,
    const PrimitiveContext & context);

}  // namespace yumi_cube
