#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yumi_cube/primitives.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CubeTrajectoryPublisher : public rclcpp::Node
{
public:
    // High-level task states. Short multi-step actions keep their own phase counter.
    enum class FSMState {
        IDLE,
        PICK_CUBE,
        CUBE_SCANNING,
        CHANGE_ARM,
        CHECK_SCAN,
        WAIT_FOR_SEQUENCE,
        PRIMITIVE,
        EXECUTING_TRAJECTORY
    };

    enum class Arm {
        RIGHT,
        LEFT
    };

    CubeTrajectoryPublisher() : Node("yumi_cube_node")
    {
        declare_parameter<double>("duration_sec", 2.0);
        declare_parameter<int>("selected_profile", 3);
        declare_parameter<double>("cube_size", 0.05);

        duration_sec_ = get_parameter("duration_sec").as_double();
        selected_profile_ = get_parameter("selected_profile").as_int();
        cube_size_ = get_parameter("cube_size").as_double();

        current_pose_right_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/yumi/right/current_pose", 10,
            std::bind(&CubeTrajectoryPublisher::currentPoseRightCallback, this, _1));
        current_pose_left_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/yumi/left/current_pose", 10,
            std::bind(&CubeTrajectoryPublisher::currentPoseLeftCallback, this, _1));
        cube_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/cube_pose", 10,
            std::bind(&CubeTrajectoryPublisher::cubePoseCallback, this, _1));
        sensor_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/sensor_pose", 10,
            std::bind(&CubeTrajectoryPublisher::sensorPoseCallback, this, _1));
        sequence_to_do_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/sequnce_to_do", 10,
            std::bind(&CubeTrajectoryPublisher::sequenceToDoCallback, this, _1));

        desired_pose_right_pub_ = create_publisher<geometry_msgs::msg::Pose>("/yumi/right/desired_pose", 10);
        desired_pose_left_pub_ = create_publisher<geometry_msgs::msg::Pose>("/yumi/left/desired_pose", 10);
        gripper_cmd_right_pub_ = create_publisher<std_msgs::msg::Bool>("/yumi/right/gripper_cmd", 10);
        gripper_cmd_left_pub_ = create_publisher<std_msgs::msg::Bool>("/yumi/left/gripper_cmd", 10);

        gripper_wait_until_ = now();
        timer_ = create_wall_timer(20ms, std::bind(&CubeTrajectoryPublisher::fsm_loop, this));

        RCLCPP_INFO(get_logger(), "Node started. Awaiting all initial topics...");
    }

private:
    void currentPoseRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_right_ = *msg;
        has_received_right_pose_ = true;
        update_active_pose(Arm::RIGHT, *msg);
    }

    void currentPoseLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_left_ = *msg;
        has_received_left_pose_ = true;
        update_active_pose(Arm::LEFT, *msg);
    }

    void cubePoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        cube_pose_ = *msg;
        has_received_cube_ = true;
    }

    void sensorPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        sensor_pose_ = *msg;
        has_received_sensor_ = true;
    }

    void sequenceToDoCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sequence_queue_.push(msg->data);
    }

    // Keep the generic current_pose_ synchronized with whichever arm is currently carrying the cube.
    void update_active_pose(Arm arm, const geometry_msgs::msg::Pose & pose)
    {
        if (active_arm_ != arm) {
            return;
        }

        current_pose_ = pose;
        has_received_pose_ = true;
    }

    // Timer callback: dispatch the current finite-state-machine state.
    void fsm_loop()
    {
        switch (current_state_) {
            case FSMState::IDLE:
                if (has_received_pose_) {
                    RCLCPP_INFO(get_logger(), "Pose received. Initializing trajectory...");
                    desired_pose_ = current_pose_;
                    current_state_ = FSMState::PICK_CUBE;
                }
                break;

            case FSMState::PICK_CUBE:
                pick_cube();
                break;

            case FSMState::CUBE_SCANNING:
                cube_scan();
                break;

            case FSMState::CHANGE_ARM:
                change_arm();
                break;

            case FSMState::CHECK_SCAN:
                RCLCPP_INFO(get_logger(), "SCAN COMPLETED");
                current_state_ = FSMState::WAIT_FOR_SEQUENCE;
                break;

            case FSMState::WAIT_FOR_SEQUENCE:
                wait_for_sequence();
                break;

            case FSMState::PRIMITIVE:
                execute_primitive();
                break;

            case FSMState::EXECUTING_TRAJECTORY:
                execute_smooth_trajectory(active_path_);
                break;

            default:
                RCLCPP_WARN(get_logger(), "Unknown state!");
                break;
        }
    }

    void wait_for_sequence()
    {
        if (sequence_queue_.empty()) {
            return;
        }

        current_primitive_ = sequence_queue_.front();
        sequence_queue_.pop();
        current_state_ = FSMState::PRIMITIVE;
    }

    void execute_primitive()
    {
        const auto status = yumi_cube::execute_primitive(
            current_primitive_,
            current_pose_,
            get_logger(),
            primitive_context());
        if (status == yumi_cube::PrimitiveStatus::COMPLETED) {
            current_state_ = FSMState::WAIT_FOR_SEQUENCE;
        }
    }

    Arm to_node_arm(yumi_cube::PrimitiveArm arm) const
    {
        return arm == yumi_cube::PrimitiveArm::RIGHT ? Arm::RIGHT : Arm::LEFT;
    }

    yumi_cube::PrimitiveContext primitive_context()
    {
        yumi_cube::PrimitiveContext context;
        context.is_active_arm = [this](yumi_cube::PrimitiveArm arm) {
            return active_arm_ == to_node_arm(arm);
        };
        context.has_pose_for_arm = [this](yumi_cube::PrimitiveArm arm) {
            return has_pose_for_arm(to_node_arm(arm));
        };
        context.waiting_for_gripper = [this]() {
            return waiting_for_gripper();
        };
        context.request_change_arm = [this]() {
            start_change_arm(FSMState::PRIMITIVE);
        };
        context.send_gripper_command = [this](bool close, yumi_cube::PrimitiveArm arm) {
            send_gripper_command(close, to_node_arm(arm));
        };
        context.pose_for_arm = [this](yumi_cube::PrimitiveArm arm) {
            return pose_for_arm(to_node_arm(arm));
        };
        context.desired_pose_for_arm = [this](yumi_cube::PrimitiveArm arm) {
            return desired_pose_for_arm(to_node_arm(arm));
        };
        context.build_handoff_change_path = [this](
            const geometry_msgs::msg::Point & from,
            const geometry_msgs::msg::Point & cube_position,
            yumi_cube::PrimitiveArm target_arm) {
            return build_handoff_change_path(from, cube_position, to_node_arm(target_arm));
        };
        context.opposite_handoff_orientation = [this](
            const geometry_msgs::msg::Quaternion & source_q,
            const geometry_msgs::msg::Quaternion & target_reference_q) {
            return opposite_handoff_orientation(source_q, target_reference_q);
        };
        context.hold_path = [](const geometry_msgs::msg::Point & p) {
            return hold_path(p);
        };
        context.rotated_orientation_from = [this](
            const geometry_msgs::msg::Quaternion & base_orientation,
            double roll,
            double pitch,
            double yaw) {
            return rotated_orientation_from(base_orientation, roll, pitch, yaw);
        };
        context.start_path = [this](
            const std::vector<geometry_msgs::msg::Point> & path,
            const geometry_msgs::msg::Quaternion & orientation,
            int n_segments,
            yumi_cube::PrimitiveArm execution_arm) {
            start_path(path, orientation, FSMState::PRIMITIVE, n_segments, to_node_arm(execution_arm));
        };
        context.build_path = [this](const std::vector<geometry_msgs::msg::Point> & corners) {
            return build_path(corners);
        };
        context.restore_active_pose = [this]() {
            current_pose_ = pose_for_arm(active_arm_);
            desired_pose_ = desired_pose_for_arm(active_arm_);
        };
        return context;
    }

    // Initial sequence: grasp the cube with the active arm and place it in front of the sensor.
    void pick_cube()
    {
        if (!has_received_cube_ || !has_received_sensor_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for /cube_pose and /sensor_pose before picking the cube...");
            return;
        }

        if (waiting_for_gripper()) {
            return;
        }

        const auto grasp_q = grasp_orientation(cube_pose_.orientation);
        const auto sensor_q = sensor_view_orientation(sensor_pose_.orientation);
        const auto view_point = sensor_front_position(sensor_pose_);

        switch (pick_phase_) {
            case 0:
                RCLCPP_INFO(get_logger(), "Opening right gripper before picking the cube");
                send_gripper_command(false);
                ++pick_phase_;
                break;

            case 1:
                RCLCPP_INFO(get_logger(), "Moving right arm above the cube");
                ++pick_phase_;
                cube_pose_.position.z += cube_size_/2 + 0.025;
                start_path(build_pick_path(cube_pose_.position), grasp_q, FSMState::PICK_CUBE, 2);
                break;

            case 2:
                RCLCPP_INFO(get_logger(), "Closing right gripper on the cube");
                send_gripper_command(true);
                ++pick_phase_;
                break;

            case 3:
                RCLCPP_INFO(get_logger(), "Moving cube in front of the sensor");
                ++pick_phase_;
                start_path(build_place_path(view_point), sensor_q, FSMState::PICK_CUBE, 3);
                break;

            case 4:
                RCLCPP_INFO(get_logger(), "Cube positioned in front of the sensor");
                current_state_ = FSMState::CUBE_SCANNING;
                break;
        }
    }

    // Scan sequence is intentionally split by phase so the handoff can happen mid-scan.
    void cube_scan()
    {
        const auto hold_position = hold_path(current_pose_.position);

        switch (scan_phase_) {
            case 0:
                scan_start_pose_ = current_pose_;
                RCLCPP_INFO(get_logger(), "Scanning cube: rotating pi around local z");
                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(0.0, 0.0, M_PI), FSMState::CUBE_SCANNING);
                break;

            case 1:
                RCLCPP_INFO(get_logger(), "Scanning cube: rotating pi/2 around local x");
                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(M_PI_2, 0.0, 0.0), FSMState::CUBE_SCANNING);
                break;

            case 2:
                return_to_scan_start();
                break;

            case 3:
                ++scan_phase_;
                start_change_arm(FSMState::CUBE_SCANNING);
                break;

            case 4:
                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(0.0, 0.0, M_PI_2), FSMState::CUBE_SCANNING);
                break;

            case 5:
                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(0.0, 0.0, -M_PI), FSMState::CUBE_SCANNING);
                break;

            case 6:
                return_to_scan_start();
                break;

            default:
                RCLCPP_INFO(get_logger(), "Cube scan completed");
                scan_phase_ = 0;
                start_change_arm(FSMState::CHECK_SCAN);
                break;
        }
    }

    // Return to the pose saved at the start of the current scan segment before continuing.
    void return_to_scan_start()
    {
        RCLCPP_INFO(get_logger(), "Scanning cube: returning to initial scanning pose");
        ++scan_phase_;
        start_path(
            build_segment(current_pose_.position, scan_start_pose_.position),
            scan_start_pose_.orientation,
            FSMState::CUBE_SCANNING);
    }

    // CHANGE_ARM is reusable: callers decide which FSM state should resume after handoff.
    void start_change_arm(FSMState return_state)
    {
        change_return_state_ = return_state;
        current_state_ = FSMState::CHANGE_ARM;
    }

    // Transfers the cube from the current active arm to the opposite arm.
    void change_arm()
    {
        const Arm source_arm = active_arm_;
        const Arm target_arm = other_arm(source_arm);

        if (!has_pose_for_arm(target_arm)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for %s arm pose before changing arm...", arm_name(target_arm));
            return;
        }

        if (waiting_for_gripper()) {
            return;
        }

        const auto source_pose = pose_for_arm(source_arm);
        const auto target_pose = pose_for_arm(target_arm);
        const auto target_orientation = opposite_handoff_orientation(
            source_pose.orientation, target_pose.orientation);

        switch (change_phase_) {
            case 0:
                RCLCPP_INFO(get_logger(), "Opening %s gripper for cube handoff", arm_name(target_arm));
                send_gripper_command(false, target_arm);
                ++change_phase_;
                break;

            case 1:
                RCLCPP_INFO(get_logger(), "Moving %s arm to cube handoff pose", arm_name(target_arm));
                ++change_phase_;
                start_path(
                    build_handoff_change_path(target_pose.position, source_pose.position, target_arm),
                    target_orientation,
                    FSMState::CHANGE_ARM,
                    2,
                    target_arm);
                break;

            case 2:
                RCLCPP_INFO(get_logger(), "Closing %s gripper on the cube", arm_name(target_arm));
                send_gripper_command(true, target_arm);
                ++change_phase_;
                break;

            case 3:
                RCLCPP_INFO(get_logger(), "Opening %s gripper to release the cube", arm_name(source_arm));
                send_gripper_command(false, source_arm);
                ++change_phase_;
                break;

            case 4:
                RCLCPP_INFO(get_logger(), "Moving %s arm away from the handoff", arm_name(source_arm));
                ++change_phase_;
                start_path(
                    build_handoff_retreat_path(source_pose.position, source_arm),
                    source_pose.orientation,
                    FSMState::CHANGE_ARM,
                    1,
                    source_arm);
                break;

            default:
                active_arm_ = target_arm;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                scan_start_pose_ = current_pose_;
                RCLCPP_INFO(get_logger(), "Cube handoff completed. Active arm is now %s", arm_name(active_arm_));
                change_phase_ = 0;
                current_state_ = change_return_state_;
                break;
        }
    }

    bool waiting_for_gripper() const
    {
        return now() < gripper_wait_until_;
    }

    geometry_msgs::msg::Point above(const geometry_msgs::msg::Point & p) const
    {
        geometry_msgs::msg::Point q = p;
        q.z += 0.10;
        return q;
    }

    // Two identical waypoints let the trajectory executor rotate in place.
    static std::vector<geometry_msgs::msg::Point> hold_path(const geometry_msgs::msg::Point & p)
    {
        return {p, p};
    }

    // Mirror y-offsets for the two YuMi arms so handoff approach/retreat stays symmetric.
    double handoff_side_sign(Arm arm) const
    {
        return arm == Arm::LEFT ? 1.0 : -1.0;
    }

    geometry_msgs::msg::Point side_offset(
        const geometry_msgs::msg::Point & p,
        Arm arm,
        double distance) const
    {
        geometry_msgs::msg::Point q = p;
        q.y += handoff_side_sign(arm) * distance;
        return q;
    }

    // Applies roll/pitch/yaw in the local frame of the provided base orientation.
    geometry_msgs::msg::Quaternion rotated_orientation_from(
        const geometry_msgs::msg::Quaternion & base_orientation,
        double roll,
        double pitch,
        double yaw) const
    {
        tf2::Quaternion q_current;
        tf2::fromMsg(base_orientation, q_current);

        tf2::Quaternion q_delta;
        q_delta.setRPY(roll, pitch, yaw);

        tf2::Quaternion q_target = q_current * q_delta;
        q_target.normalize();
        return tf2::toMsg(q_target);
    }

    geometry_msgs::msg::Quaternion local_rotated_orientation(
        double roll,
        double pitch,
        double yaw) const
    {
        return rotated_orientation_from(desired_pose_.orientation, roll, pitch, yaw);
    }

    // Stores a path and switches execution to the shared trajectory publisher.
    void start_path(
        const std::vector<geometry_msgs::msg::Point> & path,
        const geometry_msgs::msg::Quaternion & orientation,
        FSMState return_state,
        int n_segments = 1)
    {
        start_path(path, orientation, return_state, n_segments, active_arm_);
    }

    void start_path(
        const std::vector<geometry_msgs::msg::Point> & path,
        const geometry_msgs::msg::Quaternion & orientation,
        FSMState return_state,
        int n_segments,
        Arm execution_arm)
    {
        execution_arm_ = execution_arm;
        active_path_ = path;
        active_duration_ = duration_sec_ * std::max(1, n_segments);
        path_start_orientation_ = desired_pose_for_arm(execution_arm_).orientation;
        path_target_orientation_ = orientation;
        start_time_ = now();
        next_state_ = return_state;
        current_state_ = FSMState::EXECUTING_TRAJECTORY;
    }

    void send_gripper_command(bool close)
    {
        send_gripper_command(close, active_arm_);
    }

    void send_gripper_command(bool close, Arm arm)
    {
        std_msgs::msg::Bool msg;
        msg.data = close;
        gripper_pub_for_arm(arm)->publish(msg);
        gripper_wait_until_ = now() + rclcpp::Duration::from_seconds(1.0);
    }

    static double distance(
        const geometry_msgs::msg::Point & a,
        const geometry_msgs::msg::Point & b)
    {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static geometry_msgs::msg::Point interpolate(
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & to,
        double s)
    {
        geometry_msgs::msg::Point p;
        p.x = from.x + s * (to.x - from.x);
        p.y = from.y + s * (to.y - from.y);
        p.z = from.z + s * (to.z - from.z);
        return p;
    }

    // Distance-based interpolation keeps blend points stable even when path segments have different lengths.
    static geometry_msgs::msg::Point point_at_distance(
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & to,
        double target_distance,
        double total_distance)
    {
        const double s = total_distance > 0.0 ? target_distance / total_distance : 0.0;
        return interpolate(from, to, s);
    }

    std::vector<geometry_msgs::msg::Point> build_segment(
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & to) const
    {
        std::vector<geometry_msgs::msg::Point> path;
        constexpr int n_points = 20;
        path.reserve(n_points + 1);

        for (int i = 0; i <= n_points; ++i) {
            path.push_back(interpolate(from, to, static_cast<double>(i) / n_points));
        }

        return path;
    }

    // Builds a blended polyline through corner points, preserving the existing straight-line endpoints.
    std::vector<geometry_msgs::msg::Point> build_path(
        const std::vector<geometry_msgs::msg::Point> & corners,
        double blend_radius = 0.05,
        int curve_points = 10) const
    {
        std::vector<geometry_msgs::msg::Point> path;
        if (corners.empty()) {
            return path;
        }
        if (corners.size() == 1) {
            return {corners.front()};
        }

        path.push_back(corners.front());

        for (size_t c = 1; c < corners.size() - 1; ++c) {
            const auto & p_prev = corners[c - 1];
            const auto & p_curr = corners[c];
            const auto & p_next = corners[c + 1];

            const double dist_in = distance(p_prev, p_curr);
            const double dist_out = distance(p_curr, p_next);
            const double safe_radius = std::min({blend_radius, dist_in / 2.0, dist_out / 2.0});

            const auto blend_start = point_at_distance(p_prev, p_curr, dist_in - safe_radius, dist_in);
            const auto blend_end = point_at_distance(p_curr, p_next, safe_radius, dist_out);

            append_without_first(path, build_segment(path.back(), blend_start));
            append_bezier(path, blend_start, p_curr, blend_end, curve_points);
        }

        append_without_first(path, build_segment(path.back(), corners.back()));
        return path;
    }

    static void append_without_first(
        std::vector<geometry_msgs::msg::Point> & dst,
        const std::vector<geometry_msgs::msg::Point> & src)
    {
        dst.insert(dst.end(), std::next(src.begin()), src.end());
    }

    static void append_bezier(
        std::vector<geometry_msgs::msg::Point> & path,
        const geometry_msgs::msg::Point & p0,
        const geometry_msgs::msg::Point & p1,
        const geometry_msgs::msg::Point & p2,
        int n_points)
    {
        for (int i = 1; i <= n_points; ++i) {
            const double t = static_cast<double>(i) / n_points;
            const double u = 1.0 - t;

            geometry_msgs::msg::Point p;
            p.x = u * u * p0.x + 2.0 * u * t * p1.x + t * t * p2.x;
            p.y = u * u * p0.y + 2.0 * u * t * p1.y + t * t * p2.y;
            p.z = u * u * p0.z + 2.0 * u * t * p1.z + t * t * p2.z;
            path.push_back(p);
        }
    }

    std::vector<geometry_msgs::msg::Point> build_pick_path(
        const geometry_msgs::msg::Point & cube_position) const
    {
        return build_handoff_pick_path(current_pose_.position, cube_position);
    }

    std::vector<geometry_msgs::msg::Point> build_handoff_pick_path(
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & cube_position) const
    {
        return build_path({from, above(cube_position), cube_position});
    }

    // Receiver approaches the cube from its own side, then closes at the handoff point.
    std::vector<geometry_msgs::msg::Point> build_handoff_change_path(
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & cube_position,
        Arm target_arm) const
    {
        const auto handoff_position = side_offset(cube_position, target_arm, cube_size_ + 0.052);
        return build_path({
            from,
            side_offset(handoff_position, target_arm, 0.05),
            handoff_position
        });
    }

    // Releasing arm retreats only along y so it does not disturb the newly active arm.
    std::vector<geometry_msgs::msg::Point> build_handoff_retreat_path(
        const geometry_msgs::msg::Point & source_position,
        Arm source_arm) const
    {
        return build_path({source_position, side_offset(source_position, source_arm, 0.10)});
    }

    std::vector<geometry_msgs::msg::Point> build_place_path(
        const geometry_msgs::msg::Point & place_position) const
    {
        return build_path({current_pose_.position, above(current_pose_.position), place_position});
    }

    // Aligns the gripper with the cube while choosing the yaw-equivalent pose closest to the current wrist.
    geometry_msgs::msg::Quaternion grasp_orientation(
        const geometry_msgs::msg::Quaternion & cube_q) const
    {
        tf2::Quaternion q_cube;
        tf2::fromMsg(cube_q, q_cube);

        tf2::Vector3 z = -tf2::quatRotate(q_cube, tf2::Vector3(0, 0, 1));
        z.normalize();

        tf2::Vector3 x = tf2::quatRotate(q_cube, tf2::Vector3(1, 0, 0)) -
            z * tf2::quatRotate(q_cube, tf2::Vector3(1, 0, 0)).dot(z);
        if (x.length2() < 1e-8) {
            x = tf2::quatRotate(q_cube, tf2::Vector3(0, 1, 0)) -
                z * tf2::quatRotate(q_cube, tf2::Vector3(0, 1, 0)).dot(z);
        }
        x.normalize();

        tf2::Vector3 y = z.cross(x);
        y.normalize();

        tf2::Matrix3x3 R(
            x.x(), y.x(), z.x(),
            x.y(), y.y(), z.y(),
            x.z(), y.z(), z.z());

        tf2::Quaternion base_q;
        R.getRotation(base_q);
        base_q.normalize();

        return closest_symmetric_yaw(base_q, desired_pose_.orientation);
    }

    // Points the held cube toward the sensor while keeping the wrist x-axis as continuous as possible.
    geometry_msgs::msg::Quaternion sensor_view_orientation(
        const geometry_msgs::msg::Quaternion & sensor_q) const
    {
        tf2::Quaternion q_sensor;
        tf2::fromMsg(sensor_q, q_sensor);

        tf2::Vector3 z = tf2::quatRotate(q_sensor, tf2::Vector3(0, 1, 0));
        if (z.length2() < 1e-8) {
            z = tf2::Vector3(0, 1, 0);
        }
        z.normalize();

        tf2::Quaternion q_ref;
        tf2::fromMsg(desired_pose_.orientation, q_ref);
        tf2::Vector3 x = project_axis_onto_plane(tf2::quatRotate(q_ref, tf2::Vector3(1, 0, 0)), z);

        if (x.length2() < 1e-8) {
            x = project_axis_onto_plane(tf2::quatRotate(q_sensor, tf2::Vector3(1, 0, 0)), z);
        }
        if (x.length2() < 1e-8) {
            x = project_axis_onto_plane(tf2::quatRotate(q_sensor, tf2::Vector3(0, 0, 1)), z);
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

    // Removes the component along normal, leaving an axis tangent to the view plane.
    static tf2::Vector3 project_axis_onto_plane(const tf2::Vector3 & axis, const tf2::Vector3 & normal)
    {
        return axis - normal * axis.dot(normal);
    }

    geometry_msgs::msg::Quaternion closest_symmetric_yaw(
        const tf2::Quaternion & base_q,
        const geometry_msgs::msg::Quaternion & reference_q) const
    {
        geometry_msgs::msg::Quaternion best;
        double best_dot = -1.0;

        for (int k = 0; k < 4; ++k) {
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, k * M_PI_2);

            tf2::Quaternion q_candidate = base_q * q_yaw;
            q_candidate.normalize();
            const auto candidate = tf2::toMsg(q_candidate);
            const double dot = quaternion_abs_dot(candidate, reference_q);

            if (dot > best_dot) {
                best_dot = dot;
                best = candidate;
            }
        }

        return best;
    }

    // Computes the receiving-arm orientation and selects the mirrored finger clearance with less wrist motion.
    geometry_msgs::msg::Quaternion opposite_handoff_orientation(
        const geometry_msgs::msg::Quaternion & source_q,
        const geometry_msgs::msg::Quaternion & target_reference_q) const
    {
        tf2::Quaternion q_source;
        tf2::fromMsg(source_q, q_source);

        tf2::Quaternion q_flip_x;
        q_flip_x.setRPY(M_PI, 0.0, 0.0);

        geometry_msgs::msg::Quaternion best;
        double best_dot = -1.0;

        for (const double yaw_clearance : {M_PI_2, -M_PI_2}) {
            tf2::Quaternion q_finger_clearance;
            q_finger_clearance.setRPY(0.0, 0.0, yaw_clearance);

            tf2::Quaternion q_candidate = q_source * q_flip_x * q_finger_clearance;
            q_candidate.normalize();
            const auto candidate = tf2::toMsg(q_candidate);
            const double dot = quaternion_abs_dot(candidate, target_reference_q);

            if (dot > best_dot) {
                best_dot = dot;
                best = candidate;
            }
        }

        return best;
    }

    static double quaternion_abs_dot(
        const geometry_msgs::msg::Quaternion & a,
        const geometry_msgs::msg::Quaternion & b)
    {
        return std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    }

    static Arm other_arm(Arm arm)
    {
        return arm == Arm::RIGHT ? Arm::LEFT : Arm::RIGHT;
    }

    static const char * arm_name(Arm arm)
    {
        return arm == Arm::RIGHT ? "right" : "left";
    }

    bool has_pose_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? has_received_right_pose_ : has_received_left_pose_;
    }

    const geometry_msgs::msg::Pose & pose_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? current_pose_right_ : current_pose_left_;
    }

    const geometry_msgs::msg::Pose & desired_pose_for_arm(Arm arm) const
    {
        if (arm == Arm::RIGHT) {
            return has_commanded_right_pose_ ? desired_pose_right_ : current_pose_right_;
        }
        return has_commanded_left_pose_ ? desired_pose_left_ : current_pose_left_;
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pub_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? desired_pose_right_pub_ : desired_pose_left_pub_;
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? gripper_cmd_right_pub_ : gripper_cmd_left_pub_;
    }

    void store_desired_pose_for_arm(Arm arm, const geometry_msgs::msg::Pose & pose)
    {
        if (arm == Arm::RIGHT) {
            desired_pose_right_ = pose;
            has_commanded_right_pose_ = true;
        } else {
            desired_pose_left_ = pose;
            has_commanded_left_pose_ = true;
        }
    }

    geometry_msgs::msg::Point sensor_front_position(
        const geometry_msgs::msg::Pose & sensor_pose) const
    {
        tf2::Quaternion q_sensor;
        tf2::fromMsg(sensor_pose.orientation, q_sensor);

        tf2::Vector3 sensor_z = tf2::quatRotate(q_sensor, tf2::Vector3(0, 0, 1));
        if (sensor_z.length2() < 1e-8) {
            sensor_z = tf2::Vector3(0, 0, 1);
        }
        sensor_z.normalize();

        geometry_msgs::msg::Point p = sensor_pose.position;
        p.x += sensor_front_distance_ * sensor_z.x() + 0.2;
        p.y += sensor_front_distance_ * sensor_z.y() - cube_size_ / 2.0;
        p.z += sensor_front_distance_ * sensor_z.z();
        return p;
    }

    // Shortest-arc quaternion interpolation for smooth orientation commands.
    static geometry_msgs::msg::Quaternion slerp(
        const geometry_msgs::msg::Quaternion & qa,
        const geometry_msgs::msg::Quaternion & qb,
        double t)
    {
        double dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
        double sign = 1.0;
        if (dot < 0.0) {
            dot = -dot;
            sign = -1.0;
        }

        double wa;
        double wb;
        if (dot > 0.9995) {
            wa = 1.0 - t;
            wb = t;
        } else {
            const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
            const double sin_theta = std::sin(theta);
            wa = std::sin((1.0 - t) * theta) / sin_theta;
            wb = std::sin(t * theta) / sin_theta;
        }
        wb *= sign;

        geometry_msgs::msg::Quaternion q;
        q.x = wa * qa.x + wb * qb.x;
        q.y = wa * qa.y + wb * qb.y;
        q.z = wa * qa.z + wb * qb.z;
        q.w = wa * qa.w + wb * qb.w;

        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
        q.w /= norm;
        return q;
    }

    // Publishes one time-scaled pose sample per timer tick until the active path is complete.
    void execute_smooth_trajectory(const std::vector<geometry_msgs::msg::Point> & path)
    {
        if (path.size() < 2) {
            RCLCPP_ERROR(get_logger(), "Empty path, nothing to execute.");
            return;
        }

        if (selected_profile_ != 3 && selected_profile_ != 5 && selected_profile_ != 7) {
            RCLCPP_ERROR(get_logger(), "Invalid time law profile %d, using 5th degree", selected_profile_);
            selected_profile_ = 5;
        }

        const double t = (now() - start_time_).seconds();
        const double tau = std::min(t / active_duration_, 1.0);
        const double s = time_law(tau);
        const auto cumulative_lengths = path_lengths(path);
        const double target_len = s * cumulative_lengths.back();

        size_t i = 1;
        while (i < path.size() - 1 && cumulative_lengths[i] < target_len) {
            ++i;
        }

        const double seg_len = cumulative_lengths[i] - cumulative_lengths[i - 1];
        const double alpha = seg_len > 0.0 ? (target_len - cumulative_lengths[i - 1]) / seg_len : 1.0;

        desired_pose_.position = interpolate(path[i - 1], path[i], alpha);
        desired_pose_.orientation = slerp(path_start_orientation_, path_target_orientation_, s);
        desired_pub_for_arm(execution_arm_)->publish(desired_pose_);
        store_desired_pose_for_arm(execution_arm_, desired_pose_);

        if (tau >= 1.0) {
            RCLCPP_INFO(get_logger(), "Path completed.");
            current_state_ = next_state_;
        }
    }

    // Polynomial time laws with zero endpoint velocity; higher degrees smooth more derivatives.
    double time_law(double tau) const
    {
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;

        switch (selected_profile_) {
            case 3:
                return tau2 * (3.0 - 2.0 * tau);
            case 5:
                return tau3 * (10.0 - 15.0 * tau + 6.0 * tau2);
            case 7: {
                const double tau4 = tau3 * tau;
                return tau4 * (35.0 - 84.0 * tau + 70.0 * tau2 - 20.0 * tau3);
            }
            default:
                return tau3 * (10.0 - 15.0 * tau + 6.0 * tau2);
        }
    }

    // Cumulative arc lengths let the time law move at uniform speed along blended paths.
    static std::vector<double> path_lengths(const std::vector<geometry_msgs::msg::Point> & path)
    {
        std::vector<double> lengths(path.size(), 0.0);
        for (size_t i = 1; i < path.size(); ++i) {
            lengths[i] = lengths[i - 1] + distance(path[i - 1], path[i]);
        }
        return lengths;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_right_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_left_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cube_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sensor_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sequence_to_do_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_right_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_left_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // FSM state and return targets used by the shared trajectory and handoff states.
    FSMState current_state_ = FSMState::IDLE;
    FSMState next_state_ = FSMState::CHECK_SCAN;
    FSMState change_return_state_ = FSMState::CUBE_SCANNING;
    Arm active_arm_ = Arm::RIGHT;
    Arm execution_arm_ = Arm::RIGHT;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose desired_pose_;
    geometry_msgs::msg::Pose current_pose_right_;
    geometry_msgs::msg::Pose current_pose_left_;
    geometry_msgs::msg::Pose desired_pose_right_;
    geometry_msgs::msg::Pose desired_pose_left_;
    geometry_msgs::msg::Pose scan_start_pose_;
    geometry_msgs::msg::Pose cube_pose_;
    geometry_msgs::msg::Pose sensor_pose_;

    // Topic readiness flags prevent the FSM from planning before all needed scene data exists.
    bool has_received_pose_ = false;
    bool has_received_right_pose_ = false;
    bool has_received_left_pose_ = false;
    bool has_commanded_right_pose_ = false;
    bool has_commanded_left_pose_ = false;
    bool has_received_cube_ = false;
    bool has_received_sensor_ = false;

    rclcpp::Time start_time_;
    rclcpp::Time gripper_wait_until_;
    double duration_sec_ = 2.0;
    double active_duration_ = 5.0;
    double cube_size_ = 0.05;
    double sensor_front_distance_ = 0.15;
    int selected_profile_ = 3;
    int current_primitive_ = 0;
    std::queue<int> sequence_queue_;
    // Phase counters make long actions resumable across timer callbacks.
    int pick_phase_ = 0;
    int scan_phase_ = 0;
    int change_phase_ = 0;

    std::vector<geometry_msgs::msg::Point> active_path_;
    geometry_msgs::msg::Quaternion path_start_orientation_;
    geometry_msgs::msg::Quaternion path_target_orientation_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
