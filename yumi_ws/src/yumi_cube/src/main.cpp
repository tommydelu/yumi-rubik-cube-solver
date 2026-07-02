#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <functional>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
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
        GET_CUBE_POSE,
        PICK_CUBE,
        CUBE_SCANNING,
        CHANGE_ARM,
        CHECK_SCAN,
        WAIT_FOR_SEQUENCE,
        PRIMITIVE,
        EXECUTING_TRAJECTORY,
        EXECUTING_JOINT_TRAJECTORY,
        EXECUTING_JOINT_DELTA
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

        get_cube_pose_sub_ =  create_subscription<geometry_msgs::msg::Pose>(
            "/get_cube_pose", 10,
            std::bind(&CubeTrajectoryPublisher::getCubePoseCallback, this, _1));
        current_pose_right_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/yumi/right/current_pose", 10,
            std::bind(&CubeTrajectoryPublisher::currentPoseRightCallback, this, _1));
        current_pose_left_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/yumi/left/current_pose", 10,
            std::bind(&CubeTrajectoryPublisher::currentPoseLeftCallback, this, _1));
        current_joint_state_right_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/yumi/right/current_joint_state", 10,
            std::bind(&CubeTrajectoryPublisher::currentJointStateRightCallback, this, _1));
        current_joint_state_left_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/yumi/left/current_joint_state", 10,
            std::bind(&CubeTrajectoryPublisher::currentJointStateLeftCallback, this, _1));
        vision_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/yumi/vision", rclcpp::SensorDataQoS(),
            std::bind(&CubeTrajectoryPublisher::visionImageCallback, this, _1));
        vision_sensor_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/yumi/vision_sensor", rclcpp::SensorDataQoS(),
            std::bind(&CubeTrajectoryPublisher::visionImageCallback, this, _1));
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
        desired_joint_state_right_pub_ = create_publisher<sensor_msgs::msg::JointState>("/yumi/right/desired_joint_state", 10);
        desired_joint_state_left_pub_ = create_publisher<sensor_msgs::msg::JointState>("/yumi/left/desired_joint_state", 10);
        joint7_delta_right_pub_ = create_publisher<std_msgs::msg::Float64>("/yumi/right/joint7_delta", 10);
        joint7_delta_left_pub_ = create_publisher<std_msgs::msg::Float64>("/yumi/left/joint7_delta", 10);
        gripper_cmd_right_pub_ = create_publisher<std_msgs::msg::Bool>("/yumi/right/gripper_cmd", 10);
        gripper_cmd_left_pub_ = create_publisher<std_msgs::msg::Bool>("/yumi/left/gripper_cmd", 10);
        rubik_key_pub_ = create_publisher<std_msgs::msg::String>("/cube_cmd", 10);
        locate_cube_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/locate_cube/ready", 10);

        gripper_wait_until_ = now();
        timer_ = create_wall_timer(20ms, std::bind(&CubeTrajectoryPublisher::fsm_loop, this));

        RCLCPP_INFO(get_logger(), "Node started. Awaiting all initial topics...");
    }

    ~CubeTrajectoryPublisher() override
    {
        restore_keyboard_mode();
    }

private:
    void currentPoseRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_right_ = *msg;
        has_received_right_pose_ = true;
        update_active_pose(Arm::RIGHT, *msg);
    }

    void getCubePoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        cube_pose_pcl_ = *msg;
        has_received_cube_pcl_ = true;
        publish_locate_cube_ready(false);
        get_cube_pose_request_sent_ = false;
    }

    void currentPoseLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_left_ = *msg;
        has_received_left_pose_ = true;
        update_active_pose(Arm::LEFT, *msg);
    }

    void currentJointStateRightCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joints_right_ = msg->position;
        has_received_right_joints_ = current_joints_right_.size() >= 7;
    }

    void currentJointStateLeftCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joints_left_ = msg->position;
        has_received_left_joints_ = current_joints_left_.size() >= 7;
    }

    void visionImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_vision_image_ = *msg;
        has_received_vision_image_ = true;
        ++vision_image_counter_;
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
                    current_state_ = FSMState::GET_CUBE_POSE;
                }
                break;

            case FSMState::GET_CUBE_POSE:
                get_cube_pose();
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

            case FSMState::EXECUTING_JOINT_TRAJECTORY:
                execute_joint_trajectory();
                break;

            case FSMState::EXECUTING_JOINT_DELTA:
                execute_joint_delta_wait();
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
        context.has_joint_state_for_arm = [this](yumi_cube::PrimitiveArm arm) {
            return has_joint_state_for_arm(to_node_arm(arm));
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
        context.joint_state_for_arm = [this](yumi_cube::PrimitiveArm arm) {
            return joint_state_for_arm(to_node_arm(arm));
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
        context.start_joint_path = [this](
            const std::vector<double> & joint_positions,
            int n_segments,
            yumi_cube::PrimitiveArm execution_arm) {
            start_joint_path(joint_positions, FSMState::PRIMITIVE, n_segments, to_node_arm(execution_arm));
        };
        context.rotate_last_joint = [this](
            double angle,
            int n_segments,
            yumi_cube::PrimitiveArm execution_arm) {
            start_joint_delta(angle, FSMState::PRIMITIVE, n_segments, to_node_arm(execution_arm));
        };
        context.build_path = [this](const std::vector<geometry_msgs::msg::Point> & corners) {
            return build_path(corners);
        };
        context.restore_active_pose = [this]() {
            current_pose_ = pose_for_arm(active_arm_);
            desired_pose_ = desired_pose_for_arm(active_arm_);
        };
        context.send_rubik_key = [this](const std::string & keys) {
            send_rubik_key(keys);
        };
        return context;
    }

    void send_rubik_key(const std::string & keys)
    {
        for (const char key : keys) {
            if (std::isspace(static_cast<unsigned char>(key)) || key == ',') {
                continue;
            }

            std_msgs::msg::String msg;
            switch (std::tolower(static_cast<unsigned char>(key))) {
                case 'u': msg.data = "2,1,90"; break;
                case 'd': msg.data = "2,-1,-90"; break;
                case 'r': msg.data = "1,1,90"; break;
                case 'l': msg.data = "1,-1,-90"; break;
                case 'f': msg.data = "3,1,90"; break;
                case 'b': msg.data = "3,-1,-90"; break;
                default:
                    RCLCPP_WARN(get_logger(), "Ignoring unsupported Rubik keyboard command '%c'", key);
                    continue;
            }

            rubik_key_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Published Rubik command '%s' for key '%c'", msg.data.c_str(), key);
        }
    }

    void get_cube_pose()
    {
        if (!has_joint_state_for_arm(Arm::RIGHT) || !has_joint_state_for_arm(Arm::LEFT)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for both arm joint states before getting cube pose.");
            return;
        }

        switch (get_cube_pose_phase_) {
            case 0: {
                has_received_cube_pcl_ = false;
                get_cube_pose_request_sent_ = false;
                publish_locate_cube_ready(false);

                get_cube_pose_start_joints_ = joint_state_for_arm(Arm::RIGHT);
                get_cube_pose_left_start_joints_ = joint_state_for_arm(Arm::LEFT);
                if (get_cube_pose_start_joints_.size() < 7 || get_cube_pose_left_start_joints_.size() < 7) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Get cube pose received incomplete joint state: right=%zu left=%zu positions.",
                        get_cube_pose_start_joints_.size(),
                        get_cube_pose_left_start_joints_.size());
                    return;
                }

                get_cube_pose_start_joints_.resize(7);
                get_cube_pose_left_start_joints_.resize(7);
                auto target_joints = get_cube_pose_start_joints_;
                target_joints[0] -= M_PI_2;

                active_arm_ = Arm::RIGHT;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                RCLCPP_INFO(get_logger(), "Getting cube pose: rotating right joint 1 by -pi/2");
                ++get_cube_pose_phase_;
                start_joint_path(target_joints, FSMState::GET_CUBE_POSE, 1, Arm::RIGHT);
                break;
            }

            case 1: {
                if (get_cube_pose_left_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Get cube pose cannot move left arm: missing initial left joint snapshot.");
                    get_cube_pose_phase_ = 0;
                    return;
                }

                auto target_joints = get_cube_pose_left_start_joints_;
                target_joints[0] += M_PI_2;

                active_arm_ = Arm::LEFT;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                RCLCPP_INFO(get_logger(), "Getting cube pose: rotating left joint 1 by +pi/2");
                ++get_cube_pose_phase_;
                start_joint_path(target_joints, FSMState::GET_CUBE_POSE, 1, Arm::LEFT);
                break;
            }

            case 2:
                if (!has_received_cube_pcl_) {
                    if (!get_cube_pose_request_sent_) {
                        publish_locate_cube_ready(true);
                        get_cube_pose_request_sent_ = true;
                        RCLCPP_INFO(get_logger(), "Requested locate_cube_node to compute /get_cube_pose.");
                    }
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Waiting for /get_cube_pose before capturing cube pose.");
                        // ++get_cube_pose_phase_;
                    return;
                }else {
                    publish_locate_cube_ready(false);
                    get_cube_pose_request_sent_ = false;
                    // print values
                    RCLCPP_INFO(get_logger(), "Cube pose received: x=%.3f y=%.3f z=%.3f", cube_pose_pcl_.position.x, cube_pose_pcl_.position.y, cube_pose_pcl_.position.z);
                    RCLCPP_INFO(get_logger(), "Cube orientation received: x=%.3f y=%.3f z=%.3f w=%.3f", cube_pose_pcl_.orientation.x, cube_pose_pcl_.orientation.y, cube_pose_pcl_.orientation.z, cube_pose_pcl_.orientation.w);
                    // compute the error between the cube pose and the cube pose pcl
                    double error_x = cube_pose_pcl_.position.x - cube_pose_.position.x;
                    double error_y = cube_pose_pcl_.position.y - cube_pose_.position.y;
                    double error_z = cube_pose_pcl_.position.z - cube_pose_.position.z;
                    RCLCPP_INFO(get_logger(), "Cube pose error: x=%.3f y=%.3f z=%.3f", error_x, error_y, error_z);
                    ++get_cube_pose_phase_;
                }
                break;

            case 3:
                if (get_cube_pose_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Get cube pose cannot restore right arm: missing initial right joint snapshot.");
                    get_cube_pose_phase_ = 0;
                    return;
                }

                active_arm_ = Arm::RIGHT;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                RCLCPP_INFO(get_logger(), "Getting cube pose: returning right arm to initial joint configuration");
                ++get_cube_pose_phase_;
                start_joint_path(get_cube_pose_start_joints_, FSMState::GET_CUBE_POSE, 1, Arm::RIGHT);
                break;

            case 4:
                if (get_cube_pose_left_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Get cube pose cannot restore left arm: missing initial left joint snapshot.");
                    get_cube_pose_phase_ = 0;
                    return;
                }

                active_arm_ = Arm::LEFT;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                RCLCPP_INFO(get_logger(), "Getting cube pose: returning left arm to initial joint configuration");
                ++get_cube_pose_phase_;
                start_joint_path(get_cube_pose_left_start_joints_, FSMState::GET_CUBE_POSE, 1, Arm::LEFT);
                break;

            default:
                active_arm_ = Arm::RIGHT;
                current_pose_ = pose_for_arm(active_arm_);
                desired_pose_ = desired_pose_for_arm(active_arm_);
                RCLCPP_INFO(get_logger(), "GETTING CUBE POSE completed");
                get_cube_pose_phase_ = 0;
                current_state_ = FSMState::PICK_CUBE;
                break;
        }
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
            case 0: {
                if (!has_joint_state_for_arm(active_arm_)) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Scanning cube waiting for %s arm joint state before joint 7 rotation.",
                        arm_name(active_arm_));
                    return;
                }

                if (!capture_scan_photo_after_new_frame("right_start")) {
                    return;
                }

                scan_start_joints_ = joint_state_for_arm(active_arm_);
                if (scan_start_joints_.size() < 7) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Scanning cube received incomplete %s arm joint state: %zu positions.",
                        arm_name(active_arm_),
                        scan_start_joints_.size());
                    return;
                }

                scan_start_joints_.resize(7);
                auto target_joints = scan_start_joints_;
                target_joints[6] += M_PI;

                RCLCPP_INFO(get_logger(), "Scanning cube: rotating %s joint 7 by pi", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(target_joints, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;
            }

            case 1:
                if (!capture_scan_photo_after_new_frame("right_after_joint7_pi")) {
                    return;
                }

                if (scan_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Scanning cube cannot restore joint scan: missing initial joint snapshot.");
                    scan_phase_ = 0;
                    return;
                }

                RCLCPP_INFO(get_logger(), "Scanning cube: returning %s arm to initial joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(scan_start_joints_, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;

            case 2: {
                std::vector<double> target_joints = {
                    1.563, -1.016, -1.113, 0.890, 3.311, -1.285, 1.393
                };

                RCLCPP_INFO(get_logger(), "Scanning cube: moving %s arm to manual scan joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(target_joints, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;
            }

            case 3:
                if (!capture_scan_photo_after_new_frame("right_after_manual_joint_config")) {
                    return;
                }

                if (scan_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Scanning cube cannot restore joint scan: missing initial joint snapshot.");
                    scan_phase_ = 0;
                    return;
                }

                RCLCPP_INFO(get_logger(), "Scanning cube: returning %s arm to initial joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(scan_start_joints_, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;

            case 4:
                ++scan_phase_;
                start_change_arm(FSMState::CUBE_SCANNING);
                break;

            case 5:
                if (!has_joint_state_for_arm(active_arm_)) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Scanning cube waiting for %s arm joint state before left-arm scan.",
                        arm_name(active_arm_));
                    return;
                }

                scan_start_joints_ = joint_state_for_arm(active_arm_);
                if (scan_start_joints_.size() < 7) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Scanning cube received incomplete %s arm joint state: %zu positions.",
                        arm_name(active_arm_),
                        scan_start_joints_.size());
                    return;
                }

                scan_start_joints_.resize(7);
                RCLCPP_INFO(get_logger(), "Scanning cube: saved initial %s arm joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(0.0, 0.0, M_PI_2), FSMState::CUBE_SCANNING);
                break;

            case 6:
                if (!capture_scan_photo_after_new_frame("left_after_yaw_pi_2")) {
                    return;
                }

                ++scan_phase_;
                start_path(hold_position, local_rotated_orientation(0.0, 0.0, -M_PI), FSMState::CUBE_SCANNING);
                break;

            case 7: {
                if (!capture_scan_photo_after_new_frame("left_after_yaw_minus_pi")) {
                    return;
                }

                std::vector<double> target_joints = {
                    -1.263, -0.916, 0.551, 1.061, -3.254, -1.174, -1.826
                };

                RCLCPP_INFO(get_logger(), "Scanning cube: moving %s arm to left scan joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(target_joints, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;
            }

            case 8:
                if (!capture_scan_photo_after_new_frame("left_after_scan_joint_config")) {
                    return;
                }

                if (scan_start_joints_.size() < 7) {
                    RCLCPP_WARN(get_logger(), "Scanning cube cannot restore left-arm scan: missing initial joint snapshot.");
                    scan_phase_ = 0;
                    return;
                }

                RCLCPP_INFO(get_logger(), "Scanning cube: returning %s arm to initial joint configuration", arm_name(active_arm_));
                ++scan_phase_;
                start_joint_path(scan_start_joints_, FSMState::CUBE_SCANNING, 1, active_arm_);
                break;

            default:
                RCLCPP_INFO(get_logger(), "Cube scan completed");
                scan_phase_ = 0;
                start_change_arm(FSMState::CHECK_SCAN);
                break;
        }
    }

    bool capture_scan_photo(const std::string & label)
    {
        if (!has_received_vision_image_) {
            RCLCPP_WARN(get_logger(), "Cannot save scan photo '%s': no image received yet on /yumi/vision or /yumi/vision_sensor.", label.c_str());
            return false;
        }

        const std::string directory = "src/yumi_cube/scan_images";
        if (mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST) {
            RCLCPP_WARN(get_logger(), "Cannot create scan image directory '%s'.", directory.c_str());
            return false;
        }

        const auto & image = latest_vision_image_;
        if (image.width == 0 || image.height == 0 || image.data.empty()) {
            RCLCPP_WARN(get_logger(), "Cannot save scan photo '%s': latest image is empty.", label.c_str());
            return false;
        }

        const bool mono = image.encoding == "mono8" || image.encoding == "8UC1";
        const bool rgb = image.encoding == "rgb8" || image.encoding == "bgr8" ||
            image.encoding == "rgba8" || image.encoding == "bgra8";
        if (!mono && !rgb) {
            RCLCPP_WARN(
                get_logger(),
                "Cannot save scan photo '%s': unsupported image encoding '%s'.",
                label.c_str(),
                image.encoding.c_str());
            return false;
        }

        const std::string extension = mono ? ".pgm" : ".ppm";
        std::ostringstream name;
        name << directory << "/scan_" << std::setw(2) << std::setfill('0')
             << scan_photo_counter_++ << "_phase_" << scan_phase_ << "_" << label << extension;

        std::ofstream out(name.str(), std::ios::binary);
        if (!out) {
            RCLCPP_WARN(get_logger(), "Cannot open scan photo file '%s'.", name.str().c_str());
            return false;
        }

        if (mono) {
            const size_t step = image.step != 0 ? image.step : image.width;
            out << "P5\n" << image.width << " " << image.height << "\n255\n";
            for (uint32_t y = 0; y < image.height; ++y) {
                const size_t offset = y * step;
                if (offset + image.width > image.data.size()) {
                    RCLCPP_WARN(get_logger(), "Cannot save scan photo '%s': image buffer is truncated.", label.c_str());
                    return false;
                }
                out.write(reinterpret_cast<const char *>(image.data.data() + offset), image.width);
            }
        } else {
            const size_t channels = (image.encoding == "rgba8" || image.encoding == "bgra8") ? 4 : 3;
            const size_t step = image.step != 0 ? image.step : image.width * channels;
            const bool bgr_order = image.encoding == "bgr8" || image.encoding == "bgra8";
            out << "P6\n" << image.width << " " << image.height << "\n255\n";
            for (uint32_t y = 0; y < image.height; ++y) {
                const size_t row = y * step;
                if (row + image.width * channels > image.data.size()) {
                    RCLCPP_WARN(get_logger(), "Cannot save scan photo '%s': image buffer is truncated.", label.c_str());
                    return false;
                }
                for (uint32_t x = 0; x < image.width; ++x) {
                    const size_t pixel = row + x * channels;
                    const char rgb_pixel[3] = {
                        static_cast<char>(image.data[pixel + (bgr_order ? 2 : 0)]),
                        static_cast<char>(image.data[pixel + 1]),
                        static_cast<char>(image.data[pixel + (bgr_order ? 0 : 2)])
                    };
                    out.write(rgb_pixel, 3);
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Saved scan photo '%s'.", name.str().c_str());
        return true;
    }

    bool capture_scan_photo_after_new_frame(const std::string & label)
    {
        if (!has_received_vision_image_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for a vision image before saving scan photo '%s'.",
                label.c_str());
            return false;
        }

        if (pending_scan_photo_label_ != label) {
            pending_scan_photo_label_ = label;
            pending_scan_photo_image_counter_ = vision_image_counter_;
            RCLCPP_INFO(get_logger(), "Waiting for fresh vision frame for scan photo '%s'.", label.c_str());
            return false;
        }

        if (vision_image_counter_ == pending_scan_photo_image_counter_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for fresh vision frame for scan photo '%s'.",
                label.c_str());
            return false;
        }

        if (!capture_scan_photo(label)) {
            return false;
        }

        pending_scan_photo_label_.clear();
        return true;
    }

    void manual_joint_scan_control()
    {
        if (!has_joint_state_for_arm(active_arm_)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Manual scan waiting for %s arm joint state.",
                arm_name(active_arm_));
            return;
        }

        if (!manual_scan_active_) {
            manual_scan_active_ = true;
            selected_manual_joint_ = 6;
            enable_keyboard_mode();
            RCLCPP_INFO(
                get_logger(),
                "Manual joint scan active. Press 1-7 to jog that joint by %.3f rad, 'd' or '-' to reverse direction, 'e' to continue.",
                manual_joint_step_);
        }

        print_manual_joint_state();

        char key = 0;
        if (!read_keyboard_key(key)) {
            return;
        }

        if (key == 'e' || key == 'E') {
            manual_scan_active_ = false;
            restore_keyboard_mode();
            ++scan_phase_;
            RCLCPP_INFO(get_logger(), "Manual joint scan completed from keyboard.");
            return;
        }

        if (key == 'd' || key == 'D' || key == '-') {
            manual_joint_direction_ *= -1.0;
            RCLCPP_INFO(get_logger(), "Manual joint jog direction is now %+0.0f.", manual_joint_direction_);
            return;
        }

        if (key < '1' || key > '7') {
            RCLCPP_INFO(get_logger(), "Manual joint scan ignored key '%c'. Use 1-7, 'd', '-', or 'e'.", key);
            return;
        }

        const int joint_index = key - '1';
        selected_manual_joint_ = joint_index;
        auto target_joints = joint_state_for_arm(active_arm_);
        if (target_joints.size() < 7) {
            RCLCPP_WARN(
                get_logger(),
                "Manual joint scan received incomplete %s arm joint state: %zu positions.",
                arm_name(active_arm_),
                target_joints.size());
            return;
        }

        target_joints.resize(7);
        target_joints[joint_index] += manual_joint_direction_ * manual_joint_step_;
        RCLCPP_INFO(
            get_logger(),
            "Manual joint scan: jogging %s joint %d to %.4f rad.",
            arm_name(active_arm_),
            joint_index + 1,
            target_joints[joint_index]);
        start_joint_path(target_joints, FSMState::CUBE_SCANNING, 1, active_arm_);
    }

    void print_manual_joint_state()
    {
        const double now_sec = now().seconds();
        if (now_sec - last_manual_joint_print_sec_ < 1.0) {
            return;
        }
        last_manual_joint_print_sec_ = now_sec;

        const auto & joints = joint_state_for_arm(active_arm_);
        if (joints.size() < 7) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "%s joints: [1]=%.3f [2]=%.3f [3]=%.3f [4]=%.3f [5]=%.3f [6]=%.3f [7]=%.3f | direction=%+.0f | d=reverse | e=continue",
            arm_name(active_arm_),
            joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6],
            manual_joint_direction_);
    }

    void enable_keyboard_mode()
    {
        if (keyboard_raw_enabled_) {
            return;
        }

        if (keyboard_fd_ < 0) {
            keyboard_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);
            if (keyboard_fd_ < 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "Manual joint scan could not open /dev/tty. Run yumi_cube_node from an interactive terminal, then press keys in that terminal.");
                return;
            }
        }

        if (tcgetattr(keyboard_fd_, &saved_terminal_state_) != 0) {
            RCLCPP_WARN(get_logger(), "Manual joint scan could not read terminal settings; press keys followed by Enter if needed.");
            close(keyboard_fd_);
            keyboard_fd_ = -1;
            return;
        }

        termios raw = saved_terminal_state_;
        raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(keyboard_fd_, TCSANOW, &raw) == 0) {
            keyboard_raw_enabled_ = true;
            RCLCPP_INFO(get_logger(), "Manual joint scan keyboard input is active on /dev/tty.");
        }
    }

    void restore_keyboard_mode()
    {
        if (keyboard_raw_enabled_) {
            tcsetattr(keyboard_fd_, TCSANOW, &saved_terminal_state_);
            keyboard_raw_enabled_ = false;
        }

        if (keyboard_fd_ >= 0) {
            close(keyboard_fd_);
            keyboard_fd_ = -1;
        }
    }

    bool read_keyboard_key(char & key) const
    {
        if (keyboard_fd_ < 0) {
            return false;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(keyboard_fd_, &fds);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        const int ready = select(keyboard_fd_ + 1, &fds, nullptr, nullptr, &timeout);
        if (ready <= 0 || !FD_ISSET(keyboard_fd_, &fds)) {
            return false;
        }

        return read(keyboard_fd_, &key, 1) == 1;
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
        path_start_orientation_ = has_pose_for_arm(execution_arm_)
            ? pose_for_arm(execution_arm_).orientation
            : desired_pose_for_arm(execution_arm_).orientation;
        path_target_orientation_ = orientation;
        start_time_ = now();
        next_state_ = return_state;
        current_state_ = FSMState::EXECUTING_TRAJECTORY;
    }

    void start_joint_path(
        const std::vector<double> & joint_positions,
        FSMState return_state,
        int n_segments,
        Arm execution_arm)
    {
        if (!has_joint_state_for_arm(execution_arm)) {
            RCLCPP_WARN(get_logger(), "Cannot start %s joint path before receiving joint state.", arm_name(execution_arm));
            return;
        }
        if (joint_positions.size() < 7) {
            RCLCPP_ERROR(get_logger(), "Expected 7 target joint positions, got %zu.", joint_positions.size());
            return;
        }

        execution_arm_ = execution_arm;
        active_joint_start_ = joint_state_for_arm(execution_arm_);
        active_joint_target_ = joint_positions;
        active_duration_ = duration_sec_ * std::max(1, n_segments);
        start_time_ = now();
        waiting_for_joint_pose_refresh_ = false;
        next_state_ = return_state;
        current_state_ = FSMState::EXECUTING_JOINT_TRAJECTORY;
    }

    void start_joint_delta(
        double angle,
        FSMState return_state,
        int n_segments,
        Arm execution_arm)
    {
        execution_arm_ = execution_arm;
        active_duration_ = duration_sec_ * std::max(1, n_segments);
        start_time_ = now();
        next_state_ = return_state;

        std_msgs::msg::Float64 msg;
        msg.data = angle;
        joint7_delta_pub_for_arm(execution_arm_)->publish(msg);
        current_state_ = FSMState::EXECUTING_JOINT_DELTA;
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

    void publish_locate_cube_ready(bool ready)
    {
        std_msgs::msg::Bool msg;
        msg.data = ready;
        locate_cube_ready_pub_->publish(msg);
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

    static double quaternion_angle(
        const geometry_msgs::msg::Quaternion & a,
        const geometry_msgs::msg::Quaternion & b)
    {
        const double dot = std::clamp(quaternion_abs_dot(a, b), 0.0, 1.0);
        return 2.0 * std::acos(dot);
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

    bool has_joint_state_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? has_received_right_joints_ : has_received_left_joints_;
    }

    const std::vector<double> & joint_state_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? current_joints_right_ : current_joints_left_;
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

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_pub_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? desired_joint_state_right_pub_ : desired_joint_state_left_pub_;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint7_delta_pub_for_arm(Arm arm) const
    {
        return arm == Arm::RIGHT ? joint7_delta_right_pub_ : joint7_delta_left_pub_;
    }

    static std::vector<std::string> joint_names_for_arm(Arm arm)
    {
        std::vector<std::string> names;
        names.reserve(7);
        const char * prefix = arm == Arm::RIGHT ? "rightJoint" : "leftJoint";
        for (int i = 1; i <= 7; ++i) {
            names.push_back(std::string(prefix) + std::to_string(i));
        }
        return names;
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
        p.x += sensor_front_distance_ * sensor_z.x() + scan_workspace_x_offset_;
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
            const auto & actual_pose = pose_for_arm(execution_arm_);
            const double position_error = distance(actual_pose.position, desired_pose_.position);
            const double orientation_error = quaternion_angle(actual_pose.orientation, desired_pose_.orientation);

            if (position_error > position_settle_tolerance_ ||
                orientation_error > orientation_settle_tolerance_) {
                const double settle_time = t - active_duration_;
                if (settle_time < settle_timeout_sec_) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Waiting for %s arm to settle: position error %.4f m, orientation error %.4f rad",
                        arm_name(execution_arm_), position_error, orientation_error);
                    desired_pub_for_arm(execution_arm_)->publish(desired_pose_);
                    return;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Accepting best reachable %s arm pose after %.1f s: position error %.4f m, orientation error %.4f rad",
                    arm_name(execution_arm_), settle_timeout_sec_, position_error, orientation_error);
                desired_pose_ = actual_pose;
                store_desired_pose_for_arm(execution_arm_, desired_pose_);
            }

            RCLCPP_INFO(get_logger(), "Path completed.");
            current_state_ = next_state_;
        }
    }

    void execute_joint_trajectory()
    {
        if (active_joint_start_.size() < 7 || active_joint_target_.size() < 7) {
            RCLCPP_ERROR(get_logger(), "Joint trajectory is missing start or target positions.");
            current_state_ = next_state_;
            return;
        }

        if (selected_profile_ != 3 && selected_profile_ != 5 && selected_profile_ != 7) {
            RCLCPP_ERROR(get_logger(), "Invalid time law profile %d, using 5th degree", selected_profile_);
            selected_profile_ = 5;
        }

        const double t = (now() - start_time_).seconds();
        const double tau = std::min(t / active_duration_, 1.0);
        const double s = time_law(tau);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names_for_arm(execution_arm_);
        msg.position.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            msg.position[i] = active_joint_start_[i] + s * (active_joint_target_[i] - active_joint_start_[i]);
        }
        desired_joint_pub_for_arm(execution_arm_)->publish(msg);

        if (tau >= 1.0) {
            const auto & actual_joints = joint_state_for_arm(execution_arm_);
            double max_error = 0.0;
            if (actual_joints.size() >= 7) {
                for (size_t i = 0; i < 7; ++i) {
                    max_error = std::max(max_error, std::abs(actual_joints[i] - active_joint_target_[i]));
                }
            }

            if (max_error > joint_settle_tolerance_) {
                const double settle_time = t - active_duration_;
                if (settle_time < settle_timeout_sec_) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Waiting for %s arm joints to settle: max error %.4f rad",
                        arm_name(execution_arm_), max_error);
                    desired_joint_pub_for_arm(execution_arm_)->publish(msg);
                    return;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Accepting best reachable %s joint state after %.1f s: max error %.4f rad",
                    arm_name(execution_arm_), settle_timeout_sec_, max_error);
            }

            if (!waiting_for_joint_pose_refresh_) {
                waiting_for_joint_pose_refresh_ = true;
                joint_pose_refresh_until_ = now() + rclcpp::Duration::from_seconds(0.12);
                desired_joint_pub_for_arm(execution_arm_)->publish(msg);
                return;
            }

            if (now() < joint_pose_refresh_until_) {
                desired_joint_pub_for_arm(execution_arm_)->publish(msg);
                return;
            }

            if (has_pose_for_arm(execution_arm_)) {
                desired_pose_ = pose_for_arm(execution_arm_);
                store_desired_pose_for_arm(execution_arm_, desired_pose_);
                update_active_pose(execution_arm_, desired_pose_);
            }

            waiting_for_joint_pose_refresh_ = false;
            RCLCPP_INFO(get_logger(), "Joint path completed.");
            current_state_ = next_state_;
        }
    }

    void execute_joint_delta_wait()
    {
        const double t = (now() - start_time_).seconds();
        if (t < active_duration_) {
            return;
        }

        if (has_pose_for_arm(execution_arm_)) {
            desired_pose_ = pose_for_arm(execution_arm_);
            store_desired_pose_for_arm(execution_arm_, desired_pose_);
        }

        RCLCPP_INFO(get_logger(), "Joint 7 delta completed.");
        current_state_ = next_state_;
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
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr get_cube_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_sensor_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cube_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sensor_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sequence_to_do_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_right_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint7_delta_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint7_delta_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_left_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rubik_key_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr locate_cube_ready_pub_;
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
    geometry_msgs::msg::Pose cube_pose_pcl_;
    geometry_msgs::msg::Pose current_pose_left_;
    geometry_msgs::msg::Pose desired_pose_right_;
    geometry_msgs::msg::Pose desired_pose_left_;
    geometry_msgs::msg::Pose scan_start_pose_;
    geometry_msgs::msg::Pose cube_pose_;
    geometry_msgs::msg::Pose sensor_pose_;
    sensor_msgs::msg::Image latest_vision_image_;
    std::vector<double> current_joints_right_;
    std::vector<double> current_joints_left_;
    std::vector<double> scan_start_joints_;
    std::vector<double> get_cube_pose_start_joints_;
    std::vector<double> get_cube_pose_left_start_joints_;

    // Topic readiness flags prevent the FSM from planning before all needed scene data exists.
    bool has_received_pose_ = false;
    bool has_received_right_pose_ = false;
    bool has_received_left_pose_ = false;
    bool has_received_right_joints_ = false;
    bool has_received_left_joints_ = false;
    bool has_commanded_right_pose_ = false;
    bool has_commanded_left_pose_ = false;
    bool waiting_for_joint_pose_refresh_ = false;
    bool has_received_cube_ = false;
    bool has_received_cube_pcl_ = false;
    bool has_received_sensor_ = false;
    bool has_received_vision_image_ = false;
    bool manual_scan_active_ = false;
    bool keyboard_raw_enabled_ = false;
    bool get_cube_pose_request_sent_ = false;

    termios saved_terminal_state_{};
    int keyboard_fd_ = -1;
    rclcpp::Time start_time_;
    rclcpp::Time gripper_wait_until_;
    rclcpp::Time joint_pose_refresh_until_;
    double duration_sec_ = 2.0;
    double active_duration_ = 5.0;
    double cube_size_ = 0.05;
    double sensor_front_distance_ = 0.15;
    double scan_workspace_x_offset_ = 0.2;
    double position_settle_tolerance_ = 0.015;
    double orientation_settle_tolerance_ = 0.15;
    double joint_settle_tolerance_ = 0.02;
    double settle_timeout_sec_ = 2.0;
    int selected_profile_ = 3;
    int current_primitive_ = 0;
    int scan_photo_counter_ = 0;
    int selected_manual_joint_ = 6;
    uint64_t vision_image_counter_ = 0;
    uint64_t pending_scan_photo_image_counter_ = 0;
    std::string pending_scan_photo_label_;
    double manual_joint_step_ = 0.05;
    double manual_joint_direction_ = 1.0;
    double last_manual_joint_print_sec_ = 0.0;
    std::queue<int> sequence_queue_;
    // Phase counters make long actions resumable across timer callbacks.
    int get_cube_pose_phase_ = 0;
    int pick_phase_ = 0;
    int scan_phase_ = 0;
    int change_phase_ = 0;

    std::vector<geometry_msgs::msg::Point> active_path_;
    std::vector<double> active_joint_start_;
    std::vector<double> active_joint_target_;
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
