#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

class Controller : rclcpp::Node {

    public:
        Controller() : Node("/controller") {

            current_pose = this->create_subscription<geometry_msgs::msg::Pose>("/current_pose", 10, std::bind(&Controller::currentPoseCallback, this, std::placeholders::_1));

            required_pose = this->create_publisher<geometry_msgs::msg::Pose>("/left_arm_pose", 10);

            /*

            */
        }

        ~Controller() {

        }

        bool publicPose(bool arm/*true for left, false for right*/ ) {

            auto msg = std::make_unique<geometry_msgs::msg::Pose>();

            /*
            
            */
           
            required_pose->publish(std::move(msg));

        }
    
    private: 

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr required_pose;
        
        void currentPoseCallback(geometry_msgs::msg::Pose::ConstPtr msg) {

        }



};

int main(int argc, char *argv[]) {

    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<Controller>());

    
}