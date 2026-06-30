#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SequenceParser : public rclcpp::Node
{
public:
    SequenceParser() : Node("sequence_parser_node")
    {
        sequence_sub_ = create_subscription<std_msgs::msg::String>(
            "/sequence", 10,
            std::bind(&SequenceParser::sequence_callback, this, std::placeholders::_1));
        sequence_pub_ = create_publisher<std_msgs::msg::Int32>("/sequnce_to_do", 10);
        publish_timer_ = create_wall_timer(100ms, std::bind(&SequenceParser::publish_next, this));

        RCLCPP_INFO(get_logger(), "Sequence parser ready. Listening on /sequence.");
    }

private:
    void sequence_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string & sequence = msg->data;

        for (size_t i = 0; i < sequence.size(); ++i) {
            std::string token(1, static_cast<char>(std::tolower(sequence[i])));
            if (i + 1 < sequence.size() && sequence[i + 1] == '\'') {
                token += "'";
                ++i;
            }

            const auto primitive = primitive_map_.find(token);
            if (primitive == primitive_map_.end()) {
                RCLCPP_WARN(get_logger(), "Ignoring invalid sequence token '%s'", token.c_str());
                continue;
            }

            pending_primitives_.push(primitive->second);
            RCLCPP_INFO(get_logger(), "Queued primitive %d", primitive->second);
        }
    }

    void publish_next()
    {
        if (pending_primitives_.empty()) {
            return;
        }

        std_msgs::msg::Int32 out;
        out.data = pending_primitives_.front();
        pending_primitives_.pop();
        sequence_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Published primitive %d", out.data);
    }

    const std::unordered_map<std::string, int> primitive_map_ = {
        {"a", 0},
        {"a'", 1},
        {"b", 2},
        {"b'", 3},
        {"c", 4},
        {"c'", 5},
        {"d", 6},
        {"d'", 7},
        {"e", 8},
        {"e'", 9},
        {"f", 10},
        {"f'", 11},
        {"g", 12},
        {"g'", 13},
    };

    std::queue<int> pending_primitives_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sequence_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sequence_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceParser>());
    rclcpp::shutdown();
    return 0;
}
