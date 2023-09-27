#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protobuf_client/protobuf_client_node.hpp"

class MessageParser : public rclcpp::Node {
    public:
        MessageParser() : Node("message_parser") {
            m_rudder_publisher = this->create_publisher<std_msgs::msg::Float64>("/moos/desired_rudder", 10);
            m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>("/moos/desired_thrust", 10);

            m_subscription = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
                "/gateway_msg",
                10,
                std::bind(&MessageParser::topic_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_subscription;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_rudder_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrust_publisher;

        void topic_callback(const protobuf_client_interfaces::msg::Gateway & msg) const {
            auto float_msg = std_msgs::msg::Float64();
            float_msg.data = msg.gateway_double;
            auto string_msg = std_msgs::msg::String();
            string_msg.data = msg.gateway_string;
            if (msg.gateway_key == "DESIRED_RUDDER") m_rudder_publisher->publish(float_msg);
            if (msg.gateway_key == "DESIRED_THRUST") m_thrust_publisher->publish(float_msg);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageParser>());
    rclcpp::shutdown();
    return 0;
}
