#include "rclcpp/rclcpp.hpp"
#include "protobuf_client/protobuf_client_node.hpp"

class MessageParser : public rclcpp::Node {
    public:
        MessageParser() : Node("message_parser") {
            m_subscription = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
                "/gateway_msg",
                10,
                std::bind(&MessageParser::topic_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_subscription;

        void topic_callback(const protobuf_client_interfaces::msg::Gateway & msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.gateway_double);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageParser>());
    rclcpp::shutdown();
    return 0;
}
