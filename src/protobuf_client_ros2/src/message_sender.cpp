#include "rclcpp/rclcpp.hpp"
#include "protobuf_client/protobuf_client_node.hpp"
#include "asv_interfaces/msg/asv_state.hpp"

using namespace std::chrono_literals;

class MessageSender : public rclcpp::Node {
    public:
        MessageSender() : Node("message_parser") {
            m_state_sub = this->create_subscription<asv_interfaces::msg::ASVState>(
                "/allseaing_main/state",
                10,
                std::bind(&MessageSender::state_callback, this, std::placeholders::_1)
            );

            m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
        }

    private:
        rclcpp::Subscription<asv_interfaces::msg::ASVState>::SharedPtr m_state_sub;
        rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_pub;

        void state_callback(const asv_interfaces::msg::ASVState & msg) {
            auto nav_lat_msg = protobuf_client_interfaces::msg::Gateway();
            auto nav_long_msg = protobuf_client_interfaces::msg::Gateway();
            auto nav_heading_msg = protobuf_client_interfaces::msg::Gateway();
            auto nav_speed_msg = protobuf_client_interfaces::msg::Gateway();

            nav_lat_msg.gateway_key = "NAV_LAT";
            nav_lat_msg.gateway_double = msg.nav_lat;
            nav_long_msg.gateway_key = "NAV_LONG";
            nav_long_msg.gateway_double = msg.nav_long;
            nav_heading_msg.gateway_key = "NAV_HEADING";
            nav_heading_msg.gateway_double = msg.nav_heading;
            nav_speed_msg.gateway_key = "NAV_SPEED";
            nav_speed_msg.gateway_double = msg.nav_speed;

            m_gateway_pub->publish(nav_lat_msg);
            m_gateway_pub->publish(nav_long_msg);
            m_gateway_pub->publish(nav_heading_msg);
            m_gateway_pub->publish(nav_speed_msg);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageSender>());
    rclcpp::shutdown();
    return 0;
}
