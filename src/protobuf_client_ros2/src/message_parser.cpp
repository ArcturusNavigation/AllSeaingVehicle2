#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "protobuf_client/protobuf_client_node.hpp"

class MessageParser : public rclcpp::Node {
    public:
        MessageParser() : Node("message_parser") {
            m_rudder_publisher = this->create_publisher<std_msgs::msg::Float64>("/moos/desired_rudder", 10);
            m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>("/moos/desired_thrust", 10);
            m_coord_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("/moos/local_utm", 10);

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
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_coord_publisher;

        double m_nav_x;

        void topic_callback(const protobuf_client_interfaces::msg::Gateway & msg) {
            auto float_msg = std_msgs::msg::Float64();
            float_msg.data = msg.gateway_double;

            // Publish desired rudder and thrust
            if (msg.gateway_key == "DESIRED_RUDDER") m_rudder_publisher->publish(float_msg);
            if (msg.gateway_key == "DESIRED_THRUST") m_thrust_publisher->publish(float_msg);

            // Publisher local utm coordinates (when NAV_Y is received)
            if (msg.gateway_key == "NAV_X") m_nav_x = msg.gateway_double;
            if (msg.gateway_key == "NAV_Y" && m_nav_x) {
                auto point_msg = geometry_msgs::msg::PointStamped();
                point_msg.header.stamp = msg.gateway_time;
                point_msg.point.x = m_nav_x;
                point_msg.point.y = msg.gateway_double;
                point_msg.point.z = 0;
                m_coord_publisher->publish(point_msg);
            }
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageParser>());
    rclcpp::shutdown();
    return 0;
}
