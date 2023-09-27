#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "asv_interfaces/msg/asv_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class StateReporter : public rclcpp::Node {
    public:
        StateReporter() : Node("state_reporter") {
            this->declare_parameter("lat_origin", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("lon_origin", rclcpp::PARAMETER_DOUBLE);

            m_lat_origin = this->get_parameter("lat_origin").as_double();
            m_lon_origin = this->get_parameter("lon_origin").as_double();

            m_timer = this->create_wall_timer(
                //16.67ms,
                100ms,
                std::bind(&StateReporter::timer_callback, this)
            );

            m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/wamv/sensors/imu/imu/data",
                10,
                std::bind(&StateReporter::imu_callback, this, std::placeholders::_1)
            );

            m_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/wamv/sensors/gps/gps/fix",
                10,
                std::bind(&StateReporter::gps_callback, this, std::placeholders::_1)
            );

            m_state_pub = this->create_publisher<asv_interfaces::msg::ASVState>("/allseaing_main/state", 10);

        }
    
    private:
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_sub;
        rclcpp::Publisher<asv_interfaces::msg::ASVState>::SharedPtr m_state_pub;

        double m_lat_origin, m_lon_origin;
        asv_interfaces::msg::ASVState m_state = asv_interfaces::msg::ASVState();

        void imu_callback(const sensor_msgs::msg::Imu & msg) {
            tf2::Quaternion q;
            q.setW(msg.orientation.w);
            q.setX(msg.orientation.x);
            q.setY(msg.orientation.y);
            q.setZ(msg.orientation.z);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            //std::cout << "Roll: " << r << ", Pitch: " << p << ", Yaw: " << y << std::endl;
            m_state.nav_heading = y;
        }

        void gps_callback(const sensor_msgs::msg::NavSatFix & msg) {
            m_state.nav_long = msg.longitude;
            m_state.nav_lat = msg.latitude;
            m_state.nav_x = msg.longitude - m_lon_origin;
            m_state.nav_y = msg.latitude - m_lat_origin;
        }

        void timer_callback() {
            m_state.header.stamp = this->get_clock()->now();
            m_state_pub->publish(m_state);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateReporter>());
    rclcpp::shutdown();
    return 0;
}