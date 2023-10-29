#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PCDTransformer: public rclcpp::Node {
    public:
        PCDTransformer() : Node("pcd_transformer") {

            // TODO: CHANGE THIS TO PARAMETERS
            //m_source_frame = this->declare_parameter<std::string>("source_frame");
            //m_target_frame = this->declare_parameter<std::string>("target_frame");
            m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

            m_pcd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/wamv/sensors/lidars/lidar_wamv_sensor/transformed_points", 10);
            m_pcd_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/wamv/sensors/lidars/lidar_wamv_sensor/points",
                10,
                std::bind(&PCDTransformer::pcd_callback, this, std::placeholders::_1)
            );

        }

    private:
        std::string m_source_frame = "wamv/wamv/base_link/lidar_wamv_sensor";
        std::string m_target_frame = "wamv/wamv/base_link/front_left_camera_sensor";
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcd_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcd_pub;

        void pcd_callback(const sensor_msgs::msg::PointCloud2 & msg) const {

            std::string source_frame_rel = m_source_frame.c_str();
            std::string target_frame_rel = m_target_frame.c_str();

            geometry_msgs::msg::TransformStamped t;
            try {
                t = m_tf_buffer->lookupTransform(
                    target_frame_rel, source_frame_rel,
                    tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    m_source_frame.c_str(), m_target_frame.c_str(), ex.what());
                return;
            }

            sensor_msgs::msg::PointCloud2 cloud_out;
            tf2::doTransform<sensor_msgs::msg::PointCloud2>(msg, cloud_out, t);
            m_pcd_pub->publish(cloud_out);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDTransformer>());
    rclcpp::shutdown();
    return 0;
}
