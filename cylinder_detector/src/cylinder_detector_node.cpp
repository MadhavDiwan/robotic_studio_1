#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class CylinderDetector : public rclcpp::Node {
public:
    CylinderDetector() : Node("cylinder_detector") {
        // Subscriber to laser scans
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);

        // TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float cylinder_radius = 0.15;  // Half of 30cm diameter
        float tolerance = 0.05;
        int min_points = 5;
        std::vector<geometry_msgs::msg::PointStamped> potential_cylinder_points;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float distance = msg->ranges[i];

            if (!std::isfinite(distance)) {
                continue;
            }

            if (std::abs(distance - cylinder_radius) < tolerance) {
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = distance * std::cos(angle);
                float y = distance * std::sin(angle);

                geometry_msgs::msg::PointStamped laser_point;
                laser_point.header.frame_id = msg->header.frame_id;
                laser_point.header.stamp = msg->header.stamp;
                laser_point.point.x = x;
                laser_point.point.y = y;
                laser_point.point.z = 0.0;

                potential_cylinder_points.push_back(laser_point);
            } else {
                if (potential_cylinder_points.size() >= min_points) {
                    processCylinderPoints(potential_cylinder_points);
                }
                potential_cylinder_points.clear();
            }
        }

        if (potential_cylinder_points.size() >= min_points) {
            processCylinderPoints(potential_cylinder_points);
        }
    }

    void processCylinderPoints(const std::vector<geometry_msgs::msg::PointStamped>& points) {
        // Average the points to get the center of the cylinder
        float sum_x = 0.0;
        float sum_y = 0.0;
        for (const auto& point : points) {
            sum_x += point.point.x;
            sum_y += point.point.y;
        }
        float avg_x = sum_x / points.size();
        float avg_y = sum_y / points.size();

        // Transform the averaged point to the map frame
        try {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform(
                "map",
                points[0].header.frame_id,
                tf2::TimePointZero);

            geometry_msgs::msg::PointStamped map_point;
            geometry_msgs::msg::PointStamped avg_point = points[0];
            avg_point.point.x = avg_x;
            avg_point.point.y = avg_y;
            tf2::doTransform(avg_point, map_point, transformStamped);

            // Publish the marker
            publishCylinderMarker(map_point.point.x, map_point.point.y);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    void publishCylinderMarker(float x, float y) {

        RCLCPP_INFO(this->get_logger(), "Publishing cylinder marker at map coordinates (%f, %f)", x, y);
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.30;  // 30cm diameter
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means the marker persists
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
