#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class ModelSpawner : public rclcpp::Node
{
public:
    ModelSpawner() : Node("model_spawner")
    {
        marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&ModelSpawner::publish_marker, this));

        RCLCPP_INFO(this->get_logger(), "Model spawner node started.");
    }

private:
    void publish_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();

        marker.ns = "workpiece";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.mesh_resource = "package://final_project/meshes/tie.stl";
        marker.mesh_use_embedded_materials = false;

        // poloha objektu pred robotom
        marker.pose.position.x = 2.0;
        marker.pose.position.y = 0.5;
        marker.pose.position.z = 0.0;

        // rotacia
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        marker.color.r = 0.7f;
        marker.color.g = 0.7f;
        marker.color.b = 0.7f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.0);

        marker_publisher_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelSpawner>());
    rclcpp::shutdown();
    return 0;
}