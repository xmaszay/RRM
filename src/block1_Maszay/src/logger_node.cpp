#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "maszay_interface/srv/save_point.hpp"

#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class JointLogger : public rclcpp::Node
{
public:
    JointLogger();

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void save_point_callback(
        const std::shared_ptr<maszay_interface::srv::SavePoint::Request> request,
        std::shared_ptr<maszay_interface::srv::SavePoint::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request received: velocity = %f", request->velocity);

        if (!has_joint_state_)
        {
            response->result = false;
            response->message = "No joint state received yet";
            return;
        }

        if (request->velocity <= 0.0)
        {
            response->result = false;
            response->message = "Velocity must be positive";
            return;
        }

        bool ok = save_point_to_file(request->velocity);

        if (ok)
        {
            response->result = true;
            response->message = "Point saved to trajectory.txt";
        }
        else
        {
            response->result = false;
            response->message = "Failed to save point to file";
        }
    }

private:
    bool save_point_to_file(double velocity)
    {
        std::ofstream file("trajectory.txt", std::ios::app);

        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory.txt");
            return false;
        }

        file << point_id_ << " "
             << current_[0] << " "
             << current_[1] << " "
             << current_[2] << " "
             << velocity << std::endl;

        file.close();

        RCLCPP_INFO(this->get_logger(),
                    "Saved point %d: %f %f %f vel=%f",
                    point_id_, current_[0], current_[1], current_[2], velocity);

        point_id_++;
        return true;
    }

    Eigen::Matrix4d rotZ(double angle)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T(0,0) = std::cos(angle);
        T(0,1) = -std::sin(angle);
        T(1,0) = std::sin(angle);
        T(1,1) = std::cos(angle);

        return T;
    }

    Eigen::Matrix4d rotY(double angle)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T(0,0) = std::cos(angle);
        T(0,2) = std::sin(angle);
        T(2,0) = -std::sin(angle);
        T(2,2) = std::cos(angle);

        return T;
    }

    Eigen::Matrix4d transZ(double d)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(2,3) = d;
        return T;
    }

    void publish_tf(const Eigen::Matrix4d& T)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "tool0_calculated";

        transform.transform.translation.x = T(0, 3);
        transform.transform.translation.y = T(1, 3);
        transform.transform.translation.z = T(2, 3);

        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(transform);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Service<maszay_interface::srv::SavePoint>::SharedPtr save_service_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double current_[3];
    bool has_joint_state_;
    int point_id_;
};

JointLogger::JointLogger()
    : Node("joint_logger"),
      tf_broadcaster_(this),
      has_joint_state_(false),
      point_id_(1)
{
    current_[0] = 0.0;
    current_[1] = 0.0;
    current_[2] = 0.0;

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&JointLogger::joint_states_callback, this, std::placeholders::_1));

    save_service_ = this->create_service<maszay_interface::srv::SavePoint>(
        "save_point",
        std::bind(&JointLogger::save_point_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "JointLogger initialized");
    RCLCPP_INFO(this->get_logger(), "Service /save_point created");
}

void JointLogger::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough joint positions received");
        return;
    }

    current_[0] = msg->position[0];
    current_[1] = msg->position[1];
    current_[2] = msg->position[2];

    has_joint_state_ = true;

    double q1 = current_[0];
    double q2 = current_[1];
    double q3 = current_[2];

    // Podla URDF:
    // joint_1: rotacia okolo z
    // joint_2: rotacia okolo y
    // joint_3: rotacia okolo y, posunuta o 0.203 po lokalnej z
    // koniec link_3 je dalsich 0.203 po lokalnej z
    const double l2 = 0.203;
    const double l3 = 0.203;

    Eigen::Matrix4d T01 = rotZ(q1);
    Eigen::Matrix4d T12 = rotY(q2);
    Eigen::Matrix4d T23 = transZ(l2) * rotY(q3);
    Eigen::Matrix4d T3E = transZ(l3);

    Eigen::Matrix4d T0E = T01 * T12 * T23 * T3E;

    RCLCPP_INFO(this->get_logger(),
                "tool0_calculated: x=%f y=%f z=%f",
                T0E(0,3), T0E(1,3), T0E(2,3));

    publish_tf(T0E);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto logger = std::make_shared<JointLogger>();

    rclcpp::spin(logger);

    rclcpp::shutdown();
    return 0;
}