#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class JointLogger : public rclcpp::Node
{
public:
    JointLogger();

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    Eigen::Matrix4d dhMatrix(double theta, double d, double a, double alpha) const //vytvara dh transformacnu maticu 
    {
        const double ct = std::cos(theta); // prepocitavanie pre lepsiu prehladnost 
        const double st = std::sin(theta);
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        Eigen::Matrix4d A; // DH matica
        A << ct, -st * ca,  st * sa, a * ct, 
             st,  ct * ca, -ct * sa, a * st,
             0.0,      sa,      ca,      d,
             0.0,     0.0,     0.0,    1.0; // perspektiva , mierka

        return A;
    }

    Eigen::Matrix4d computeFK(double q1, double q2, double q3) const // DH parametre
    {
                                        // tu pocitame 3 matice 
        const Eigen::Matrix4d A1 = dhMatrix(M_PI + q1,      0.0, 0.0, M_PI / 2.0);
        const Eigen::Matrix4d A2 = dhMatrix(M_PI / 2.0 + q2, 0.0, l2_, 0.0);
        const Eigen::Matrix4d A3 = dhMatrix(q3,              0.0, l3_, 0.0);

        return A1 * A2 * A3;
    }

    void broadcastTransform(const Eigen::Matrix4d& T)
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->get_clock()->now(); // cas aby sme mali konkretnu tranformaciu v case
        ts.header.frame_id = "base_link";
        ts.child_frame_id = "tool0_calculated";

        ts.transform.translation.x = T(0, 3); // translacia
        ts.transform.translation.y = T(1, 3);
        ts.transform.translation.z = T(2, 3);

        Eigen::Quaterniond q(T.block<3, 3>(0, 0)); // rotacia 
        q.normalize();

        ts.transform.rotation.x = q.x(); // prevod do ROS  
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(ts);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double l2_;
    double l3_;
};

JointLogger::JointLogger()
    : Node("joint_logger"),
      tf_broadcaster_(this),
      l2_(0.203),
      l3_(0.203)
{
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&JointLogger::joint_states_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JointLogger initialized");
}

void JointLogger::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough joint positions received");
        return;
    }

    const double q1 = msg->position[0]; // nacitanie uhlov 
    const double q2 = msg->position[1];
    const double q3 = msg->position[2];

    const Eigen::Matrix4d T = computeFK(q1, q2, q3);
    broadcastTransform(T);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto logger = std::make_shared<JointLogger>();
    rclcpp::spin(logger);

    rclcpp::shutdown();
    return 0;
}