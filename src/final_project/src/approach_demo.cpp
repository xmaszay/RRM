#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "final_project/manipulator.hpp"

#include <Eigen/Geometry>

using namespace std::chrono_literals;

class ApproachDemo : public rclcpp::Node
{
public:
    ApproachDemo() : Node("approach_demo"), index_(0)
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

       Manipulator::JointVector q_t1 = {
    -0.335961, 0.189184, -0.083887, 1.863155, 0.351478, -1.881027
};

Manipulator::Pose pose_t1 = makePoseQuat(
    1.378681, -0.434293, 1.452074,
    0.000000, 0.706824, 0.000000, 0.707390
);

const double edge1_length = 0.685024;

Manipulator::Pose pose_t2 = pose_t1;
pose_t2.translation() += Eigen::Vector3d(0.0, edge1_length, 0.0);

        Manipulator::Pose pose_approach_t1 = manipulator_.offsetAlongToolX(pose_t1, -0.15);
        Manipulator::Pose pose_retract_t2  = manipulator_.offsetAlongToolX(pose_t2, -0.15);

        try {
            Manipulator::JointVector q_approach_t1 = q_t1;

            auto traj_approach = manipulator_.generateLIN(
                pose_approach_t1, pose_t1, q_approach_t1, 2.0, 0.01);

            auto traj_machining = manipulator_.generateLIN(
                pose_t1, pose_t2, q_t1, 4.0, 0.01);

            Manipulator::JointVector q_t2_cartesian = traj_machining.back().q;

            auto traj_retract = manipulator_.generateLIN(
                pose_t2, pose_retract_t2, q_t2_cartesian, 2.0, 0.01);

            appendTrajectory(traj_approach);
            appendTrajectory(traj_machining);
            appendTrajectory(traj_retract);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Approach demo failed: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            10ms, std::bind(&ApproachDemo::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "approach_demo started");
    }

private:
    Manipulator::Pose makePoseQuat(
        double x, double y, double z,
        double qx, double qy, double qz, double qw)
    {
        Eigen::Translation3d translation(Eigen::Vector3d(x, y, z));
        Eigen::Quaterniond q(qw, qx, qy, qz);

        Manipulator::Pose pose = Manipulator::Pose::Identity();
        pose = translation * q.normalized();

        return pose;
    }

    void appendTrajectory(const std::vector<Manipulator::Sample> &segment)
    {
        if (segment.empty()) {
            return;
        }

        if (trajectory_.empty()) {
            trajectory_ = segment;
            return;
        }

        for (size_t i = 1; i < segment.size(); ++i) {
            trajectory_.push_back(segment[i]);
        }
    }

    void timerCallback()
    {
        if (trajectory_.empty()) {
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        if (index_ < trajectory_.size()) {
            msg.position.assign(trajectory_[index_].q.begin(), trajectory_[index_].q.end());
            ++index_;
        } else {
            msg.position.assign(trajectory_.back().q.begin(), trajectory_.back().q.end());
        }

        joint_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    Manipulator manipulator_;
    std::vector<Manipulator::Sample> trajectory_;
    size_t index_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApproachDemo>());
    rclcpp::shutdown();
    return 0;
}