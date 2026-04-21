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
            -0.346254, 0.197993, -0.105621, -1.563038, -0.344984, 1.532185
        };

        Manipulator::JointVector q_t2 = {
            0.212505, 0.141679, -0.044757, 1.521074, -0.212008, -1.506295
        };

        Manipulator::Pose pose_t1 = makePoseQuat(
            1.380018, -0.449362, 1.451754,
            -0.000000, 0.736001, -0.000000, 0.676980
        );

        Manipulator::Pose pose_t2 = makePoseQuat(
            1.380366, 0.268806, 1.455895,
            -0.001664, 0.736000, -0.001530, 0.676978
        );

        Manipulator::Pose pose_approach_t1 = manipulator_.offsetAlongToolX(pose_t1, -0.15);
        Manipulator::Pose pose_retract_t2  = manipulator_.offsetAlongToolX(pose_t2, -0.15);

        try {
            auto traj_approach = manipulator_.generateLIN(pose_approach_t1, pose_t1, q_t1, 2.0, 0.01);
            auto traj_machining = manipulator_.generateLIN(pose_t1, pose_t2, q_t1, 4.0, 0.01);
            auto traj_retract = manipulator_.generateLIN(pose_t2, pose_retract_t2, q_t2, 2.0, 0.01);

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
    Manipulator::Pose makePoseQuat(double x, double y, double z,
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