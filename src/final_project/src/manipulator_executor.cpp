#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"

#include "final_project/manipulator.hpp"
#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

#include <Eigen/Geometry>

using namespace std::chrono_literals;

class ManipulatorExecutor : public rclcpp::Node
{
public:
    enum State : int8_t
    {
        IDLE = 0,
        PTP = 1,
        APPROACHING = 2,
        MACHINING = 3,
        RETRACTING = 4,
        TRANSITION = 5
    };

    ManipulatorExecutor() : Node("manipulator_executor"), index_(0)
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::Int8>("manipulator/state", 10);

        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        try {
            buildFullSequence();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build full sequence: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            10ms, std::bind(&ManipulatorExecutor::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "manipulator_executor started");
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

    Manipulator::JointVector selectNearestSolution(
        const ikfast_abb::Solutions &solutions,
        const Manipulator::JointVector &reference)
    {
        if (solutions.empty()) {
            throw std::runtime_error("IK returned no solutions.");
        }

        double best_dist = std::numeric_limits<double>::max();
        Manipulator::JointVector best = solutions.front();

        for (const auto &sol : solutions) {
            double dist = 0.0;
            for (size_t i = 0; i < 6; ++i) {
                double d = sol[i] - reference[i];
                dist += d * d;
            }

            if (dist < best_dist) {
                best_dist = dist;
                best = sol;
            }
        }

        return best;
    }

    Manipulator::JointVector solveNearestIK(
        const Manipulator::Pose &pose,
        const Manipulator::JointVector &reference)
    {
        auto solutions = ikfast_abb::computeIK(pose);
        return selectNearestSolution(solutions, reference);
    }

    void appendSegment(const std::vector<Manipulator::Sample> &segment, State state)
    {
        if (segment.empty()) {
            return;
        }

        if (trajectory_.empty()) {
            trajectory_ = segment;
            states_.assign(segment.size(), static_cast<int8_t>(state));
            return;
        }

        for (size_t i = 1; i < segment.size(); ++i) {
            trajectory_.push_back(segment[i]);
            states_.push_back(static_cast<int8_t>(state));
        }
    }

    void buildFullSequence()
    {
        const double dt = 0.01;

        Manipulator::JointVector q_home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        Manipulator::JointVector q_t1 = {
            -0.346254, 0.197993, -0.105621, -1.563038, -0.344984, 1.532185
        };

        Manipulator::JointVector q_t2 = {
            0.212505, 0.141679, -0.044757, 1.521074, -0.212008, -1.506295
        };

        Manipulator::JointVector q_tvia = {
            0.135446, -0.144309, 0.451832, 0.460273, -0.308750, -0.437340
        };

        Manipulator::JointVector q_t3 = {
            0.023699, 0.155486, 0.456767, -3.098603, 0.583753, 3.106401
        };

        Manipulator::JointVector q_t4 = {
            -0.179721, 0.175691, 0.433183, -0.320224, -0.603631, 0.261282
        };

        Manipulator::Pose pose_t1 = makePoseQuat(
            1.380018, -0.449362, 1.451754,
            -0.000000, 0.736001, -0.000000, 0.676980
        );

        Manipulator::Pose pose_t2 = makePoseQuat(
            1.380366, 0.268806, 1.455895,
            -0.001664, 0.736000, -0.001530, 0.676978
        );

        Manipulator::Pose pose_t3 = makePoseQuat(
            1.335176, 0.028450, 0.971728,
            0.000000, 0.716977, 0.000000, 0.697096
        );

        Manipulator::Pose pose_t4 = makePoseQuat(
            1.335176, -0.218060, 0.971727,
            0.000000, 0.716978, -0.000000, 0.697096
        );

        Manipulator::Pose pose_approach_t1 = manipulator_.offsetAlongToolX(pose_t1, -0.15);
        Manipulator::Pose pose_retract_t2  = manipulator_.offsetAlongToolX(pose_t2, -0.15);
        Manipulator::Pose pose_approach_t3 = manipulator_.offsetAlongToolX(pose_t3, -0.15);
        Manipulator::Pose pose_retract_t4  = manipulator_.offsetAlongToolX(pose_t4, -0.15);

        Manipulator::JointVector q_approach_t1 = solveNearestIK(pose_approach_t1, q_t1);
        Manipulator::JointVector q_retract_t2  = solveNearestIK(pose_retract_t2, q_t2);
        Manipulator::JointVector q_approach_t3 = solveNearestIK(pose_approach_t3, q_t3);
        Manipulator::JointVector q_retract_t4  = solveNearestIK(pose_retract_t4, q_t4);

        auto seg_home_to_approach_t1 = manipulator_.generatePTP(q_home, q_approach_t1, 4.0, dt);
        auto seg_approach_t1         = manipulator_.generateLIN(pose_approach_t1, pose_t1, q_approach_t1, 2.0, dt);
        auto seg_t1_t2               = manipulator_.generateLIN(pose_t1, pose_t2, q_t1, 4.0, dt);
        auto seg_retract_t2          = manipulator_.generateLIN(pose_t2, pose_retract_t2, q_t2, 2.0, dt);
        auto seg_transition          = manipulator_.generatePTPVia(q_retract_t2, q_tvia, q_approach_t3, 2.5, 2.5, dt);
        auto seg_approach_t3         = manipulator_.generateLIN(pose_approach_t3, pose_t3, q_approach_t3, 2.0, dt);
        auto seg_t3_t4               = manipulator_.generateLIN(pose_t3, pose_t4, q_t3, 4.0, dt);
        auto seg_retract_t4          = manipulator_.generateLIN(pose_t4, pose_retract_t4, q_t4, 2.0, dt);
        auto seg_home_return         = manipulator_.generatePTP(q_retract_t4, q_home, 4.0, dt);

        appendSegment(seg_home_to_approach_t1, PTP);
        appendSegment(seg_approach_t1, APPROACHING);
        appendSegment(seg_t1_t2, MACHINING);
        appendSegment(seg_retract_t2, RETRACTING);
        appendSegment(seg_transition, TRANSITION);
        appendSegment(seg_approach_t3, APPROACHING);
        appendSegment(seg_t3_t4, MACHINING);
        appendSegment(seg_retract_t4, RETRACTING);
        appendSegment(seg_home_return, PTP);

        RCLCPP_INFO(this->get_logger(), "Full sequence prepared, samples: %zu", trajectory_.size());
    }

    void timerCallback()
    {
        if (trajectory_.empty()) {
            publishState(IDLE);
            return;
        }

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->now();
        joint_msg.name = joint_names_;

        int8_t state_to_publish = IDLE;

        if (index_ < trajectory_.size()) {
            joint_msg.position.assign(
                trajectory_[index_].q.begin(),
                trajectory_[index_].q.end());
            state_to_publish = states_[index_];
            ++index_;
        } else {
            joint_msg.position.assign(
                trajectory_.back().q.begin(),
                trajectory_.back().q.end());
            state_to_publish = IDLE;
        }

        joint_pub_->publish(joint_msg);
        publishState(static_cast<State>(state_to_publish));
    }

    void publishState(State state)
    {
        std_msgs::msg::Int8 msg;
        msg.data = static_cast<int8_t>(state);
        state_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    Manipulator manipulator_;
    std::vector<Manipulator::Sample> trajectory_;
    std::vector<int8_t> states_;
    size_t index_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorExecutor>());
    rclcpp::shutdown();
    return 0;
}