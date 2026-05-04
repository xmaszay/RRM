#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

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

    ManipulatorExecutor()
        : Node("manipulator_executor"),
          index_(0),
          active_(false),
          current_path_marker_id_(0),
          last_path_state_(-1),
          has_active_path_marker_(false)
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        state_pub_ = this->create_publisher<std_msgs::msg::Int8>(
            "manipulator/state", 10);

        path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "manipulator/path",
            rclcpp::QoS(1).transient_local());

        execute_service_ = this->create_service<std_srvs::srv::Trigger>(
            "execute_machining",
            std::bind(
                &ManipulatorExecutor::executeMachiningCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        joint_names_ = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        };

        q_home_ = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        try {
            buildFullSequence();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build full sequence: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&ManipulatorExecutor::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "manipulator_executor started");
        RCLCPP_INFO(this->get_logger(), "Waiting for service call: /execute_machining");
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

        Manipulator::JointVector q_t1 = {
            -0.335961, 0.189184, -0.083887,
             1.863155, 0.351478, -1.881027
        };

        Manipulator::JointVector q_tvia = {
             0.256884, -0.168304, 0.513256,
             0.660456, -0.427003, -0.615465
        };

        Manipulator::JointVector q_t3 = {
             0.021435, 0.153536, 0.465015,
             0.036955, -0.618874, -0.030106
        };

        Manipulator::Pose pose_t1 = makePoseQuat(
            1.378681, -0.434293, 1.452074,
            0.000000, 0.706824, 0.000000, 0.707390
        );

        Manipulator::Pose pose_t3 = makePoseQuat(
            1.330971, 0.025640, 0.970327,
            0.000000, 0.706824, 0.000000, 0.707390
        );

        // Hrana 1: T2 vznikne čistým karteziánskym posunom z T1 v osi Y.
        const double edge1_length = 0.685024;
        Manipulator::Pose pose_t2 = pose_t1;
        pose_t2.translation() += Eigen::Vector3d(0.0, edge1_length, 0.0);

        // Hrana 2: T4 vznikne čistým karteziánskym posunom z T3 v osi Y.
        const double edge2_length = -0.231941;
        Manipulator::Pose pose_t4 = pose_t3;
        pose_t4.translation() += Eigen::Vector3d(0.0, edge2_length, 0.0);

        // Poznámka:
        // offsetAlongToolX môžeš mať interne upravený na inú lokálnu os nástroja,
        // napr. local_y = pose.linear().col(1), ak chceš nájazd z inej strany.
        Manipulator::Pose pose_approach_t1 = manipulator_.offsetAlongToolX(pose_t1, -0.15);
        Manipulator::Pose pose_retract_t2  = manipulator_.offsetAlongToolX(pose_t2, -0.15);
        Manipulator::Pose pose_approach_t3 = manipulator_.offsetAlongToolX(pose_t3, -0.15);
        Manipulator::Pose pose_retract_t4  = manipulator_.offsetAlongToolX(pose_t4, -0.15);

        Manipulator::JointVector q_approach_t1 = solveNearestIK(pose_approach_t1, q_t1);
        Manipulator::JointVector q_approach_t3 = solveNearestIK(pose_approach_t3, q_t3);

        auto seg_home_to_approach_t1 =
            manipulator_.generatePTP(q_home_, q_approach_t1, 4.0, dt);

        auto seg_approach_t1 =
            manipulator_.generateLIN(pose_approach_t1, pose_t1, q_approach_t1, 2.0, dt);

        auto seg_t1_t2 =
            manipulator_.generateLIN(pose_t1, pose_t2, q_t1, 4.0, dt);

        Manipulator::JointVector q_t2_cartesian = seg_t1_t2.back().q;

        auto seg_retract_t2 =
            manipulator_.generateLIN(pose_t2, pose_retract_t2, q_t2_cartesian, 2.0, dt);

        Manipulator::JointVector q_retract_t2 = seg_retract_t2.back().q;

        auto seg_transition =
            manipulator_.generatePTPVia(q_retract_t2, q_tvia, q_approach_t3, 2.5, 2.5, dt);

        auto seg_approach_t3 =
            manipulator_.generateLIN(pose_approach_t3, pose_t3, q_approach_t3, 2.0, dt);

        auto seg_t3_t4 =
            manipulator_.generateLIN(pose_t3, pose_t4, q_t3, 4.0, dt);

        Manipulator::JointVector q_t4_cartesian = seg_t3_t4.back().q;

        auto seg_retract_t4 =
            manipulator_.generateLIN(pose_t4, pose_retract_t4, q_t4_cartesian, 2.0, dt);

        Manipulator::JointVector q_retract_t4 = seg_retract_t4.back().q;

        auto seg_home_return =
            manipulator_.generatePTP(q_retract_t4, q_home_, 4.0, dt);

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

    void executeMachiningCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (trajectory_.empty()) {
            response->success = false;
            response->message = "Trajectory is empty. Cannot execute machining.";
            return;
        }

        if (active_) {
            response->success = false;
            response->message = "Machining sequence is already running.";
            return;
        }

        index_ = 0;
        active_ = true;

        clearPathMarkers();

        response->success = true;
        response->message = "Machining sequence started.";

        RCLCPP_INFO(this->get_logger(), "execute_machining service called. Sequence started.");
    }

    void timerCallback()
{
    if (trajectory_.empty()) {
        publishState(IDLE);
        publishHomeJointState();
        return;
    }

    if (!active_) {
        publishState(IDLE);
        publishHomeJointState();
        return;
    }

    if (index_ < trajectory_.size()) {
        const auto &sample = trajectory_[index_];
        int8_t current_state = states_[index_];

        publishJointState(sample.q, sample.dq, sample.ddq);
        publishState(static_cast<State>(current_state));
        appendCurrentPathPoint(sample.q, current_state);

        ++index_;
    } else {
        active_ = false;

        const auto &last_sample = trajectory_.back();
        publishJointState(last_sample.q, last_sample.dq, last_sample.ddq);
        publishState(IDLE);

        RCLCPP_INFO(this->get_logger(), "Machining sequence finished.");
    }
}

    void publishJointState(
    const Manipulator::JointVector &q,
    const Manipulator::JointVector &dq,
    const Manipulator::JointVector &ddq)
{
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    joint_msg.name = joint_names_;

    joint_msg.position.assign(q.begin(), q.end());
    joint_msg.velocity.assign(dq.begin(), dq.end());

    // Pre PlotJuggler použijeme effort pole ako zrýchlenie ddq.
    joint_msg.effort.assign(ddq.begin(), ddq.end());

    joint_pub_->publish(joint_msg);
}

    void publishHomeJointState()
{
    Manipulator::JointVector zero{};
    zero.fill(0.0);

    publishJointState(q_home_, zero, zero);
}

    void publishState(State state)
    {
        std_msgs::msg::Int8 msg;
        msg.data = static_cast<int8_t>(state);
        state_pub_->publish(msg);
    }

    std_msgs::msg::ColorRGBA colorForState(int8_t state)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0f;

    if (state == PTP || state == TRANSITION) {
        // Modrá = PTP pohyby vrátane PTP cez VIA
        color.r = 0.0f;
        color.g = 0.2f;
        color.b = 1.0f;
    } else if (state == APPROACHING || state == RETRACTING) {
        // Červená = approach / retract
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 0.0f;
    } else if (state == MACHINING) {
        // Zelená = machining
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 0.0f;
    } else {
        // Fallback, normálne by sa nemal použiť
        color.r = 0.5f;
        color.g = 0.5f;
        color.b = 0.5f;
    }

    return color;
}

    geometry_msgs::msg::Point tcpPointFromJoints(const Manipulator::JointVector &q)
    {
        Eigen::Affine3d pose = ikfast_abb::computeFk(q);
        Eigen::Vector3d p = pose.translation();

        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();

        return point;
    }

    void clearPathMarkers()
    {
        visualization_msgs::msg::MarkerArray clear_array;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_array.markers.push_back(clear_marker);

        path_pub_->publish(clear_array);

        path_markers_.markers.clear();
        current_path_marker_id_ = 0;
        last_path_state_ = -1;
        has_active_path_marker_ = false;
    }

    void appendCurrentPathPoint(const Manipulator::JointVector &q, int8_t state)
    {
        geometry_msgs::msg::Point point = tcpPointFromJoints(q);

        bool need_new_marker =
            !has_active_path_marker_ ||
            state != last_path_state_;

        if (need_new_marker) {
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();

            marker.ns = "tool_path";
            marker.id = current_path_marker_id_++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.015;
            marker.color = colorForState(state);
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);

            marker.points.push_back(point);

            path_markers_.markers.push_back(marker);

            last_path_state_ = state;
            has_active_path_marker_ = true;
        } else {
            path_markers_.markers.back().header.stamp = this->now();
            path_markers_.markers.back().points.push_back(point);
        }

        path_pub_->publish(path_markers_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;

    Manipulator manipulator_;

    std::vector<Manipulator::Sample> trajectory_;
    std::vector<int8_t> states_;

    visualization_msgs::msg::MarkerArray path_markers_;

    size_t index_;
    bool active_;

    int current_path_marker_id_;
    int8_t last_path_state_;
    bool has_active_path_marker_;

    Manipulator::JointVector q_home_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorExecutor>());
    rclcpp::shutdown();
    return 0;
}