#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"

#include <Eigen/Geometry>

#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

class PoseTeacher : public rclcpp::Node
{
public:
    PoseTeacher()
        : Node("pose_teacher"),
          server_(std::make_shared<interactive_markers::InteractiveMarkerServer>(
              "pose_teacher_marker", this))
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        current_q_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        publishJointState(current_q_);
        makeInteractiveMarker();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PoseTeacher::publishCurrentJointState, this));

        RCLCPP_INFO(this->get_logger(), "pose_teacher started");
        RCLCPP_INFO(this->get_logger(), "Move the interactive marker in RViz.");
    }

private:
    void makeInteractiveMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.name = "tool_target";
        int_marker.description = "Pose Teacher";
        int_marker.scale = 0.35;

        int_marker.pose.position.x = 1.0;
        int_marker.pose.position.y = 0.3;
        int_marker.pose.position.z = 1.2;
        int_marker.pose.orientation.w = 1.0;

        addVisibleControl(int_marker);
        add6DofControls(int_marker);

        server_->insert(
            int_marker,
            std::bind(&PoseTeacher::processFeedback, this, std::placeholders::_1));

        server_->applyChanges();
    }

    void addVisibleControl(visualization_msgs::msg::InteractiveMarker &int_marker)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.name = "visual";

        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.1f;
        marker.color.a = 1.0f;

        control.markers.push_back(marker);
        int_marker.controls.push_back(control);
    }

    void add6DofControls(visualization_msgs::msg::InteractiveMarker &int_marker)
    {
        using visualization_msgs::msg::InteractiveMarkerControl;

        InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    void processFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        Eigen::Affine3d target_pose = poseMsgToEigen(feedback->pose);

        auto solutions = ikfast_abb::computeIK(target_pose);

        if (solutions.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "No IK solution for current marker pose.");
            return;
        }

        auto best_solution = selectNearestSolution(solutions, current_q_);
        current_q_ = best_solution;

       if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    RCLCPP_INFO(
        this->get_logger(),
        "Joint values: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
        current_q_[0], current_q_[1], current_q_[2],
        current_q_[3], current_q_[4], current_q_[5]);

    RCLCPP_INFO(
        this->get_logger(),
        "Pose position: [%.6f, %.6f, %.6f]",
        feedback->pose.position.x,
        feedback->pose.position.y,
        feedback->pose.position.z);

    RCLCPP_INFO(
        this->get_logger(),
        "Pose orientation (quat): [%.6f, %.6f, %.6f, %.6f]",
        feedback->pose.orientation.x,
        feedback->pose.orientation.y,
        feedback->pose.orientation.z,
        feedback->pose.orientation.w);
}
    }

    void publishCurrentJointState()
    {
        publishJointState(current_q_);
    }

    Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::Pose &pose_msg)
    {
        Eigen::Translation3d translation(
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z);

        Eigen::Quaterniond q(
            pose_msg.orientation.w,
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z);

        return translation * q;
    }

    ikfast_abb::JointValues selectNearestSolution(
        const ikfast_abb::Solutions &solutions,
        const ikfast_abb::JointValues &reference)
    {
        double best_dist = std::numeric_limits<double>::max();
        ikfast_abb::JointValues best = solutions.front();

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

    void publishJointState(const ikfast_abb::JointValues &q)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position.assign(q.begin(), q.end());
        joint_pub_->publish(msg);
    }

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    ikfast_abb::JointValues current_q_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseTeacher>());
    rclcpp::shutdown();
    return 0;
}