#include <memory>
#include <array>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"

#include "maszay_interface/srv/solve_best_ik.hpp"
#include "maszay_interface/srv/move_to_point.hpp"

class MotionManagerNode : public rclcpp::Node
{
public:
    MotionManagerNode()
        : Node("motion_manager_node"),
          has_joint_state_(false)
    {
        current_joints_ = {0.0, 0.0, 0.0};

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&MotionManagerNode::jointStatesCallback, this, std::placeholders::_1));

        helper_node_ = std::make_shared<rclcpp::Node>("motion_manager_helper");

        ik_client_ = helper_node_->create_client<maszay_interface::srv::SolveBestIK>("solve_best_ik");
        move_client_ = helper_node_->create_client<rrm_msgs::srv::Command>("move_command");

        move_service_ = this->create_service<maszay_interface::srv::MoveToPoint>(
            "move_to_point",
            std::bind(&MotionManagerNode::handleMoveToPoint, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Motion manager node initialized");
    }

private:
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() == msg->position.size())
        {
            bool found1 = false;
            bool found2 = false;
            bool found3 = false;

            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == "joint_1")
                {
                    current_joints_[0] = msg->position[i];
                    found1 = true;
                }
                else if (msg->name[i] == "joint_2")
                {
                    current_joints_[1] = msg->position[i];
                    found2 = true;
                }
                else if (msg->name[i] == "joint_3")
                {
                    current_joints_[2] = msg->position[i];
                    found3 = true;
                }
            }

            if (found1 && found2 && found3)
            {
                has_joint_state_ = true;
                return;
            }
        }

        if (msg->position.size() >= 3)
        {
            current_joints_[0] = msg->position[0];
            current_joints_[1] = msg->position[1];
            current_joints_[2] = msg->position[2];
            has_joint_state_ = true;
        }
    }

    void handleMoveToPoint(
        const std::shared_ptr<maszay_interface::srv::MoveToPoint::Request> request,
        std::shared_ptr<maszay_interface::srv::MoveToPoint::Response> response)
    {
        if (!has_joint_state_)
        {
            response->success = false;
            response->message = "Current joint state not available";
            return;
        }

        if (request->velocity <= 0.0)
        {
            response->success = false;
            response->message = "Velocity must be > 0";
            return;
        }

        if (!ik_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "IK solver service not available";
            return;
        }

        if (!move_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "move_command service not available";
            return;
        }

        auto ik_request = std::make_shared<maszay_interface::srv::SolveBestIK::Request>();
        ik_request->target = request->target;

        auto ik_future = ik_client_->async_send_request(ik_request);

        auto ik_result_code =
            rclcpp::spin_until_future_complete(helper_node_, ik_future, std::chrono::seconds(5));

        if (ik_result_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            response->success = false;
            response->message = "Failed to call solve_best_ik";
            return;
        }

        auto ik_result = ik_future.get();

        if (!ik_result->success || ik_result->solution.size() < 3)
        {
            response->success = false;
            response->message = "IK solver did not return a valid solution";
            return;
        }

        auto move_request = std::make_shared<rrm_msgs::srv::Command::Request>();
        move_request->positions = ik_result->solution;
        move_request->velocities = {
            request->velocity,
            request->velocity,
            request->velocity
        };

        auto move_future = move_client_->async_send_request(move_request);

        auto move_result_code =
            rclcpp::spin_until_future_complete(helper_node_, move_future, std::chrono::seconds(10));

        if (move_result_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            response->success = false;
            response->message = "Failed to call move_command";
            return;
        }

        auto move_result = move_future.get();

        if (move_result->result_code == 0)
        {
            response->success = true;
            response->message = "Motion completed successfully";
            RCLCPP_INFO(this->get_logger(), "Motion completed successfully");
        }
        else
        {
            response->success = false;
            response->message = "Simulator reported motion failure";
            RCLCPP_ERROR(this->get_logger(), "Simulator reported motion failure");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Client<maszay_interface::srv::SolveBestIK>::SharedPtr ik_client_;
    rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;
    rclcpp::Service<maszay_interface::srv::MoveToPoint>::SharedPtr move_service_;

    std::array<double, 3> current_joints_;
    bool has_joint_state_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotionManagerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}