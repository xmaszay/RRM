#include <memory>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

#include "maszay_interface/srv/solve_all_ik.hpp"
#include "maszay_interface/srv/solve_best_ik.hpp"

class IKNode : public rclcpp::Node
{
public:
    IKNode()
        : Node("ik_node"),
          l2_(0.203),
          l3_(0.203),
          has_joint_state_(false)
    {
        current_joints_ = {0.0, 0.0, 0.0};

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&IKNode::jointStatesCallback, this, std::placeholders::_1));

        if (!loadJointLimitsFromURDF())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to load joint limits from URDF");
            throw std::runtime_error("Failed to load joint limits from URDF");
        }

        solve_all_service_ = this->create_service<maszay_interface::srv::SolveAllIK>(
            "solve_all_ik",
            std::bind(&IKNode::handleSolveAllIK, this,
                      std::placeholders::_1, std::placeholders::_2));

        solve_best_service_ = this->create_service<maszay_interface::srv::SolveBestIK>(
            "solve_best_ik",
            std::bind(&IKNode::handleSolveBestIK, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "IK node initialized");
    }

private:
    struct IkSolution
    {
        double q1;
        double q2;
        double q3;
    };

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

    double normalizeAngle(double a) const
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    double clampValue(double v, double lo, double hi) const
    {
        return std::max(lo, std::min(v, hi));
    }

    bool withinLimits(double q, double qmin, double qmax) const
    {
        return q >= qmin && q <= qmax;
    }

    bool isValidSolution(const IkSolution & s) const
    {
        return withinLimits(s.q1, q1_min_, q1_max_) &&
               withinLimits(s.q2, q2_min_, q2_max_) &&
               withinLimits(s.q3, q3_min_, q3_max_);
    }

    std::vector<IkSolution> computeAllIK(double x, double y, double z) const
    {
        std::vector<IkSolution> solutions;
        int raw_count = 0;

        const double r = std::sqrt(x * x + y * y);
        const double base = std::atan2(y, x);

        const std::array<std::pair<double, double>, 2> base_variants = {{
            {normalizeAngle(base),        +r},
            {normalizeAngle(base + M_PI), -r}
        }};

        for (const auto & variant : base_variants)
        {
            const double q1 = variant.first;
            const double s  = variant.second;

            double D = (s * s + z * z - l2_ * l2_ - l3_ * l3_) / (2.0 * l2_ * l3_);

            if (D < -1.000001 || D > 1.000001)
            {
                continue;
            }

            D = clampValue(D, -1.0, 1.0);

            const double q3a = std::acos(D);
            const double q3b = -std::acos(D);

            for (double q3_raw : {q3a, q3b})
            {
                raw_count++;

                const double q2_raw =
                    std::atan2(s, z) -
                    std::atan2(l3_ * std::sin(q3_raw),
                               l2_ + l3_ * std::cos(q3_raw));

                IkSolution sol;
                sol.q1 = normalizeAngle(q1);
                sol.q2 = normalizeAngle(q2_raw);
                sol.q3 = normalizeAngle(q3_raw);

                if (isValidSolution(sol))
                {
                    bool duplicate = false;

                    for (const auto & existing : solutions)
                    {
                        if (std::fabs(normalizeAngle(existing.q1 - sol.q1)) < 1e-6 &&
                            std::fabs(normalizeAngle(existing.q2 - sol.q2)) < 1e-6 &&
                            std::fabs(normalizeAngle(existing.q3 - sol.q3)) < 1e-6)
                        {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate)
                    {
                        solutions.push_back(sol);
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Raw IK solutions generated: %d", raw_count);
        RCLCPP_INFO(this->get_logger(), "Valid IK solutions after filtering: %ld", solutions.size());

        return solutions;
    }

    double jointDistance(const IkSolution & target, const std::array<double, 3> & current) const
    {
        const double d1 = normalizeAngle(target.q1 - current[0]);
        const double d2 = normalizeAngle(target.q2 - current[1]);
        const double d3 = normalizeAngle(target.q3 - current[2]);

        return std::sqrt(d1 * d1 + d2 * d2 + d3 * d3);
    }

    bool findBestIK(const std::vector<IkSolution> & solutions,
                    const std::array<double, 3> & current,
                    IkSolution & best) const
    {
        if (solutions.empty())
        {
            return false;
        }

        double best_cost = std::numeric_limits<double>::infinity();

        for (const auto & s : solutions)
        {
            const double cost = jointDistance(s, current);
            if (cost < best_cost)
            {
                best_cost = cost;
                best = s;
            }
        }

        return true;
    }

    bool loadJointLimitsFromURDF()
    {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");

        if (!client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter service /robot_state_publisher not available");
            return false;
        }

        auto params = client->get_parameters({"robot_description"});
        if (params.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "robot_description parameter not found");
            return false;
        }

        const std::string urdf_xml = params[0].as_string();

        urdf::Model model;
        if (!model.initString(urdf_xml))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }

        auto j1 = model.getJoint("joint_1");
        auto j2 = model.getJoint("joint_2");
        auto j3 = model.getJoint("joint_3");

        if (!j1 || !j2 || !j3)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to find joints joint_1/joint_2/joint_3 in URDF");
            return false;
        }

        if (!j1->limits || !j2->limits || !j3->limits)
        {
            RCLCPP_ERROR(this->get_logger(), "Joint limits missing in URDF");
            return false;
        }

        q1_min_ = j1->limits->lower;
        q1_max_ = j1->limits->upper;
        q2_min_ = j2->limits->lower;
        q2_max_ = j2->limits->upper;
        q3_min_ = j3->limits->lower;
        q3_max_ = j3->limits->upper;

        RCLCPP_INFO(this->get_logger(),
                    "Loaded joint limits: q1[%.3f, %.3f], q2[%.3f, %.3f], q3[%.3f, %.3f]",
                    q1_min_, q1_max_, q2_min_, q2_max_, q3_min_, q3_max_);

        return true;
    }

    void handleSolveAllIK(
        const std::shared_ptr<maszay_interface::srv::SolveAllIK::Request> request,
        std::shared_ptr<maszay_interface::srv::SolveAllIK::Response> response)
    {
        const auto & p = request->target;
        auto solutions = computeAllIK(p.x, p.y, p.z);

        if (solutions.empty())
        {
            response->success = false;
            response->message = "No valid IK solution found";
            return;
        }

        response->success = true;
        response->message = "Valid IK solutions found";

        for (const auto & s : solutions)
        {
            response->q1.push_back(s.q1);
            response->q2.push_back(s.q2);
            response->q3.push_back(s.q3);
        }
    }

    void handleSolveBestIK(
        const std::shared_ptr<maszay_interface::srv::SolveBestIK::Request> request,
        std::shared_ptr<maszay_interface::srv::SolveBestIK::Response> response)
    {
        if (!has_joint_state_)
        {
            response->success = false;
            response->message = "Current joint state not available";
            return;
        }

        const auto & p = request->target;
        auto solutions = computeAllIK(p.x, p.y, p.z);

        if (solutions.empty())
        {
            response->success = false;
            response->message = "No valid IK solution found";
            return;
        }

        IkSolution best;
        if (!findBestIK(solutions, current_joints_, best))
        {
            response->success = false;
            response->message = "Failed to select best IK solution";
            return;
        }

        response->success = true;
        response->message = "Best IK solution found";
        response->solution = {best.q1, best.q2, best.q3};
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<maszay_interface::srv::SolveAllIK>::SharedPtr solve_all_service_;
    rclcpp::Service<maszay_interface::srv::SolveBestIK>::SharedPtr solve_best_service_;

    double l2_;
    double l3_;
    double q1_min_, q1_max_;
    double q2_min_, q2_max_;
    double q3_min_, q3_max_;

    std::array<double, 3> current_joints_;
    bool has_joint_state_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<IKNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception & e)
    {
        std::cerr << "IK node failed to start: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}