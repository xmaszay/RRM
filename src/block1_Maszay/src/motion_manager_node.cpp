#include <memory>      
#include <array>       
#include <string>      
#include <chrono>     

#include "rclcpp/rclcpp.hpp"               
#include "sensor_msgs/msg/joint_state.hpp" 
#include "rrm_msgs/srv/command.hpp"        

#include "maszay_interface/srv/solve_best_ik.hpp" // service na výber najlepšieho IK riešenia
#include "maszay_interface/srv/move_to_point.hpp" // service pre používateľa: pohyb do bodu

class MotionManagerNode : public rclcpp::Node
{
public:
    MotionManagerNode()
        : Node("motion_manager_node"), /
          has_joint_state_(false)      // na začiatku ešte nepoznáme aktuálny stav robota
    {
        // počiatočné nastavenie jointov
        current_joints_ = {0.0, 0.0, 0.0};

        // subscriber na joint_states, aby motion manager vedel aktuálnu konfiguráciu robota
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
            std::bind(&MotionManagerNode::jointStatesCallback, this, std::placeholders::_1));

        // pomocný node sa používa na synchronné čakanie na service odpovede
        helper_node_ = std::make_shared<rclcpp::Node>("motion_manager_helper");

        // klient na service solve_best_ik z IK node
        ik_client_ = helper_node_->create_client<maszay_interface::srv::SolveBestIK>("solve_best_ik");

        // klient na service move_command zo simulátora
        move_client_ = helper_node_->create_client<rrm_msgs::srv::Command>("move_command");

        // service, ktorú volá používateľ:
        // zadá kartézsky bod a rýchlosť, manager vybaví celý pohyb
        move_service_ = this->create_service<maszay_interface::srv::MoveToPoint>(
            "move_to_point",
            std::bind(&MotionManagerNode::handleMoveToPoint, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Motion manager node initialized");
    }

private:
    // callback na joint_states, ukladá aktuálne natočenia kĺbov robota
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() == msg->position.size())
        {
            bool found1 = false;
            bool found2 = false;
            bool found3 = false;

            // hľadanie jointov podľa mena
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

            // ak sú všetky tri jointy načítané, máme validný stav robota
            if (found1 && found2 && found3)
            {
                has_joint_state_ = true;
                return;
            }
        }
    }

    // hlavný callback služby move_to_point
    // prijme cieľový bod v priestore a rýchlosť, nájde IK riešenie a pošle pohyb do simulátora
    void handleMoveToPoint(
        const std::shared_ptr<maszay_interface::srv::MoveToPoint::Request> request,
        std::shared_ptr<maszay_interface::srv::MoveToPoint::Response> response)
    {
        // bez aktuálneho stavu kĺbov nevieme vybrať najlepšie IK riešenie
        if (!has_joint_state_)
        {
            response->success = false;
            response->message = "Current joint state not available";
            return;
        }

        // rýchlosť pohybu musí byť kladná
        if (request->velocity <= 0.0)
        {
            response->success = false;
            response->message = "Velocity must be > 0";
            return;
        }

        // kontrola, či je dostupný IK solver
        if (!ik_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "IK solver service not available";
            return;
        }

        // kontrola, či je dostupný simulátorový service move_command
        if (!move_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "move_command service not available";
            return;
        }

        // príprava requestu pre IK solver:
        // posielame cieľový bod a aktuálnu konfiguráciu robota
        auto ik_request = std::make_shared<maszay_interface::srv::SolveBestIK::Request>();
        ik_request->target = request->target;
        ik_request->current_joints = {
            current_joints_[0],
            current_joints_[1],
            current_joints_[2]
        };

        // asynchrónne zavolanie IK service
        auto ik_future = ik_client_->async_send_request(ik_request);

        // synchronné počkanie na výsledok cez helper_node_
        auto ik_result_code =
            rclcpp::spin_until_future_complete(helper_node_, ik_future, std::chrono::seconds(5));

        // ak service zlyhá alebo timeoutne, pohyb sa nevykoná
        if (ik_result_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            response->success = false;
            response->message = "Failed to call solve_best_ik";
            return;
        }

        auto ik_result = ik_future.get();

        // ak IK solver nevrátil validné riešenie, manager vráti chybu
        if (!ik_result->success || ik_result->solution.size() < 3)
        {
            response->success = false;
            response->message = "IK solver did not return a valid solution";
            return;
        }

        // príprava requestu pre simulátor:
        // pošleme joint riešenie a rovnakú rýchlosť pre všetky kĺby
        auto move_request = std::make_shared<rrm_msgs::srv::Command::Request>();
        move_request->positions = ik_result->solution;
        move_request->velocities = {
            request->velocity,
            request->velocity,
            request->velocity
        };

        // asynchrónne volanie service move_command
        auto move_future = move_client_->async_send_request(move_request);

        // synchronné počkanie na výsledok simulátora
        auto move_result_code =
            rclcpp::spin_until_future_complete(helper_node_, move_future, std::chrono::seconds(10));

        // ak simulátor neodpovie korektne, vráti sa chyba
        if (move_result_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            response->success = false;
            response->message = "Failed to call move_command";
            return;
        }

        auto move_result = move_future.get();

        // result_code == 0 znamená úspešne dokončený pohyb
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
    // subscriber na aktuálne joint stavy
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // pomocný node na bezpečné synchronné čakanie na service odpovede
    rclcpp::Node::SharedPtr helper_node_;

    // klienti na IK solver a simulátor
    rclcpp::Client<maszay_interface::srv::SolveBestIK>::SharedPtr ik_client_;
    rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;

    // service server, ktorý poskytuje rozhranie používateľovi
    rclcpp::Service<maszay_interface::srv::MoveToPoint>::SharedPtr move_service_;

    // aktuálny stav kĺbov robota
    std::array<double, 3> current_joints_;
    bool has_joint_state_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);                               
    auto node = std::make_shared<MotionManagerNode>();     
    rclcpp::spin(node);                                    // spracovanie callbackov a service požiadaviek
    rclcpp::shutdown();                                    
    return 0;
}