#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/srv/command.hpp" //menene msg za srv 

class Teleop : public rclcpp::Node
{
public:
  Teleop() : Node("Teleop")
  {
    RCLCPP_INFO(this->get_logger(), "Teleop initialized");
    client_ = this->create_client<rrm_msgs::srv::Command>("move_command"); //aj toto upravene

    while (!client_ ->wait_for_service (std::chrono::seconds (1))) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR (this->get_logger (), "Interrupted while waiting for the service. Exiting." );
        return;
      }
      RCLCPP_INFO (this->get_logger (), "Service not available, waiting again..." );
    }
    current_[0] = 0.0;
    current_[1] = 0.0;
    current_[2] = 0.0;

  }
  bool move(const std::vector<double>& positions, double max_velocity)
  {
    std::vector<double> velocities = compute_velocities(positions, max_velocity); 

     RCLCPP_INFO(this->get_logger(),
      "Joint velocities: v1 = %f, v2 = %f, v3 = %f",
        velocities[0], velocities[1], velocities[2]);

    auto request = std::make_shared<rrm_msgs::srv::Command::Request>(); // req pre serv
    request->positions = positions; // takto nastavujem hodnoty ktore odoslem
    request->velocities = velocities;

    auto result = client_->async_send_request(request); // odoslanie req

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(10)) // cakanie na odpoved 10s
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      return false;
    }

    auto response = result.get(); // odpoved zo serv

    if (response->result_code == 0) { // ukladanie polohy robota
      current_[0] = positions[0];
      current_[1] = positions[1];
      current_[2] = positions[2];
      return true;
    }

    return false;
  }

private:
  std::vector<double> compute_velocities(const std::vector<double>& positions, double max_velocity)
  {
    double d0 = std::abs(positions[0] - current_[0]); // cielova poloha - aktualna
    double d1 = std::abs(positions[1] - current_[1]);
    double d2 = std::abs(positions[2] - current_[2]);

    double max_delta = d0; // zistime max posun 
    if (d1 > max_delta) max_delta = d1;
    if (d2 > max_delta) max_delta = d2;

    std::vector<double> velocities = {0.0, 0.0, 0.0}; // vektor rychlosti 

    if (max_delta > 0.0) { // ochrana pred delenim 0
      double T = max_delta / max_velocity;   // T = s/v, cas pohybu 
      velocities[0] = d0 / T; // v = s/t, rychlost pohybu
      velocities[1] = d1 / T;
      velocities[2] = d2 / T;
    }

    return velocities;
  }

private:
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_; //msg za srv a publisher za client
  double current_[3];
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<Teleop>();

  while (true)
  {
    std::string input;

    std::cout << "Enter p1 p2 p3 max_velocity (q to quit): ";

    std::cin >> input;
    if (input == "q") break;

    double p1 = std::stod(input);
    double p2, p3, max_velocity;

    std::cin >> p2 >> p3 >> max_velocity;

    if (max_velocity <= 0.0) continue;

    std::vector<double> positions = {p1, p2, p3};

    if (robot->move(positions, max_velocity)) {
      std::cout << "Move OK\n";
    } else {
      std::cout << "Move failed\n";
    }
  }
  
  rclcpp::shutdown();
  return 0;
}