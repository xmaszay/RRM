#include "rclcpp/rclcpp.hpp"
#include "maszay_interface/srv/play_trajectory.hpp"
#include "rrm_msgs/srv/command.hpp"

#include <fstream>
#include <vector>
#include <chrono>
#include <future>
#include <thread>
#include <atomic>
#include <mutex>

class TrajectoryPlayer : public rclcpp::Node
{
public:
    TrajectoryPlayer()
        : Node("trajectory_player"), is_playing_(false)
    {
        // service pre spustenie prehratia trajektorie zo suboru
        play_service_ = this->create_service<maszay_interface::srv::PlayTrajectory>(
            "play_trajectory",
            std::bind(&TrajectoryPlayer::play_trajectory_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // client na move_command - tymto posielame body robotovi
        move_client_ = this->create_client<rrm_msgs::srv::Command>("move_command");

        RCLCPP_INFO(this->get_logger(), "TrajectoryPlayer initialized");
        RCLCPP_INFO(this->get_logger(), "Service /play_trajectory created");
    }

    ~TrajectoryPlayer()
    {
        // ak este bezi worker thread, pockame na jeho ukoncenie
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }

private:
    // callback pre service /play_trajectory
    void play_trajectory_callback(
        const std::shared_ptr<maszay_interface::srv::PlayTrajectory::Request> request,
        std::shared_ptr<maszay_interface::srv::PlayTrajectory::Response> response)
    {
        // ak start=false, trajektoria sa nespusti
        if (!request->start)
        {
            response->result = false;
            response->message = "Start flag is false";
            return;
        }

        // kontrola, ci sa uz nejaka trajektoria prehrava
        if (is_playing_)
        {
            response->result = false;
            response->message = "Trajectory is already running";
            return;
        }

        // kontrola dostupnosti service move_command
        if (!move_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->result = false;
            response->message = "move_command service is not available";
            return;
        }

        is_playing_ = true;

        {
            std::lock_guard<std::mutex> lock(worker_mutex_);

            // ak by uz thread existoval, pockame na jeho ukoncenie
            if (worker_thread_.joinable())
            {
                worker_thread_.join();
            }

            // spustenie prehravania trajektorie v samostatnom vlakne
            worker_thread_ = std::thread(&TrajectoryPlayer::worker_function, this);
        }

        // service odpovie okamzite
        response->result = true;
        response->message = "Trajectory execution started";
    }

    // worker thread pre prehravanie trajektorie
    void worker_function()
    {
        bool ok = play_trajectory_from_file();

        if (ok)
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory execution finished successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
        }

        // po dokonceni uz mozeme spustit dalsiu trajektoriu
        is_playing_ = false;
    }

    // precita trajectory.txt a postupne odosle body robotovi
    bool play_trajectory_from_file()
    {
        std::ifstream file("trajectory.txt");

        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory.txt");
            return false;
        }

        int id;
        double p1, p2, p3, max_velocity;

        // format riadku:
        // ID joint1 joint2 joint3 velocity
        while (file >> id >> p1 >> p2 >> p3 >> max_velocity)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Executing point %d: %f %f %f vel=%f",
                        id, p1, p2, p3, max_velocity);

            // cielove pozicie robota
            std::vector<double> positions = {p1, p2, p3};

            // rovnaku velocity posleme vsetkym trom klbom
            std::vector<double> velocities = {max_velocity, max_velocity, max_velocity};

            // request na service move_command
            auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
            request->positions = positions;
            request->velocities = velocities;

            auto future = move_client_->async_send_request(request);

            // cakame na odpoved zo service
            future.wait();

            auto response = future.get();

            if (!response)
            {
                RCLCPP_ERROR(this->get_logger(), "Null response received at point %d", id);
                file.close();
                return false;
            }

            if (response->result_code != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Robot failed to move to point %d", id);
                file.close();
                return false;
            }

            // mala pauza medzi bodmi
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        file.close();
        return true;
    }

private:
    // service pre spustenie prehravania trajektorie
    rclcpp::Service<maszay_interface::srv::PlayTrajectory>::SharedPtr play_service_;

    // client na service move_command
    rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;

    // worker thread pre prehravanie trajektorie
    std::thread worker_thread_;

    // mutex pre bezpecnu pracu s threadom
    std::mutex worker_mutex_;

    // priznak, ci sa momentalne prehrava trajektoria
    std::atomic<bool> is_playing_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto player = std::make_shared<TrajectoryPlayer>();

    // MultiThreadedExecutor aby service a client mohli bezat paralelne
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(player);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}