#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

#include <Eigen/Geometry>

class FkDumpNode : public rclcpp::Node
{
public:
    FkDumpNode() : Node("fk_dump")
    {
        printPose("T1", {-0.346254,  0.197993, -0.105621, -1.563038, -0.344984,  1.532185});
        printPose("T2", { 0.212505,  0.141679, -0.044757,  1.521074, -0.212008, -1.506295});
        printPose("Tvia", { 0.135446, -0.144309,  0.451832,  0.460273, -0.308750, -0.437340});
        printPose("T3", { 0.023699,  0.155486,  0.456767, -3.098603,  0.583753,  3.106401});
        printPose("T4", {-0.179721,  0.175691,  0.433183, -0.320224, -0.603631,  0.261282});

        rclcpp::shutdown();
    }

private:
    void printPose(const std::string &name, const ikfast_abb::JointValues &q)
    {
        Eigen::Affine3d pose = ikfast_abb::computeFk(q);
        Eigen::Vector3d p = pose.translation();
        Eigen::Quaterniond quat(pose.rotation());
        quat.normalize();

        RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "  position = [%.6f, %.6f, %.6f]",
                    p.x(), p.y(), p.z());
        RCLCPP_INFO(this->get_logger(),
                    "  quat     = [%.6f, %.6f, %.6f, %.6f]",
                    quat.x(), quat.y(), quat.z(), quat.w());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FkDumpNode>();
    rclcpp::spin_some(node);
    return 0;
}