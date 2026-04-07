#include <memory>      
#include <vector>      
#include <array>       
#include <string>      
#include <cmath>       
#include <algorithm>   
#include <chrono>      
#include <limits>      

#include "rclcpp/rclcpp.hpp"              
#include "rclcpp/parameter_client.hpp"    
#include "urdf/model.h"                   

#include "maszay_interface/srv/solve_all_ik.hpp"   // service na vrátenie všetkých IK riešení
#include "maszay_interface/srv/solve_best_ik.hpp"  // service na vrátenie najlepšieho IK riešenia

class IKNode : public rclcpp::Node
{
public:
    IKNode()
        : Node("ik_node"),     
          l2_(0.203),          // dĺžka druhého článku robota
          l3_(0.203)           // dĺžka tretieho článku robota
    {

        // service, ktorá vráti všetky validné IK riešenia pre zadaný bod
        solve_all_service_ = this->create_service<maszay_interface::srv::SolveAllIK>(
            "solve_all_ik",
            std::bind(&IKNode::handleSolveAllIK, this,
                      std::placeholders::_1, std::placeholders::_2));

        // service, ktorá vráti iba najlepšie IK riešenie vzhľadom na aktuálny stav robota
        solve_best_service_ = this->create_service<maszay_interface::srv::SolveBestIK>(
            "solve_best_ik",
            std::bind(&IKNode::handleSolveBestIK, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "IK node initialized");
    }

private:
    // pomocná štruktúra na uloženie jedného IK riešenia
    struct IkSolution
    {
        double q1;
        double q2;
        double q3;
    };

    // normalizácia uhla do intervalu <-pi, pi>
    double normalizeAngle(double a) const
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // orezanie hodnoty do intervalu <lo, hi>
    double clampValue(double v, double lo, double hi) const
    {
        return std::max(lo, std::min(v, hi));
    }

    // kontrola, či je uhol v zadaných limitoch kĺbu
    bool withinLimits(double q, double qmin, double qmax) const
    {
        return q >= qmin && q <= qmax;
    }

    // kontrola, či celé riešenie spĺňa limity všetkých troch kĺbov
    bool isValidSolution(const IkSolution & s) const
    {
        return withinLimits(s.q1, q1_min_, q1_max_) &&
               withinLimits(s.q2, q2_min_, q2_max_) &&
               withinLimits(s.q3, q3_min_, q3_max_);
    }

    // hlavná funkcia analytickej inverznej kinematiky
    // pre zadaný bod (x,y,z) vypočíta všetky validné riešenia
    std::vector<IkSolution> computeAllIK(double x, double y, double z) const
    {
        std::vector<IkSolution> solutions; // sem sa ukladajú validné riešenia
        int raw_count = 0;                 // počet všetkých raw kandidátov pred filtrovaním

        // vzdialenosť cieľa od osi z v rovine XY
        const double r = std::sqrt(x * x + y * y);

        // základný uhol natočenia základne smerom k bodu
        const double base = std::atan2(y, x);

        // dve možné konfigurácie základne:
        // 1. priamy smer k bodu
        // 2. otočenie o pi, pričom "rameno ide opačne"
        const std::array<std::pair<double, double>, 2> base_variants = {{
            {normalizeAngle(base),        +r},
            {normalizeAngle(base + M_PI), -r}
        }};

        // pre každú vetvu základne skúšame dve vetvy lakťa => spolu max 4 riešenia
        for (const auto & variant : base_variants)
        {
            const double q1 = variant.first;
            const double s  = variant.second;

            // kosínusová veta pre výpočet q3
            double D = (s * s + z * z - l2_ * l2_ - l3_ * l3_) / (2.0 * l2_ * l3_);

            // ak je D výrazne mimo <-1,1>, bod je pre túto vetvu nedosiahnuteľný
            if (D < -1.000001 || D > 1.000001)
            {
                continue;
            }

            // kvôli numerickým chybám ešte D orežeme do <-1,1>
            D = clampValue(D, -1.0, 1.0);

            // dve riešenia pre q3: elbow-up a elbow-down
            const double q3a = std::acos(D);
            const double q3b = -std::acos(D);

            for (double q3_raw : {q3a, q3b})
            {
                raw_count++;

                // dopočet q2 z geometrie rovinného 2R manipulátora
                const double q2_raw =
                    std::atan2(s, z) -
                    std::atan2(l3_ * std::sin(q3_raw),
                               l2_ + l3_ * std::cos(q3_raw));

                // vytvorenie kandidátneho riešenia a normalizácia uhlov
                IkSolution sol;
                sol.q1 = normalizeAngle(q1);
                sol.q2 = normalizeAngle(q2_raw);
                sol.q3 = normalizeAngle(q3_raw);

                // ponechá sa iba riešenie, ktoré je v joint limitoch
                if (isValidSolution(sol))
                {
                    bool duplicate = false;

                    // kontrola duplicity:
                    // niekedy dve vetvy dajú prakticky rovnaké riešenie
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

                    // ak riešenie nie je duplicita, pridáme ho do zoznamu
                    if (!duplicate)
                    {
                        solutions.push_back(sol);
                    }
                }
            }
        }

        // debug výpis: koľko riešení sa vygenerovalo a koľko ostalo po filtrovaní
        RCLCPP_INFO(this->get_logger(), "Raw IK solutions generated: %d", raw_count);
        RCLCPP_INFO(this->get_logger(), "Valid IK solutions after filtering: %ld", solutions.size());

        return solutions;
    }

    // euklidovská vzdialenosť dvoch konfigurácií v priestore kĺbov
    // slúži na výber "najlepšieho" riešenia vzhľadom na aktuálny stav
    double jointDistance(const IkSolution & target, const std::array<double, 3> & current) const
    {
        const double d1 = normalizeAngle(target.q1 - current[0]);
        const double d2 = normalizeAngle(target.q2 - current[1]);
        const double d3 = normalizeAngle(target.q3 - current[2]);

        return std::sqrt(d1 * d1 + d2 * d2 + d3 * d3);
    }

    // vyberie najlepšie riešenie zo zoznamu validných riešení
    // najlepšie je ktoré vyžaduje najmenšiu zmenu kĺbov od aktuálneho stavu
    bool findBestIK(const std::vector<IkSolution> & solutions, const std::array<double, 3> & current, IkSolution & best) const
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

    // načítanie joint limitov z URDF cez parameter robot_description z robot_state_publisher
    bool loadJointLimitsFromURDF()
    {
        // klient na čítanie parametrov cudzieho node-u
        auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");

        // počkanie na dostupnosť parameter service
        if (!client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter service /robot_state_publisher not available");
            return false;
        }

        // načítanie textového URDF modelu z parametra robot_description
        auto params = client->get_parameters({"robot_description"});
        if (params.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "robot_description parameter not found");
            return false;
        }

        const std::string urdf_xml = params[0].as_string();

        // URDF string do urdf::Model
        urdf::Model model;
        if (!model.initString(urdf_xml))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }

        // získanie kĺbov z modelu
        auto j1 = model.getJoint("joint_1");
        auto j2 = model.getJoint("joint_2");
        auto j3 = model.getJoint("joint_3");

        if (!j1 || !j2 || !j3)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to find joints joint_1/joint_2/joint_3 in URDF");
            return false;
        }

        // jointy musia mať definované limity
        if (!j1->limits || !j2->limits || !j3->limits)
        {
            RCLCPP_ERROR(this->get_logger(), "Joint limits missing in URDF");
            return false;
        }

        // uloženie limitov do členských premenných
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

    // callback služby solve_all_ik:
    // pre zadaný bod vráti všetky validné riešenia
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

        // riešenia sa vracajú ako tri polia q1[], q2[], q3[]
        for (const auto & s : solutions)
        {
            response->q1.push_back(s.q1);
            response->q2.push_back(s.q2);
            response->q3.push_back(s.q3);
        }
    }

    // callback služby solve_best_ik:
    // pre zadaný bod a aktuálny stav robota vráti iba najlepšie riešenie
    void handleSolveBestIK(
        const std::shared_ptr<maszay_interface::srv::SolveBestIK::Request> request,
        std::shared_ptr<maszay_interface::srv::SolveBestIK::Response> response)
    {
        // musíme mať tri aktuálne jointy
        if (request->current_joints.size() < 3)
        {
            response->success = false;
            response->message = "current_joints must contain 3 values";
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

        // aktuálny stav robota, voči ktorému sa vyberá najlepšie riešenie
        std::array<double, 3> current = {
            request->current_joints[0],
            request->current_joints[1],
            request->current_joints[2]
        };

        IkSolution best;
        if (!findBestIK(solutions, current, best))
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
    // service servery pre všetky a najlepšie IK riešenie
    rclcpp::Service<maszay_interface::srv::SolveAllIK>::SharedPtr solve_all_service_;
    rclcpp::Service<maszay_interface::srv::SolveBestIK>::SharedPtr solve_best_service_;

    // dĺžky článkov robota
    double l2_;
    double l3_;

    // limity kĺbov
    double q1_min_, q1_max_;
    double q2_min_, q2_max_;
    double q3_min_, q3_max_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);                    
    auto node = std::make_shared<IKNode>();     
    rclcpp::spin(node);                         // spracovanie callbackov a iných volaní
    rclcpp::shutdown();                         
    return 0;
}