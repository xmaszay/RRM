#include "final_project/manipulator.hpp"
#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

#include <cmath>
#include <limits>
#include <stdexcept>

Manipulator::QuinticCoefficients Manipulator::computeGeneralQuintic(
    double q0, double v0, double a0,
    double qf, double vf, double af,
    double T) const
{
    QuinticCoefficients c;

    c.a0 = q0;
    c.a1 = v0;
    c.a2 = a0 / 2.0;

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    c.a3 = (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
    c.a4 = (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / (2.0 * T4);
    c.a5 = (12.0 * (qf - q0) - (6.0 * vf + 6.0 * v0) * T - (a0 - af) * T2) / (2.0 * T5);

    return c;
}

double Manipulator::evalPosition(const QuinticCoefficients &c, double t) const
{
    return c.a0
         + c.a1 * t
         + c.a2 * t * t
         + c.a3 * t * t * t
         + c.a4 * t * t * t * t
         + c.a5 * t * t * t * t * t;
}

double Manipulator::evalVelocity(const QuinticCoefficients &c, double t) const
{
    return c.a1
         + 2.0 * c.a2 * t
         + 3.0 * c.a3 * t * t
         + 4.0 * c.a4 * t * t * t
         + 5.0 * c.a5 * t * t * t * t;
}

double Manipulator::evalAcceleration(const QuinticCoefficients &c, double t) const
{
    return 2.0 * c.a2
         + 6.0 * c.a3 * t
         + 12.0 * c.a4 * t * t
         + 20.0 * c.a5 * t * t * t;
}

double Manipulator::quinticScale(double tau) const
{
    return 10.0 * tau * tau * tau
         - 15.0 * tau * tau * tau * tau
         +  6.0 * tau * tau * tau * tau * tau;
}

double Manipulator::quinticScaleDot(double tau, double T) const
{
    return (30.0 * tau * tau
          - 60.0 * tau * tau * tau
          + 30.0 * tau * tau * tau * tau) / T;
}

double Manipulator::quinticScaleDDot(double tau, double T) const
{
    return (60.0 * tau
          - 180.0 * tau * tau
          + 120.0 * tau * tau * tau) / (T * T);
}

std::vector<Manipulator::Sample> Manipulator::generatePTP(
    const JointVector &q_start,
    const JointVector &q_goal,
    double duration,
    double dt) const
{
    std::vector<Sample> trajectory;

    for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        double local_t = (t > duration) ? duration : t;
        double tau = local_t / duration;

        double s = quinticScale(tau);
        double ds = quinticScaleDot(tau, duration);
        double dds = quinticScaleDDot(tau, duration);

        Sample sample;
        sample.t = local_t;

        for (size_t i = 0; i < 6; ++i) {
            double dq_total = q_goal[i] - q_start[i];
            sample.q[i] = q_start[i] + s * dq_total;
            sample.dq[i] = ds * dq_total;
            sample.ddq[i] = dds * dq_total;
        }

        trajectory.push_back(sample);
    }

    return trajectory;
}

std::vector<Manipulator::Sample> Manipulator::generatePTPVia(
    const JointVector &q_start,
    const JointVector &q_via,
    const JointVector &q_goal,
    double duration1,
    double duration2,
    double dt) const
{
    std::array<QuinticCoefficients, 6> coeffs1;
    std::array<QuinticCoefficients, 6> coeffs2;
    JointVector v_via{};
    JointVector a_via{};

    for (size_t i = 0; i < 6; ++i) {
        v_via[i] = (q_goal[i] - q_start[i]) / (duration1 + duration2);
        a_via[i] = 0.0;

        coeffs1[i] = computeGeneralQuintic(
            q_start[i], 0.0, 0.0,
            q_via[i], v_via[i], a_via[i],
            duration1);

        coeffs2[i] = computeGeneralQuintic(
            q_via[i], v_via[i], a_via[i],
            q_goal[i], 0.0, 0.0,
            duration2);
    }

    std::vector<Sample> trajectory;

    for (double t = 0.0; t <= duration1 + 1e-9; t += dt) {
        Sample sample;
        double local_t = (t > duration1) ? duration1 : t;
        sample.t = local_t;

        for (size_t i = 0; i < 6; ++i) {
            sample.q[i] = evalPosition(coeffs1[i], local_t);
            sample.dq[i] = evalVelocity(coeffs1[i], local_t);
            sample.ddq[i] = evalAcceleration(coeffs1[i], local_t);
        }

        trajectory.push_back(sample);
    }

    for (double t = dt; t <= duration2 + 1e-9; t += dt) {
        Sample sample;
        double local_t = (t > duration2) ? duration2 : t;
        sample.t = duration1 + local_t;

        for (size_t i = 0; i < 6; ++i) {
            sample.q[i] = evalPosition(coeffs2[i], local_t);
            sample.dq[i] = evalVelocity(coeffs2[i], local_t);
            sample.ddq[i] = evalAcceleration(coeffs2[i], local_t);
        }

        trajectory.push_back(sample);
    }

    return trajectory;
}

Manipulator::JointVector Manipulator::selectNearestSolution(
    const std::vector<JointVector> &solutions,
    const JointVector &reference) const
{
    double best_dist = std::numeric_limits<double>::max();
    JointVector best = solutions.front();

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

std::vector<Manipulator::Sample> Manipulator::generateLIN(
    const Pose &pose_start,
    const Pose &pose_goal,
    const JointVector &q_seed,
    double duration,
    double dt) const
{
    std::vector<Sample> trajectory;
    JointVector q_prev = q_seed;

    const Eigen::Vector3d p0 = pose_start.translation();
    const Eigen::Vector3d pf = pose_goal.translation();

    const Eigen::Matrix3d R0 = pose_start.rotation();

    for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        double local_t = (t > duration) ? duration : t;
        double tau = local_t / duration;

        double s = quinticScale(tau);

        Eigen::Vector3d p = p0 + s * (pf - p0);

        Pose desired = Pose::Identity();
        desired.linear() = R0;
        desired.translation() = p;

        auto ik_solutions_raw = ikfast_abb::computeIK(desired);
        if (ik_solutions_raw.empty()) {
            throw std::runtime_error("generateLIN: IK returned no solution.");
        }

        std::vector<JointVector> ik_solutions;
        for (const auto &sol : ik_solutions_raw) {
            JointVector q{};
            for (size_t i = 0; i < 6; ++i) {
                q[i] = sol[i];
            }
            ik_solutions.push_back(q);
        }

        JointVector q = selectNearestSolution(ik_solutions, q_prev);

        Sample sample;
        sample.t = local_t;
        sample.q = q;

        if (trajectory.empty()) {
            sample.dq.fill(0.0);
            sample.ddq.fill(0.0);
        } else {
            const auto &prev = trajectory.back();
            for (size_t i = 0; i < 6; ++i) {
                sample.dq[i] = (sample.q[i] - prev.q[i]) / dt;
                sample.ddq[i] = (sample.dq[i] - prev.dq[i]) / dt;
            }
        }

        q_prev = q;
        trajectory.push_back(sample);
    }

    if (!trajectory.empty()) {
        trajectory.front().dq.fill(0.0);
        trajectory.front().ddq.fill(0.0);
        trajectory.back().dq.fill(0.0);
        trajectory.back().ddq.fill(0.0);
    }

    return trajectory;
}

Manipulator::Pose Manipulator::offsetAlongToolX(const Pose &pose, double offset) const
{
    Pose result = pose;

    Eigen::Vector3d local_x = pose.linear().col(0);
    result.translation() = pose.translation() + offset * local_x;

    return result;
}