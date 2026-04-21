#pragma once

#include <array>
#include <vector>
#include <Eigen/Geometry>

class Manipulator
{
public:
    using JointVector = std::array<double, 6>;
    using Pose = Eigen::Affine3d;

    struct Sample
    {
        double t;
        JointVector q;
        JointVector dq;
        JointVector ddq;
    };

    std::vector<Sample> generatePTP(
        const JointVector &q_start,
        const JointVector &q_goal,
        double duration,
        double dt) const;

    std::vector<Sample> generatePTPVia(
        const JointVector &q_start,
        const JointVector &q_via,
        const JointVector &q_goal,
        double duration1,
        double duration2,
        double dt) const;

    std::vector<Sample> generateLIN(
        const Pose &pose_start,
        const Pose &pose_goal,
        const JointVector &q_seed,
        double duration,
        double dt) const;

    Pose offsetAlongToolX(const Pose &pose, double offset) const;

private:
    struct QuinticCoefficients
    {
        double a0, a1, a2, a3, a4, a5;
    };

    QuinticCoefficients computeGeneralQuintic(
        double q0, double v0, double a0,
        double qf, double vf, double af,
        double T) const;

    double evalPosition(const QuinticCoefficients &c, double t) const;
    double evalVelocity(const QuinticCoefficients &c, double t) const;
    double evalAcceleration(const QuinticCoefficients &c, double t) const;

    double quinticScale(double tau) const;
    double quinticScaleDot(double tau, double T) const;
    double quinticScaleDDot(double tau, double T) const;

    JointVector selectNearestSolution(
        const std::vector<JointVector> &solutions,
        const JointVector &reference) const;
};