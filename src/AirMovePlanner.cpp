#include "airmove/AirMovePlanner.hpp"

#include "airmove/BSpline.hpp"
#include "airmove/RuckigExecutor.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace airmove {
namespace {

double minClearance(const Path3& path, const CollisionWorld& world) {
    if (path.empty()) {
        return 0.0;
    }

    double result = std::numeric_limits<double>::infinity();
    for (const auto& point : path) {
        result = std::min(result, world.distanceToNearestObstacle(point));
    }
    return result;
}

double averageClearance(const Path3& path, const CollisionWorld& world) {
    if (path.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (const auto& point : path) {
        sum += world.distanceToNearestObstacle(point);
    }
    return sum / static_cast<double>(path.size());
}

double maxCurvature(const Path3& path) {
    if (path.size() < 3) {
        return 0.0;
    }

    double result = 0.0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i] - path[i - 1];
        const Vec3 b = path[i + 1] - path[i];
        const double len_a = a.norm();
        const double len_b = b.norm();
        if (len_a < 1e-9 || len_b < 1e-9) {
            continue;
        }

        const double cos_angle = std::clamp(a.dot(b) / (len_a * len_b), -1.0, 1.0);
        const double angle = std::acos(cos_angle);
        const double local_curvature = angle / (0.5 * (len_a + len_b));
        result = std::max(result, local_curvature);
    }
    return result;
}

bool isPathValid(const Path3& path, const CollisionWorld& world, double resolution) {
    if (path.empty()) {
        return false;
    }
    for (const auto& point : path) {
        if (!world.isStateValid(point)) {
            return false;
        }
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!world.isSegmentValid(path[i - 1], path[i], resolution)) {
            return false;
        }
    }
    return true;
}

bool isPositiveVec(const Vec3& value) {
    return (value.array() > 0.0).all();
}

void updateTrajectoryMetrics(PlanningResult& result) {
    result.max_velocity_abs = Vec3::Zero();
    result.max_acceleration_abs = Vec3::Zero();
    result.max_jerk_abs = Vec3::Zero();
    result.jerk_integral = 0.0;

    for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
        const auto& sample = result.trajectory[i];
        result.max_velocity_abs = result.max_velocity_abs.cwiseMax(sample.velocity.cwiseAbs());
        result.max_acceleration_abs =
            result.max_acceleration_abs.cwiseMax(sample.acceleration.cwiseAbs());
        result.max_jerk_abs = result.max_jerk_abs.cwiseMax(sample.jerk.cwiseAbs());

        if (i > 0) {
            const double dt = sample.time - result.trajectory[i - 1].time;
            if (dt > 0.0) {
                result.jerk_integral += sample.jerk.squaredNorm() * dt;
            }
        }
    }
}

ob::PlannerPtr createPlanner(const PlannerConfig& config, const ob::SpaceInformationPtr& si) {
    const auto applyRange = [&config](auto& planner) {
        if (config.planner_range > 0.0) {
            planner->setRange(config.planner_range);
        }
    };

    const auto applyGoalBias = [&config](auto& planner) {
        if (config.planner_goal_bias >= 0.0 && config.planner_goal_bias <= 1.0) {
            planner->setGoalBias(config.planner_goal_bias);
        }
    };

    const auto& planner_type = config.planner_type;
    if (planner_type == "rrtconnect") {
        auto planner = std::make_shared<og::RRTConnect>(si);
        applyRange(planner);
        return planner;
    }
    if (planner_type == "rrtstar") {
        auto planner = std::make_shared<og::RRTstar>(si);
        applyRange(planner);
        applyGoalBias(planner);
        return planner;
    }
    if (planner_type == "informed_rrtstar") {
        auto planner = std::make_shared<og::InformedRRTstar>(si);
        applyRange(planner);
        applyGoalBias(planner);
        return planner;
    }

    throw std::runtime_error(
        "Unsupported planner type: " + planner_type +
        ". Supported values: rrtconnect, rrtstar, informed_rrtstar.");
}

} // namespace

AirMovePlanner::AirMovePlanner(PlannerConfig config) : config_(std::move(config)) {}

PlanningResult AirMovePlanner::plan(const PlanningRequest& request, const CollisionWorld& world) const {
    PlanningResult result;
    result.planner_type = config_.planner_type;
    const Vec3 start = request.start.position;
    const Vec3 goal = request.goal.position;

    if ((config_.workspace_max.array() <= config_.workspace_min.array()).any()) {
        result.message = "Workspace max must be greater than workspace min on all axes.";
        return result;
    }
    if (config_.head_radius <= 0.0 || config_.safety_margin < 0.0) {
        result.message = "Head radius must be positive and safety margin must be non-negative.";
        return result;
    }
    if (config_.validity_resolution <= 0.0 || config_.smoothing_samples <= 0) {
        result.message = "Validity resolution and smoothing samples must be positive.";
        return result;
    }
    if (!isInsideWorkspace(start) || !isInsideWorkspace(goal)) {
        result.message = "Start or goal is outside the configured workspace.";
        return result;
    }
    if (request.planning_time <= 0.0 || request.sample_dt <= 0.0) {
        result.message = "Planning time and sample_dt must be positive.";
        return result;
    }
    if (!isPositiveVec(request.limits.max_velocity) ||
        !isPositiveVec(request.limits.max_acceleration) ||
        !isPositiveVec(request.limits.max_jerk)) {
        result.message = "Motion limits must be positive on all axes.";
        return result;
    }
    if (config_.planner_range < 0.0) {
        result.message = "Planner range must be zero for auto range or a positive value.";
        return result;
    }
    if (config_.planner_goal_bias < 0.0 || config_.planner_goal_bias > 1.0) {
        result.message = "Planner goal_bias must be in [0, 1].";
        return result;
    }
    if (!world.isStateValid(start)) {
        result.message = "Start state is in collision or violates the safety margin.";
        return result;
    }
    if (!world.isStateValid(goal)) {
        result.message = "Goal state is in collision or violates the safety margin.";
        return result;
    }

    try {
        if (world.isSegmentValid(start, goal, config_.validity_resolution)) {
            result.raw_path = {start, goal};
        } else {
            result.raw_path = planWithOMPL(start, goal, world, request.planning_time);
        }

        BSplineSmoother smoother;
        result.smoothed_path = smoother.smooth(result.raw_path, config_.smoothing_samples);
        if (!isPathValid(result.smoothed_path, world, config_.validity_resolution)) {
            result.smoothed_path = result.raw_path;
        }

        MotionLimits limits = request.limits;
        limits.control_cycle = request.sample_dt;
        result.trajectory = RuckigExecutor(limits).generateStopToStop(result.smoothed_path);
        result.raw_path_length = pathLength(result.raw_path);
        result.smoothed_path_length = pathLength(result.smoothed_path);
        result.min_clearance = minClearance(result.smoothed_path, world);
        result.average_clearance = averageClearance(result.smoothed_path, world);
        result.max_curvature = maxCurvature(result.smoothed_path);
        updateTrajectoryMetrics(result);
        result.success = true;
        result.message = "Planning succeeded.";
    } catch (const std::exception& e) {
        result.success = false;
        result.message = e.what();
    }

    return result;
}

Path3 AirMovePlanner::plan(const Vec3& start, const Vec3& goal, const CollisionWorld& world) const {
    PlanningRequest request;
    request.start.position = start;
    request.goal.position = goal;
    request.planning_time = config_.planning_time_limit;
    auto result = plan(request, world);
    if (!result.success) {
        throw std::runtime_error(result.message);
    }
    return result.raw_path;
}

bool AirMovePlanner::isInsideWorkspace(const Vec3& point) const {
    return (point.array() >= config_.workspace_min.array()).all() &&
           (point.array() <= config_.workspace_max.array()).all();
}

Path3 AirMovePlanner::planWithOMPL(const Vec3& start,
                                   const Vec3& goal,
                                   const CollisionWorld& world,
                                   double planning_time) const {
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; ++i) {
        bounds.setLow(i, config_.workspace_min[i]);
        bounds.setHigh(i, config_.workspace_max[i]);
    }
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([&world](const ob::State* state) {
        const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
        return world.isStateValid(Vec3(rv->values[0], rv->values[1], rv->values[2]));
    });

    const double extent = std::max(1e-9, (config_.workspace_max - config_.workspace_min).norm());
    const double resolution = std::clamp(config_.validity_resolution / extent, 1e-6, 1.0);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(resolution);

    ob::ScopedState<> s(space);
    s[0] = start.x();
    s[1] = start.y();
    s[2] = start.z();
    ob::ScopedState<> g(space);
    g[0] = goal.x();
    g[1] = goal.y();
    g[2] = goal.z();
    ss.setStartAndGoalStates(s, g);

    ss.setPlanner(createPlanner(config_, ss.getSpaceInformation()));

    const auto solved = ss.solve(planning_time);
    if (!solved) {
        throw std::runtime_error("OMPL failed to find a collision-free air-move path.");
    }

    if (config_.simplify_solution) {
        ss.simplifySolution();
    }
    auto path = ss.getSolutionPath();
    path.interpolate();

    Path3 result;
    const auto state_count = path.getStateCount();
    result.reserve(static_cast<std::size_t>(state_count));
    for (unsigned int i = 0; i < state_count; ++i) {
        const auto* rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        result.emplace_back(rv->values[0], rv->values[1], rv->values[2]);
    }

    if (!isPathValid(result, world, config_.validity_resolution)) {
        throw std::runtime_error("OMPL returned a path that failed dense collision recheck.");
    }
    return result;
}

double pathLength(const Path3& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

} // namespace airmove
