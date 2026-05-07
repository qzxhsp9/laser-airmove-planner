#include "airmove/AirMovePlanner.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathGeometric.h>

#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace airmove {

AirMovePlanner::AirMovePlanner(PlannerConfig config) : config_(std::move(config)) {}

Path3 AirMovePlanner::plan(const Vec3& start, const Vec3& goal, const CollisionWorld& world) const {
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

    ss.getSpaceInformation()->setStateValidityCheckingResolution(
        config_.validity_resolution / (config_.workspace_max - config_.workspace_min).norm());

    ob::ScopedState<> s(space);
    s[0] = start.x(); s[1] = start.y(); s[2] = start.z();
    ob::ScopedState<> g(space);
    g[0] = goal.x(); g[1] = goal.y(); g[2] = goal.z();
    ss.setStartAndGoalStates(s, g);

    auto planner = std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    const auto solved = ss.solve(config_.planning_time_limit);
    if (!solved) {
        throw std::runtime_error("OMPL failed to find a collision-free air-move path.");
    }

    ss.simplifySolution();
    auto path = ss.getSolutionPath();
    path.interpolate();

    Path3 result;
    result.reserve(path.getStateCount());
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        result.emplace_back(rv->values[0], rv->values[1], rv->values[2]);
    }
    return result;
}

} // namespace airmove
