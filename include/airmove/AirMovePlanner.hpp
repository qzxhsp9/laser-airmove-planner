#pragma once

#include "airmove/CollisionWorld.hpp"
#include "airmove/Types.hpp"

namespace airmove {

class AirMovePlanner {
public:
    explicit AirMovePlanner(PlannerConfig config);

    PlanningResult plan(const PlanningRequest& request, const CollisionWorld& world) const;
    Path3 plan(const Vec3& start, const Vec3& goal, const CollisionWorld& world) const;

private:
    bool isInsideWorkspace(const Vec3& point) const;
    Path3 planWithOMPL(const Vec3& start,
                       const Vec3& goal,
                       const CollisionWorld& world,
                       double planning_time) const;

    PlannerConfig config_;
};

double pathLength(const Path3& path);

} // namespace airmove
