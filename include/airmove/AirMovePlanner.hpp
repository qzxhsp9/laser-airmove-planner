#pragma once

#include "airmove/CollisionWorld.hpp"
#include "airmove/Types.hpp"

namespace airmove {

class AirMovePlanner {
public:
    explicit AirMovePlanner(PlannerConfig config);

    // 单段空移避障：输入起点/终点 TCP 坐标，输出无碰撞折线路径。
    Path3 plan(const Vec3& start, const Vec3& goal, const CollisionWorld& world) const;

private:
    PlannerConfig config_;
};

} // namespace airmove
