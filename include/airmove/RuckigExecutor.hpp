#pragma once

#include "airmove/Types.hpp"

namespace airmove {

struct TimedPoint {
    double time{0.0};
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{0.0, 0.0, 0.0};
    Vec3 acceleration{0.0, 0.0, 0.0};
};

using TimedTrajectory = std::vector<TimedPoint>;

class RuckigExecutor {
public:
    explicit RuckigExecutor(MotionLimits limits);

    // 对路径相邻点逐段生成 jerk-limited S 曲线轨迹。
    // 初版按 stop-to-stop 处理，后续可升级为连续速度衔接。
    TimedTrajectory generateStopToStop(const Path3& path) const;

private:
    MotionLimits limits_;
};

} // namespace airmove
