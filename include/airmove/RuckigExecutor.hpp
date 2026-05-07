#pragma once

#include "airmove/Types.hpp"

namespace airmove {

using TimedTrajectory = std::vector<TrajectorySample>;

class RuckigExecutor {
public:
    explicit RuckigExecutor(MotionLimits limits);

    TimedTrajectory generateStopToStop(const Path3& path) const;

private:
    MotionLimits limits_;
};

} // namespace airmove
