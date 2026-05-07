#include "airmove/RuckigExecutor.hpp"

#include <ruckig/ruckig.hpp>

#include <array>
#include <stdexcept>

namespace airmove {
namespace {

std::array<double, 3> toArray(const Vec3& v) {
    return {v.x(), v.y(), v.z()};
}

Vec3 toVec3(const std::array<double, 3>& v) {
    return Vec3(v[0], v[1], v[2]);
}

} // namespace

RuckigExecutor::RuckigExecutor(MotionLimits limits) : limits_(std::move(limits)) {}

TimedTrajectory RuckigExecutor::generateStopToStop(const Path3& path) const {
    TimedTrajectory trajectory;
    if (path.empty()) {
        return trajectory;
    }

    ruckig::Ruckig<3> otg(limits_.control_cycle);
    double global_time = 0.0;

    for (std::size_t i = 1; i < path.size(); ++i) {
        ruckig::InputParameter<3> input;
        ruckig::OutputParameter<3> output;

        input.current_position = toArray(path[i - 1]);
        input.current_velocity = {0.0, 0.0, 0.0};
        input.current_acceleration = {0.0, 0.0, 0.0};

        input.target_position = toArray(path[i]);
        input.target_velocity = {0.0, 0.0, 0.0};
        input.target_acceleration = {0.0, 0.0, 0.0};

        input.max_velocity = toArray(limits_.max_velocity);
        input.max_acceleration = toArray(limits_.max_acceleration);
        input.max_jerk = toArray(limits_.max_jerk);

        while (true) {
            const auto result = otg.update(input, output);
            if (result == ruckig::Result::Error) {
                throw std::runtime_error("Ruckig failed to generate a jerk-limited segment.");
            }

            trajectory.push_back({
                global_time,
                toVec3(output.new_position),
                toVec3(output.new_velocity),
                toVec3(output.new_acceleration)
            });

            global_time += limits_.control_cycle;
            output.pass_to_input(input);

            if (result == ruckig::Result::Finished) {
                break;
            }
        }
    }

    return trajectory;
}

} // namespace airmove
