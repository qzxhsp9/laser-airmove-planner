#include "airmove/RuckigExecutor.hpp"

#include <ruckig/ruckig.hpp>

#include <array>
#include <stdexcept>
#include <utility>

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

    Pose3 initial_pose;
    initial_pose.position = path.front();
    trajectory.push_back({0.0, initial_pose, Vec3::Zero(), Vec3::Zero(), Vec3::Zero()});

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

        Vec3 previous_acceleration = toVec3(input.current_acceleration);
        while (true) {
            const auto result = otg.update(input, output);
            if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
                throw std::runtime_error("Ruckig failed to generate a jerk-limited segment.");
            }

            const Vec3 acceleration = toVec3(output.new_acceleration);
            const Vec3 jerk = (acceleration - previous_acceleration) / limits_.control_cycle;
            previous_acceleration = acceleration;

            Pose3 pose;
            pose.position = toVec3(output.new_position);
            global_time += limits_.control_cycle;
            trajectory.push_back({
                global_time,
                pose,
                toVec3(output.new_velocity),
                acceleration,
                jerk
            });

            output.pass_to_input(input);

            if (result == ruckig::Result::Finished) {
                break;
            }
        }
    }

    return trajectory;
}

} // namespace airmove
