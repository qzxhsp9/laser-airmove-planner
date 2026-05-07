#include "airmove/AirMovePlanner.hpp"
#include "airmove/BSpline.hpp"
#include "airmove/RuckigExecutor.hpp"

#include <iostream>

int main() {
    using namespace airmove;

    PlannerConfig config;
    config.workspace_min = Vec3(-100.0, -100.0, 0.0);
    config.workspace_max = Vec3(300.0, 300.0, 200.0);
    config.head_radius = 8.0;
    config.safety_margin = 2.0;
    config.planning_time_limit = 1.5;

    CollisionWorld world(config.head_radius, config.safety_margin);

    // 示例障碍：把模具凸起/夹具简化为一个盒体。
    world.addBoxObstacle(Vec3(100.0, 100.0, 50.0), Vec3(60.0, 60.0, 100.0));

    const Vec3 start(0.0, 0.0, 50.0);
    const Vec3 goal(220.0, 220.0, 50.0);

    try {
        AirMovePlanner planner(config);
        auto raw_path = planner.plan(start, goal, world);

        BSplineSmoother smoother;
        auto smooth_path = smoother.smooth(raw_path, config.smoothing_samples);

        MotionLimits limits;
        RuckigExecutor executor(limits);
        auto timed = executor.generateStopToStop(smooth_path);

        std::cout << "raw path points: " << raw_path.size() << '\n';
        std::cout << "smooth path points: " << smooth_path.size() << '\n';
        std::cout << "timed trajectory points: " << timed.size() << '\n';

        for (const auto& p : smooth_path) {
            std::cout << p.x() << ',' << p.y() << ',' << p.z() << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "Planning failed: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
