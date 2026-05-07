#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <filesystem>
#include <iostream>

int main() {
    using namespace airmove;

    PlannerConfig config;
    config.workspace_min = Vec3(-100.0, -100.0, 0.0);
    config.workspace_max = Vec3(300.0, 300.0, 200.0);
    config.head_radius = 8.0;
    config.safety_margin = 2.0;
    config.planning_time_limit = 1.5;
    config.validity_resolution = 2.0;
    config.smoothing_samples = 80;

    CollisionWorld world(config.head_radius, config.safety_margin);
    world.addBoxObstacle(Vec3(100.0, 100.0, 50.0), Vec3(60.0, 60.0, 100.0));

    PlanningRequest request;
    request.start.position = Vec3(0.0, 0.0, 50.0);
    request.goal.position = Vec3(220.0, 220.0, 50.0);
    request.planning_time = config.planning_time_limit;
    request.safety_margin = config.safety_margin;
    request.sample_dt = 0.004;

    AirMovePlanner planner(config);
    const PlanningResult result = planner.plan(request, world);
    if (!result.success) {
        std::cerr << "Planning failed: " << result.message << '\n';
        return 1;
    }

    const std::filesystem::path output_dir = "airmove_demo_output";
    writePlanningOutputs(output_dir, result);

    std::cout << "Planning succeeded\n";
    std::cout << "raw path points: " << result.raw_path.size() << '\n';
    std::cout << "smooth path points: " << result.smoothed_path.size() << '\n';
    std::cout << "timed trajectory points: " << result.trajectory.size() << '\n';
    std::cout << "min clearance: " << result.min_clearance << '\n';
    std::cout << "output: " << output_dir.string() << '\n';
    return 0;
}
