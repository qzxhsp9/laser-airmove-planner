#include "airmove/AirMovePlanner.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

void writePathCsv(const std::filesystem::path& path, const airmove::Path3& points) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write CSV: " + path.string());
    }
    out << "index,x,y,z\n";
    for (std::size_t i = 0; i < points.size(); ++i) {
        out << i << ',' << points[i].x() << ',' << points[i].y() << ',' << points[i].z() << '\n';
    }
}

void writeTrajectoryCsv(const std::filesystem::path& path,
                        const std::vector<airmove::TrajectorySample>& trajectory) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write CSV: " + path.string());
    }
    out << "time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";
    for (const auto& sample : trajectory) {
        const auto& p = sample.pose.position;
        const auto& v = sample.velocity;
        const auto& a = sample.acceleration;
        const auto& j = sample.jerk;
        out << sample.time << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
            << v.x() << ',' << v.y() << ',' << v.z() << ','
            << a.x() << ',' << a.y() << ',' << a.z() << ','
            << j.x() << ',' << j.y() << ',' << j.z() << '\n';
    }
}

void writeSummaryJson(const std::filesystem::path& path, const airmove::PlanningResult& result) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write JSON: " + path.string());
    }

    const double duration = result.trajectory.empty() ? 0.0 : result.trajectory.back().time;
    out << std::fixed << std::setprecision(6)
        << "{\n"
        << "  \"success\": " << (result.success ? "true" : "false") << ",\n"
        << "  \"message\": \"" << result.message << "\",\n"
        << "  \"raw_waypoints\": " << result.raw_path.size() << ",\n"
        << "  \"smoothed_waypoints\": " << result.smoothed_path.size() << ",\n"
        << "  \"trajectory_samples\": " << result.trajectory.size() << ",\n"
        << "  \"raw_path_length\": " << result.raw_path_length << ",\n"
        << "  \"smoothed_path_length\": " << result.smoothed_path_length << ",\n"
        << "  \"trajectory_duration\": " << duration << ",\n"
        << "  \"min_clearance\": " << result.min_clearance << "\n"
        << "}\n";
}

} // namespace

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
    std::filesystem::create_directories(output_dir);
    writePathCsv(output_dir / "raw_path.csv", result.raw_path);
    writePathCsv(output_dir / "smoothed_path.csv", result.smoothed_path);
    writeTrajectoryCsv(output_dir / "trajectory.csv", result.trajectory);
    writeSummaryJson(output_dir / "summary.json", result);

    std::cout << "Planning succeeded\n";
    std::cout << "raw path points: " << result.raw_path.size() << '\n';
    std::cout << "smooth path points: " << result.smoothed_path.size() << '\n';
    std::cout << "timed trajectory points: " << result.trajectory.size() << '\n';
    std::cout << "min clearance: " << result.min_clearance << '\n';
    std::cout << "output: " << output_dir.string() << '\n';
    return 0;
}
