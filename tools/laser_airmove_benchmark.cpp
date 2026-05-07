#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cout << "Usage:\n"
              << "  laser_airmove_benchmark --output <report.csv> <config1.json> [config2.json ...]\n";
}

std::string csvEscape(const std::string& value) {
    std::string result;
    result.reserve(value.size() + 2);
    result.push_back('"');
    for (const char c : value) {
        if (c == '"') {
            result.push_back('"');
        }
        result.push_back(c);
    }
    result.push_back('"');
    return result;
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path output_path = "benchmark_report.csv";
    std::vector<std::filesystem::path> configs;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else {
            configs.emplace_back(arg);
        }
    }

    if (configs.empty()) {
        std::cerr << "At least one config file is required.\n";
        printUsage();
        return 2;
    }

    std::ofstream report(output_path);
    if (!report) {
        std::cerr << "Cannot write benchmark report: " << output_path.string() << '\n';
        return 1;
    }

    report << "config,planner,success,wall_time_ms,message,raw_waypoints,smoothed_waypoints,"
           << "trajectory_samples,raw_path_length,smoothed_path_length,trajectory_duration,"
           << "min_clearance,average_clearance,max_curvature,jerk_integral,"
           << "max_vx,max_vy,max_vz,max_ax,max_ay,max_az,max_jx,max_jy,max_jz\n";

    int success_count = 0;
    for (const auto& config : configs) {
        airmove::PlanningResult result;
        std::string planner_type;
        const auto begin = std::chrono::steady_clock::now();

        try {
            const auto problem = airmove::loadPlanningProblemJson(config);
            planner_type = problem.planner_config.planner_type;
            auto world = airmove::buildCollisionWorld(problem);
            airmove::AirMovePlanner planner(problem.planner_config);
            result = planner.plan(problem.request, world);
        } catch (const std::exception& e) {
            result.success = false;
            result.message = e.what();
        }

        const auto end = std::chrono::steady_clock::now();
        const auto wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        const double duration = result.trajectory.empty() ? 0.0 : result.trajectory.back().time;
        if (result.success) {
            ++success_count;
        }

        const std::string report_planner = !result.planner_type.empty() ? result.planner_type : planner_type;
        report << csvEscape(config.string()) << ','
               << csvEscape(report_planner) << ','
               << (result.success ? 1 : 0) << ','
               << wall_time_ms << ','
               << csvEscape(result.message) << ','
               << result.raw_path.size() << ','
               << result.smoothed_path.size() << ','
               << result.trajectory.size() << ','
               << result.raw_path_length << ','
               << result.smoothed_path_length << ','
               << duration << ','
               << result.min_clearance << ','
               << result.average_clearance << ','
               << result.max_curvature << ','
               << result.jerk_integral << ','
               << result.max_velocity_abs.x() << ','
               << result.max_velocity_abs.y() << ','
               << result.max_velocity_abs.z() << ','
               << result.max_acceleration_abs.x() << ','
               << result.max_acceleration_abs.y() << ','
               << result.max_acceleration_abs.z() << ','
               << result.max_jerk_abs.x() << ','
               << result.max_jerk_abs.y() << ','
               << result.max_jerk_abs.z() << '\n';
    }

    std::cout << "Benchmark finished: " << success_count << '/' << configs.size() << " succeeded\n";
    std::cout << "report: " << output_path.string() << '\n';
    return success_count == static_cast<int>(configs.size()) ? 0 : 1;
}
