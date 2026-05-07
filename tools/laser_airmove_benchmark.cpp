#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct BenchmarkMetadata {
    std::string scene;
    std::string category;
    std::optional<bool> expected_success;
};

void printUsage() {
    std::cout << "Usage:\n"
              << "  laser_airmove_benchmark --output <report.csv> <config1.json> [config2.json ...]\n";
}

bool containsWildcard(const std::string& value) {
    return value.find('*') != std::string::npos || value.find('?') != std::string::npos;
}

std::string wildcardToRegex(const std::string& pattern) {
    std::string result;
    result.reserve(pattern.size() * 2);
    result.push_back('^');
    for (const char c : pattern) {
        switch (c) {
        case '*':
            result += ".*";
            break;
        case '?':
            result.push_back('.');
            break;
        case '.':
        case '\\':
        case '+':
        case '^':
        case '$':
        case '(':
        case ')':
        case '[':
        case ']':
        case '{':
        case '}':
        case '|':
            result.push_back('\\');
            result.push_back(c);
            break;
        default:
            result.push_back(c);
            break;
        }
    }
    result.push_back('$');
    return result;
}

std::vector<std::filesystem::path> expandConfigArg(const std::string& arg) {
    if (!containsWildcard(arg)) {
        return {std::filesystem::path(arg)};
    }

    const std::filesystem::path pattern_path(arg);
    const auto dir = pattern_path.has_parent_path() ? pattern_path.parent_path() : std::filesystem::path(".");
    const auto filename_pattern = pattern_path.filename().string();
    const std::regex matcher(wildcardToRegex(filename_pattern), std::regex::icase);

    std::vector<std::filesystem::path> result;
    if (!std::filesystem::exists(dir)) {
        return result;
    }
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (std::regex_match(entry.path().filename().string(), matcher)) {
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
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

BenchmarkMetadata loadMetadata(const std::filesystem::path& config_path) {
    BenchmarkMetadata metadata;
    metadata.scene = config_path.stem().string();
    metadata.category = config_path.parent_path().filename().string();

    std::ifstream in(config_path);
    if (!in) {
        return metadata;
    }

    nlohmann::json root;
    in >> root;
    if (!root.contains("benchmark")) {
        return metadata;
    }

    const auto& benchmark = root.at("benchmark");
    metadata.scene = benchmark.value("scene", metadata.scene);
    metadata.category = benchmark.value("category", metadata.category);
    if (benchmark.contains("expected_success")) {
        metadata.expected_success = benchmark.at("expected_success").get<bool>();
    }
    return metadata;
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
            auto expanded = expandConfigArg(arg);
            configs.insert(configs.end(), expanded.begin(), expanded.end());
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

    report << "config,scene,category,planner,expected_success,success,outcome_matched,"
           << "wall_time_ms,message,raw_waypoints,shortcut_waypoints,smoothed_waypoints,"
           << "trajectory_samples,raw_path_length,shortcut_path_length,smoothed_path_length,"
           << "trajectory_duration,smoothing_used,smoothing_fallback,smoothing_message,"
           << "min_clearance,average_clearance,max_curvature,jerk_integral,"
           << "max_vx,max_vy,max_vz,max_ax,max_ay,max_az,max_jx,max_jy,max_jz\n";

    int success_count = 0;
    int matched_count = 0;
    for (const auto& config : configs) {
        airmove::PlanningResult result;
        const auto metadata = loadMetadata(config);
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
        const bool expected_success = metadata.expected_success.value_or(true);
        const bool outcome_matched = result.success == expected_success;
        if (outcome_matched) {
            ++matched_count;
        }

        const std::string report_planner = !result.planner_type.empty() ? result.planner_type : planner_type;
        report << csvEscape(config.string()) << ','
               << csvEscape(metadata.scene) << ','
               << csvEscape(metadata.category) << ','
               << csvEscape(report_planner) << ','
               << (expected_success ? 1 : 0) << ','
               << (result.success ? 1 : 0) << ','
               << (outcome_matched ? 1 : 0) << ','
               << wall_time_ms << ','
               << csvEscape(result.message) << ','
               << result.raw_path.size() << ','
               << result.shortcut_path.size() << ','
               << result.smoothed_path.size() << ','
               << result.trajectory.size() << ','
               << result.raw_path_length << ','
               << result.shortcut_path_length << ','
               << result.smoothed_path_length << ','
               << duration << ','
               << (result.smoothing_used ? 1 : 0) << ','
               << (result.smoothing_fallback ? 1 : 0) << ','
               << csvEscape(result.smoothing_message) << ','
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

    std::cout << "Benchmark finished: " << success_count << '/' << configs.size() << " succeeded, "
              << matched_count << '/' << configs.size() << " matched expectation\n";
    std::cout << "report: " << output_path.string() << '\n';
    return matched_count == static_cast<int>(configs.size()) ? 0 : 1;
}
