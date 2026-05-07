#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void printUsage() {
    std::cout << "Usage:\n"
              << "  laser_airmove_plan --config <config.json> --output <output_dir>\n";
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path config_path;
    std::filesystem::path output_dir = "airmove_output";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            config_path = argv[++i];
        } else if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << '\n';
            printUsage();
            return 2;
        }
    }

    if (config_path.empty()) {
        std::cerr << "Missing required --config argument.\n";
        printUsage();
        return 2;
    }

    try {
        const auto problem = airmove::loadPlanningProblemJson(config_path);
        auto world = airmove::buildCollisionWorld(problem);
        airmove::AirMovePlanner planner(problem.planner_config);
        const auto result = planner.plan(problem.request, world);

        airmove::writePlanningOutputs(output_dir, result);
        if (!result.success) {
            std::cerr << "Planning failed: " << result.message << '\n';
            return 1;
        }

        std::cout << "Planning succeeded\n";
        std::cout << "raw path points: " << result.raw_path.size() << '\n';
        std::cout << "smooth path points: " << result.smoothed_path.size() << '\n';
        std::cout << "trajectory samples: " << result.trajectory.size() << '\n';
        std::cout << "min clearance: " << result.min_clearance << '\n';
        std::cout << "output: " << output_dir.string() << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
