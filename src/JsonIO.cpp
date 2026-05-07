#include "airmove/JsonIO.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace airmove {
namespace {

using json = nlohmann::json;

std::string resolvePath(const std::filesystem::path& base_dir, const std::string& file) {
    const std::filesystem::path path(file);
    if (path.is_absolute()) {
        return path.string();
    }
    return (base_dir / path).lexically_normal().string();
}

Vec3 readVec3(const json& value, const char* name) {
    if (!value.is_array() || value.size() != 3) {
        throw std::runtime_error(std::string(name) + " must be an array with 3 numbers.");
    }
    return Vec3(value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>());
}

void readVec3IfPresent(const json& object, const char* key, Vec3& target) {
    if (object.contains(key)) {
        target = readVec3(object.at(key), key);
    }
}

double readDouble(const json& object, const char* key, double fallback) {
    return object.contains(key) ? object.at(key).get<double>() : fallback;
}

int readInt(const json& object, const char* key, int fallback) {
    return object.contains(key) ? object.at(key).get<int>() : fallback;
}

bool readBool(const json& object, const char* key, bool fallback) {
    return object.contains(key) ? object.at(key).get<bool>() : fallback;
}

std::string readString(const json& object, const char* key, const std::string& fallback) {
    return object.contains(key) ? object.at(key).get<std::string>() : fallback;
}

Pose3 readPose(const json& object, const char* name) {
    if (!object.contains("xyz")) {
        throw std::runtime_error(std::string(name) + " requires xyz.");
    }

    Pose3 pose;
    pose.position = readVec3(object.at("xyz"), "xyz");
    return pose;
}

json vecToJson(const Vec3& value) {
    return json::array({value.x(), value.y(), value.z()});
}

bool isPositiveVec(const Vec3& value) {
    return (value.array() > 0.0).all();
}

void requirePositive(double value, const std::string& name) {
    if (value <= 0.0) {
        throw std::runtime_error(name + " must be positive.");
    }
}

void requireNonNegative(double value, const std::string& name) {
    if (value < 0.0) {
        throw std::runtime_error(name + " must be non-negative.");
    }
}

void validateProblem(const PlanningProblem& problem) {
    const auto& config = problem.planner_config;
    if ((config.workspace_max.array() <= config.workspace_min.array()).any()) {
        throw std::runtime_error("workspace.max must be greater than workspace.min on all axes.");
    }
    requirePositive(config.head_radius, "tool.head_radius");
    requireNonNegative(config.safety_margin, "planning.safety_margin");
    requirePositive(config.planning_time_limit, "planning.planning_time");
    requirePositive(config.validity_resolution, "planning.validity_resolution");
    if (config.smoothing_samples <= 0) {
        throw std::runtime_error("planning.smoothing_samples must be positive.");
    }
    requireNonNegative(config.planner_range, "planning.range");
    if (config.planner_goal_bias < 0.0 || config.planner_goal_bias > 1.0) {
        throw std::runtime_error("planning.goal_bias must be in [0, 1].");
    }

    const auto& limits = problem.request.limits;
    if (!isPositiveVec(limits.max_velocity) ||
        !isPositiveVec(limits.max_acceleration) ||
        !isPositiveVec(limits.max_jerk)) {
        throw std::runtime_error("motion_limits values must be positive on all axes.");
    }
    requirePositive(problem.request.sample_dt, "motion_limits.sample_dt");

    for (const auto& box : problem.box_obstacles) {
        if (!isPositiveVec(box.size)) {
            throw std::runtime_error("box obstacle size must be positive on all axes.");
        }
    }
    for (const auto& sphere : problem.sphere_obstacles) {
        requirePositive(sphere.radius, "sphere obstacle radius");
    }
    for (const auto& cylinder : problem.cylinder_obstacles) {
        requirePositive(cylinder.radius, "cylinder obstacle radius");
        requirePositive(cylinder.height, "cylinder obstacle height");
    }
    for (const auto& mesh : problem.mesh_obstacles) {
        if (mesh.file.empty()) {
            throw std::runtime_error("mesh obstacle file must not be empty.");
        }
    }
}

} // namespace

PlanningProblem loadPlanningProblemJson(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open config file: " + path.string());
    }

    json root;
    in >> root;

    const auto base_dir = path.parent_path();
    PlanningProblem problem;

    if (root.contains("workspace")) {
        const auto& workspace = root.at("workspace");
        readVec3IfPresent(workspace, "min", problem.planner_config.workspace_min);
        readVec3IfPresent(workspace, "max", problem.planner_config.workspace_max);
    }

    if (root.contains("tool")) {
        const auto& tool = root.at("tool");
        problem.planner_config.head_radius =
            readDouble(tool, "head_radius", problem.planner_config.head_radius);
    }

    if (root.contains("planning")) {
        const auto& planning = root.at("planning");
        problem.planner_config.planner_type =
            readString(planning, "planner", problem.planner_config.planner_type);
        problem.planner_config.planner_range =
            readDouble(planning, "range", problem.planner_config.planner_range);
        problem.planner_config.planner_goal_bias =
            readDouble(planning, "goal_bias", problem.planner_config.planner_goal_bias);
        problem.planner_config.safety_margin =
            readDouble(planning, "safety_margin", problem.planner_config.safety_margin);
        problem.planner_config.planning_time_limit =
            readDouble(planning, "planning_time", problem.planner_config.planning_time_limit);
        problem.planner_config.validity_resolution =
            readDouble(planning, "validity_resolution", problem.planner_config.validity_resolution);
        problem.planner_config.smoothing_samples =
            readInt(planning, "smoothing_samples", problem.planner_config.smoothing_samples);
        problem.planner_config.simplify_solution =
            readBool(planning, "simplify_solution", problem.planner_config.simplify_solution);
    }

    problem.request.safety_margin = problem.planner_config.safety_margin;
    problem.request.planning_time = problem.planner_config.planning_time_limit;

    if (root.contains("motion_limits")) {
        const auto& limits = root.at("motion_limits");
        readVec3IfPresent(limits, "max_velocity", problem.request.limits.max_velocity);
        readVec3IfPresent(limits, "max_acceleration", problem.request.limits.max_acceleration);
        readVec3IfPresent(limits, "max_jerk", problem.request.limits.max_jerk);
        problem.request.sample_dt =
            readDouble(limits, "sample_dt", problem.request.sample_dt);
    }

    if (!root.contains("request")) {
        throw std::runtime_error("Config requires request.");
    }
    const auto& request = root.at("request");
    if (!request.contains("start") || !request.contains("goal")) {
        throw std::runtime_error("Config request requires start and goal.");
    }
    problem.request.start = readPose(request.at("start"), "request.start");
    problem.request.goal = readPose(request.at("goal"), "request.goal");

    if (root.contains("obstacles")) {
        for (const auto& obstacle : root.at("obstacles")) {
            const std::string type = obstacle.value("type", "");
            if (type == "box") {
                BoxObstacle box;
                box.center = readVec3(obstacle.at("center"), "obstacle.center");
                box.size = readVec3(obstacle.at("size"), "obstacle.size");
                problem.box_obstacles.push_back(box);
            } else if (type == "sphere") {
                SphereObstacle sphere;
                sphere.center = readVec3(obstacle.at("center"), "obstacle.center");
                sphere.radius = obstacle.at("radius").get<double>();
                problem.sphere_obstacles.push_back(sphere);
            } else if (type == "cylinder") {
                CylinderObstacle cylinder;
                cylinder.center = readVec3(obstacle.at("center"), "obstacle.center");
                cylinder.radius = obstacle.at("radius").get<double>();
                cylinder.height = obstacle.at("height").get<double>();
                problem.cylinder_obstacles.push_back(cylinder);
            } else if (type == "ascii_stl") {
                MeshObstacle mesh;
                mesh.file = resolvePath(base_dir, obstacle.at("file").get<std::string>());
                problem.mesh_obstacles.push_back(mesh);
            } else {
                throw std::runtime_error("Unsupported obstacle type: " + type);
            }
        }
    }

    validateProblem(problem);
    return problem;
}

CollisionWorld buildCollisionWorld(const PlanningProblem& problem) {
    CollisionWorld world(problem.planner_config.head_radius, problem.planner_config.safety_margin);
    for (const auto& box : problem.box_obstacles) {
        world.addBoxObstacle(box.center, box.size);
    }
    for (const auto& sphere : problem.sphere_obstacles) {
        world.addSphereObstacle(sphere.center, sphere.radius);
    }
    for (const auto& cylinder : problem.cylinder_obstacles) {
        world.addCylinderObstacle(cylinder.center, cylinder.radius, cylinder.height);
    }
    for (const auto& mesh : problem.mesh_obstacles) {
        world.addAsciiStlObstacle(mesh.file);
    }
    return world;
}

void writePathCsv(const std::filesystem::path& path, const Path3& points) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write CSV: " + path.string());
    }
    out << "index,x,y,z\n";
    for (std::size_t i = 0; i < points.size(); ++i) {
        out << i << ',' << points[i].x() << ',' << points[i].y() << ',' << points[i].z() << '\n';
    }
}

void writeTrajectoryCsv(const std::filesystem::path& path, const std::vector<TrajectorySample>& trajectory) {
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

void writeSummaryJson(const std::filesystem::path& path, const PlanningResult& result) {
    const double duration = result.trajectory.empty() ? 0.0 : result.trajectory.back().time;

    json summary{
        {"success", result.success},
        {"planner", result.planner_type},
        {"message", result.message},
        {"raw_waypoints", result.raw_path.size()},
        {"smoothed_waypoints", result.smoothed_path.size()},
        {"trajectory_samples", result.trajectory.size()},
        {"raw_path_length", result.raw_path_length},
        {"smoothed_path_length", result.smoothed_path_length},
        {"trajectory_duration", duration},
        {"min_clearance", result.min_clearance},
        {"average_clearance", result.average_clearance},
        {"max_curvature", result.max_curvature},
        {"jerk_integral", result.jerk_integral},
        {"max_velocity_abs", vecToJson(result.max_velocity_abs)},
        {"max_acceleration_abs", vecToJson(result.max_acceleration_abs)},
        {"max_jerk_abs", vecToJson(result.max_jerk_abs)},
    };

    if (!result.raw_path.empty()) {
        summary["start"] = vecToJson(result.raw_path.front());
        summary["goal"] = vecToJson(result.raw_path.back());
    }

    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write JSON: " + path.string());
    }
    out << std::setw(2) << summary << '\n';
}

void writeGCodePrototype(const std::filesystem::path& path, const Path3& path_points) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Cannot write G-code: " + path.string());
    }

    out << "(laser-airmove-planner prototype air-move path)\n";
    out << "G21\n";
    out << "G90\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& point : path_points) {
        out << "G0 X" << point.x()
            << " Y" << point.y()
            << " Z" << point.z() << '\n';
    }
    out << "M30\n";
}

void writePlanningOutputs(const std::filesystem::path& output_dir, const PlanningResult& result) {
    std::filesystem::create_directories(output_dir);
    writePathCsv(output_dir / "raw_path.csv", result.raw_path);
    writePathCsv(output_dir / "smoothed_path.csv", result.smoothed_path);
    writeTrajectoryCsv(output_dir / "trajectory.csv", result.trajectory);
    writeSummaryJson(output_dir / "summary.json", result);
    writeGCodePrototype(output_dir / "path_gcode.nc", result.smoothed_path);
}

} // namespace airmove
