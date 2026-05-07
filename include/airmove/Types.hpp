#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace airmove {

using Vec3 = Eigen::Vector3d;

struct Pose3 {
    Vec3 position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct PlannerConfig {
    Vec3 workspace_min{-500.0, -500.0, 0.0};
    Vec3 workspace_max{500.0, 500.0, 500.0};

    std::string planner_type{"informed_rrtstar"};
    double planner_range{0.0};
    double planner_goal_bias{0.05};
    double head_radius{8.0};
    double safety_margin{2.0};
    double planning_time_limit{2.0};
    double validity_resolution{2.0};
    int smoothing_samples{80};
    bool simplify_solution{true};
};

struct MotionLimits {
    Vec3 max_velocity{800.0, 800.0, 300.0};
    Vec3 max_acceleration{3000.0, 3000.0, 1200.0};
    Vec3 max_jerk{20000.0, 20000.0, 8000.0};
    double control_cycle{0.001};
};

using Path3 = std::vector<Vec3>;

struct PlanningRequest {
    Pose3 start;
    Pose3 goal;
    MotionLimits limits;
    double safety_margin{2.0};
    double planning_time{2.0};
    double sample_dt{0.001};
};

struct TrajectorySample {
    double time{0.0};
    Pose3 pose;
    Vec3 velocity{0.0, 0.0, 0.0};
    Vec3 acceleration{0.0, 0.0, 0.0};
    Vec3 jerk{0.0, 0.0, 0.0};
};

struct PlanningResult {
    bool success{false};
    std::string planner_type;
    Path3 raw_path;
    Path3 smoothed_path;
    std::vector<TrajectorySample> trajectory;
    double raw_path_length{0.0};
    double smoothed_path_length{0.0};
    double min_clearance{0.0};
    double average_clearance{0.0};
    double max_curvature{0.0};
    double jerk_integral{0.0};
    Vec3 max_velocity_abs{0.0, 0.0, 0.0};
    Vec3 max_acceleration_abs{0.0, 0.0, 0.0};
    Vec3 max_jerk_abs{0.0, 0.0, 0.0};
    std::string message;
};

struct BoxObstacle {
    Vec3 center{0.0, 0.0, 0.0};
    Vec3 size{1.0, 1.0, 1.0};
};

struct SphereObstacle {
    Vec3 center{0.0, 0.0, 0.0};
    double radius{1.0};
};

struct CylinderObstacle {
    Vec3 center{0.0, 0.0, 0.0};
    double radius{1.0};
    double height{1.0};
};

struct MeshObstacle {
    std::string file;
};

struct PlanningProblem {
    PlannerConfig planner_config;
    PlanningRequest request;
    std::vector<BoxObstacle> box_obstacles;
    std::vector<SphereObstacle> sphere_obstacles;
    std::vector<CylinderObstacle> cylinder_obstacles;
    std::vector<MeshObstacle> mesh_obstacles;
};

} // namespace airmove
