#pragma once

#include <Eigen/Core>
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

    // 激光头/喷嘴简化为球体包络，用于保守碰撞检测。
    double head_radius = 8.0;

    // 与模具/夹具的安全距离。
    double safety_margin = 2.0;

    // OMPL 单段规划最大时间，单位秒。
    double planning_time_limit = 2.0;

    // 路径离散检查步长，单位同输入模型。
    double validity_resolution = 2.0;

    // 平滑后采样点数。
    int smoothing_samples = 80;
};

struct MotionLimits {
    Vec3 max_velocity{800.0, 800.0, 300.0};
    Vec3 max_acceleration{3000.0, 3000.0, 1200.0};
    Vec3 max_jerk{20000.0, 20000.0, 8000.0};
    double control_cycle = 0.001;
};

using Path3 = std::vector<Vec3>;

} // namespace airmove
