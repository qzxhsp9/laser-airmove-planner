#include "airmove/AirMovePlanner.hpp"
#include "airmove/BSpline.hpp"
#include "airmove/JsonIO.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>

int main() {
    using namespace airmove;

    CollisionWorld world(1.0, 0.5);
    world.addBoxObstacle(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 10.0, 10.0));

    assert(!world.isStateValid(Vec3(0.0, 0.0, 0.0)));
    assert(world.isStateValid(Vec3(20.0, 0.0, 0.0)));
    assert(!world.isSegmentValid(Vec3(-20.0, 0.0, 0.0), Vec3(20.0, 0.0, 0.0), 1.0));
    assert(world.isSegmentValid(Vec3(-20.0, 20.0, 0.0), Vec3(20.0, 20.0, 0.0), 1.0));

    Path3 line{Vec3(0.0, 0.0, 0.0), Vec3(3.0, 4.0, 0.0)};
    assert(std::abs(pathLength(line) - 5.0) < 1e-9);

    Path3 polyline{
        Vec3(0.0, 0.0, 0.0),
        Vec3(1.0, 1.0, 0.0),
        Vec3(2.0, 0.0, 0.0),
    };
    auto smooth = BSplineSmoother().smooth(polyline, 12);
    assert(smooth.size() > polyline.size());
    assert((smooth.front() - polyline.front()).norm() < 1e-9);
    assert((smooth.back() - polyline.back()).norm() < 1e-9);

    const auto temp_dir = std::filesystem::temp_directory_path() / "airmove_basic_tests";
    std::filesystem::create_directories(temp_dir);
    const auto config_path = temp_dir / "simple_box_config.json";
    const auto stl_path = temp_dir / "triangle.stl";

    {
        std::ofstream out(stl_path);
        out << R"(solid triangle
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
endsolid triangle
)";
    }

    {
        std::ofstream out(config_path);
        out << R"({
  "workspace": {"min": [-10, -10, 0], "max": [20, 20, 20]},
  "tool": {"head_radius": 1.0},
  "planning": {"safety_margin": 0.5, "planning_time": 0.1, "validity_resolution": 1.0},
  "motion_limits": {
    "max_velocity": [10, 10, 10],
    "max_acceleration": [100, 100, 100],
    "max_jerk": [1000, 1000, 1000],
    "sample_dt": 0.01
  },
  "request": {
    "start": {"xyz": [-5, -5, 5]},
    "goal": {"xyz": [5, 5, 5]}
  },
  "obstacles": [
    {"type": "box", "center": [0, 0, 5], "size": [2, 2, 2]},
    {"type": "ascii_stl", "file": "triangle.stl"}
  ]
})";
    }

    const auto problem = loadPlanningProblemJson(config_path);
    assert(problem.box_obstacles.size() == 1);
    assert(problem.mesh_obstacles.size() == 1);
    assert(problem.planner_config.head_radius == 1.0);
    assert(problem.request.sample_dt == 0.01);

    const auto output_dir = temp_dir / "output";
    PlanningResult fake_result;
    fake_result.success = true;
    fake_result.message = "ok";
    fake_result.raw_path = {Vec3(0, 0, 0), Vec3(1, 0, 0)};
    fake_result.smoothed_path = fake_result.raw_path;
    fake_result.raw_path_length = 1.0;
    fake_result.smoothed_path_length = 1.0;
    fake_result.min_clearance = 2.0;
    fake_result.max_velocity_abs = Vec3(1, 2, 3);
    fake_result.max_acceleration_abs = Vec3(4, 5, 6);
    fake_result.max_jerk_abs = Vec3(7, 8, 9);
    writePlanningOutputs(output_dir, fake_result);
    assert(std::filesystem::exists(output_dir / "raw_path.csv"));
    assert(std::filesystem::exists(output_dir / "smoothed_path.csv"));
    assert(std::filesystem::exists(output_dir / "trajectory.csv"));
    assert(std::filesystem::exists(output_dir / "summary.json"));

    return 0;
}
