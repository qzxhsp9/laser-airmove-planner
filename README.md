# laser-airmove-planner

面向模具激光加工的**单段空移避障**项目骨架。

目标路线：

```text
几何避障：OMPL + FCL
运动执行：Ruckig
轨迹表达：Bezier / B-spline
```

当前版本先实现一个最小闭环：

```text
起点 / 终点 TCP
    ↓
FCL 碰撞检测
    ↓
OMPL InformedRRT* 单段避障路径
    ↓
Catmull-Rom / B-spline 风格平滑
    ↓
Ruckig jerk-limited stop-to-stop 轨迹生成
```

## 适用场景

- 模具深腔空移避障
- 激光熔覆 / 修复前后的非工作移动
- 激光清洗、纹理加工中的安全跨越
- 三轴 TCP 空移避障原型验证

> 当前骨架先把激光头包络简化为球体。如果要用于真实设备，需要替换为喷嘴、保护镜、摆头等更完整的几何包络，并加入机床轴限位、姿态约束和安全联锁。

## 目录结构

```text
include/airmove/
  AirMovePlanner.hpp     # OMPL 单段规划接口
  CollisionWorld.hpp     # FCL 碰撞世界
  BSpline.hpp            # 轨迹几何平滑
  RuckigExecutor.hpp     # jerk-limited 运动执行轨迹
  StlMeshLoader.hpp      # ASCII STL 加载器
  Types.hpp              # 公共数据结构

src/
  AirMovePlanner.cpp
  CollisionWorld.cpp
  BSpline.cpp
  RuckigExecutor.cpp
  StlMeshLoader.cpp

examples/
  single_airmove_demo.cpp
```

## 依赖

推荐用 vcpkg：

```bash
git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh
./vcpkg/vcpkg install ompl fcl eigen3 ruckig
```

Windows PowerShell：

```powershell
git clone https://github.com/microsoft/vcpkg.git
.\vcpkg\bootstrap-vcpkg.bat
.\vcpkg\vcpkg.exe install ompl fcl eigen3 ruckig
```

## 编译

Linux/macOS：

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build -j
./build/single_airmove_demo
```

Windows：

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
.\build\Release\single_airmove_demo.exe
```

## 使用示例

```cpp
PlannerConfig config;
config.workspace_min = Vec3(-100, -100, 0);
config.workspace_max = Vec3(300, 300, 200);
config.head_radius = 8.0;
config.safety_margin = 2.0;

CollisionWorld world(config.head_radius, config.safety_margin);
world.addBoxObstacle(Vec3(100, 100, 50), Vec3(60, 60, 100));

AirMovePlanner planner(config);
PlanningRequest request;
request.start.position = Vec3(0, 0, 50);
request.goal.position = Vec3(220, 220, 50);
request.planning_time = config.planning_time_limit;
request.sample_dt = 0.004;

PlanningResult result = planner.plan(request, world);
if (result.success) {
    // result.raw_path / result.smoothed_path / result.trajectory
}
```

## 命令行工具

配置并编译时打开 `AIRMOVE_BUILD_TOOLS` 后，会生成：

```powershell
.\build\Debug\laser_airmove_plan.exe
```

使用示例：

```powershell
.\build\Debug\laser_airmove_plan.exe --config examples\simple_box_config.json --output airmove_cli_output
```

输出目录包含：

```text
raw_path.csv
smoothed_path.csv
trajectory.csv
summary.json
```

当前 JSON 配置支持：

- `box` 障碍物
- `ascii_stl` 障碍物
- workspace / tool / planning / motion_limits / request 参数

`summary.json` 会输出路径长度、轨迹时长、最小 clearance，以及速度、加速度、jerk 的各轴最大绝对值。

## Benchmark

可以批量运行一个或多个配置文件并输出 CSV 指标报告：

```powershell
.\build\Debug\laser_airmove_benchmark.exe --output benchmark_report.csv examples\simple_box_config.json
```

报告包含：

- success / wall_time_ms
- raw/smoothed waypoint count
- path length
- trajectory duration
- min/average clearance
- max curvature
- jerk integral
- max velocity / acceleration / jerk per axis

## 后续工程路线

### 1. 几何避障增强

- 支持二进制 STL / STEP / OBJ
- 使用模具和夹具真实网格
- 激光头从球体包络升级为多体包络：喷嘴、镜筒、五轴摆头
- 加入 signed distance / clearance 查询，用于路径代价优化

### 2. 轨迹平滑增强

当前是 Catmull-Rom 风格平滑，后续建议替换为：

- clamped cubic B-spline
- quintic B-spline
- Bezier blend
- 曲率约束 smoothing
- 碰撞感知 smoothing

### 3. 运动执行增强

当前 Ruckig 是 stop-to-stop 分段轨迹，简单可靠但不够快。后续建议：

- 连续速度衔接
- look-ahead
- path parameterization
- 加入 TCP 方向和五轴姿态
- 与 CNC / PLC 插补周期对齐

### 4. 工业安全

实际设备必须额外考虑：

- 软限位 / 硬限位
- 安全高度策略
- 夹具和工件热变形余量
- 急停和回退轨迹
- 控制器最大速度、加速度、jerk 限制

## 创建 GitHub 仓库

本环境没有 GitHub CLI 登录态，不能直接在 `https://github.com/qzxhsp9` 下创建远程仓库。你可以在本地执行：

```bash
cd laser-airmove-planner
git init
git add .
git commit -m "Initial laser air-move planner scaffold"

gh repo create qzxhsp9/laser-airmove-planner --public --source=. --remote=origin --push
```

没有 `gh` 时：

```bash
git remote add origin https://github.com/qzxhsp9/laser-airmove-planner.git
git branch -M main
git push -u origin main
```

需要先在 GitHub 网页端创建同名仓库。
