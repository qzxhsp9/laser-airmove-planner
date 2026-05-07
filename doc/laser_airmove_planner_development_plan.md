# Laser Air-Move Planner 开发计划

## 1. 项目目标

本项目面向模具激光加工中的**单段非工作空移避障**场景，目标是在给定起点、终点、模具/夹具几何模型、激光头几何包络和运动学约束的条件下，生成一条：

- 几何无碰撞；
- 满足安全距离；
- 路径尽量短；
- 曲率连续或近似连续；
- 可由运动控制器执行；
- 满足速度、加速度、jerk 限制；
- 可导出给 CNC / 机器人 / 激光加工系统使用；

的单段空移轨迹。

当前技术路线：

```text
几何避障：OMPL + FCL
运动执行：Ruckig
轨迹表达：Bezier / B-spline
```

---

## 2. 适用场景边界

### 2.1 当前阶段只覆盖

- 单段空移：从一个 TCP 位姿移动到另一个 TCP 位姿；
- 非工作状态：激光关闭，不考虑加工能量分布；
- 静态障碍：模具、夹具、工作台、禁行区在规划期间不变化；
- 三轴或简化五轴几何避障；
- 离线规划优先，后续再扩展在线重规划；
- 输入模型以 STL / mesh / primitive obstacle 为主。

### 2.2 暂不覆盖

- 全局切割顺序优化；
- 多段空移调度；
- 激光加工热变形反馈；
- 视觉/传感器在线地图更新；
- 工艺能量模型；
- 完整 CNC 控制器替代；
- 多机器人协同避障。

---

## 3. 总体系统架构

```text
输入层
  ├─ 起点/终点 TCP 位姿
  ├─ 模具/夹具 STL 模型
  ├─ 激光头几何包络
  ├─ 安全距离参数
  ├─ 运动学约束 vmax/amax/jmax
  └─ 规划参数

几何建模层
  ├─ STL mesh loader
  ├─ FCL collision object
  ├─ 激光头 capsule/cylinder/box 模型
  ├─ 安全膨胀模型
  └─ workspace boundary

路径规划层
  ├─ OMPL state space
  ├─ state validity checker
  ├─ motion validity checker
  ├─ RRTConnect / RRT* / InformedRRT*
  └─ path simplification

轨迹表达层
  ├─ polyline path
  ├─ Bezier smoothing
  ├─ B-spline fitting
  ├─ curvature estimation
  └─ path sampling

轨迹时间参数化层
  ├─ Ruckig point-to-point motion
  ├─ segment-wise jerk-limited profile
  ├─ velocity/acceleration continuity
  └─ trajectory sampling

输出层
  ├─ JSON trajectory
  ├─ CSV sampled trajectory
  ├─ G-code prototype
  ├─ visualization data
  └─ debug report
```

---

## 4. 开发里程碑

## Milestone 0：项目基础设施完善

### 目标

让项目具备稳定的构建、测试和示例运行能力。

### 任务

1. 完善 CMake 工程结构；
2. 固定 C++ 标准，建议 C++17 或 C++20；
3. 引入依赖管理方案：
   - 优先 vcpkg；
   - 可选 Conan；
4. 增加基础 CI：
   - Ubuntu；
   - Windows；
5. 增加单元测试框架：
   - Catch2 或 GoogleTest；
6. 增加 examples；
7. 增加 docs；
8. 增加 clang-format；
9. 增加基础日志库。

### 验收标准

- `cmake --build` 可成功；
- 示例程序可运行；
- 单元测试可运行；
- README 中有完整构建说明；
- GitHub Actions 可通过。

---

## Milestone 1：基础数据结构与坐标系统

### 目标

统一项目内部的几何、路径、轨迹和运动约束表达。

### 关键数据结构

#### Pose

```cpp
struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
```

#### Waypoint

```cpp
struct Waypoint {
    Pose tcp_pose;
    double clearance;
};
```

#### MotionLimits

```cpp
struct MotionLimits {
    Eigen::Vector3d max_velocity;
    Eigen::Vector3d max_acceleration;
    Eigen::Vector3d max_jerk;
};
```

#### PlanningRequest

```cpp
struct PlanningRequest {
    Pose start;
    Pose goal;
    MotionLimits limits;
    double safety_margin;
    double planning_time;
};
```

#### PlanningResult

```cpp
struct PlanningResult {
    bool success;
    std::vector<Pose> raw_path;
    std::vector<Pose> smoothed_path;
    std::vector<TrajectorySample> trajectory;
    std::string message;
};
```

### 坐标系统约定

必须明确：

- 世界坐标系 `World`；
- 模具坐标系 `Mold`；
- 机床坐标系 `Machine`；
- 激光头 TCP 坐标系 `Tool`；
- STL 模型单位，建议统一为 mm；
- OMPL 内部单位，建议也使用 mm；
- Ruckig 输入单位必须与路径单位一致。

### 验收标准

- 所有模块只通过统一数据结构通信；
- 单元测试覆盖 Pose、Transform、MotionLimits；
- README 中说明单位和坐标系。

---

## Milestone 2：FCL 碰撞世界

### 目标

建立可复用的几何碰撞检测模块。

### 核心功能

1. 加载 STL 模具；
2. 加载夹具模型；
3. 支持基础几何障碍：
   - box；
   - sphere；
   - cylinder；
   - capsule 近似；
4. 支持激光头几何包络；
5. 支持安全距离；
6. 支持点位碰撞检测；
7. 支持路径段碰撞检测；
8. 支持最小距离查询。

### 模块接口建议

```cpp
class CollisionWorld {
public:
    bool loadMeshObstacle(const std::string& path, const Transform& tf);
    void addBoxObstacle(const Box& box, const Transform& tf);
    void setToolGeometry(const ToolGeometry& tool);
    void setSafetyMargin(double margin);

    bool isStateValid(const Pose& pose) const;
    bool isSegmentValid(const Pose& a, const Pose& b, double resolution) const;
    double distanceToNearestObstacle(const Pose& pose) const;
};
```

### 关键实现点

- FCL BVHModel 构建；
- STL 三角面片转换；
- Tool collision object 动态更新位姿；
- 安全距离既可以通过模型膨胀实现，也可以通过 distance query 实现；
- segment validity 不能只检查端点，必须按分辨率采样。

### 验收标准

- 简单 box obstacle 测试通过；
- STL obstacle 测试通过；
- 激光头从障碍物旁经过时能正确判断安全/碰撞；
- distance query 可返回合理距离。

---

## Milestone 3：OMPL 单段路径规划

### 目标

基于 OMPL 实现 start → goal 的几何避障路径规划。

### 规划空间

第一阶段建议实现两种模式：

#### 3D 点位模式

```text
State = x, y, z
```

适合三轴空移。

#### SE3 简化模式

```text
State = x, y, z, qx, qy, qz, qw
```

适合带姿态的激光头或五轴加工。

### Planner 优先级

建议依次实现：

1. StraightLinePlanner：先尝试直线；
2. RRTConnect：快速可行解；
3. RRTstar：优化路径质量；
4. InformedRRTstar：有初解后加速优化；
5. PRM：静态环境多次查询时使用。

### 核心流程

```text
1. 检查 start 是否有效
2. 检查 goal 是否有效
3. 尝试直线连接
4. 直线失败，调用 OMPL planner
5. 得到 raw path
6. path simplification
7. 输出 waypoints
```

### 模块接口建议

```cpp
class AirMovePlanner {
public:
    PlanningResult plan(const PlanningRequest& request);

private:
    bool tryStraightLine(...);
    bool planWithOMPL(...);
    std::vector<Pose> simplifyPath(...);
};
```

### 参数建议

```yaml
planner:
  type: InformedRRTstar
  planning_time: 1.0
  range: 20.0
  goal_bias: 0.05
  collision_check_resolution: 2.0
  simplify: true
```

### 验收标准

- 无障碍时返回直线路径；
- 有障碍时返回绕障路径；
- start/goal 在障碍物内时返回失败；
- planning_time 限制有效；
- raw path 每个点均无碰撞；
- 相邻路径段采样后无碰撞。

---

## Milestone 4：路径平滑与轨迹表达

### 目标

把 OMPL 输出的折线路径转为适合运动执行的平滑路径。

### 第一阶段推荐

- Catmull-Rom spline；
- cubic B-spline；
- quintic Bezier 局部平滑。

### 路径平滑必须满足

- 不穿越障碍物；
- 与原始路径偏差受限；
- 曲率不超过设定阈值；
- 起点终点保持不变；
- 可采样输出。

### 核心问题

OMPL 的路径通常是折线：

```text
P0 -> P1 -> P2 -> P3
```

直接执行会导致：

- 速度不连续；
- 加速度突变；
- jerk 冲击；
- 拐角处必须急减速。

所以需要转换为：

```text
smooth curve s(u), u ∈ [0, 1]
```

### 接口建议

```cpp
class PathSmoother {
public:
    SmoothPath smooth(const std::vector<Pose>& raw_path,
                      const CollisionWorld& collision_world,
                      const SmoothOptions& options);
};
```

### 平滑流程

```text
1. 输入 raw path
2. 删除冗余点
3. 对折线拐角做 shortcut
4. 拟合 B-spline
5. 按固定步长采样
6. 碰撞检测
7. 若碰撞，降低平滑强度或回退局部折线
8. 输出 smoothed path
```

### 验收标准

- 平滑后路径比 raw path 点数更多但曲率更连续；
- 平滑后不碰撞；
- 能导出 CSV 可视化；
- 在典型 box obstacle 场景下路径可绕过障碍且无尖角。

---

## Milestone 5：Ruckig jerk-limited 运动执行层

### 目标

将几何路径转为带时间戳的轨迹样本。

### 第一阶段实现策略

先做 segment-wise Ruckig：

```text
waypoint[i] -> waypoint[i+1]
```

每段使用 Ruckig 生成 jerk-limited 轨迹。

### 后续优化

segment-wise 的问题是段间可能需要降速甚至停顿。后续应升级为：

- 中间点速度连续；
- 曲线弧长参数化；
- TOPP-RA 或自研路径速度规划；
- Ruckig tracking interface 或 Pro 版本能力替代。

### 接口建议

```cpp
class RuckigExecutor {
public:
    std::vector<TrajectorySample> generate(
        const std::vector<Pose>& path,
        const MotionLimits& limits,
        double sample_dt);
};
```

### TrajectorySample

```cpp
struct TrajectorySample {
    double time;
    Pose pose;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d jerk;
};
```

### 验收标准

- 输出轨迹时间单调递增；
- 每个采样点速度不超过 vmax；
- 每个采样点加速度不超过 amax；
- 每个采样点 jerk 不超过 jmax；
- 可以导出 CSV；
- 可画出 x/y/z 位置、速度、加速度曲线。

---

## Milestone 6：输入输出与配置系统

### 目标

让项目从 demo 变成可以被实际业务调用的工具。

### 输入格式

建议支持 YAML / JSON：

```yaml
workspace:
  unit: mm
  bounds:
    x: [-500, 500]
    y: [-500, 500]
    z: [0, 600]

obstacles:
  - type: mesh
    file: ./data/mold.stl
    transform:
      xyz: [0, 0, 0]
      rpy: [0, 0, 0]

laser_head:
  type: cylinder
  radius: 25
  height: 120
  tcp_offset: [0, 0, -100]

planning:
  planner: InformedRRTstar
  planning_time: 1.0
  safety_margin: 5.0
  resolution: 2.0

motion_limits:
  max_velocity: [500, 500, 300]
  max_acceleration: [3000, 3000, 2000]
  max_jerk: [20000, 20000, 10000]

request:
  start:
    xyz: [0, 0, 200]
    rpy: [0, 0, 0]
  goal:
    xyz: [200, 100, 150]
    rpy: [0, 0, 0]
```

### 输出格式

#### JSON summary

```json
{
  "success": true,
  "path_length": 384.2,
  "trajectory_duration": 1.82,
  "min_clearance": 8.6,
  "raw_waypoints": 12,
  "smoothed_waypoints": 120
}
```

#### CSV trajectory

```csv
time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz
0.000,0,0,200,0,0,0,0,0,0,0,0,0
```

### 验收标准

- 可通过配置文件运行规划；
- 可导出 summary.json；
- 可导出 raw_path.csv；
- 可导出 smoothed_path.csv；
- 可导出 trajectory.csv。

---

## Milestone 7：可视化与调试工具

### 目标

提高开发和调参效率。

### 第一阶段

使用 Python / matplotlib / plotly：

- 绘制 raw path；
- 绘制 smoothed path；
- 绘制障碍物 bounding box；
- 绘制速度曲线；
- 绘制加速度曲线；
- 绘制 jerk 曲线。

### 第二阶段

可选：

- Open3D 可视化 STL + path；
- MeshCat；
- RViz，如果后续接 ROS；
- Qt 小工具，如果项目需要桌面端集成。

### 验收标准

- demo 运行后自动生成可视化图片；
- 能直观看到路径是否穿模；
- 能看到 jerk 是否超限；
- 能比较 raw path 和 smoothed path。

---

## Milestone 8：算法质量指标与 Benchmark

### 目标

建立可量化评价体系，避免只凭肉眼判断路径好坏。

### 指标

#### 几何指标

- path length；
- raw waypoint count；
- smoothed path length；
- minimum clearance；
- average clearance；
- maximum curvature；
- collision check count；
- planning time。

#### 动力学指标

- trajectory duration；
- max velocity；
- max acceleration；
- max jerk；
- jerk integral；
- number of stop points；
- segment transition smoothness。

#### 工程指标

- success rate；
- average planning time；
- worst-case planning time；
- parameter sensitivity；
- memory usage。

### Benchmark 场景

1. 无障碍直线场景；
2. 单个 box obstacle；
3. 狭窄通道；
4. 深腔模具简化模型；
5. 多夹具场景；
6. 起点/终点靠近障碍物；
7. start invalid；
8. goal invalid；
9. 无可行路径。

### 验收标准

- 每个 benchmark 可重复运行；
- 输出统一报告；
- 能比较不同 planner 的表现；
- 能比较不同 smoothing 参数的表现。

---

## Milestone 9：工程接口封装

### 目标

将项目封装为可被外部系统调用的库或服务。

### 形态一：C++ Library

```cpp
AirMovePlanner planner;
planner.loadConfig("config.yaml");
auto result = planner.plan(request);
```

### 形态二：Command Line Tool

```bash
laser_airmove_plan --config config.yaml --output ./out
```

### 形态三：REST Service，可选

```http
POST /plan_airmove
```

用于后续和上位机、MES 或 CAM 系统集成。

### 验收标准

- C++ API 稳定；
- CLI 可运行；
- 错误码清晰；
- 输入参数校验完整；
- 输出结果可被后续系统解析。

---

## Milestone 10：面向模具激光业务的增强

### 目标

加入模具激光加工中特有的业务约束。

### 业务约束

1. 激光头安全高度；
2. 喷嘴与模具表面的最小安全距离；
3. 深腔入口约束；
4. 五轴摆角限制；
5. TCP 姿态保持；
6. 禁止穿越已加工区域；
7. 避免路径过低导致飞溅污染；
8. 空移尽量减少 Z 轴大幅升降；
9. 某些区域只能从特定方向进入。

### 可实现功能

- preferred height cost；
- clearance cost；
- vertical lift penalty；
- orientation change penalty；
- workspace soft boundary；
- forbidden volume；
- approach corridor。

### 验收标准

- 支持禁行区配置；
- 支持最小安全高度配置；
- 支持姿态变化权重；
- 支持路径代价函数扩展。

---

## 5. 推荐开发优先级

### P0：必须先做

1. 构建系统稳定；
2. 基础数据结构；
3. FCL 碰撞检测；
4. OMPL 直线失败后的避障规划；
5. raw path 输出；
6. CSV 可视化。

### P1：核心能力

1. B-spline 平滑；
2. 平滑后碰撞回检；
3. Ruckig jerk-limited 轨迹；
4. YAML 配置；
5. benchmark。

### P2：工程增强

1. STL 大模型性能优化；
2. distance query；
3. clearance cost；
4. 多 planner 对比；
5. JSON summary；
6. CLI 工具。

### P3：高级能力

1. SE3 / 五轴规划；
2. 姿态约束；
3. TrajOpt / CHOMP 风格优化；
4. SDF 距离场；
5. 在线重规划。

---

## 6. 关键技术风险

## 风险 1：OMPL 找到的路径不可执行

### 原因

OMPL 输出的是几何路径，不考虑速度、加速度、jerk。

### 对策

- 路径平滑；
- 曲率约束；
- Ruckig 时间参数化；
- benchmark 中增加 jerk 指标。

---

## 风险 2：B-spline 平滑后发生碰撞

### 原因

平滑曲线可能偏离原始安全路径。

### 对策

- 平滑后必须密集采样碰撞检测；
- 控制最大偏移；
- 对危险区域降低平滑强度；
- 必要时回退原始折线。

---

## 风险 3：FCL 大 STL 模型性能不足

### 原因

模具 STL 面片多，频繁碰撞检测会变慢。

### 对策

- 模型简化；
- BVH 缓存；
- voxel / SDF 加速；
- 使用 bounding volume 粗筛；
- 分区加载。

---

## 风险 4：安全距离处理不准确

### 原因

碰撞检测只判断是否相交，不一定保证 clearance。

### 对策

- 使用 distance query；
- 对 tool 或 obstacle 做膨胀；
- 每个采样点记录 min clearance；
- 安全距离作为路径代价。

---

## 风险 5：三轴和五轴需求混杂

### 原因

三轴规划只需 XYZ，五轴需要姿态和机构约束。

### 对策

- 第一阶段明确只做 XYZ；
- 代码接口保留 Pose；
- SE3 模块单独实现；
- 不要一开始把五轴复杂度引入主线。

---

## 7. 建议目录结构

```text
laser-airmove-planner/
├─ CMakeLists.txt
├─ vcpkg.json
├─ README.md
├─ docs/
│  ├─ architecture.md
│  ├─ coordinate_system.md
│  ├─ collision_world.md
│  ├─ planner.md
│  └─ trajectory.md
├─ include/airmove/
│  ├─ types/
│  ├─ geometry/
│  ├─ collision/
│  ├─ planning/
│  ├─ smoothing/
│  ├─ trajectory/
│  ├─ io/
│  └─ benchmark/
├─ src/
│  ├─ geometry/
│  ├─ collision/
│  ├─ planning/
│  ├─ smoothing/
│  ├─ trajectory/
│  ├─ io/
│  └─ benchmark/
├─ examples/
├─ tests/
├─ scripts/
├─ data/
│  ├─ simple_box/
│  ├─ narrow_passage/
│  └─ mold_demo/
└─ tools/
   ├─ visualize_path.py
   └─ benchmark_report.py
```

---

## 8. 第一阶段 Sprint 计划

## Sprint 1：基础工程与类型系统

### 目标

项目可构建，可运行空 demo。

### 任务

- 整理 CMake；
- 添加 Eigen；
- 添加基础 types；
- 添加 config 占位；
- 添加 demo；
- 添加 tests；
- 添加格式化配置。

### 产出

- `airmove::Pose`；
- `airmove::PlanningRequest`；
- `airmove::PlanningResult`；
- `single_airmove_demo`。

---

## Sprint 2：FCL 碰撞检测

### 目标

实现基础 obstacle + tool 的碰撞检测。

### 任务

- FCL box obstacle；
- FCL mesh obstacle；
- tool cylinder；
- state validity；
- segment validity；
- collision unit tests。

### 产出

- `CollisionWorld`；
- `StlMeshLoader`；
- `collision_demo`。

---

## Sprint 3：OMPL 规划

### 目标

实现三维单段避障。

### 任务

- OMPL RealVectorStateSpace；
- workspace bounds；
- validity checker；
- RRTConnect；
- InformedRRTstar；
- path simplification；
- raw path CSV 输出。

### 产出

- `AirMovePlanner::plan()`；
- `raw_path.csv`；
- box obstacle demo。

---

## Sprint 4：B-spline 平滑

### 目标

将 raw path 转为 smooth path。

### 任务

- Catmull-Rom spline；
- B-spline sampling；
- smoothing options；
- collision recheck；
- fallback strategy；
- smooth path CSV。

### 产出

- `PathSmoother`；
- `smoothed_path.csv`；
- 可视化脚本。

---

## Sprint 5：Ruckig 轨迹生成

### 目标

生成 jerk-limited trajectory。

### 任务

- Ruckig wrapper；
- per-segment trajectory；
- trajectory sample；
- velocity/acceleration/jerk CSV；
- motion limit tests。

### 产出

- `RuckigExecutor`；
- `trajectory.csv`；
- 运动曲线图。

---

## Sprint 6：配置、CLI、Benchmark

### 目标

形成最小可用工程工具。

### 任务

- YAML config；
- CLI；
- JSON summary；
- benchmark scenes；
- benchmark report；
- README 使用说明。

### 产出

```bash
laser_airmove_plan --config config.yaml --output ./out
```

---

## 9. 最小可用版本 MVP 定义

MVP 必须能完成：

```text
输入：
  start pose
  goal pose
  box/STL obstacle
  tool geometry
  safety margin
  motion limits

输出：
  collision-free raw path
  smoothed path
  jerk-limited trajectory
  CSV/JSON report
```

### MVP 验收 demo

场景：

- 起点：障碍物一侧；
- 终点：障碍物另一侧；
- 中间放置 box obstacle；
- 激光头用 cylinder 或 capsule 近似；
- 安全距离 5 mm；
- 规划时间 1 秒；
- 轨迹采样周期 1 ms 或 4 ms。

成功标准：

- raw path 不碰撞；
- smoothed path 不碰撞；
- trajectory 不超 vmax/amax/jmax；
- 输出报告显示 min clearance；
- 可视化图能看到绕障轨迹。

---

## 10. 后续高级路线

### 10.1 从 OMPL 到 TrajOpt

当 B-spline + Ruckig 仍不能满足平滑性时，可引入：

- TrajOpt；
- CHOMP；
- STOMP；
- SDF-based optimization。

目标是直接优化：

```text
collision cost + smoothness cost + path length cost + clearance cost
```

### 10.2 从三轴到五轴

扩展内容：

- SE3 state space；
- 姿态约束；
- A/B/C 轴限位；
- TCP 姿态插值；
- 机构正逆运动学；
- 自碰撞检测。

### 10.3 从离线到在线

扩展内容：

- 动态障碍；
- 增量重规划；
- anytime planner；
- 局部避障；
- 与传感器地图集成。

---

## 11. 当前最建议立刻开始的任务

建议下一步直接进入 Sprint 1 和 Sprint 2：

1. 重构目录；
2. 完善基础类型；
3. 加入测试框架；
4. 实现 `CollisionWorld`；
5. 用 box obstacle 完成第一个可验证碰撞检测 demo；
6. 再接 OMPL。

不要一开始就做 Ruckig 或复杂 B-spline。原因是：

```text
碰撞检测是整个系统的地基。
```

如果 state validity 不可靠，后面的 OMPL、平滑、Ruckig 都没有意义。

