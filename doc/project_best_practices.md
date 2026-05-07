# Laser Air-Move Planner 项目最佳实践

本文档用于约定 `laser-airmove-planner` 的日常开发、验证、调参和交付方式。目标是让项目在 Windows + vcpkg 环境下保持可复现、可测试、可比较，并避免把构建产物或临时实验数据混入代码仓库。

## 1. 基本原则

- 优先保持工程可构建、示例可运行、测试可通过。
- 每次功能开发尽量保持改动范围清晰，避免把重构、格式化和功能变更混在一起。
- 算法参数、输入场景和输出指标都应可复现，不依赖手动操作结果。
- 规划结果不能只看“是否成功”，还要同时看路径长度、clearance、曲率、jerk、轨迹耗时和 benchmark 表现。
- 失败场景也应纳入 benchmark，用于验证错误处理是否符合预期。

## 2. 推荐开发环境

Windows 下推荐使用：

- Visual Studio 2022
- CMake 3.20+
- vcpkg manifest mode
- x64-windows triplet
- PowerShell
- Python 3.10+，用于可视化脚本

依赖安装和常见问题参考：

```text
doc/project_setup_windows_vcpkg.md
```

建议始终使用单独的构建目录，例如：

```powershell
cmake -S . -B build `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DAIRMOVE_BUILD_EXAMPLES=ON `
  -DAIRMOVE_BUILD_TOOLS=ON `
  -DAIRMOVE_BUILD_TESTS=ON
```

## 3. 分支与提交习惯

建议按任务建立短分支：

```text
feature/benchmark-scenes
feature/path-smoothing
fix/ompl-approximate-solution
doc/project-best-practices
```

提交前建议至少检查：

```powershell
git status --short --untracked-files=all
git diff --check
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

不要提交以下内容：

- `build/`
- `vcpkg_installed/`
- `.vs/`
- `airmove_*_output/`
- `visualization_output/`
- `benchmark_visualization/`
- 临时 CSV、PNG、NC 输出

如果新增 example 或 benchmark JSON，需要确认没有被 `.gitignore` 误忽略。

## 4. C++ 代码实践

### 4.1 数据结构

公共数据结构优先放在：

```text
include/airmove/Types.hpp
```

新增字段时应同步检查：

- JSON 输出是否需要写入 `summary.json`
- benchmark 报告是否需要新增列
- 测试里的 `PlanningResult` 构造是否需要补默认数据
- 可视化工具是否需要读取新字段

### 4.2 错误处理

库代码中推荐使用清晰的异常或 `PlanningResult::message` 返回错误原因。错误信息应能直接定位问题，例如：

```text
Start state is in collision or violates the safety margin.
OMPL only found an approximate path and did not reach the goal.
```

不要把 approximate solution 当作成功路径。对加工空移来说，路径没有到达 goal 就不应继续生成轨迹。

### 4.3 碰撞检查

路径相关逻辑必须做两层检查：

- 每个 waypoint 的 state validity。
- 相邻 waypoint 之间按 `validity_resolution` 做 dense segment validity。

平滑后的路径必须重新碰撞检查。如果检查失败，应回退到已验证安全的路径，例如 shortcut path 或 raw path。

### 4.4 路径后处理

推荐流程：

```text
raw path -> shortcut path -> smoothed path -> dense collision recheck -> trajectory
```

输出中应保留各阶段指标：

- `raw_waypoints`
- `shortcut_waypoints`
- `smoothed_waypoints`
- `raw_path_length`
- `shortcut_path_length`
- `smoothed_path_length`
- `smoothing_used`
- `smoothing_fallback`
- `smoothing_message`

这样可以区分“规划器产生的路径质量”和“后处理产生的路径质量”。

## 5. 配置文件实践

示例配置建议保留在：

```text
examples/
examples/benchmark/
```

配置文件应包含：

- `workspace`
- `tool`
- `planning`
- `motion_limits`
- `request`
- `obstacles`

benchmark 场景建议额外包含：

```json
{
  "benchmark": {
    "scene": "single_box_rrtconnect",
    "category": "single_obstacle",
    "expected_success": true
  }
}
```

`expected_success=false` 的场景很重要，应覆盖：

- start invalid
- goal invalid
- no feasible path
- 参数非法

## 6. 测试实践

单元测试优先覆盖稳定、低成本、可重复的逻辑：

- 几何和路径长度计算
- primitive obstacle 碰撞
- segment validity
- JSON 解析和参数校验
- 输出文件是否生成
- 典型错误配置是否被拒绝

运行测试：

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

涉及 OMPL 的随机规划结果可能有波动。测试里应避免对随机路径点数、精确长度做过强断言，更适合放到 benchmark 中观察。

## 7. Benchmark 实践

运行整套 benchmark：

```powershell
.\build\Debug\laser_airmove_benchmark.exe --output benchmark_report.csv examples\benchmark\*.json
```

报告中重点关注：

- `outcome_matched`
- `wall_time_ms`
- `raw_path_length`
- `shortcut_path_length`
- `smoothed_path_length`
- `min_clearance`
- `average_clearance`
- `max_curvature`
- `jerk_integral`
- `smoothing_fallback`

推荐 benchmark 场景至少覆盖：

- 无障碍直线
- 单 box obstacle
- 狭窄通道
- 靠近障碍
- 多障碍组合
- start invalid
- goal invalid
- no feasible path

新增 planner 或 smoothing 策略时，应先跑 benchmark 对比，不要只凭单个 demo 判断效果。

## 8. 可视化实践

单场景规划：

```powershell
.\build\Debug\laser_airmove_plan.exe --config examples\simple_box_config.json --output airmove_cli_output
```

生成单场景可视化：

```powershell
py -3.14 tools\visualize_path.py `
  --config examples\simple_box_config.json `
  --input airmove_cli_output `
  --output visualization_output
```

生成 benchmark 汇总图：

```powershell
py -3.14 tools\visualize_benchmark.py `
  --report benchmark_report.csv `
  --output benchmark_visualization
```

推荐检查：

- `path_3d.png`：路径是否绕开障碍。
- `path_xy.png`：平面投影是否合理。
- `path_lengths.png`：shortcut 和 smooth 是否异常变长。
- `motion_profiles.png`：速度、加速度、jerk 是否接近限制。
- `summary_dashboard.png`：核心指标是否异常。
- `benchmark_*` 图：不同场景是否存在明显退化。

## 9. 调参实践

常见参数影响：

- `planning_time`：提高成功率和优化时间，但会增加运行耗时。
- `validity_resolution`：越小碰撞检查越密，越安全但越慢。
- `range`：影响 RRT 扩展步长，过大可能穿过窄通道，过小可能效率低。
- `goal_bias`：提高接近 goal 的采样概率，过高可能降低探索能力。
- `smoothing_samples`：提高平滑采样密度，但不能替代碰撞回检。
- `safety_margin`：提高安全距离，但可能让狭窄通道不可行。

调参时推荐一次只改一个维度，并通过 benchmark 记录结果。

## 10. 输出文件约定

规划工具输出目录通常包含：

```text
raw_path.csv
shortcut_path.csv
smoothed_path.csv
trajectory.csv
summary.json
path_gcode.nc
```

这些文件属于生成物，一般不提交。需要长期保存的实验结果应放入单独的实验目录，并附带配置文件、版本号和说明。

## 11. 文档实践

新增重要功能时建议同步补充文档：

- 构建和依赖变化：更新 `project_setup_windows_vcpkg.md`
- 开发路线变化：更新 `laser_airmove_planner_development_plan.md`
- 使用方式变化：更新 README 或对应工具文档
- benchmark 场景变化：更新 `examples/benchmark/README.md`

文档应优先写清楚：

- 适用场景
- 输入输出
- 命令示例
- 常见失败原因
- 如何验证

## 12. 推荐提交前检查清单

提交前建议完成：

```powershell
git status --short --untracked-files=all
git diff --check
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\laser_airmove_benchmark.exe --output benchmark_report.csv examples\benchmark\*.json
```

如果改动涉及可视化，再运行：

```powershell
.\build\Debug\laser_airmove_plan.exe --config examples\simple_box_config.json --output airmove_cli_output
py -3.14 tools\visualize_path.py --config examples\simple_box_config.json --input airmove_cli_output --output visualization_output
py -3.14 tools\visualize_benchmark.py --report benchmark_report.csv --output benchmark_visualization
```

最后确认生成物仍被忽略，只提交源码、配置、测试和文档。
