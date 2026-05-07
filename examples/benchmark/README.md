# Benchmark 场景集

本目录提供可重复运行的规划场景，用于比较不同 planner、障碍构型、平滑参数和失败处理。

运行示例：

```powershell
.\build\Debug\laser_airmove_benchmark.exe --output benchmark_report.csv examples\benchmark\*.json
```

生成 benchmark 汇总图：

```powershell
py -3.14 tools\visualize_benchmark.py --report benchmark_report.csv --output benchmark_visualization
```

场景说明：

- `01_open_space_informed.json`：无障碍直线路径，验证基础耗时和轨迹输出。
- `02_single_box_rrtconnect.json`：单 box 障碍绕行，验证基础避障。
- `03_narrow_passage_rrtconnect.json`：狭窄通道，验证采样规划和碰撞分辨率。
- `04_near_obstacle_informed.json`：起点和路径靠近障碍，验证 clearance 指标。
- `05_no_path_blocked.json`：工作空间被墙体阻断，预期规划失败。
- `06_start_invalid.json`：起点在障碍内，预期规划失败。

每个配置的 `benchmark.expected_success` 用于报告中判断 `outcome_matched`，因此预期失败场景不会导致整套 benchmark 误判。
