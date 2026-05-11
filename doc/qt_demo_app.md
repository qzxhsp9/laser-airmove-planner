# Qt 演示界面使用说明

本文档说明 `airmove_qt_demo` 的构建和使用方式。该程序定位为轻量级演示工具，目标是快速加载规划配置、交互调整少量参数、运行单段空移规划，并查看路径、指标和可视化图片。

## 1. 功能范围

当前 Qt Demo 提供：

- 选择 JSON 配置文件。
- 选择规划输出目录。
- 在右侧 `Scene editor` 中结构化编辑并保存 JSON 配置文件。
- 调整常用规划参数：
  - `planner`
  - `planning_time`
  - `validity_resolution`
  - `smoothing_samples`
  - `safety_margin`
  - `simplify_solution`
- 在界面内调用 C++ planner 运行轻量规划。
- 输出 `raw_path.csv`、`shortcut_path.csv`、`smoothed_path.csv`、`trajectory.csv`、`summary.json`、`path_gcode.nc`。
- 表格展示 smoothed path。
- 显示规划摘要指标。
- 内置轻量 3D 交互视图，支持旋转、平移、缩放。
- 3D 视图显示 box、sphere、cylinder 和 `ascii_stl` 障碍物线框。
- 调用 `tools/visualize_path.py` 导出高质量 PNG 可视化图片。
- 保留运行日志，便于演示和排查。

该工具不是完整 CAM 前端，暂不承担复杂场景编辑、STL 交互建模或实时 3D 渲染。

## 2. Qt 安装路径

当前推荐使用本机 Qt：

```text
C:\Qt\6.11.0\msvc2022_64
```

需要确保该 Qt Kit 与 Visual Studio 2022 的 MSVC x64 工具链匹配。

## 3. CMake 配置

建议使用独立构建目录，避免影响已有 `build/`：

```powershell
cmake -S . -B build-qt `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.11.0\msvc2022_64" `
  -DAIRMOVE_BUILD_QT_DEMO=ON `
  -DAIRMOVE_BUILD_EXAMPLES=ON `
  -DAIRMOVE_BUILD_TOOLS=ON `
  -DAIRMOVE_BUILD_TESTS=ON
```

构建：

```powershell
cmake --build build-qt --config Debug
```

运行：

```powershell
.\build-qt\Debug\airmove_qt_demo.exe
```

项目的 CMake 已经在构建 `airmove_qt_demo` 后自动调用 `windeployqt`，正常情况下会把 Qt DLL 和 `platforms/qwindowsd.dll` 部署到 exe 附近。

当前工程改为复制 Qt Demo 的最小运行依赖，不再强依赖 `windeployqt`。构建日志应出现：

```text
Copying minimal Qt runtime dependencies for airmove_qt_demo
```

如果运行时仍提示缺少 Qt DLL，可先在当前 PowerShell 中加入 Qt bin 目录：

```powershell
$env:Path = "C:\Qt\6.11.0\msvc2022_64\bin;$env:Path"
.\build-qt\Debug\airmove_qt_demo.exe
```

也可以手动使用 Qt 自带部署工具：

```powershell
C:\Qt\6.11.0\msvc2022_64\bin\windeployqt.exe .\build-qt\Debug\airmove_qt_demo.exe
```

## 4. 使用流程

1. 打开程序。
2. 选择配置文件，例如：

```text
examples/simple_box_config.json
```

3. 选择输出目录，例如：

```text
airmove_qt_output
```

4. 按需微调参数。
5. 如需修改配置，在右侧 `Scene editor` 中编辑场景。
6. 点击 `Save scene config` 保存配置，并同步刷新 `Interactive 3D View`。
7. 点击 `Run planning`。
8. 查看左侧摘要和 `smoothed path` 表格。
7. 在 `Interactive 3D View` 标签页拖拽查看路径。
8. 点击 `Export PNG visualization` 可导出 `path_3d.png` 等高质量图片。

可视化脚本依赖 matplotlib。如果未安装：

```powershell
py -3.14 -m pip install -r tools\requirements-visualization.txt
```

3D 交互视图操作：

```text
Left drag   : rotate
Right drag  : pan
Mouse wheel : zoom
Reset 3D view button: reset camera
```

配置制作入口：

```text
Save scene config : 在 Scene editor 中生成 JSON，保存到当前配置文件，并刷新 3D 视图
Reload config     : 从磁盘重新读取当前配置文件
```

`Run planning` 和 `Export PNG visualization` 会在配置有未保存修改时先保存当前配置。

## 5. 常见问题

### 5.1 CMake GUI 找不到 Qt6

典型现象：

```text
Qt6_DIR-NOTFOUND
Could not find a package configuration file provided by "Qt6"
```

如果 Qt 安装目录是：

```text
C:\Qt\6.11.0\msvc2022_64
```

在 CMake GUI 中推荐填写 `CMAKE_PREFIX_PATH`：

```text
C:/Qt/6.11.0/msvc2022_64
```

如果选择直接填写 `Qt6_DIR`，则必须填到 Qt6 的 CMake 配置目录：

```text
C:/Qt/6.11.0/msvc2022_64/lib/cmake/Qt6
```

注意不要把 `Qt6_DIR` 填成：

```text
C:/Qt/6.11.0/msvc2022_64
```

这个路径是 Qt 安装前缀，不是 `Qt6Config.cmake` 所在目录。

CMake GUI 操作建议：

1. 勾选 `Advanced`。
2. 如果已有 `Qt6_DIR-NOTFOUND`，改成 `C:/Qt/6.11.0/msvc2022_64/lib/cmake/Qt6`。
3. 或点击 `Add Entry` 添加：

```text
Name: CMAKE_PREFIX_PATH
Type: PATH
Value: C:/Qt/6.11.0/msvc2022_64
```

4. 点击 `Configure`。
5. 不再报 Qt6 错误后点击 `Generate`。

### 5.2 运行时找不到 Qt6Widgetsd.dll

典型现象：

```text
由于找不到 Qt6Widgetsd.dll，无法继续执行代码。
```

这不是编译错误，而是运行时 DLL 搜索路径问题。`Qt6Widgetsd.dll` 中的 `d` 表示 Debug 版本 Qt DLL。

当前 CMake 会在构建后复制最小 Qt 运行依赖：

```text
Qt6Cored.dll
Qt6Guid.dll
Qt6Widgetsd.dll
platforms\qwindowsd.dll
```

如果你是从 CMake GUI 或 Visual Studio 构建，先重新生成并完整构建 `airmove_qt_demo`，不要只运行旧 exe。

开发阶段推荐在 PowerShell 中临时加入 Qt bin 目录后启动：

```powershell
$env:Path = "C:\Qt\6.11.0\msvc2022_64\bin;$env:Path"
.\build-qt\Debug\airmove_qt_demo.exe
```

如果从 Visual Studio 或资源管理器双击运行，可以把下面路径加入系统 `PATH`：

```text
C:\Qt\6.11.0\msvc2022_64\bin
```

然后重新打开 Visual Studio、PowerShell 或资源管理器窗口。

演示或拷贝给别人运行时，推荐使用 Qt 部署工具把 DLL 复制到 exe 附近：

```powershell
C:\Qt\6.11.0\msvc2022_64\bin\windeployqt.exe --debug .\build-qt\Debug\airmove_qt_demo.exe
```

Release 版本则使用：

```powershell
C:\Qt\6.11.0\msvc2022_64\bin\windeployqt.exe .\build-qt\Release\airmove_qt_demo.exe
```

### 5.3 Debug 和 Release 不要混用

Debug 程序会寻找带 `d` 后缀的 Qt DLL，例如：

```text
Qt6Widgetsd.dll
Qt6Cored.dll
Qt6Guid.dll
```

Release 程序会寻找不带 `d` 后缀的 Qt DLL，例如：

```text
Qt6Widgets.dll
Qt6Core.dll
Qt6Gui.dll
```

如果 Debug exe 配了 Release Qt DLL，或者 Release exe 配了 Debug Qt DLL，都可能启动失败。开发阶段建议统一使用：

```text
Debug + C:\Qt\6.11.0\msvc2022_64\bin
```

正式演示包建议统一使用：

```text
Release + windeployqt
```

### 5.4 Export PNG visualization 找不到脚本

典型现象：

```text
Cannot find tools/visualize_path.py
```

Qt Demo 会从以下位置向上查找仓库根目录：

- 当前工作目录
- `airmove_qt_demo.exe` 所在目录
- 当前配置文件所在目录

只要目录树中存在：

```text
tools/visualize_path.py
CMakeLists.txt
```

即可自动定位。若仍失败，优先检查是否把 exe 单独复制到了仓库外部运行。演示时建议直接从仓库构建目录运行：

```powershell
.\build-qt\Debug\airmove_qt_demo.exe
```

### 5.5 ascii_stl 不显示

Qt Demo 当前支持显示 JSON 中的 `ascii_stl` 障碍物线框，例如：

```json
{
  "type": "ascii_stl",
  "file": "simple_triangle_obstacle.stl"
}
```

相对路径会按配置文件所在目录解析。若不显示，检查：

- STL 是否为 ASCII STL，不是 binary STL。
- `file` 路径是否相对于 JSON 配置文件可访问。
- STL 是否至少包含一个三角面片。

## 6. 演示建议

推荐演示顺序：

1. 使用 `examples/simple_box_config.json` 展示多障碍绕行。
2. 修改 `planner` 为 `rrtconnect`，比较耗时和路径质量。
3. 调整 `smoothing_samples`，观察 smoothed path 点数和路径图变化。
4. 调整 `safety_margin`，观察 clearance 和可行性变化。
5. 用 `examples/benchmark/06_start_invalid.json` 展示失败处理。

演示时重点说明：

- Qt 界面直接调用 C++ planner，不是只包装命令行。
- 可视化仍复用现有 Python 后处理工具，避免在 Qt 端重复实现绘图逻辑。
- 输出文件与 CLI 保持一致，方便后续接入其他系统。

## 7. 后续扩展方向

可继续增强：

- 内置 benchmark 场景选择器。
- 多场景批量运行和报告预览。
- 显示 `path_xy.png`、`motion_profiles.png`、`summary_dashboard.png` 多图切换。
- 配置编辑器支持 JSON 校验和保存。
- 使用 Qt 3D 或 VTK/Open3D 做真正交互式 3D 场景。
- 将 Python 可视化改为异步执行，避免界面阻塞。
- 增加 planner 参数 preset，便于现场演示对比。
## Scene editor 场景配置界面

`Scene editor` 位于界面右侧，是当前推荐的 config 制作入口，不再要求用户直接手写完整 JSON。它用于把实际遇到的空移避障场景结构化录入，然后生成 planner 可读取的 JSON 输入文件。

当前支持编辑：

- 工作空间边界：`workspace min/max`
- 起点和终点：`start xyz`、`goal xyz`
- 工具头半径：`tool head radius`
- 运动限制：最大速度、最大加速度、最大 jerk、采样周期
- 障碍物表：`box`、`sphere`、`cylinder`、`ascii_stl`
- 常用规划参数：左侧 `Planner Params` 区域

推荐工作流：

1. 在右侧 `Scene editor` 中编辑场景。
2. 填写工作空间、起点、终点和工具头半径。
3. 在障碍物表中添加或修改障碍物。
4. 点击 `Save scene config`，生成 JSON、保存输入文件，并刷新 `Interactive 3D View`。
5. 点击 `Run planning` 执行规划。

点击 `Run planning` 或 `Export PNG visualization` 时，程序会先把 `Scene editor` 中的结构化场景生成 JSON，再保存并继续执行。

障碍物表字段说明：

| type | cx/cy/cz | size/radius/file | size_y/height | size_z |
| --- | --- | --- | --- | --- |
| `box` | 中心点 | X 尺寸 | Y 尺寸 | Z 尺寸 |
| `sphere` | 球心 | 半径 | 留空 | 留空 |
| `cylinder` | 圆柱中心 | 半径 | 高度 | 留空 |
| `ascii_stl` | 暂不使用 | STL 文件路径 | 留空 | 留空 |

`ascii_stl` 路径建议使用相对 config 文件所在目录的相对路径，便于项目移动和演示打包。Qt Demo 会在交互 3D 视图中加载 ASCII STL 并以线框方式显示。
