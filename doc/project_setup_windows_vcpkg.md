# Laser Air-Move Planner Windows 工程搭建指南

本文档记录本项目在 Windows 环境下的推荐工程搭建流程，重点覆盖：

- Visual Studio 2022
- CMake
- vcpkg
- OMPL / FCL / Eigen / Ruckig
- CMake GUI 与 Visual Studio 生成器
- 常见构建错误排查

## 1. 基础工具

需要安装：

- Visual Studio 2022
- CMake 3.20 或更高版本
- Git
- vcpkg
- 7-Zip

Visual Studio 2022 需要勾选以下组件：

- 使用 C++ 的桌面开发
- MSVC v143 x64/x86 生成工具
- Windows 10/11 SDK

推荐 7-Zip 安装路径：

```powershell
C:\Program Files\7-Zip
```

如果当前 PowerShell 找不到 `7z`，可以临时加入 `PATH`：

```powershell
$env:Path = "C:\Program Files\7-Zip;$env:Path"
where.exe 7z
```

确认输出中包含：

```text
C:\Program Files\7-Zip\7z.exe
```

## 2. 安装 vcpkg

示例安装路径：

```powershell
cd D:\software
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

设置 `VCPKG_ROOT`：

```powershell
setx VCPKG_ROOT D:\software\vcpkg
```

重新打开 PowerShell 后验证：

```powershell
echo $env:VCPKG_ROOT
Test-Path "$env:VCPKG_ROOT\vcpkg.exe"
```

`Test-Path` 应返回：

```text
True
```

## 3. 代理配置说明

如果 vcpkg 从 GitHub 下载依赖或工具时失败，可以显式设置代理。

示例：

```powershell
$env:HTTP_PROXY = "http://127.0.0.1:10808"
$env:HTTPS_PROXY = "http://127.0.0.1:10808"
Test-NetConnection 127.0.0.1 -Port 10808
```

除非你的代理确实是 HTTPS 代理，否则 `HTTP_PROXY` 和 `HTTPS_PROXY` 都建议使用 `http://` 前缀。

如果 vcpkg 一直尝试下载 `7zr.exe`、PowerShell Core 等工具并失败，可以优先在本机安装这些工具，然后设置：

```powershell
$env:VCPKG_FORCE_SYSTEM_BINARIES = "1"
```

## 4. 安装项目依赖

本项目使用 vcpkg manifest 模式。依赖声明在仓库根目录的 `vcpkg.json` 中：

- `ompl`
- `fcl`
- `eigen3`
- `ruckig`

安装依赖：

```powershell
cd D:\data\workspace\github\laser-airmove-planner
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows
```

注意 PowerShell 中不能直接写：

```powershell
$env:VCPKG_ROOT\vcpkg.exe
```

应使用调用运算符 `&` 和引号：

```powershell
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows
```

## 5. 使用 Visual Studio 生成器配置 CMake

Windows 下推荐使用 Visual Studio 生成器。它会自动处理 MSVC 和 Windows SDK 环境，避免 `kernel32.lib` 等系统库找不到的问题。

如果已有旧的构建目录，先删除：

```powershell
Remove-Item -Recurse -Force build
```

如果 Visual Studio 正在打开该解决方案，可能会锁定 `build\.vs` 下的数据库文件。此时先关闭 Visual Studio，再删除构建目录。

配置命令：

```powershell
$toolchain = Join-Path $env:VCPKG_ROOT "scripts\buildsystems\vcpkg.cmake"

cmake -S . -B build `
  -G "Visual Studio 17 2022" `
  -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$toolchain" `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DAIRMOVE_BUILD_EXAMPLES=ON `
  -DAIRMOVE_BUILD_TESTS=ON
```

成功后会生成：

```text
build\laser_airmove_planner.sln
```

## 6. 使用 CMake GUI 配置

CMake GUI 中设置：

```text
Where is the source code:
D:/data/workspace/github/laser-airmove-planner

Where to build the binaries:
D:/data/workspace/github/laser-airmove-planner/build
```

点击 `Add Entry`，新增：

```text
Name: CMAKE_TOOLCHAIN_FILE
Type: FILEPATH
Value: D:/software/vcpkg/scripts/buildsystems/vcpkg.cmake
```

再新增：

```text
Name: VCPKG_TARGET_TRIPLET
Type: STRING
Value: x64-windows
```

然后按顺序执行：

1. 点击 `Configure`
2. 生成器选择 `Visual Studio 17 2022`
3. 平台选择 `x64`
4. 如有红色配置项，继续点击 `Configure`
5. 点击 `Generate`
6. 点击 `Open Project`

如果忘记设置 `CMAKE_TOOLCHAIN_FILE`，CMake 通常会报找不到 OMPL。

## 7. 编译

命令行编译：

```powershell
cmake --build build --config Debug
```

也可以打开：

```text
build\laser_airmove_planner.sln
```

在 Visual Studio 中选择：

```text
Debug | x64
```

然后生成解决方案。

## 8. 运行测试

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

期望结果：

```text
100% tests passed
```

## 9. 运行示例

```powershell
.\build\Debug\single_airmove_demo.exe
```

示例会生成：

```text
airmove_demo_output\raw_path.csv
airmove_demo_output\smoothed_path.csv
airmove_demo_output\trajectory.csv
airmove_demo_output\summary.json
```

这些文件是本地生成产物，不需要提交到 Git。

## 10. 运行命令行工具

项目也提供可配置的命令行工具：

```powershell
.\build\Debug\laser_airmove_plan.exe --config examples\simple_box_config.json --output airmove_cli_output
```

其中：

- `--config` 指定 JSON 配置文件
- `--output` 指定输出目录

输出文件与 demo 一致：

```text
raw_path.csv
smoothed_path.csv
trajectory.csv
summary.json
path_gcode.nc
```

## 11. 运行可视化工具

先确认已经生成规划输出目录，例如 `airmove_cli_output`。然后执行：

```powershell
python tools\visualize_path.py --config examples\simple_box_config.json --input airmove_cli_output --output visualization_output
```

输出图片：

```text
visualization_output\path_3d.png
visualization_output\path_xy.png
visualization_output\motion_profiles.png
```

该工具依赖 `matplotlib`：

```powershell
python -m pip install -r tools\requirements-visualization.txt
```

## 12. 常见问题

### 12.1 CMake 找不到 OMPL

典型错误：

```text
Could not find a package configuration file provided by "ompl"
ompl_DIR-NOTFOUND
```

原因：

CMake 没有使用 vcpkg toolchain。

处理方式：

```powershell
$toolchain = Join-Path $env:VCPKG_ROOT "scripts\buildsystems\vcpkg.cmake"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="$toolchain" -DVCPKG_TARGET_TRIPLET=x64-windows
```

也可以检查 OMPL 是否已经安装：

```powershell
Test-Path "D:\software\vcpkg\installed\x64-windows\share\ompl\omplConfig.cmake"
```

如果使用 manifest 模式，也可能安装在：

```text
build\vcpkg_installed\x64-windows\share\ompl\omplConfig.cmake
```

### 12.2 找不到 kernel32.lib

典型错误：

```text
LINK : fatal error LNK1104: cannot open file 'kernel32.lib'
```

原因：

普通 PowerShell 或 Ninja 环境没有加载 MSVC / Windows SDK 的库路径。

推荐处理方式：

使用 Visual Studio 生成器：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="$toolchain"
```

如果坚持使用 Ninja，需要先进入 Visual Studio 开发者环境。

### 12.3 Ruckig 报 nlohmann_json target not found

典型错误：

```text
The link interface of target "ruckig::ruckig" contains:
nlohmann_json::nlohmann_json
but the target was not found
```

处理方式：

确保 `CMakeLists.txt` 中先加载 `nlohmann_json`：

```cmake
find_package(nlohmann_json CONFIG REQUIRED)
find_package(ruckig REQUIRED)
```

### 12.4 链接时找不到 ompl.lib

典型错误：

```text
LINK : fatal error LNK1104: cannot open file 'ompl.lib'
```

原因：

项目链接了裸库名 `ompl`，但 vcpkg 导出的 CMake target 是 `ompl::ompl`。

处理方式：

```cmake
target_link_libraries(airmove_planner PUBLIC ompl::ompl)
```

### 12.5 无法删除 build 目录

典型错误：

```text
The process cannot access the file because it is being used by another process
```

原因：

Visual Studio 正在占用 `build\.vs` 下的索引数据库。

处理方式：

- 关闭 Visual Studio 后重试
- 或换一个新的构建目录

### 12.6 vcpkg 下载 7zr.exe 或 PowerShell Core 失败

典型现象：

```text
A suitable version of 7zip was not found
Download timed out
curl operation failed
```

处理方式：

1. 确认 7-Zip 已安装并加入 `PATH`
2. 确认代理端口可用
3. 显式设置 `HTTP_PROXY` 和 `HTTPS_PROXY`
4. 必要时设置 `VCPKG_FORCE_SYSTEM_BINARIES=1`

## 13. Git 注意事项

以下是本地生成产物，不应提交：

```text
.vs/
build/
out/
vcpkg_installed/
airmove_demo_output/
```

以下内容应提交：

```text
CMakeLists.txt
vcpkg.json
include/
src/
examples/
tests/
doc/
README.md
.gitignore
```

## 14. Qt Demo 补充说明

Qt 演示界面是可选模块，默认不参与构建。启用时需要本机 Qt 6，例如：

```text
C:\Qt\6.11.0\msvc2022_64
```

### 14.1 CMake GUI 中 Qt 路径怎么填

如果勾选了：

```text
AIRMOVE_BUILD_QT_DEMO
```

但 CMake GUI 报：

```text
Qt6_DIR-NOTFOUND
Could not find a package configuration file provided by "Qt6"
```

推荐添加：

```text
Name: CMAKE_PREFIX_PATH
Type: PATH
Value: C:/Qt/6.11.0/msvc2022_64
```

或者直接设置：

```text
Name: Qt6_DIR
Type: PATH
Value: C:/Qt/6.11.0/msvc2022_64/lib/cmake/Qt6
```

注意：

```text
Qt6_DIR = C:/Qt/6.11.0/msvc2022_64
```

是不对的。`Qt6_DIR` 必须指向包含 `Qt6Config.cmake` 的目录。

### 14.2 运行时找不到 Qt6Widgetsd.dll

如果启动 `airmove_qt_demo.exe` 时提示：

```text
由于找不到 Qt6Widgetsd.dll，无法继续执行代码。
```

说明运行时没有找到 Qt 的 DLL。开发阶段可临时设置：

当前工程已经在 `AIRMOVE_BUILD_QT_DEMO=ON` 时自动调用 `windeployqt`，完整构建后通常不需要手动复制 Qt DLL。确认你执行过：

```powershell
cmake --build build-qt --config Debug --target airmove_qt_demo
```

构建日志中应出现：

```text
Deploying Qt runtime dependencies for airmove_qt_demo
Updating Qt6Widgetsd.dll.
Updating qwindowsd.dll.
```

如果仍然失败，再临时设置：

```powershell
$env:Path = "C:\Qt\6.11.0\msvc2022_64\bin;$env:Path"
.\build-qt\Debug\airmove_qt_demo.exe
```

也可以使用 Qt 部署工具：

```powershell
C:\Qt\6.11.0\msvc2022_64\bin\windeployqt.exe --debug .\build-qt\Debug\airmove_qt_demo.exe
```

Debug 版本会找 `Qt6Widgetsd.dll`，Release 版本会找 `Qt6Widgets.dll`。不要混用 Debug exe 和 Release DLL。
