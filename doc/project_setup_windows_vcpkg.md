# Laser Air-Move Planner Windows Setup Guide

This document records the recommended Windows engineering setup for this project.

## 1. Prerequisites

Install the following tools:

- Visual Studio 2022
- CMake 3.20 or newer
- Git
- vcpkg
- 7-Zip

Visual Studio 2022 must include:

- Desktop development with C++
- MSVC v143 x64/x86 build tools
- Windows 10/11 SDK

Recommended 7-Zip path:

```powershell
C:\Program Files\7-Zip
```

Add it to the current PowerShell session when needed:

```powershell
$env:Path = "C:\Program Files\7-Zip;$env:Path"
where.exe 7z
```

## 2. Install vcpkg

Example installation path:

```powershell
cd D:\software
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

Set `VCPKG_ROOT`:

```powershell
setx VCPKG_ROOT D:\software\vcpkg
```

Open a new PowerShell window and verify:

```powershell
echo $env:VCPKG_ROOT
Test-Path "$env:VCPKG_ROOT\vcpkg.exe"
```

## 3. Proxy Notes

If vcpkg downloads from GitHub fail, explicitly set proxy variables.

Example:

```powershell
$env:HTTP_PROXY = "http://127.0.0.1:10808"
$env:HTTPS_PROXY = "http://127.0.0.1:10808"
Test-NetConnection 127.0.0.1 -Port 10808
```

Use `http://` for both variables unless your proxy is truly an HTTPS proxy.

If vcpkg keeps downloading tool binaries such as `7zr.exe` or PowerShell Core and fails, install those tools locally and then try:

```powershell
$env:VCPKG_FORCE_SYSTEM_BINARIES = "1"
```

## 4. Install Dependencies

The project uses vcpkg manifest mode. Dependencies are declared in `vcpkg.json`:

- ompl
- fcl
- eigen3
- ruckig

Install dependencies:

```powershell
cd D:\data\workspace\github\laser-airmove-planner
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows
```

If PowerShell reports an unexpected token near `vcpkg.exe`, use the call operator and quotes exactly as shown above.

## 5. Configure With Visual Studio Generator

Using the Visual Studio generator is recommended on Windows because it loads the MSVC and Windows SDK environment correctly.

Clean any old build directory first:

```powershell
Remove-Item -Recurse -Force build
```

If Visual Studio is open and files under `build\.vs` are locked, close Visual Studio and retry. Alternatively use a fresh build directory.

Configure:

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

Generated solution:

```text
build\laser_airmove_planner.sln
```

## 6. Configure With CMake GUI

Set:

```text
Where is the source code:
D:/data/workspace/github/laser-airmove-planner

Where to build the binaries:
D:/data/workspace/github/laser-airmove-planner/build
```

Add entries before pressing Configure:

```text
Name: CMAKE_TOOLCHAIN_FILE
Type: FILEPATH
Value: D:/software/vcpkg/scripts/buildsystems/vcpkg.cmake
```

```text
Name: VCPKG_TARGET_TRIPLET
Type: STRING
Value: x64-windows
```

Then press:

1. Configure
2. Select `Visual Studio 17 2022`
3. Select platform `x64`
4. Configure again if needed
5. Generate
6. Open Project

## 7. Build

Command line:

```powershell
cmake --build build --config Debug
```

Or open:

```text
build\laser_airmove_planner.sln
```

Then build the solution in Visual Studio with:

```text
Debug | x64
```

## 8. Run Tests

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Expected result:

```text
100% tests passed
```

## 9. Run Demo

```powershell
.\build\Debug\single_airmove_demo.exe
```

The demo writes:

```text
airmove_demo_output\raw_path.csv
airmove_demo_output\smoothed_path.csv
airmove_demo_output\trajectory.csv
airmove_demo_output\summary.json
```

These files are local generated artifacts and are ignored by Git.

## 10. Common Errors

### CMake cannot find OMPL

Error:

```text
Could not find a package configuration file provided by "ompl"
ompl_DIR-NOTFOUND
```

Cause: CMake was not configured with vcpkg toolchain.

Fix:

```powershell
$toolchain = Join-Path $env:VCPKG_ROOT "scripts\buildsystems\vcpkg.cmake"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="$toolchain" -DVCPKG_TARGET_TRIPLET=x64-windows
```

### CMake cannot find kernel32.lib

Error:

```text
LINK : fatal error LNK1104: cannot open file 'kernel32.lib'
```

Cause: Ninja or command shell was used without a loaded MSVC/Windows SDK environment.

Fix: use the Visual Studio generator:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="$toolchain"
```

Or start a Visual Studio developer shell before using Ninja.

### Ruckig target references nlohmann_json

Error:

```text
The link interface of target "ruckig::ruckig" contains:
nlohmann_json::nlohmann_json
but the target was not found
```

Fix: ensure `CMakeLists.txt` contains:

```cmake
find_package(nlohmann_json CONFIG REQUIRED)
find_package(ruckig REQUIRED)
```

### Cannot open ompl.lib

Error:

```text
LINK : fatal error LNK1104: cannot open file 'ompl.lib'
```

Fix: link the imported target, not the raw library name:

```cmake
target_link_libraries(airmove_planner PUBLIC ompl::ompl)
```

### Build directory cannot be deleted

Error:

```text
The process cannot access the file because it is being used by another process
```

Cause: Visual Studio keeps database files under `build\.vs` open.

Fix: close Visual Studio and retry, or use another build directory.

## 11. Git Hygiene

The following are local artifacts and should not be committed:

```text
.vs/
build/
out/
vcpkg_installed/
airmove_demo_output/
```

The following should be committed:

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
