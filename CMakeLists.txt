# ============================================================================
# CMakeLists.txt — Physics Debug Platform 构建配置
# ============================================================================
# 本文件定义了项目的构建规则，包括：
#   1. C++17 标准设置
#   2. raylib 依赖获取（FetchContent 或系统安装）
#   3. 源文件列表与编译目标
#   4. MSVC / GCC / Clang 编译选项
#   5. 平台特定链接库（Windows / macOS / Linux）
#
# 使用方法：
#   cmake -B build -G "Visual Studio 17 2022"   # 配置
#   cmake --build build --config Debug           # 编译
# ============================================================================

# 要求 CMake 3.24+（FetchContent 的 FIND_PACKAGE_ARGS 支持等特性）
cmake_minimum_required(VERSION 3.24)

# 项目名称：physics_debug_platform，仅使用 C++ 语言
project(physics_debug_platform LANGUAGES CXX)

# ---------------------------------------------------------------------------
# C++ 标准设置
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)          # 使用 C++17 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 若编译器不支持 C++17 则报错
set(CMAKE_CXX_EXTENSIONS OFF)       # 禁用 GNU 等编译器扩展，确保可移植性

# ---------------------------------------------------------------------------
# 外部依赖（FetchContent）
# ---------------------------------------------------------------------------
include(FetchContent)

# --- raylib：窗口/渲染/输入 ---
option(PDP_FETCH_RAYLIB "Fetch raylib with CMake if it is not already installed" ON)
find_package(raylib 5.5 QUIET)
if (NOT raylib_FOUND AND PDP_FETCH_RAYLIB)
    FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    )
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)
endif()

# --- Jolt Physics：物理引擎后端 ---
# 从 GitHub 拉取 v5.3.0，SOURCE_SUBDIR 指向 Build/ 子目录（Jolt 的 CMakeLists 所在处）
option(PDP_FETCH_JOLT "Fetch Jolt Physics with CMake" ON)
if (PDP_FETCH_JOLT)
    # Jolt 构建选项：关闭不需要的功能，减小编译规模
    set(TARGET_HELLO_WORLD       OFF CACHE BOOL "" FORCE)
    set(TARGET_UNIT_TESTS        OFF CACHE BOOL "" FORCE)
    set(TARGET_PERFORMANCE_TEST  OFF CACHE BOOL "" FORCE)
    set(TARGET_SAMPLES           OFF CACHE BOOL "" FORCE)
    set(TARGET_VIEWER            OFF CACHE BOOL "" FORCE)
    set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        joltphysics
        GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
        GIT_TAG        v5.3.0
        SOURCE_SUBDIR  Build
    )
    FetchContent_MakeAvailable(joltphysics)
endif()

# ---------------------------------------------------------------------------
# 构建目标
# ---------------------------------------------------------------------------
# 可执行文件，包含所有源文件
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/world.cpp
    src/physics_null_v1.cpp
    src/physics_null_v2.cpp
    src/physics_null_v3.cpp
    src/physics_null_v3.1.cpp
    src/physics_null_v4.cpp
    src/physics_jolt.cpp
    src/renderer.cpp
    src/scenes.cpp
)

# 包含目录：src/ 下的头文件可直接 #include
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 再次确认 C++17（双重保障，兼容某些 IDE）
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# ---------------------------------------------------------------------------
# 编译器选项
# ---------------------------------------------------------------------------
if (MSVC)
    # /W4: 高警告等级   /permissive-: 严格标准一致性   /utf-8: 源码 UTF-8
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive- /utf-8)
    # 保留控制台窗口，方便 printf / std::cout 调试输出
    # （若不需要控制台可改为 /SUBSYSTEM:WINDOWS）
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")
else()
    # GCC / Clang: 开启常用警告
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 链接库
# ---------------------------------------------------------------------------
# 链接 raylib（3D 渲染、窗口管理、输入等）和 Jolt Physics
target_link_libraries(${PROJECT_NAME} PRIVATE raylib Jolt)

# macOS 需要额外的系统框架
if (APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        "-framework IOKit"
        "-framework Cocoa"
        "-framework OpenGL"
    )
# Linux 需要 X11 等系统库
elseif (UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE m pthread dl rt X11)
endif()
