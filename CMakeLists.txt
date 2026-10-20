# UEVR FPV Drone plugin - standalone build.
#
# Only UEVR's MIT-licensed plugin headers and the example render helpers are used, so the
# UEVR repository is fetched WITHOUT its submodules (no EpicGames / UESDK access needed).
#
#   cmake -S . -B build -G "Visual Studio 17 2022" -A x64
#   cmake --build build --config Release
#
# Output: build/Release/uevr_fpv_drone.dll
cmake_minimum_required(VERSION 3.24)
project(uevr_fpv_drone LANGUAGES C CXX)

if(NOT WIN32)
    message(FATAL_ERROR "This plugin targets Windows (UEVR is Windows-only). Build with MSVC / Visual Studio 2022.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Static CRT, like UEVR itself: no VC++ redistributable needed on the player's PC.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

include(FetchContent)

# Pinned to the versions this plugin was written against. Bump together if needed.
# UEVR 1.05 release = plugin API 2.34. Works with UEVR 1.05 and every newer nightly.
set(FPV_UEVR_COMMIT  "1a810f69fe4cab82b352c574763959d4390b84ce" CACHE STRING "UEVR commit for the plugin API headers")
set(FPV_IMGUI_COMMIT "c6e0284ac58b3f205c95365478888f7b53b077e2" CACHE STRING "Dear ImGui commit (the one UEVR pins)")

FetchContent_Declare(uevr
    GIT_REPOSITORY https://github.com/praydog/UEVR.git
    GIT_TAG        ${FPV_UEVR_COMMIT}
    GIT_SUBMODULES ""          # skip UESDK & co.
    GIT_SHALLOW    FALSE
)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        ${FPV_IMGUI_COMMIT}
)
# Populate only (we do not want their own CMake projects).
FetchContent_GetProperties(uevr)
if(NOT uevr_POPULATED)
    FetchContent_Populate(uevr)
endif()
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

# --- Dear ImGui -------------------------------------------------------------------------
add_library(fpv_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)
target_include_directories(fpv_imgui PUBLIC ${imgui_SOURCE_DIR} ${uevr_SOURCE_DIR}/src/uevr-imgui)
target_compile_definitions(fpv_imgui PUBLIC
    "IMGUI_USER_CONFIG=\"${uevr_SOURCE_DIR}/src/uevr-imgui/uevr_imconfig.hpp\"")

# --- UEVR example render helpers (ImGui on D3D11 / D3D12) -------------------------------
file(GLOB_RECURSE FPV_RENDERLIB_SOURCES CONFIGURE_DEPENDS
    ${uevr_SOURCE_DIR}/examples/renderlib/*.cpp
    ${uevr_SOURCE_DIR}/examples/renderlib/*.c)
add_library(fpv_renderlib STATIC ${FPV_RENDERLIB_SOURCES})
target_include_directories(fpv_renderlib PUBLIC
    ${uevr_SOURCE_DIR}/examples/renderlib
    ${uevr_SOURCE_DIR}/include)
target_link_libraries(fpv_renderlib PUBLIC fpv_imgui d3d11 d3d12 dxgi d3dcompiler dwmapi)

# --- The plugin ---------------------------------------------------------------------------
add_library(uevr_fpv_drone SHARED
    src/Plugin.cpp
    src/DroneSim.cpp
    src/Gimbal.cpp
    src/FlightRecorder.cpp
    src/GameCamera.cpp
    src/SceneCaptureView.cpp
    src/SecondWindow.cpp
    src/GameThreadTicker.cpp
    src/GameUiHider.cpp
    src/GlPresenter.cpp
    src/Reflection.cpp
    src/Input.cpp
    src/Settings.cpp
)
target_include_directories(uevr_fpv_drone PRIVATE src)
set_target_properties(uevr_fpv_drone PROPERTIES PREFIX "")
target_link_libraries(uevr_fpv_drone PRIVATE fpv_renderlib winmm gdi32 user32 opengl32 d3d11 d3d12 dxgi d3dcompiler)
target_compile_definitions(uevr_fpv_drone PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
if(MSVC)
    target_compile_options(uevr_fpv_drone PRIVATE /W4 /permissive- /utf-8 /MP /EHa)
    target_compile_options(fpv_renderlib PRIVATE /EHa /MP)
endif()

# --- Flight model tests (portable) --------------------------------------------------------
add_executable(fpv_sim_tests tests/sim_tests.cpp src/DroneSim.cpp)
target_include_directories(fpv_sim_tests PRIVATE src)
add_executable(fpv_gimbal_tests tests/gimbal_tests.cpp src/Gimbal.cpp src/FlightRecorder.cpp)
target_include_directories(fpv_gimbal_tests PRIVATE src)
