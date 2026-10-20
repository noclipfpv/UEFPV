// Settings.hpp - user settings persisted as a simple key=value ini file.
#pragma once

#include <filesystem>

#include "DroneSim.hpp"
#include "Gimbal.hpp"
#include "FeedEffect.hpp"

namespace fpv {

enum class InputSource : int {
    XInput = 0,   // Xbox-style gamepad (also UEVR's emulated gamepad from VR controllers)
    Joystick = 1, // RC radio in USB joystick mode, or any DirectInput/HID joystick (winmm)
};

enum class GamepadThrottle : int {
    LeftStickFull = 0,   // left stick Y: bottom = 0 %, top = 100 %
    LeftStickUpper = 1,  // left stick Y: center = 0 %, top = 100 %
    RightTrigger = 2,    // RT = throttle (left stick Y unused)
};

// Logical channel indices.
enum Channel : int { ChThrottle = 0, ChRoll = 1, ChPitch = 2, ChYaw = 3, ChCount = 4 };

// Joystick axis indices (winmm order).
enum JoyAxis : int { AxisX = 0, AxisY, AxisZ, AxisR, AxisU, AxisV, AxisCount };

struct InputConfig {
    InputSource source{InputSource::XInput};

    // Gamepad
    int xinput_user{0};
    GamepadThrottle gamepad_throttle{GamepadThrottle::LeftStickFull};
    double stick_deadzone{0.04};
    double radio_deadzone{0.0}; // RC radio sticks (roll / pitch / yaw); radios are usually clean at center

    // Joystick / RC radio
    int joystick_id{0};
    int axis_map[ChCount]{AxisZ, AxisX, AxisY, AxisR}; // typical RC radio "mode 2" layout
    int axis_min[AxisCount]{};  // calibration; min == max means "use device caps"
    int axis_max[AxisCount]{};

    bool invert[ChCount]{false, false, false, false};

    // Arming
    int arm_mode{0};          // 0 = throttle low, 1 = arm switch on the radio / joystick
    int arm_source{0};        // 0 = joystick button, 1 = joystick axis
    int arm_button{0};        // 0-based button index
    int arm_axis{AxisU};      // axis carrying the switch channel
    double arm_threshold{0.3};// axis value (-1..1) above which the switch is "on"
    bool arm_invert{false};
};

// Pro cinema camera: gimbal and Dual OP.
struct CinemaSettings {
    int camera_mode{0};          // 0 = FPV camera (fixed to the frame), 1 = stabilized gimbal
    int gimbal_control{0};       // 0 = pilot (follows heading), 1 = Dual OP live (2nd gamepad), 2 = Dual OP replay
    int operator_user{1};        // XInput user index of the camera operator (Dual OP live)
    int operator_source{0};      // Dual OP live: 0 = gamepad, 1 = RC radio / joystick
    double operator_deadzone{0.04};      // operator gamepad sticks
    int operator_joystick_id{1};
    int op_axis_pan{AxisX};      // operator radio, mode 2: right stick = pan / tilt
    int op_axis_tilt{AxisY};
    int op_axis_roll{AxisR};     // left stick X
    int op_axis_zoom{AxisZ};     // left stick Y (throttle stick); -1 = none
    bool op_invert_pan{false}, op_invert_tilt{false}, op_invert_roll{false}, op_invert_zoom{false};
    int op_zoom_mode{1};         // 0 = zoom speed (centered stick), 1 = focal length follows the stick / pot
    double op_radio_deadzone{0.0};
    int op_axis_min[AxisCount]{};
    int op_axis_max[AxisCount]{};
    int tilt_axis{-1};           // pilot mode: joystick axis for absolute tilt (-1 = gamepad D-pad)
    bool tilt_axis_invert{false};
    double replay_speed{1.0};    // 1 = real time, 0.5 = half speed...
    bool replay_loop{true};
    int record_view{0};          // Dual OP replay: main view while flying the take: 0 = pilot FPV camera, 1 = gimbal
    double camera_drop_m{0.15};  // the gimbal hangs below the frame
    GimbalParams gimbal{};
};

// Second window (scene capture).
struct ViewSettings {
    bool enabled{false};
    int view{2};          // 0 = pilot FPV camera, 1 = gimbal camera, 2 = the view not on the main screen
    int height{480};      // capture lines
    int aspect{0};        // 0 = 4:3, 1 = 16:9
    int interval{1};      // capture every N frames
    FeedEffect fx{};

    int width() const { return (static_cast<int>(height * (aspect == 0 ? 4.0 / 3.0 : 16.0 / 9.0)) + 1) & ~1; }
};

struct Settings {
    DroneParams drone{};
    ViewSettings second{};
    CinemaSettings cinema{};
    CollisionParams collision{};
    int trace_channel{0};           // 0 = Visibility, 1 = Camera (TraceTypeQuery1 / 2)
    InputConfig input{};

    // Camera
    bool override_fov{true};
    double fov_deg{110.0};

    // World
    double units_per_meter{100.0};  // Unreal default: 1 uu = 1 cm

    // Game integration
    bool block_game_gamepad{true};  // hide gamepad input from the game while flying
    bool freeze_player{true};       // ignore the player's move / look input while flying
    bool show_osd{false};
    bool show_crosshair{false};
    bool hide_game_ui{false};

    bool load(const std::filesystem::path& path);
    bool save(const std::filesystem::path& path) const;
};

} // namespace fpv
