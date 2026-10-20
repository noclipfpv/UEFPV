// Input.hpp - reads the pilot's sticks from an XInput gamepad or an RC radio / joystick.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <windows.h>
#include <xinput.h>

#include "DroneSim.hpp"
#include "Settings.hpp"

namespace fpv {

struct RawJoystickState {
    bool connected{false};
    std::array<int, AxisCount> axes{};
    std::array<int, AxisCount> caps_min{};
    std::array<int, AxisCount> caps_max{};
    uint32_t buttons{};
};

// A joystick / RC radio currently connected (winmm id and what the driver reports).
struct JoystickInfo {
    int id{0};
    std::string name;
    int axes{0};
    int buttons{0};
    std::array<double, AxisCount> value{}; // -1..1 from the device caps (to see which one moves)
};

// Lists the connected winmm joysticks (ids 0..15). A few ms at most: call it now and then.
std::vector<JoystickInfo> enumerate_joysticks();

// One winmm joystick (RC radio in USB HID mode) with its own calibration recorder.
struct JoystickReader {
    RawJoystickState st{};
    int caps_for_id{-1};
    bool calibrating{false};
    std::array<int, AxisCount> cal_min{};
    std::array<int, AxisCount> cal_max{};

    bool poll(int id);
    // -1..1 using the stored calibration (min == max: device caps).
    double normalized(int axis, const int* cfg_min, const int* cfg_max) const;
    void begin_calibration();
    void end_calibration(int* cfg_min, int* cfg_max);
};

// Plain copy of the input state for the UI thread.
struct InputSnapshot {
    bool pad_connected{false};
    RawJoystickState joy{};
    std::array<double, ChCount> channels{};
    int arm_switch{-1};
    bool calibrating{false};
    RawJoystickState op_joy{};
    bool op_calibrating{false};
    std::vector<JoystickInfo> devices{};
};

class InputManager {
public:
    InputManager();

    // Call from UEVR's on_xinput_get_state. Hides the gamepad from the game when requested.
    void on_xinput_get_state(uint32_t* retval, uint32_t user_index, XINPUT_STATE* state, bool block_game);

    // Polls the configured device and returns normalized stick values.
    DroneInput poll(const InputConfig& cfg);

    // Latest raw states (valid after poll()).
    const XINPUT_STATE& gamepad() const { return m_pad; }
    bool gamepad_connected() const { return m_pad_connected; }
    const RawJoystickState& joystick() const { return m_pilot.st; }
    std::array<double, ChCount> channels() const { return m_channels; }
    int arm_switch() const { return m_arm_switch; } // -1 = not used, 0 = off, 1 = on

    InputSnapshot snapshot() const {
        return {m_pad_connected, m_pilot.st, m_channels, m_arm_switch, m_pilot.calibrating, m_op.st, m_op.calibrating, m_devices};
    }

    // Camera operator gamepad (Dual OP live), polled on demand.
    bool poll_operator(int user);
    const XINPUT_STATE& operator_pad() const { return m_operator; }

    // Camera operator RC radio / joystick (Dual OP live), polled on demand.
    bool poll_operator_joystick(int id) { return m_op.poll(id); }
    double operator_axis(int axis, const int* cal_min, const int* cal_max) const {
        return m_op.st.connected ? m_op.normalized(axis, cal_min, cal_max) : 0.0;
    }
    void begin_operator_calibration() { m_op.begin_calibration(); }
    void end_operator_calibration(int* cal_min, int* cal_max) { m_op.end_calibration(cal_min, cal_max); }

    // Normalized joystick axis (-1..1) with the configured calibration.
    double joystick_axis(const InputConfig& cfg, int axis) const {
        return m_pilot.st.connected ? m_pilot.normalized(axis, cfg.axis_min, cfg.axis_max) : 0.0;
    }

    // Edge-triggered gamepad buttons (pressed this poll).
    bool pad_pressed(WORD mask) const { return (m_pad.Gamepad.wButtons & mask) && !(m_prev_buttons & mask); }
    bool pad_held(WORD mask) const { return (m_pad.Gamepad.wButtons & mask) == mask; }

    // Joystick calibration: records min/max of every axis while active.
    void begin_calibration() { m_pilot.begin_calibration(); }
    void end_calibration(InputConfig& cfg) { m_pilot.end_calibration(cfg.axis_min, cfg.axis_max); }
    bool calibrating() const { return m_pilot.calibrating; }

private:
    void poll_gamepad(int user);

    using XInputGetStateFn = DWORD(WINAPI*)(DWORD, XINPUT_STATE*);
    XInputGetStateFn m_xinput_get_state{nullptr};

    XINPUT_STATE m_pad{};
    XINPUT_STATE m_operator{};
    WORD m_prev_buttons{};
    bool m_pad_connected{false};

public:
    void set_devices(std::vector<JoystickInfo> d) { m_devices = std::move(d); }
    const std::vector<JoystickInfo>& devices() const { return m_devices; }

private:
    std::vector<JoystickInfo> m_devices{};
    JoystickReader m_pilot{};
    JoystickReader m_op{};

    std::array<double, ChCount> m_channels{};
    int m_arm_switch{-1};
};

} // namespace fpv
