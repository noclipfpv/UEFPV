// Input.cpp - see Input.hpp
#include "Input.hpp"

#include <algorithm>
#include <cmath>

#include <mmsystem.h>

namespace fpv {

namespace {

// Set while we poll XInput ourselves, so our own UEVR XInput callback leaves the data alone.
thread_local bool t_self_polling = false;

double apply_deadzone(double v, double dz) {
    const double a = std::abs(v);
    if (a <= dz) return 0.0;
    return std::copysign((a - dz) / (1.0 - dz), v);
}

double thumb(SHORT v) {
    return std::clamp(static_cast<double>(v) / 32767.0, -1.0, 1.0);
}

} // namespace

InputManager::InputManager() {
    for (const auto* name : {L"xinput1_4.dll", L"xinput1_3.dll", L"xinput9_1_0.dll"}) {
        if (HMODULE mod = LoadLibraryW(name)) {
            m_xinput_get_state = reinterpret_cast<XInputGetStateFn>(GetProcAddress(mod, "XInputGetState"));
            if (m_xinput_get_state != nullptr) break;
        }
    }
}

void InputManager::on_xinput_get_state(uint32_t* retval, uint32_t /*user_index*/, XINPUT_STATE* state, bool block_game) {
    if (t_self_polling || !block_game || state == nullptr || retval == nullptr || *retval != ERROR_SUCCESS) {
        return;
    }

    // Keep the packet number so the game does not think the pad disconnected.
    state->Gamepad = XINPUT_GAMEPAD{};
}

void InputManager::poll_gamepad(int user) {
    m_prev_buttons = m_pad.Gamepad.wButtons;

    if (m_xinput_get_state == nullptr) {
        m_pad_connected = false;
        return;
    }

    XINPUT_STATE st{};
    t_self_polling = true;
    const DWORD res = m_xinput_get_state(static_cast<DWORD>(std::clamp(user, 0, 3)), &st);
    t_self_polling = false;

    m_pad_connected = (res == ERROR_SUCCESS);
    m_pad = m_pad_connected ? st : XINPUT_STATE{};
}

bool InputManager::poll_operator(int user) {
    m_operator = XINPUT_STATE{};
    if (m_xinput_get_state == nullptr) return false;
    t_self_polling = true;
    const DWORD res = m_xinput_get_state(static_cast<DWORD>(std::clamp(user, 0, 3)), &m_operator);
    t_self_polling = false;
    if (res != ERROR_SUCCESS) {
        m_operator = XINPUT_STATE{};
        return false;
    }
    return true;
}

std::vector<JoystickInfo> enumerate_joysticks() {
    std::vector<JoystickInfo> out;
    const UINT n = std::min<UINT>(joyGetNumDevs(), 16);
    for (UINT id = 0; id < n; ++id) {
        JOYINFOEX info{};
        info.dwSize = sizeof(info);
        info.dwFlags = JOY_RETURNALL;
        if (joyGetPosEx(id, &info) != JOYERR_NOERROR) continue;
        JoystickInfo j;
        j.id = static_cast<int>(id);
        JOYCAPSW caps{};
        if (joyGetDevCapsW(id, &caps, sizeof(caps)) == JOYERR_NOERROR) {
            char name[64]{};
            WideCharToMultiByte(CP_UTF8, 0, caps.szPname, -1, name, sizeof(name) - 1, nullptr, nullptr);
            j.name = name;
            j.axes = static_cast<int>(caps.wNumAxes);
            j.buttons = static_cast<int>(caps.wNumButtons);
            const UINT lo[AxisCount]{caps.wXmin, caps.wYmin, caps.wZmin, caps.wRmin, caps.wUmin, caps.wVmin};
            const UINT hi[AxisCount]{caps.wXmax, caps.wYmax, caps.wZmax, caps.wRmax, caps.wUmax, caps.wVmax};
            const DWORD pos[AxisCount]{info.dwXpos, info.dwYpos, info.dwZpos, info.dwRpos, info.dwUpos, info.dwVpos};
            for (int a = 0; a < AxisCount; ++a) {
                j.value[a] = hi[a] > lo[a] ? (static_cast<double>(pos[a]) - lo[a]) / (hi[a] - lo[a]) * 2.0 - 1.0 : 0.0;
            }
        }
        out.push_back(std::move(j));
    }
    return out;
}

bool JoystickReader::poll(int id) {
    JOYINFOEX info{};
    info.dwSize = sizeof(info);
    info.dwFlags = JOY_RETURNALL;

    if (joyGetPosEx(static_cast<UINT>(id), &info) != JOYERR_NOERROR) {
        st.connected = false;
        caps_for_id = -1;
        return false;
    }

    if (caps_for_id != id) {
        JOYCAPSW caps{};
        if (joyGetDevCapsW(static_cast<UINT_PTR>(id), &caps, sizeof(caps)) == JOYERR_NOERROR) {
            st.caps_min = {(int)caps.wXmin, (int)caps.wYmin, (int)caps.wZmin, (int)caps.wRmin, (int)caps.wUmin, (int)caps.wVmin};
            st.caps_max = {(int)caps.wXmax, (int)caps.wYmax, (int)caps.wZmax, (int)caps.wRmax, (int)caps.wUmax, (int)caps.wVmax};
        } else {
            st.caps_min.fill(0);
            st.caps_max.fill(65535);
        }
        caps_for_id = id;
    }

    st.connected = true;
    st.axes = {(int)info.dwXpos, (int)info.dwYpos, (int)info.dwZpos, (int)info.dwRpos, (int)info.dwUpos, (int)info.dwVpos};
    st.buttons = info.dwButtons;

    if (calibrating) {
        for (int a = 0; a < AxisCount; ++a) {
            cal_min[a] = std::min(cal_min[a], st.axes[a]);
            cal_max[a] = std::max(cal_max[a], st.axes[a]);
        }
    }
    return true;
}

double JoystickReader::normalized(int axis, const int* cfg_min, const int* cfg_max) const {
    axis = std::clamp(axis, 0, AxisCount - 1);
    int lo = cfg_min[axis];
    int hi = cfg_max[axis];
    if (hi <= lo) {
        lo = st.caps_min[axis];
        hi = st.caps_max[axis];
    }
    if (hi <= lo) return 0.0;

    const double t = (static_cast<double>(st.axes[axis]) - lo) / static_cast<double>(hi - lo);
    return std::clamp(t * 2.0 - 1.0, -1.0, 1.0);
}

void JoystickReader::begin_calibration() {
    calibrating = true;
    cal_min.fill(INT32_MAX);
    cal_max.fill(INT32_MIN);
}

void JoystickReader::end_calibration(int* cfg_min, int* cfg_max) {
    calibrating = false;
    for (int a = 0; a < AxisCount; ++a) {
        // Only keep axes that actually moved a meaningful amount.
        if (cal_max[a] > cal_min[a] && (cal_max[a] - cal_min[a]) > 1000) {
            cfg_min[a] = cal_min[a];
            cfg_max[a] = cal_max[a];
        }
    }
}

DroneInput InputManager::poll(const InputConfig& cfg) {
    // The gamepad is always polled so its button shortcuts work with any input source.
    poll_gamepad(cfg.xinput_user);

    std::array<double, ChCount> ch{};

    if (cfg.source == InputSource::Joystick) {
        m_pilot.poll(cfg.joystick_id);
        if (m_pilot.st.connected) {
            for (int c = 0; c < ChCount; ++c) {
                ch[c] = m_pilot.normalized(cfg.axis_map[c], cfg.axis_min, cfg.axis_max);
            }
            ch[ChThrottle] = (ch[ChThrottle] + 1.0) * 0.5;
            if (cfg.radio_deadzone > 0.0) {
                for (int c : {ChRoll, ChPitch, ChYaw}) ch[c] = apply_deadzone(ch[c], cfg.radio_deadzone);
            }
        }
    } else if (m_pad_connected) {
        const auto& g = m_pad.Gamepad;
        const double dz = cfg.stick_deadzone;
        const double ly = thumb(g.sThumbLY);

        switch (cfg.gamepad_throttle) {
        case GamepadThrottle::LeftStickUpper:
            ch[ChThrottle] = std::max(0.0, apply_deadzone(ly, dz));
            break;
        case GamepadThrottle::RightTrigger:
            ch[ChThrottle] = g.bRightTrigger / 255.0;
            break;
        case GamepadThrottle::LeftStickFull:
        default:
            ch[ChThrottle] = (ly + 1.0) * 0.5;
            break;
        }

        ch[ChYaw] = apply_deadzone(thumb(g.sThumbLX), dz);
        ch[ChRoll] = apply_deadzone(thumb(g.sThumbRX), dz);
        ch[ChPitch] = apply_deadzone(thumb(g.sThumbRY), dz);
    }

    for (int c = 0; c < ChCount; ++c) {
        if (!cfg.invert[c]) continue;
        ch[c] = (c == ChThrottle) ? 1.0 - ch[c] : -ch[c];
    }

    m_channels = ch;

    // Arm switch: read from the joystick even when the sticks come from the gamepad.
    int arm_switch = -1;
    if (cfg.arm_mode == 1) {
        if (cfg.source != InputSource::Joystick) {
            m_pilot.poll(cfg.joystick_id);
        }
        bool on = false;
        if (m_pilot.st.connected) {
            if (cfg.arm_source == 0) {
                const int b = std::clamp(cfg.arm_button, 0, 31);
                on = (m_pilot.st.buttons >> b) & 1u;
            } else {
                on = m_pilot.normalized(cfg.arm_axis, cfg.axis_min, cfg.axis_max) > cfg.arm_threshold;
            }
        }
        if (cfg.arm_invert) on = !on;
        arm_switch = (m_pilot.st.connected && on) ? 1 : 0;
    }
    m_arm_switch = arm_switch;

    DroneInput out;
    out.throttle = std::clamp(ch[ChThrottle], 0.0, 1.0);
    out.roll = ch[ChRoll];
    out.pitch = ch[ChPitch];
    out.yaw = ch[ChYaw];
    out.arm_switch = arm_switch;
    return out;
}

} // namespace fpv
