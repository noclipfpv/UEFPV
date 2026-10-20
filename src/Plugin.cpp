// UEVR FPV Drone - a universal FPV drone camera for Unreal Engine games (flat screen),
// as a UEVR plugin.
//
// How it works (flat screen, no VR runtime needed)
//  * UEVR's engine-tick / viewport callbacks only run when a VR runtime is loaded, so the plugin
//    brings its own game-thread tick (GameThreadTicker: Win32 message-pump hook).
//  * On that tick: read the sticks, step the flight model, move a spawned ACameraActor that is
//    the player's view target (GameCamera). The engine renders from it natively.
//  * Menu and OSD are drawn with ImGui from UEVR's on_present callback (works without VR).
//
// Default controls
//  F6            settings menu
//  F7            drone on / off (spawns at the current game camera)
//  F8            respawn at the launch point (motors off)
//  Gamepad: hold BACK + A = drone on / off, hold BACK + B = respawn
//  Arming: lower the throttle to arm after (re)spawn.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <windows.h>
#include <mmsystem.h>

#include "imgui/imgui_impl_dx11.h"
#include "imgui/imgui_impl_dx12.h"
#include "imgui/imgui_impl_win32.h"
#include "rendering/d3d11.hpp"
#include "rendering/d3d12.hpp"

#include "uevr/Plugin.hpp"

#include "DroneSim.hpp"
#include "FlightRecorder.hpp"
#include "Gimbal.hpp"
#include "SceneCaptureView.hpp"
#include "SecondWindow.hpp"
#include "GameCamera.hpp"
#include "GameThreadTicker.hpp"
#include "GameUiHider.hpp"
#include "Input.hpp"
#include "Settings.hpp"

using namespace uevr;
using namespace fpv;

namespace {

constexpr const char* kVersion = "0.7.7";

bool key_pressed(int vk, bool& was_down) {
    const bool down = (GetAsyncKeyState(vk) & 0x8000) != 0;
    const bool pressed = down && !was_down;
    was_down = down;
    return pressed;
}

bool game_window_focused() {
    DWORD pid{};
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    return pid == GetCurrentProcessId();
}

bool slider(const char* label, double& v, double lo, double hi, const char* fmt = "%.2f") {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &v, &lo, &hi, fmt);
}

const char* kAxisNames[AxisCount]{"X", "Y", "Z", "R", "U", "V"};
const char* kChannelNames[ChCount]{"Throttle", "Roll", "Pitch", "Yaw"};

} // namespace

class FpvDronePlugin : public uevr::Plugin {
public:
    // ------------------------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------------------------
    void on_initialize() override {
        m_settings_path = API::get()->get_persistent_dir(L"fpv_drone.ini");
        if (m_settings.load(m_settings_path)) {
            API::get()->log_info("[FPV Drone] settings loaded from %s", m_settings_path.string().c_str());
        } else {
            API::get()->log_info("[FPV Drone] no settings file yet, using defaults");
        }
        API::get()->log_info("[FPV Drone] v%s (flat screen) initialized. F6 = menu, F7 = drone on/off, F8 = respawn", kVersion);
    }

    // ------------------------------------------------------------------------------------
    // Game thread (called by GameThreadTicker at the start of every frame)
    // ------------------------------------------------------------------------------------
    // Newer UEVR builds hook UGameEngine::Tick even without a headset: then the engine tick drives the
    // drone and the message-hook ticker stands down (it may never run while UEVR owns the frame).
    void on_pre_engine_tick(API::UGameEngine* /*engine*/, float delta) override {
        if (!m_engine_tick_active.load(std::memory_order_relaxed)) {
            m_engine_tick_active = true; // from now on the message-hook ticker stands down for good
            API::get()->log_info("[FPV Drone] UEVR engine tick available: using it to drive the drone");
        }
        game_tick(std::clamp(static_cast<double>(delta), 0.0, 0.1));
    }

    void ticker_tick(double dt) {
        if (m_engine_tick_active.load(std::memory_order_relaxed)) return; // engine tick is driving
        game_tick(dt);
    }

    void game_tick(double dt) {
        if (!m_first_tick_logged) {
            m_first_tick_logged = true;
            API::get()->log_info("[FPV Drone] first game tick");
        }
        try {
            game_tick_impl(dt);
        } catch (...) {
            // Built with /EHa: also catches access violations in reflection calls.
            m_want_active = false;
            m_camera = GameCamera{};
            m_fault = true;
            API::get()->log_error("[FPV Drone] exception in game tick, drone disabled");
        }
    }

    void game_tick_impl(double dt) {
        // No Unreal object access here until the user turns the drone on (F7): some games (TRYP
        // FPV) are still starting up when UEVR injects, and looking up UObjects that early makes
        // UEVR scan for GUObjectArray before it exists and crash the game.

        // Shortcuts (keyboard only when the game has focus).
        if (game_window_focused()) {
            if (key_pressed(VK_F7, m_f7)) {
                m_want_active = !m_want_active.load();
                API::get()->log_info("[FPV Drone] F7: drone %s", m_want_active.load() ? "on" : "off");
            }
            if (key_pressed(VK_F8, m_f8)) m_respawn_requested = true;
            if (key_pressed(VK_F10, m_f10)) {
                std::scoped_lock _{m_state_mutex};
                m_settings.second.enabled = !m_settings.second.enabled;
            }

            // F9: tap = start / stop the Dual OP replay, hold 1 s = clear the recorded take.
            const bool f9 = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
            if (f9) {
                m_f9_hold += dt;
                if (m_f9_hold >= 1.0 && !m_f9_cleared) {
                    m_clear_take_requested = true;
                    m_f9_cleared = true;
                }
            } else {
                if (m_f9_hold > 0.0 && !m_f9_cleared) m_toggle_replay_requested = true;
                m_f9_hold = 0.0;
                m_f9_cleared = false;
            }
        }

        Settings settings;
        bool paused = false;
        {
            std::scoped_lock _{m_state_mutex};
            settings = m_settings;
            paused = m_menu_open.load() && m_pause_in_menu;
        }

        // Device polling must never happen under m_state_mutex: XInput / joystick calls go
        // through UEVR's hooks, which take UEVR's own locks (deadlock with the render thread).
        // Connected joysticks / radios, refreshed every 2 s. winmm ids can differ from one session
        // or game to the next (another controller plugged in first): if the pilot's radio is not at
        // its saved id, take the first other connected device.
        m_enum_wait -= dt;
        m_refresh_wait -= dt;
        if (m_enum_wait <= 0.0) {
            // Faster while the menu is open, so the axis values in the device list follow the sticks.
            m_enum_wait = m_menu_open.load() ? 0.25 : 2.0;
            if (m_refresh_wait <= 0.0) {
                m_refresh_wait = 3.0;
                joyConfigChanged(0); // refresh winmm's device list (radio plugged in after the game started)
            }
            m_input.set_devices(enumerate_joysticks());
            const auto& devs = m_input.devices();
            const auto& in = settings.input;
            const bool want_joy = in.source == InputSource::Joystick || in.arm_mode == 1;
            auto has = [&](int id) { return std::any_of(devs.begin(), devs.end(), [&](const JoystickInfo& j) { return j.id == id; }); };
            // Only once, at the first detection: afterwards the id is the user's choice in the menu.
            const bool first = !m_enum_done;
            m_enum_done = true;
            if (first && want_joy && !devs.empty() && !has(in.joystick_id)) {
                const bool op_radio = settings.cinema.gimbal_control == 1 && settings.cinema.operator_source == 1;
                const int old_id = in.joystick_id;
                for (const auto& j : devs) {
                    if (op_radio && j.id == settings.cinema.operator_joystick_id) continue;
                    {
                        std::scoped_lock _{m_state_mutex};
                        m_settings.input.joystick_id = j.id;
                    }
                    settings.input.joystick_id = j.id;
                    API::get()->log_info("[FPV Drone] radio not found at id %d, using joystick id %d (%s)", old_id, j.id,
                                         j.name.c_str());
                    break;
                }
            }
        }

        if (m_cal_begin_requested.exchange(false)) m_input.begin_calibration();
        if (m_op_cal_begin_requested.exchange(false)) m_input.begin_operator_calibration();
        const DroneInput input = m_input.poll(settings.input);
        if (settings.cinema.gimbal_control == 1 && settings.cinema.operator_source == 1) {
            m_operator_connected = m_input.poll_operator_joystick(settings.cinema.operator_joystick_id);
        }
        if (m_input.pad_held(XINPUT_GAMEPAD_BACK)) {
            if (m_input.pad_pressed(XINPUT_GAMEPAD_A)) m_want_active = !m_want_active.load();
            if (m_input.pad_pressed(XINPUT_GAMEPAD_B)) m_respawn_requested = true;
            if (m_input.pad_pressed(XINPUT_GAMEPAD_X)) m_toggle_replay_requested = true;
            if (m_input.pad_pressed(XINPUT_GAMEPAD_Y)) m_clear_take_requested = true;
        }
        {
            std::scoped_lock _{m_state_mutex};
            m_last_input = input;
            if (m_cal_end_requested.exchange(false)) m_input.end_calibration(m_settings.input);
            if (m_op_cal_end_requested.exchange(false)) {
                m_input.end_operator_calibration(m_settings.cinema.op_axis_min, m_settings.cinema.op_axis_max);
            }
            m_input_view = m_input.snapshot();
        }

        const bool want = m_want_active.load();

        // Game UI: hidden while flying (option), restored otherwise.
        m_ui_hider.update(want && settings.hide_game_ui && m_camera.active(), dt);

        // Turn off
        if (!want) {
            if (m_camera.active()) m_camera.deactivate();
            m_flying = false;
            publish_camera_status();
            return;
        }

        // First use: find the engine objects we need (retried at most every 2 s).
        if (!m_camera.resolved()) {
            m_resolve_wait -= dt;
            if (m_resolve_wait > 0.0) return;
            m_resolve_wait = 2.0;
            if (!m_camera.resolve()) {
                API::get()->log_info("[FPV Drone] engine objects not found yet, retrying");
                publish_camera_status();
                return;
            }
        }

        // Turn on
        if (!m_camera.active()) {
            CameraPov start{};
            if (!m_camera.read_game_pov(start)) {
                publish_camera_status();
                return; // no player yet (loading / menu without a controller)
            }
            if (!m_camera.activate(start, settings.freeze_player)) {
                m_want_active = false;
                publish_camera_status();
                return;
            }
            std::scoped_lock _{m_state_mutex};
            m_spawn_pov = start;
            m_sim.reset(start.rotation.yaw);
            m_gimbal.reset(start.rotation.yaw, 0.0, settings.cinema.gimbal.focal_mm);
            m_recorder.clear();
            m_flying = true;
            m_tilt_stab_init = false;
            m_respawn_requested = false;
        }

        // Respawn: back to the point where the drone was launched, motors off.
        if (m_respawn_requested.exchange(false)) {
            std::scoped_lock _{m_state_mutex};
            m_recorder.stop_playback();
            m_sim.reset(m_spawn_pov.rotation.yaw);
            m_gimbal.reset(m_spawn_pov.rotation.yaw, 0.0, m_gimbal.focal_mm());
        }

        const auto& ci = settings.cinema;
        const bool wing = settings.drone.flight_mode == FlightMode::Wing; // no gimbal / Dual OP on the wing
        const bool replay_mode = !wing && ci.camera_mode == 1 && ci.gimbal_control == 2;

        // Dual OP replay controls
        if (m_clear_take_requested.exchange(false)) {
            {
                std::scoped_lock _{m_state_mutex};
                m_recorder.clear();
            }
            API::get()->log_info("[FPV Drone] Dual OP take cleared");
        }
        if (m_toggle_replay_requested.exchange(false) && replay_mode) {
            std::scoped_lock _{m_state_mutex};
            if (m_recorder.playing()) {
                m_recorder.stop_playback();
            } else if (!m_recorder.empty()) {
                m_recorder.start_playback();
                // The operator starts the take from the drone's initial view.
                m_gimbal.reset(m_gimbal.orientation().yaw, m_gimbal.orientation().pitch, m_gimbal.focal_mm());
            }
        }
        if (!replay_mode && m_recorder.playing()) {
            std::scoped_lock _{m_state_mutex};
            m_recorder.stop_playback();
        }

        // Fly (or replay)
        const bool replaying = m_recorder.playing();
        Vec3 prev_m{}, next_m{};
        bool moved = false;
        {
            std::scoped_lock _{m_state_mutex};
            if (replaying) {
                if (!paused) m_recorder.advance(dt * ci.replay_speed, ci.replay_loop);
            } else {
                prev_m = m_sim.position_m();
                if (!paused) {
                    m_sim.update(m_last_input, settings.drone, dt);
                }
                next_m = m_sim.position_m();
                moved = !paused && m_sim.is_simulating();
            }
        }

        if (moved && settings.collision.enabled && m_camera.sweep_available()) {
            resolve_collisions(prev_m, next_m, settings);
        }

        // Record the take (Dual OP replay mode), after the collision response.
        if (replay_mode && moved) {
            std::scoped_lock _{m_state_mutex};
            m_recorder.record(dt, m_sim.position_m(), m_sim.body());
        }

        // Cameras: the main view (game camera) and, optionally, the second window's view.
        // Dual OP replay: the take can be flown with the pilot's FPV camera on the main screen; the
        // gimbal camera takes over the main screen during the replay.
        const bool use_gimbal = !wing && ci.camera_mode == 1 && !(replay_mode && !replaying && ci.record_view == 0);
        const auto& sv = settings.second;
        const bool second_view = sv.enabled;
        const bool second_gimbal = sv.view == 2 ? !use_gimbal : sv.view == 1;
        const bool need_gimbal = (!wing && ci.camera_mode == 1) || (second_view && second_gimbal);
        const GimbalInput gin = need_gimbal ? gimbal_input(settings, replaying) : GimbalInput{};

        CameraPov fpv_pov{}, gimbal_pov{};
        {
            std::scoped_lock _{m_state_mutex};
            if (const double f = m_focal_request.exchange(0.0); f > 0.0) m_gimbal.set_focal(f);

            const Vec3 pos_m = replaying ? m_recorder.position() : m_sim.position_m();
            const Basis body = replaying ? m_recorder.body() : m_sim.body();

            // Pilot FPV camera. Quad / wing: fixed to the frame with an uptilt. Inspire: the FPV camera
            // is stabilized on tilt only, so its pitch stays at the set angle from the horizon while
            // heading and roll follow the aircraft.
            fpv_pov.location = m_spawn_pov.location + pos_m * settings.units_per_meter;
            if (settings.drone.flight_mode == FlightMode::Cinema) {
                const Rotator b = body.to_rotator();
                // The stabilizer tracks the body pitch a little late (second-order, slightly
                // underdamped): when the aircraft pitches to accelerate or brake, the image follows
                // for an instant, then the tilt motor catches up with a tiny overshoot.
                const double lag = settings.drone.cinema.tilt_stab_lag_s;
                double residual = 0.0;
                if (lag > 0.005 && !paused) {
                    if (!m_tilt_stab_init) {
                        m_tilt_stab_est = b.pitch;
                        m_tilt_stab_vel = 0.0;
                        m_tilt_stab_init = true;
                    }
                    const double w = 1.0 / lag, zeta = 0.6;
                    const int n = std::max(1, static_cast<int>(std::ceil(dt / 0.004)));
                    const double h = dt / n;
                    for (int i = 0; i < n; ++i) {
                        const double acc = w * w * (b.pitch - m_tilt_stab_est) - 2.0 * zeta * w * m_tilt_stab_vel;
                        m_tilt_stab_vel += acc * h;
                        m_tilt_stab_est += m_tilt_stab_vel * h;
                    }
                    residual = std::clamp(b.pitch - m_tilt_stab_est, -15.0, 15.0);
                } else if (lag <= 0.005) {
                    m_tilt_stab_init = false;
                }
                fpv_pov.rotation = {settings.drone.cinema.camera_tilt_deg + residual, b.yaw, b.roll};
            } else {
                m_tilt_stab_init = false;
                Basis cam = body;
                cam.pitch_up((wing ? settings.drone.wing.camera_tilt_deg : settings.drone.camera_tilt_deg) * kDeg2Rad);
                fpv_pov.rotation = cam.to_rotator();
            }
            fpv_pov.fov = static_cast<float>(settings.fov_deg);

            // Gimbal camera
            if (need_gimbal) {
                const Vec3 cam_m = pos_m - Vec3{0, 0, ci.camera_drop_m};
                const bool follow = wing || ci.gimbal_control == 0 || (ci.gimbal_control == 2 && !replaying);
                const double heading = std::atan2(body.f.y, body.f.x) * kRad2Deg;
                if (!paused) {
                    const double gdt = replaying ? dt * ci.replay_speed : dt;
                    m_gimbal.update(gin, ci.gimbal, follow, heading, cam_m, gdt);
                }
                gimbal_pov.location = m_spawn_pov.location + cam_m * settings.units_per_meter;
                gimbal_pov.rotation = m_gimbal.orientation();
                gimbal_pov.fov = static_cast<float>(Gimbal::hfov_deg(m_gimbal.focal_mm(), ci.gimbal.sensor_width_mm));
            }

            m_overlay_needed = settings.show_osd || settings.show_crosshair ||
                               m_sim.telemetry().arm_state != ArmState::Armed;

            m_cine.focal = m_gimbal.focal_mm();
            m_cine.hfov = Gimbal::hfov_deg(m_gimbal.focal_mm(), ci.gimbal.sensor_width_mm);
            m_cine.drift = m_gimbal.drift_deg();
            m_cine.take_s = m_recorder.duration();
            m_cine.replaying = m_recorder.playing();
            m_cine.replay_s = m_recorder.playback_time();
            m_cine.take_frozen = m_recorder.frozen();
            m_cine.operator_connected = m_operator_connected;
            m_cine.main_gimbal = use_gimbal;
        }

        m_camera.update(use_gimbal ? gimbal_pov : fpv_pov, use_gimbal || settings.override_fov);

        // Second window: render the other view with a scene capture.
        if (second_view) {
            const auto pc = API::get()->get_player_controller(0);
            // Supersampled capture: rendered larger, filtered down by the window (much cleaner edges).
            const float ss = std::clamp(sv.fx.supersample, 1.0f, 2.0f);
            const int cw = (static_cast<int>(sv.width() * ss) + 1) & ~1;
            const int ch = (static_cast<int>(sv.height * ss) + 1) & ~1;
            if (m_capture.ensure(pc, cw, ch)) {
                m_window.set_source_exposure_available(m_capture.set_exposure_bias(m_window.source_exposure_ev()));
                m_capture.update(second_gimbal ? gimbal_pov : fpv_pov, sv.interval);
            }
        }
        {
            std::scoped_lock _{m_state_mutex};
            m_capture_status = m_capture.status();
        }
        publish_camera_status();
    }

    // Gimbal commands for this frame, depending on who operates the camera.
    GimbalInput gimbal_input(const Settings& settings, bool replaying) {
        const auto& ci = settings.cinema;
        GimbalInput g{};

        auto trig = [](BYTE v) { return v / 255.0; };
        auto thumb = [](SHORT v) { return std::clamp(v / 32767.0, -1.0, 1.0); };

        // Keyboard zoom works in every mode.
        if (game_window_focused()) {
            if (GetAsyncKeyState(VK_PRIOR) & 0x8000) g.zoom += 1.0;
            if (GetAsyncKeyState(VK_NEXT) & 0x8000) g.zoom -= 1.0;
        }

        // Game thread only: m_input is owned by this thread, no lock needed.
        const auto& pad = m_input.gamepad().Gamepad;
        const bool pad_ok = m_input.gamepad_connected();
        const bool triggers_free = settings.input.source != InputSource::XInput ||
                                   settings.input.gamepad_throttle != GamepadThrottle::RightTrigger;

        if (ci.gimbal_control == 1) {
            auto dzv = [](double v, double d) {
                const double a = std::abs(v);
                return a <= d ? 0.0 : std::copysign((a - d) / std::max(1.0 - d, 1e-3), v);
            };
            if (ci.operator_source == 1) {
                // Dual OP live: the operator's RC radio (polled with the pilot's input, see game tick).
                auto axis = [&](int a, bool inv) {
                    const double v = m_input.operator_axis(a, ci.op_axis_min, ci.op_axis_max);
                    return inv ? -v : v;
                };
                const double d = ci.op_radio_deadzone;
                g.pan = dzv(axis(ci.op_axis_pan, ci.op_invert_pan), d);
                g.tilt = dzv(axis(ci.op_axis_tilt, ci.op_invert_tilt), d);
                g.roll = dzv(axis(ci.op_axis_roll, ci.op_invert_roll), d);
                if (ci.op_axis_zoom >= 0) {
                    const double z = axis(ci.op_axis_zoom, ci.op_invert_zoom);
                    if (ci.op_zoom_mode == 1) {
                        g.has_zoom_absolute = true;
                        g.zoom_absolute = z;
                    } else {
                        g.zoom += dzv(z, std::max(d, 0.05));
                    }
                }
                return g;
            }
            // Dual OP live: a second gamepad operates the camera.
            m_operator_connected = m_input.poll_operator(ci.operator_user);
            const auto& op = m_input.operator_pad().Gamepad;
            const double d = ci.operator_deadzone;
            g.pan = dzv(thumb(op.sThumbRX), d);
            g.tilt = dzv(thumb(op.sThumbRY), d);
            g.roll = dzv(thumb(op.sThumbLX), d);
            g.zoom += trig(op.bRightTrigger) - trig(op.bLeftTrigger);
            return g;
        }

        m_operator_connected = false;
        if (ci.gimbal_control == 2 && replaying) {
            // Dual OP replay: the pilot's sticks now operate the camera.
            const auto ch = m_input.channels();
            g.pan = ch[ChRoll];
            g.tilt = ch[ChPitch];
            g.roll = ch[ChYaw];
        } else {
            // Pilot only: pan follows the heading; tilt on the D-pad or a radio pot.
            if (ci.tilt_axis >= 0 && m_input.joystick().connected) {
                g.has_tilt_absolute = true;
                g.tilt_absolute = m_input.joystick_axis(settings.input, ci.tilt_axis) * (ci.tilt_axis_invert ? -1.0 : 1.0);
            } else if (pad_ok) {
                if (pad.wButtons & XINPUT_GAMEPAD_DPAD_UP) g.tilt += 1.0;
                if (pad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) g.tilt -= 1.0;
            }
            if (pad_ok) {
                if (pad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) g.pan += 1.0;
                if (pad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) g.pan -= 1.0;
            }
        }
        if (pad_ok && triggers_free) {
            g.zoom += trig(pad.bRightTrigger) - trig(pad.bLeftTrigger);
        }
        return g;
    }

    // Sweeps the collision sphere along this frame's movement and lets the sim respond.
    // Up to 3 contacts per frame so the drone can slide along corners instead of sticking.
    void resolve_collisions(const Vec3& prev_m, const Vec3& next_m, const Settings& settings) {
        const double upm = settings.units_per_meter;
        const Vec3 origin = m_spawn_pov.location; // only written on the game thread
        constexpr double kSkin = 0.01;            // m, keeps the sphere just outside the surface

        Vec3 from = prev_m;
        Vec3 to = next_m;
        Vec3 final_pos = next_m;
        bool any_hit = false;

        for (int i = 0; i < 3; ++i) {
            if ((to - from).length() < 1e-5) {
                final_pos = from;
                break;
            }

            SweepHit hit{};
            if (!m_camera.sweep(origin + from * upm, origin + to * upm, settings.collision.radius_m * upm,
                                settings.trace_channel, hit)) {
                final_pos = to;
                break;
            }
            if (hit.start_penetrating) {
                final_pos = to; // started inside something (e.g. the player): let it fly out
                break;
            }

            const Vec3 n = hit.normal;
            const Vec3 contact_m = (hit.location - origin) / upm + n * kSkin;
            bool crashed = false;
            {
                std::scoped_lock _{m_state_mutex};
                crashed = m_sim.apply_contact(contact_m, n, settings.collision);
            }
            if (crashed) {
                API::get()->log_info("[FPV Drone] crash (%.1f m/s)", m_sim.telemetry().last_impact_ms);
            }
            any_hit = true;

            // Slide the rest of the movement along the surface.
            Vec3 remaining = (to - from) * (1.0 - std::clamp(hit.time, 0.0, 1.0));
            remaining -= n * dot(remaining, n);
            from = contact_m;
            to = contact_m + remaining;
            final_pos = from; // if we run out of iterations, stay at the last safe point
        }

        if (any_hit) {
            std::scoped_lock _{m_state_mutex};
            m_sim.set_position(final_pos);
        }
    }

    void publish_camera_status() {
        std::scoped_lock _{m_state_mutex};
        m_camera_status = m_camera.status();
        m_camera_reasserts = m_camera.view_target_reasserts();
    }

    // Hide the pad from the game while flying so the character does not move.
    void on_xinput_get_state(uint32_t* retval, uint32_t user_index, XINPUT_STATE* state) override {
        const bool block = m_flying && m_settings_block_pad.load();
        m_input.on_xinput_get_state(retval, user_index, state, block);
    }

    // ------------------------------------------------------------------------------------
    // Render thread: ImGui menu / OSD, and installing the game-thread ticker
    // ------------------------------------------------------------------------------------
    void on_present() override {
        std::scoped_lock _{m_imgui_mutex};

        const auto renderer = API::get()->param()->renderer;
        if (renderer->renderer_type == UEVR_RENDERER_D3D12 && renderer->command_queue == nullptr) {
            return;
        }

        if (!initialize_imgui()) {
            return;
        }

        // Second window (independent of the overlay)
        {
            // The render thread never waits for m_state_mutex (UEVR holds its own locks around
            // on_present); if the game thread has it right now, use last frame's values.
            std::unique_lock lk{m_state_mutex, std::try_to_lock};
            if (lk.owns_lock()) {
                if (m_window.closed_by_user()) m_settings.second.enabled = false;
                m_rt_second_enabled = m_settings.second.enabled;
                m_rt_fx = m_settings.second.fx;
                lk.unlock();
            }
            const bool want = m_rt_second_enabled && m_flying.load();
            const FeedEffect fx = m_rt_fx;
            if (want != m_window.visible()) m_window.set_visible(want);
            if (want) {
                m_window.present(renderer->renderer_type, m_capture.native_texture(), m_capture.generation(), fx);
            }
        }

        if (!m_ticker.installed() && !m_ticker_failed) {
            if (m_ticker.install(m_wnd, [this](double dt) { ticker_tick(dt); })) {
                API::get()->log_info("[FPV Drone] game thread tick installed (thread %lu)", m_ticker.game_thread_id());
            } else {
                m_ticker_failed = true;
                API::get()->log_error("[FPV Drone] could not install the game thread tick (error %lu)", GetLastError());
            }
        }

        if (game_window_focused() && key_pressed(VK_F6, m_f6)) {
            m_menu_open = !m_menu_open.load();
        }

        // Only touch the game's frame when there is something to show: drawing into the back
        // buffer every frame can flicker with frame generation (DLSS / FSR) enabled.
        if (!m_menu_open.load() && !(m_flying.load() && m_overlay_needed.load())) {
            return;
        }

        // Build a new UI frame only if the state is free right now; otherwise re-render the
        // previous frame's draw data (never block the render thread).
        {
            std::unique_lock lk{m_state_mutex, std::try_to_lock};
            if (lk.owns_lock() || ImGui::GetDrawData() == nullptr) {
                if (!lk.owns_lock()) lk.lock(); // very first frame only
                if (renderer->renderer_type == UEVR_RENDERER_D3D11) {
                    ImGui_ImplDX11_NewFrame();
                } else {
                    ImGui_ImplDX12_NewFrame();
                }
                ImGui_ImplWin32_NewFrame();
                ImGui::NewFrame();
                ImGui::GetIO().MouseDrawCursor = m_menu_open.load();
                draw_ui();
                m_settings_block_pad = m_settings.block_game_gamepad;
                ImGui::EndFrame();
                ImGui::Render();
            }
        }

        if (renderer->renderer_type == UEVR_RENDERER_D3D11) {
            g_d3d11.render_imgui();
        } else {
            g_d3d12.render_imgui();
        }
    }

    void on_device_reset() override {
        std::scoped_lock _{m_imgui_mutex};
        m_window.on_device_reset();
        if (!m_backend_ready) return;

        const auto renderer = API::get()->param()->renderer;
        if (renderer->renderer_type == UEVR_RENDERER_D3D11) {
            ImGui_ImplDX11_Shutdown();
            g_d3d11 = {};
        } else if (renderer->renderer_type == UEVR_RENDERER_D3D12) {
            g_d3d12.reset();
            ImGui_ImplDX12_Shutdown();
            g_d3d12 = {};
        }
        m_backend_ready = false;
    }

    bool on_message(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) override {
        std::scoped_lock _{m_imgui_mutex};
        if (!m_context_ready) {
            return true;
        }
        ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam);
        if (!m_menu_open.load()) {
            return true;
        }
        const auto& io = ImGui::GetIO();
        return !(io.WantCaptureMouse || io.WantCaptureKeyboard);
    }

private:
    // ------------------------------------------------------------------------------------
    // ImGui setup
    // ------------------------------------------------------------------------------------
    bool initialize_imgui() {
        if (m_backend_ready) return true;

        const auto renderer = API::get()->param()->renderer;

        if (!m_context_ready) {
            DXGI_SWAP_CHAIN_DESC swap_desc{};
            auto swapchain = static_cast<IDXGISwapChain*>(renderer->swapchain);
            if (swapchain == nullptr || FAILED(swapchain->GetDesc(&swap_desc))) {
                return false;
            }
            m_wnd = swap_desc.OutputWindow;

            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            static const auto imgui_ini = API::get()->get_persistent_dir(L"fpv_drone_imgui.ini").string();
            ImGui::GetIO().IniFilename = imgui_ini.c_str();

            if (!ImGui_ImplWin32_Init(m_wnd)) {
                return false;
            }
            m_context_ready = true;
        }

        if (renderer->renderer_type == UEVR_RENDERER_D3D11) {
            if (!g_d3d11.initialize()) return false;
        } else if (renderer->renderer_type == UEVR_RENDERER_D3D12) {
            if (!g_d3d12.initialize()) return false;
        } else {
            return false;
        }

        m_backend_ready = true;
        API::get()->log_info("[FPV Drone] overlay initialized (%s)",
                             renderer->renderer_type == UEVR_RENDERER_D3D11 ? "D3D11" : "D3D12");
        return true;
    }

    // ------------------------------------------------------------------------------------
    // UI (called with m_state_mutex held)
    // ------------------------------------------------------------------------------------
    void draw_ui() {
        if (m_flying) {
            draw_osd();
        }
        if (m_menu_open.load()) {
            draw_menu();
        }
    }

    void draw_osd() {
        const auto t = m_sim.telemetry();
        const auto& io = ImGui::GetIO();
        auto dl = ImGui::GetForegroundDrawList();

        const ImU32 white = IM_COL32(255, 255, 255, 230);
        const ImU32 shadow = IM_COL32(0, 0, 0, 160);
        const ImU32 warn = IM_COL32(255, 80, 60, 255);
        const ImVec2 c{std::floor(io.DisplaySize.x * 0.5f), std::floor(io.DisplaySize.y * 0.5f)};

        // Betaflight-style crosshair: a plain cross, outlined so it reads on any background.
        if (m_settings.show_crosshair) {
            const float r = 9.0f;
            dl->AddLine({c.x - r - 1, c.y}, {c.x + r + 2, c.y}, shadow, 4.0f);
            dl->AddLine({c.x, c.y - r - 1}, {c.x, c.y + r + 2}, shadow, 4.0f);
            dl->AddLine({c.x - r, c.y}, {c.x + r + 1, c.y}, white, 2.0f);
            dl->AddLine({c.x, c.y - r}, {c.x, c.y + r + 1}, white, 2.0f);
        }

        auto centered = [&](const char* msg, float dy, ImU32 col) {
            const ImVec2 sz = ImGui::CalcTextSize(msg);
            const ImVec2 p{c.x - sz.x * 0.5f, c.y + dy};
            dl->AddText({p.x + 1, p.y + 1}, shadow, msg);
            dl->AddText(p, col, msg);
        };

        // Warnings are always shown, even with the OSD off.
        if (m_menu_open.load() && m_pause_in_menu) {
            centered("PAUSED - close the menu (F6) to fly", 40.0f, warn);
        }
        const bool use_switch = m_input_view.arm_switch >= 0;
        const char* arm_msg = nullptr;
        switch (t.arm_state) {
        case ArmState::WaitingForThrottleLow:
        case ArmState::Disarmed:
            if (!use_switch) arm_msg = "THROTTLE LOW TO ARM";
            else if (t.throttle_blocks_arming) arm_msg = "THROTTLE HIGH - LOWER IT TO ARM";
            else arm_msg = t.arm_state == ArmState::Disarmed ? "DISARMED" : "ARM SWITCH TO ARM";
            break;
        case ArmState::Crashed:
            arm_msg = use_switch ? (t.throttle_blocks_arming ? "CRASHED - THROTTLE HIGH" : "CRASHED - ARM SWITCH OFF THEN ON")
                                 : "CRASHED - THROTTLE LOW TO RE-ARM";
            break;
        case ArmState::Armed:
            if (t.wing_waiting_launch) arm_msg = "THROTTLE UP TO LAUNCH";
            break;
        }
        if (arm_msg != nullptr) {
            centered(arm_msg, 60.0f, warn);
        }

        if (!m_settings.show_osd) {
            return;
        }

        // Cinema / Dual OP status (top left), part of the OSD
        const auto& ci = m_settings.cinema;
        if (ci.camera_mode == 1 && m_settings.drone.flight_mode != FlightMode::Wing) {
            char st[200]{};
            auto mmss = [](double sec, char* out, size_t n) {
                const int s = static_cast<int>(sec);
                std::snprintf(out, n, "%02d:%02d", s / 60, s % 60);
            };
            char a[16], b[16];
            if (ci.gimbal_control == 2) {
                mmss(m_cine.replay_s, a, sizeof(a));
                mmss(m_cine.take_s, b, sizeof(b));
                if (m_cine.replaying) {
                    std::snprintf(st, sizeof(st), "> REPLAY %s / %s  x%.2f", a, b, ci.replay_speed);
                } else if (m_cine.take_frozen) {
                    std::snprintf(st, sizeof(st), "TAKE %s READY - F9 replay, hold F9 clear", b);
                } else if (m_cine.take_s > 0.0) {
                    std::snprintf(st, sizeof(st), "REC %s  (F9 = replay)", b);
                }
            } else if (ci.gimbal_control == 1 && !m_cine.operator_connected) {
                std::snprintf(st, sizeof(st), "CAMERA OPERATOR PAD NOT FOUND (XInput user %d)", ci.operator_user);
            }
            if (st[0] != 0) {
                const ImU32 col = (m_cine.replaying || ci.gimbal_control == 1) ? warn : IM_COL32(255, 60, 60, 255);
                dl->AddText({41, 41}, shadow, st);
                dl->AddText({40, 40}, col, st);
            }
        }


        char buf[160];
        const float x = 40.0f;
        float y = io.DisplaySize.y - 160.0f;
        auto line = [&](const char* text) {
            dl->AddText({x + 1, y + 1}, shadow, text);
            dl->AddText({x, y}, white, text);
            y += 20.0f;
        };
        std::snprintf(buf, sizeof(buf), "%s  %s",
                      m_settings.drone.flight_mode == FlightMode::Cinema ? "CINE"
                      : m_settings.drone.flight_mode == FlightMode::Wing ? "WING"
                                                                         : (m_settings.drone.angle_mode ? "ANGLE" : "ACRO"),
                      t.arm_state == ArmState::Armed     ? "ARMED"
                      : t.arm_state == ArmState::Crashed ? "CRASHED"
                                                         : "DISARMED");
        line(buf);
        std::snprintf(buf, sizeof(buf), "THR %3.0f%%", m_last_input.throttle * 100.0);
        line(buf);
        std::snprintf(buf, sizeof(buf), "SPD %5.1f km/h", t.speed_ms * 3.6);
        line(buf);
        std::snprintf(buf, sizeof(buf), "ALT %+6.1f m  VS %+5.1f m/s", t.altitude_m, t.vertical_speed_ms);
        line(buf);
        std::snprintf(buf, sizeof(buf), "DST %6.1f m", t.distance_m);
        line(buf);
        if (m_settings.drone.flight_mode == FlightMode::Wing) {
            std::snprintf(buf, sizeof(buf), "AOA %+5.1f deg%s", t.aoa_deg,
                          std::abs(t.aoa_deg) > m_settings.drone.wing.stall_deg ? "  STALL" : "");
            line(buf);
        }
        if (m_cine.main_gimbal) {
            std::snprintf(buf, sizeof(buf), "%.0f mm  (%.0f deg)", m_cine.focal, m_cine.hfov);
            line(buf);
        }
    }

    void draw_menu() {
        ImGui::SetNextWindowSize({460, 640}, ImGuiCond_FirstUseEver);
        bool open = true;
        if (!ImGui::Begin("FPV Drone", &open)) {
            ImGui::End();
            m_menu_open = open;
            return;
        }
        m_menu_open = open;

        auto& s = m_settings;
        auto& d = s.drone;

        ImGui::Text("FPV Drone v%s (flat screen)", kVersion);
        if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (m_ticker.installed()) {
                ImGui::Text("Game thread tick: %.0f Hz (%llu ticks)", m_ticker.measured_hz(),
                            static_cast<unsigned long long>(m_ticker.tick_count()));
            } else {
                ImGui::TextColored({1, 0.4f, 0.3f, 1}, "Game thread tick: %s", m_ticker_failed ? "FAILED (see UEVR log)" : "not installed yet");
            }
            ImGui::Text("Drone driven by: %s", m_engine_tick_active.load() ? "UEVR engine tick" : "message-hook tick");
            ImGui::Text("Camera: %s%s", m_camera_status.c_str(), m_want_active.load() ? "" : "  (press F7)");
            if (m_camera_reasserts > 0) {
                ImGui::Text("View target taken back from the game %u time(s)", m_camera_reasserts);
            }
            if (m_fault) {
                ImGui::TextColored({1, 0.4f, 0.3f, 1}, "An error disabled the drone (see UEVR log).");
            }
        }

        if (ImGui::Button(m_want_active.load() ? "Drone OFF (F7)" : "Drone ON (F7)")) m_want_active = !m_want_active.load();
        ImGui::SameLine();
        if (ImGui::Button("Respawn (F8)")) m_respawn_requested = true; // back to the launch point
        ImGui::SameLine();
        ImGui::Checkbox("Pause in menu", &m_pause_in_menu);
        if (m_pause_in_menu) ImGui::TextDisabled("Flight (and arming) is paused while this menu is open.");

        if (ImGui::CollapsingHeader("Flight", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Angle mode (self-level)", &d.angle_mode);
            if (d.angle_mode) {
                slider("Max angle (deg)", d.max_angle_deg, 15.0, 85.0, "%.0f");
            }
            draw_rates(d);
        }

        if (ImGui::CollapsingHeader("Flight mode / Cinema / Dual OP", ImGuiTreeNodeFlags_DefaultOpen)) {
            draw_cinema(s);
        }

        if (ImGui::CollapsingHeader("Second window (F10)")) {
            draw_second_view(s);
        }

        if (ImGui::CollapsingHeader("Collisions")) {
            auto& c = s.collision;
            ImGui::Checkbox("Collisions", &c.enabled);
            if (!m_camera.sweep_available() && m_camera.resolved()) {
                ImGui::TextColored({1, 0.4f, 0.3f, 1}, "Not available in this game (see UEVR log).");
            }
            if (c.enabled) {
                slider("Drone radius (m)", c.radius_m, 0.03, 0.5, "%.2f");
                slider("Bounce", c.restitution, 0.0, 1.0, "%.2f");
                slider("Surface friction", c.friction, 0.0, 1.0, "%.2f");
                slider("Tumble on impact", c.spin_on_impact, 0.0, 2.0, "%.2f");
                ImGui::Combo("Trace channel", &s.trace_channel, "Visibility\0Camera\0");
                ImGui::Checkbox("Crash (disarm) on hard impacts", &c.crash_enabled);
                if (c.crash_enabled) slider("Crash impact speed (m/s)", c.crash_speed_ms, 2.0, 40.0, "%.1f");
                ImGui::Text("Last impact: %.1f m/s", m_sim.telemetry().last_impact_ms);
            }
        }

        if (ImGui::CollapsingHeader("Quad")) {
            slider("Thrust / weight", d.thrust_to_weight, 1.5, 15.0, "%.1f");
            slider("Idle throttle", d.idle_throttle, 0.0, 0.2, "%.2f");
            slider("Throttle expo", d.throttle_expo, 0.0, 1.0);
            slider("Motor response (s)", d.motor_time_constant, 0.005, 0.2, "%.3f");
            slider("Rate response (s)", d.rate_time_constant, 0.005, 0.2, "%.3f");
            slider("Drag (quadratic)", d.drag_quadratic, 0.0, 0.1, "%.3f");
            slider("Drag (linear)", d.drag_linear, 0.0, 1.0, "%.2f");
            slider("Vertical drag x", d.vertical_drag_mult, 0.5, 4.0, "%.1f");
            slider("Gravity (m/s2)", d.gravity, 1.0, 20.0, "%.2f");
        }

        if (ImGui::CollapsingHeader("Camera")) {
            slider("Camera uptilt (deg)", d.camera_tilt_deg, -10.0, 60.0, "%.0f");
            ImGui::Checkbox("Override FOV", &s.override_fov);
            if (s.override_fov) slider("FOV (deg, horizontal)", s.fov_deg, 60.0, 160.0, "%.0f");
            ImGui::Checkbox("Show OSD (telemetry)", &s.show_osd);
            ImGui::Checkbox("Show crosshair", &s.show_crosshair);
            ImGui::Checkbox("Hide the game's UI while flying", &s.hide_game_ui);
        }

        if (ImGui::CollapsingHeader("Input", ImGuiTreeNodeFlags_DefaultOpen)) {
            draw_input_settings();
            draw_arming_settings();
        }

        if (ImGui::CollapsingHeader("World / Game")) {
            slider("Units per meter", s.units_per_meter, 1.0, 1000.0, "%.0f");
            ImGui::TextDisabled("100 = Unreal default (1 unit = 1 cm)");
            ImGui::Checkbox("Hide gamepad from the game while flying", &s.block_game_gamepad);
            ImGui::Checkbox("Freeze the player while flying (applies on next spawn)", &s.freeze_player);
        }

        ImGui::Separator();
        if (ImGui::Button("Save")) {
            m_status = s.save(m_settings_path) ? "Saved." : "Save FAILED.";
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload")) {
            Settings fresh{};
            m_status = fresh.load(m_settings_path) ? "Reloaded." : "No settings file.";
            m_settings = fresh;
        }
        ImGui::SameLine();
        if (ImGui::Button("Defaults")) {
            m_settings = Settings{};
            m_status = "Defaults restored (not saved).";
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(m_status.c_str());

        ImGui::End();
    }

    void draw_second_view(Settings& s) {
        auto& sv = s.second;
        ImGui::Checkbox("Show the second window (F10)", &sv.enabled);
        ImGui::Combo("Shows", &sv.view, "Pilot FPV camera\0Gimbal camera\0Auto: the view not on the main screen\0");
        ImGui::TextDisabled("Main screen now: %s. Double-click the window: fullscreen on its monitor.",
                            m_cine.main_gimbal ? "gimbal camera" : "FPV camera");

        ImGui::SeparatorText("Performance");
        static const int kLines[]{240, 360, 480, 540, 720, 1080};
        int line_index = 2;
        for (int i = 0; i < 6; ++i) if (kLines[i] == sv.height) line_index = i;
        if (ImGui::Combo("Resolution", &line_index, "240p\0360p\0480p\0540p\0720p\0001080p\0")) {
            sv.height = kLines[line_index];
        }
        ImGui::Combo("Aspect", &sv.aspect, "4:3 (analog FPV)\00016:9\0");
        int ss_index = sv.fx.supersample >= 1.9f ? 2 : (sv.fx.supersample >= 1.4f ? 1 : 0);
        if (ImGui::Combo("Supersampling", &ss_index, "Off\0001.5x (recommended)\0002x (sharpest, heavier)\0")) {
            sv.fx.supersample = ss_index == 2 ? 2.0f : (ss_index == 1 ? 1.5f : 1.0f);
        }
        ImGui::Text("Capture rendered at: %d x %d", static_cast<int>(sv.width() * sv.fx.supersample),
                    static_cast<int>(sv.height * sv.fx.supersample));
        int refresh = std::clamp(sv.interval, 1, 3) - 1;
        if (ImGui::Combo("Refresh", &refresh, "Every frame\0Every 2nd frame\0Every 3rd frame\0")) sv.interval = refresh + 1;
        ImGui::TextDisabled("The second view renders the scene again: lower resolution / refresh = more FPS.");

        ImGui::SeparatorText("Look");
        int look = sv.fx.analog ? 1 : 0;
        if (ImGui::Combo("Feed look", &look, "Clean (no effect)\0Analog feed simulation\0")) sv.fx.analog = look == 1;
        if (sv.fx.analog) {
            ImGui::SliderFloat("Noise", &sv.fx.noise, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Softness", &sv.fx.softness, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("Scanlines", &sv.fx.scanlines, 0.0f, 1.0f, "%.2f");
        }
        ImGui::SliderFloat("Sharpness", &sv.fx.sharpness, 0.0f, 1.0f, "%.2f");
        ImGui::Checkbox("Auto exposure", &sv.fx.auto_exposure);
        ImGui::SameLine();
        ImGui::TextDisabled("(scene captures often come out darker than the game)");
        ImGui::SliderFloat("Exposure (EV)", &sv.fx.exposure, -2.0f, 3.0f, "%+.1f");
        ImGui::SliderFloat("Gamma", &sv.fx.gamma, 0.5f, 2.5f, "%.2f");

        ImGui::Text("Capture: %s", m_capture_status.c_str());
        ImGui::Text("Window: %s", m_window.status().c_str());
    }

    // Camera operator on an RC radio (Dual OP live).
    void draw_operator_radio(Settings& s) {
        auto& ci = s.cinema;
        const auto& joy = m_input_view.op_joy;
        ImGui::SliderInt("Operator joystick id", &ci.operator_joystick_id, 0, 15);
        for (const auto& j : m_input_view.devices) {
            char label[192];
            std::snprintf(label, sizeof(label), "id %d: %s (%d axes)%s%s", j.id, j.name.c_str(), j.axes,
                          j.id == ci.operator_joystick_id ? "  <- operator" : "",
                          s.input.source == InputSource::Joystick && j.id == s.input.joystick_id ? "  (pilot)" : "");
            if (ImGui::Selectable(label, j.id == ci.operator_joystick_id)) ci.operator_joystick_id = j.id;
        }
        ImGui::Text("Operator radio: %s", joy.connected ? "connected" : "not found");
        if (s.input.source == InputSource::Joystick && ci.operator_joystick_id == s.input.joystick_id) {
            ImGui::TextColored({1, 0.6f, 0.3f, 1}, "Same device as the pilot: pick the operator's radio id.");
        }
        if (joy.connected) {
            for (int a = 0; a < AxisCount; ++a) {
                const bool cal = ci.op_axis_max[a] > ci.op_axis_min[a];
                const int lo = cal ? ci.op_axis_min[a] : joy.caps_min[a];
                const int hi = cal ? ci.op_axis_max[a] : joy.caps_max[a];
                const float frac = hi > lo ? float(joy.axes[a] - lo) / float(hi - lo) : 0.0f;
                char label[32];
                std::snprintf(label, sizeof(label), "op %s %d", kAxisNames[a], joy.axes[a]);
                ImGui::ProgressBar(frac, {-1, 0}, label);
            }
        }
        auto map = [&](const char* name, int& axis, bool& inv, bool allow_none) {
            ImGui::PushID(name);
            ImGui::SetNextItemWidth(110);
            if (allow_none) {
                int sel = axis + 1;
                ImGui::Combo(name, &sel, "None\0X\0Y\0Z\0R\0U\0V\0");
                axis = sel - 1;
            } else {
                ImGui::Combo(name, &axis, kAxisNames, AxisCount);
            }
            ImGui::SameLine();
            ImGui::Checkbox("Invert", &inv);
            ImGui::PopID();
        };
        map("Pan", ci.op_axis_pan, ci.op_invert_pan, false);
        map("Tilt", ci.op_axis_tilt, ci.op_invert_tilt, false);
        map("Roll", ci.op_axis_roll, ci.op_invert_roll, false);
        map("Zoom", ci.op_axis_zoom, ci.op_invert_zoom, true);
        if (ci.op_axis_zoom >= 0) {
            ImGui::Combo("Zoom control", &ci.op_zoom_mode, "Zoom speed (centered stick)\0Focal length follows the stick / pot\0");
        }
        slider("Operator stick deadzone##radio", ci.op_radio_deadzone, 0.0, 0.2, "%.2f");
        if (!m_input_view.op_calibrating) {
            if (ImGui::Button("Calibrate operator radio")) m_op_cal_begin_requested = true;
            ImGui::SameLine();
            if (ImGui::Button("Clear##opcal")) {
                std::fill(std::begin(ci.op_axis_min), std::end(ci.op_axis_min), 0);
                std::fill(std::begin(ci.op_axis_max), std::end(ci.op_axis_max), 0);
            }
        } else {
            ImGui::TextColored({1, 0.8f, 0.2f, 1}, "Move every stick of the operator radio to all its corners, then:");
            if (ImGui::Button("Finish operator calibration")) m_op_cal_end_requested = true;
        }
    }

    void draw_cinema(Settings& s) {
        auto& d = s.drone;
        auto& ci = s.cinema;
        auto& g = ci.gimbal;

        int flight = static_cast<int>(d.flight_mode);
        ImGui::Combo("Flight mode", &flight, "FPV (acro)\0Cinema (Inspire-like)\0Flying wing (Mojito-like)\0");
        d.flight_mode = static_cast<FlightMode>(flight);
        if (d.flight_mode == FlightMode::Wing) {
            auto& w = d.wing;
            ImGui::TextDisabled("Same sticks as FPV acro (your rates, scaled). Arm, then throttle up: hand launch.");
            ImGui::TextDisabled("Centered sticks: it glides along its flight path. Throttle = climb. Slow = less control.");
            ImGui::TextDisabled("FPV camera only (no gimbal / Dual OP on the wing).");
            slider("Camera tilt (deg)", w.camera_tilt_deg, -10.0, 40.0, "%.0f");
            if (ImGui::TreeNode("Wing setup")) {
                slider("Roll rate (x FPV rates)", w.roll_rate_scale, 0.1, 1.5, "%.2f");
                slider("Pitch rate (x FPV rates)", w.pitch_rate_scale, 0.1, 1.5, "%.2f");
                slider("Yaw rate (x FPV rates)", w.yaw_rate_scale, 0.0, 1.0, "%.2f");
                slider("Thrust / weight", w.thrust_to_weight, 0.3, 4.0, "%.2f");
                slider("Prop pitch speed (m/s)", w.prop_speed_ms, 15.0, 80.0, "%.0f");
                ImGui::SameLine(); ImGui::TextDisabled("%.0f km/h", w.prop_speed_ms * 3.6);
                slider("Weight (kg)", w.mass_kg, 0.3, 4.0, "%.2f");
                slider("Wing area (m2)", w.wing_area_m2, 0.08, 0.6, "%.2f");
                slider("Drag (CD0)", w.cd0, 0.01, 0.08, "%.3f");
                slider("Stall angle (deg)", w.stall_deg, 6.0, 25.0, "%.0f");
                slider("Trim angle of attack (deg)", w.trim_aoa_deg, 0.0, 10.0, "%.1f");
                slider("Pitch stability", w.pitch_stability, 0.0, 10.0, "%.1f");
                slider("Yaw stability", w.yaw_stability, 0.0, 10.0, "%.1f");
                slider("Gyro hold (armed)", w.gyro_hold, 0.0, 0.95, "%.2f");
                slider("Full control speed (m/s)", w.control_speed_ms, 4.0, 30.0, "%.0f");
                slider("Launch speed (m/s)", w.launch_speed_ms, 4.0, 25.0, "%.0f");
                if (ImGui::Button("Reset wing setup")) w = WingParams{};
                ImGui::TreePop();
            }
            return;
        }
        if (d.flight_mode == FlightMode::Cinema) {
            ImGui::TextDisabled("Sticks = speeds, centered = hold position. Throttle centered = hold altitude.");
            slider("FPV camera tilt (deg)", d.cinema.camera_tilt_deg, -45.0, 30.0, "%.0f");
            ImGui::SameLine();
            ImGui::TextDisabled("(stabilized on tilt only)");
            if (ImGui::TreeNode("Cinema flight")) {
                slider("Max speed (m/s)", d.cinema.max_speed_ms, 2.0, 40.0, "%.1f");
                ImGui::SameLine(); ImGui::TextDisabled("%.0f km/h", d.cinema.max_speed_ms * 3.6);
                slider("Max climb (m/s)", d.cinema.max_climb_ms, 1.0, 15.0, "%.1f");
                slider("Max descent (m/s)", d.cinema.max_descent_ms, 1.0, 15.0, "%.1f");
                slider("Max yaw rate (deg/s)", d.cinema.max_yaw_rate_dps, 10.0, 250.0, "%.0f");
                slider("Max acceleration (m/s2)", d.cinema.max_accel_ms2, 1.0, 20.0, "%.1f");
                slider("Smoothness (s)", d.cinema.response_s, 0.1, 3.0, "%.2f");
                slider("Yaw smoothness (s)", d.cinema.yaw_response_s, 0.05, 2.0, "%.2f");
                slider("Stick expo", d.cinema.stick_expo, 0.0, 1.0, "%.2f");
                slider("Max body tilt (deg)", d.cinema.max_tilt_deg, 5.0, 45.0, "%.0f");
                slider("Body tilt smoothness (s)", d.cinema.lean_response_s, 0.05, 1.5, "%.2f");
                slider("FPV tilt stabilizer delay (s)", d.cinema.tilt_stab_lag_s, 0.0, 0.2, "%.3f");
                ImGui::SameLine();
                ImGui::TextDisabled("(0 = perfectly stable)");
                ImGui::TreePop();
            }
        }

        ImGui::Combo("Camera", &ci.camera_mode, "FPV camera (fixed to the frame)\0Gimbal (stabilized)\0");
        if (ci.camera_mode != 1) {
            return;
        }

        ImGui::Combo("Camera operator", &ci.gimbal_control,
                     "Pilot (gimbal follows heading)\0Dual OP - second operator (live)\0Dual OP - replay the flight\0");
        switch (ci.gimbal_control) {
        case 0:
            ImGui::TextDisabled("Tilt: D-pad up/down (or a radio pot below). Pan offset: D-pad left/right.");
            {
                int axis = ci.tilt_axis + 1;
                const char* axes = "Gamepad D-pad\0Joystick X\0Joystick Y\0Joystick Z\0Joystick R\0Joystick U\0Joystick V\0";
                ImGui::Combo("Tilt control", &axis, axes);
                ci.tilt_axis = axis - 1;
                if (ci.tilt_axis >= 0) ImGui::Checkbox("Invert tilt pot", &ci.tilt_axis_invert);
            }
            break;
        case 1:
            ImGui::Combo("Operator input", &ci.operator_source, "Gamepad\0RC radio / joystick\0");
            if (ci.operator_source == 0) {
                ImGui::TextDisabled("Right stick = pan / tilt, left stick X = roll, RT / LT = zoom in / out.");
                ImGui::SliderInt("Operator XInput user", &ci.operator_user, 0, 3);
                slider("Operator stick deadzone", ci.operator_deadzone, 0.0, 0.3, "%.2f");
                ImGui::Text("Operator gamepad: %s", m_cine.operator_connected ? "connected" : "not found");
                if (s.input.source == InputSource::XInput && ci.operator_user == s.input.xinput_user) {
                    ImGui::TextColored({1, 0.6f, 0.3f, 1}, "Same gamepad as the pilot: pick another user.");
                }
            } else {
                draw_operator_radio(s);
            }
            break;
        case 2:
        default:
            ImGui::Combo("View while flying the take", &ci.record_view, "Pilot FPV camera\0Gimbal camera\0");
            ImGui::TextDisabled("Flight mode above (FPV acro or Cinema) is used for the take.");
            ImGui::TextDisabled("1) Fly: the flight is recorded.  2) F9 (or BACK+X): replay it while you");
            ImGui::TextDisabled("operate the camera with the same sticks: roll = pan, pitch = tilt, yaw = roll.");
            ImGui::TextDisabled("Hold F9 1 s (or BACK+Y) to clear the take and record a new one.");
            slider("Replay speed", ci.replay_speed, 0.1, 1.0, "x%.2f");
            ImGui::Checkbox("Loop replay", &ci.replay_loop);
            ImGui::Text("Take: %.1f s %s", m_cine.take_s, m_cine.replaying ? "(replaying)" : (m_cine.take_frozen ? "(ready)" : "(recording while flying)"));
            if (ImGui::Button(m_cine.replaying ? "Stop replay (F9)" : "Replay (F9)")) m_toggle_replay_requested = true;
            ImGui::SameLine();
            if (ImGui::Button("Clear take")) m_clear_take_requested = true;
            break;
        }

        ImGui::SeparatorText("Lens");
        double focal = m_cine.focal;
        if (slider("Focal length (mm)", focal, g.focal_min, g.focal_max, "%.0f mm")) {
            m_focal_request = focal;
            g.focal_mm = focal;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%.0f deg", m_cine.hfov);
        int sensor = g.sensor_width_mm > 30.0 ? 0 : (g.sensor_width_mm > 20.0 ? 1 : 2);
        if (ImGui::Combo("Sensor", &sensor, "Full frame (36 mm)\0Super 35 (24.9 mm)\0Micro 4/3 (17.3 mm)\0")) {
            g.sensor_width_mm = sensor == 0 ? 36.0 : (sensor == 1 ? 24.9 : 17.3);
        }
        slider("Zoom speed", g.zoom_speed, 0.1, 3.0, "%.1f");
        ImGui::TextDisabled("Zoom: RT / LT (when free) or Page Up / Page Down.");

        ImGui::SeparatorText("Gimbal");
        ImGui::Checkbox("Ronin roll drift", &g.ronin_drift);
        if (g.ronin_drift) {
            ImGui::SameLine();
            ImGui::TextDisabled("(now %+.1f deg)", m_cine.drift);
            slider("Drift per g (deg)", g.drift_deg_per_g, 0.5, 20.0, "%.1f");
            slider("Max drift (deg)", g.drift_max_deg, 1.0, 25.0, "%.0f");
            slider("Drift recovery (s)", g.drift_time_s, 0.1, 2.0, "%.2f");
        }
        if (ImGui::TreeNode("Gimbal response")) {
            slider("Pan speed (deg/s)", g.max_pan_speed, 5.0, 360.0, "%.0f");
            slider("Tilt speed (deg/s)", g.max_tilt_speed, 5.0, 360.0, "%.0f");
            slider("Roll speed (deg/s)", g.max_roll_speed, 1.0, 180.0, "%.0f");
            slider("Stick expo##gimbal", g.stick_expo, 0.0, 1.0, "%.2f");
            slider("Smoothing (s)", g.smoothing_s, 0.02, 2.0, "%.2f");
            slider("Follow smoothing (s)", g.follow_time_s, 0.05, 3.0, "%.2f");
            slider("Tilt min (deg)", g.tilt_min, -135.0, 0.0, "%.0f");
            slider("Tilt max (deg)", g.tilt_max, 0.0, 90.0, "%.0f");
            slider("Roll limit (deg)", g.roll_limit, 0.0, 180.0, g.roll_limit >= 180.0 ? "free" : "%.0f");
            slider("Camera below frame (m)", ci.camera_drop_m, 0.0, 1.0, "%.2f");
            ImGui::TreePop();
        }
    }

    void draw_rates(DroneParams& d) {
        int type = static_cast<int>(d.rates_type);
        ImGui::Combo("Rates type", &type, "Betaflight\0KISS\0Actual\0");
        d.rates_type = static_cast<RatesType>(type);

        const char* headers[3][3]{
            {"RC Rate", "Super Rate", "RC Expo"},
            {"RC Rate", "Rate", "RC Curve"},
            {"Center Sens.", "Max Rate", "Expo"},
        };
        static const char* kAxes[AxisCount3]{"Roll", "Pitch", "Yaw"};

        if (ImGui::BeginTable("rates", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("");
            for (const char* h : headers[type]) ImGui::TableSetupColumn(h);
            ImGui::TableSetupColumn("Max deg/s");
            ImGui::TableHeadersRow();

            for (int a = 0; a < AxisCount3; ++a) {
                auto& r = d.rates[a];
                ImGui::PushID(a);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(kAxes[a]);

                auto cell = [](const char* id, double& v, double lo, double hi, double step, const char* fmt) {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(-1);
                    ImGui::DragScalar(id, ImGuiDataType_Double, &v, static_cast<float>(step), &lo, &hi, fmt, ImGuiSliderFlags_AlwaysClamp);
                };
                switch (d.rates_type) {
                case RatesType::Kiss:
                    cell("##a", r.kiss.rc_rate, 0.01, 2.55, 0.01, "%.2f");
                    cell("##b", r.kiss.rate, 0.0, 0.99, 0.01, "%.2f");
                    cell("##c", r.kiss.rc_curve, 0.0, 1.0, 0.01, "%.2f");
                    break;
                case RatesType::Actual:
                    cell("##a", r.actual.center_sensitivity, 10.0, 2000.0, 1.0, "%.0f");
                    cell("##b", r.actual.max_rate, 10.0, 2000.0, 5.0, "%.0f");
                    cell("##c", r.actual.expo, 0.0, 1.0, 0.01, "%.2f");
                    break;
                case RatesType::Betaflight:
                default:
                    cell("##a", r.betaflight.rc_rate, 0.01, 2.55, 0.01, "%.2f");
                    cell("##b", r.betaflight.super_rate, 0.0, 0.99, 0.01, "%.2f");
                    cell("##c", r.betaflight.expo, 0.0, 1.0, 0.01, "%.2f");
                    break;
                }
                ImGui::TableNextColumn();
                ImGui::Text("%.0f", DroneSim::rate_curve(1.0, d.rates_type, r));
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Drag a value, or Ctrl+click to type it.");

        if (ImGui::Button("Copy roll -> pitch")) d.rates[AxisPitch] = d.rates[AxisRoll];

        // Rate curves (stick 0..100 % -> deg/s), same scale for the three axes.
        float curves[AxisCount3][33]{};
        float top = 100.0f;
        for (int a = 0; a < AxisCount3; ++a) {
            for (int i = 0; i <= 32; ++i) {
                curves[a][i] = static_cast<float>(DroneSim::rate_curve(i / 32.0, d.rates_type, d.rates[a]));
                top = std::max(top, curves[a][i]);
            }
        }
        const float w = (ImGui::GetContentRegionAvail().x - 2.0f * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
        for (int a = 0; a < AxisCount3; ++a) {
            if (a > 0) ImGui::SameLine();
            ImGui::PlotLines(("##curve" + std::to_string(a)).c_str(), curves[a], 33, 0, kAxes[a], 0.0f, top, {w, 70.0f});
        }
    }

    void draw_arming_settings() {
        auto& in = m_settings.input;
        ImGui::SeparatorText("Arming");
        ImGui::RadioButton("Throttle low", &in.arm_mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Arm switch (radio / joystick)", &in.arm_mode, 1);
        if (in.arm_mode != 1) {
            return;
        }

        const auto& joy = m_input_view.joy;
        if (in.source != InputSource::Joystick) {
            ImGui::SliderInt("Joystick id##arm", &in.joystick_id, 0, 15);
        }
        if (!joy.connected) {
            ImGui::TextColored({1, 0.4f, 0.3f, 1}, "Joystick not found: the drone cannot arm.");
        }

        ImGui::Combo("Switch is a", &in.arm_source, "Button\0Axis\0");
        if (in.arm_source == 0) {
            int button = in.arm_button + 1;
            ImGui::SliderInt("Button", &button, 1, 32);
            in.arm_button = button - 1;
            std::string pressed;
            for (int b = 0; b < 32; ++b) {
                if ((joy.buttons >> b) & 1u) pressed += std::to_string(b + 1) + " ";
            }
            ImGui::Text("Buttons pressed now: %s", pressed.empty() ? "none" : pressed.c_str());
        } else {
            ImGui::Combo("Axis", &in.arm_axis, kAxisNames, AxisCount);
            slider("Threshold", in.arm_threshold, -0.9, 0.9, "%.2f");
            if (joy.connected) {
                const bool cal = in.axis_max[in.arm_axis] > in.axis_min[in.arm_axis];
                const int lo = cal ? in.axis_min[in.arm_axis] : joy.caps_min[in.arm_axis];
                const int hi = cal ? in.axis_max[in.arm_axis] : joy.caps_max[in.arm_axis];
                const float v = hi > lo ? float(joy.axes[in.arm_axis] - lo) / float(hi - lo) * 2.0f - 1.0f : 0.0f;
                ImGui::Text("Axis value now: %+.2f", v);
            }
        }
        ImGui::Checkbox("Invert switch", &in.arm_invert);

        const int sw = m_input_view.arm_switch;
        ImGui::Text("Arm switch: ");
        ImGui::SameLine();
        if (sw > 0) ImGui::TextColored({0.3f, 1, 0.4f, 1}, "ON");
        else ImGui::TextColored({1, 0.6f, 0.3f, 1}, "OFF");
        ImGui::TextDisabled("Arming needs the throttle low. Switch off = motors off (the quad falls).");
    }

    void draw_input_settings() {
        auto& in = m_settings.input;

        int source = static_cast<int>(in.source);
        ImGui::RadioButton("Gamepad (XInput)", &source, 0);
        ImGui::SameLine();
        ImGui::RadioButton("RC radio / joystick", &source, 1);
        in.source = static_cast<InputSource>(source);

        if (in.source == InputSource::XInput) {
            ImGui::Text("Pad: %s  (mode 2: left = throttle/yaw, right = pitch/roll)",
                        m_input_view.pad_connected ? "connected" : "not found");
            ImGui::SliderInt("XInput user", &in.xinput_user, 0, 3);
            int thr = static_cast<int>(in.gamepad_throttle);
            ImGui::Combo("Throttle", &thr, "Left stick, full range\0Left stick, upper half\0Right trigger\0");
            in.gamepad_throttle = static_cast<GamepadThrottle>(thr);
            slider("Stick deadzone", in.stick_deadzone, 0.0, 0.3, "%.2f");
        } else {
            const auto& joy = m_input_view.joy;
            ImGui::SliderInt("Joystick id", &in.joystick_id, 0, 15);
            ImGui::Text("Device: %s", joy.connected ? "connected" : "not found");
            if (m_input_view.devices.empty()) {
                ImGui::TextColored({1, 0.6f, 0.3f, 1}, "No joystick / radio detected. Radio in USB joystick (HID) mode?");
                ImGui::TextDisabled("Works in other games but not here? Steam Input hides controllers from the game:");
                ImGui::TextDisabled("Steam > game Properties > Controller > Disable Steam Input, then restart the game.");
            } else {
                ImGui::TextDisabled("Detected (click to use). Radio missing? Steam > game Properties > Controller > Disable Steam Input.");
                for (const auto& j : m_input_view.devices) {
                    char label[192];
                    std::snprintf(label, sizeof(label), "id %d: %s (%d axes, %d buttons)  X%+.1f Y%+.1f Z%+.1f R%+.1f%s", j.id,
                                  j.name.c_str(), j.axes, j.buttons, j.value[0], j.value[1], j.value[2], j.value[3],
                                  j.id == in.joystick_id ? "  <- pilot" : "");
                    if (ImGui::Selectable(label, j.id == in.joystick_id)) in.joystick_id = j.id;
                }
            }
            slider("Stick deadzone (roll / pitch / yaw)", in.radio_deadzone, 0.0, 0.2, "%.2f");

            if (joy.connected) {
                for (int a = 0; a < AxisCount; ++a) {
                    const bool cal = in.axis_max[a] > in.axis_min[a];
                    const int lo = cal ? in.axis_min[a] : joy.caps_min[a];
                    const int hi = cal ? in.axis_max[a] : joy.caps_max[a];
                    const float frac = hi > lo ? float(joy.axes[a] - lo) / float(hi - lo) : 0.0f;
                    char label[32];
                    std::snprintf(label, sizeof(label), "%s %d", kAxisNames[a], joy.axes[a]);
                    ImGui::ProgressBar(frac, {-1, 0}, label);
                }
            }

            for (int c = 0; c < ChCount; ++c) {
                ImGui::PushID(c);
                ImGui::SetNextItemWidth(80);
                ImGui::Combo(kChannelNames[c], &in.axis_map[c], kAxisNames, AxisCount);
                ImGui::SameLine();
                ImGui::Checkbox("Invert", &in.invert[c]);
                ImGui::PopID();
            }

            if (!m_input_view.calibrating) {
                if (ImGui::Button("Calibrate")) m_cal_begin_requested = true;
                ImGui::SameLine();
                if (ImGui::Button("Clear calibration")) {
                    std::fill(std::begin(in.axis_min), std::end(in.axis_min), 0);
                    std::fill(std::begin(in.axis_max), std::end(in.axis_max), 0);
                }
            } else {
                ImGui::TextColored({1, 0.8f, 0.2f, 1}, "Move every stick to all its corners, then:");
                if (ImGui::Button("Finish calibration")) m_cal_end_requested = true;
            }
        }

        if (ImGui::BeginTable("channels", 3)) {
            const auto ch = m_input_view.channels;
            for (int c = 0; c < ChCount; ++c) {
                ImGui::PushID(200 + c);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(kChannelNames[c]);
                ImGui::TableNextColumn();
                const float frac = c == ChThrottle ? float(ch[c]) : float((ch[c] + 1.0) * 0.5);
                char label[32];
                std::snprintf(label, sizeof(label), "%+.2f", ch[c]);
                ImGui::ProgressBar(frac, {-1, 0}, label);
                ImGui::TableNextColumn();
                if (in.source == InputSource::XInput) ImGui::Checkbox("Invert", &in.invert[c]);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

    // ------------------------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------------------------
    // Shared between the game thread and the render thread: guarded by m_state_mutex.
    std::mutex m_state_mutex{};
    Settings m_settings{};
    InputManager m_input{};       // game thread only
    InputSnapshot m_input_view{}; // copy for the UI
    DroneInput m_last_input{};
    DroneSim m_sim{};
    CameraPov m_spawn_pov{};
    Gimbal m_gimbal{};
    SceneCaptureView m_capture{};      // game thread
    SecondWindow m_window{};           // render thread (+ its own window thread)
    std::string m_capture_status{"off"};
    FlightRecorder m_recorder{};
    std::atomic<bool> m_operator_connected{false};
    struct CineStatus {
        double focal{24.0}, hfov{73.7}, drift{0.0};
        double take_s{0.0}, replay_s{0.0};
        bool replaying{false}, take_frozen{false}, operator_connected{false}, main_gimbal{false};
    } m_cine{};
    bool m_pause_in_menu{true};
    std::string m_status{};
    std::string m_camera_status{"waiting for the game"};
    uint32_t m_camera_reasserts{0};

    // Game thread only
    GameCamera m_camera{};
    GameUiHider m_ui_hider{};        // game thread
    double m_resolve_wait{0.0};      // game thread
    double m_enum_wait{0.0};         // game thread
    bool m_enum_done{false};         // game thread
    double m_refresh_wait{0.0};      // game thread
    bool m_f7{false}, m_f8{false}, m_f10{false};
    double m_f9_hold{0.0};
    bool m_f9_cleared{false};

    // Cross-thread flags
    std::atomic<bool> m_want_active{false};
    std::atomic<bool> m_respawn_requested{false};
    std::atomic<bool> m_toggle_replay_requested{false};
    std::atomic<bool> m_cal_begin_requested{false};
    std::atomic<bool> m_cal_end_requested{false};
    std::atomic<bool> m_op_cal_begin_requested{false};
    std::atomic<bool> m_op_cal_end_requested{false};
    std::atomic<bool> m_clear_take_requested{false};
    std::atomic<double> m_focal_request{0.0};
    std::atomic<bool> m_flying{false};
    std::atomic<bool> m_menu_open{false};
    std::atomic<bool> m_settings_block_pad{true};
    std::atomic<bool> m_overlay_needed{false}; // something to draw while flying
    std::atomic<bool> m_fault{false};

    std::filesystem::path m_settings_path{};

    // Render thread
    GameThreadTicker m_ticker{};
    bool m_ticker_failed{false};
    std::atomic<bool> m_engine_tick_active{false};
    // Inspire FPV camera tilt stabilizer (game thread)
    bool m_tilt_stab_init{false};
    double m_tilt_stab_est{0.0};
    double m_tilt_stab_vel{0.0}; // the UEVR build never changes during a session
    bool m_first_tick_logged{false};
    bool m_rt_second_enabled{false}; // render thread copies
    FeedEffect m_rt_fx{};
    std::recursive_mutex m_imgui_mutex{};
    HWND m_wnd{nullptr};
    bool m_context_ready{false};
    bool m_backend_ready{false};
    bool m_f6{false};
};

std::unique_ptr<FpvDronePlugin> g_plugin{new FpvDronePlugin()};
