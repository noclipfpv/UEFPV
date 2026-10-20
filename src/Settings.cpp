// Settings.cpp - see Settings.hpp
#include "Settings.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>

namespace fpv {

namespace {

// Visits every persisted field with (key, reference). Used for both loading and saving,
// so the list of keys only exists in one place.
template <typename S, typename Fn>
void visit_fields(S& s, Fn&& fn) {
    auto& d = s.drone;
    fn("rates.type", d.rates_type);
    static const char* rate_axis_names[AxisCount3]{"roll", "pitch", "yaw"};
    for (int a = 0; a < AxisCount3; ++a) {
        const std::string ax = rate_axis_names[a];
        auto& r = d.rates[a];
        fn(("rates.betaflight." + ax + ".rc_rate").c_str(), r.betaflight.rc_rate);
        fn(("rates.betaflight." + ax + ".super_rate").c_str(), r.betaflight.super_rate);
        fn(("rates.betaflight." + ax + ".expo").c_str(), r.betaflight.expo);
        fn(("rates.kiss." + ax + ".rc_rate").c_str(), r.kiss.rc_rate);
        fn(("rates.kiss." + ax + ".rate").c_str(), r.kiss.rate);
        fn(("rates.kiss." + ax + ".rc_curve").c_str(), r.kiss.rc_curve);
        fn(("rates.actual." + ax + ".center_sensitivity").c_str(), r.actual.center_sensitivity);
        fn(("rates.actual." + ax + ".max_rate").c_str(), r.actual.max_rate);
        fn(("rates.actual." + ax + ".expo").c_str(), r.actual.expo);
    }

    auto& cm = d.cinema;
    fn("flight.mode", d.flight_mode);
    fn("cinema.max_speed_ms", cm.max_speed_ms);
    fn("cinema.max_climb_ms", cm.max_climb_ms);
    fn("cinema.max_descent_ms", cm.max_descent_ms);
    fn("cinema.max_yaw_rate_dps", cm.max_yaw_rate_dps);
    fn("cinema.max_accel_ms2", cm.max_accel_ms2);
    fn("cinema.response_s", cm.response_s);
    fn("cinema.yaw_response_s", cm.yaw_response_s);
    fn("cinema.stick_expo", cm.stick_expo);
    fn("cinema.max_tilt_deg", cm.max_tilt_deg);
    fn("cinema.lean_response_s", cm.lean_response_s);
    fn("cinema.camera_tilt_deg", cm.camera_tilt_deg);
    fn("cinema.tilt_stab_lag_s", cm.tilt_stab_lag_s);
    auto& wg = d.wing;
    fn("wing.mass_kg", wg.mass_kg);
    fn("wing.wing_area_m2", wg.wing_area_m2);
    fn("wing.thrust_to_weight", wg.thrust_to_weight);
    fn("wing.prop_speed_ms", wg.prop_speed_ms);
    fn("wing.stall_deg", wg.stall_deg);
    fn("wing.cd0", wg.cd0);
    fn("wing.trim_aoa_deg", wg.trim_aoa_deg);
    fn("wing.gyro_hold", wg.gyro_hold);
    fn("wing.pitch_stability", wg.pitch_stability);
    fn("wing.yaw_stability", wg.yaw_stability);
    fn("wing.control_speed_ms", wg.control_speed_ms);
    fn("wing.roll_rate_scale", wg.roll_rate_scale);
    fn("wing.pitch_rate_scale", wg.pitch_rate_scale);
    fn("wing.yaw_rate_scale", wg.yaw_rate_scale);
    fn("wing.launch_speed_ms", wg.launch_speed_ms);
    fn("wing.camera_tilt_deg", wg.camera_tilt_deg);

    auto& ci = s.cinema;
    fn("camera.mode", ci.camera_mode);
    fn("gimbal.control", ci.gimbal_control);
    fn("gimbal.operator_user", ci.operator_user);
    fn("gimbal.operator_source", ci.operator_source);
    fn("gimbal.operator_deadzone", ci.operator_deadzone);
    fn("gimbal.operator_joystick_id", ci.operator_joystick_id);
    fn("gimbal.op_axis_pan", ci.op_axis_pan);
    fn("gimbal.op_axis_tilt", ci.op_axis_tilt);
    fn("gimbal.op_axis_roll", ci.op_axis_roll);
    fn("gimbal.op_axis_zoom", ci.op_axis_zoom);
    fn("gimbal.op_invert_pan", ci.op_invert_pan);
    fn("gimbal.op_invert_tilt", ci.op_invert_tilt);
    fn("gimbal.op_invert_roll", ci.op_invert_roll);
    fn("gimbal.op_invert_zoom", ci.op_invert_zoom);
    fn("gimbal.op_zoom_mode", ci.op_zoom_mode);
    fn("gimbal.op_radio_deadzone", ci.op_radio_deadzone);
    {
        static const char* names[AxisCount]{"x", "y", "z", "r", "u", "v"};
        for (int a = 0; a < AxisCount; ++a) {
            fn((std::string{"gimbal.op_cal.min."} + names[a]).c_str(), ci.op_axis_min[a]);
            fn((std::string{"gimbal.op_cal.max."} + names[a]).c_str(), ci.op_axis_max[a]);
        }
    }
    fn("gimbal.tilt_axis", ci.tilt_axis);
    fn("gimbal.tilt_axis_invert", ci.tilt_axis_invert);
    fn("gimbal.replay_speed", ci.replay_speed);
    fn("gimbal.replay_loop", ci.replay_loop);
    fn("gimbal.record_view", ci.record_view);
    fn("gimbal.camera_drop_m", ci.camera_drop_m);
    auto& g = ci.gimbal;
    fn("gimbal.max_pan_speed", g.max_pan_speed);
    fn("gimbal.max_tilt_speed", g.max_tilt_speed);
    fn("gimbal.max_roll_speed", g.max_roll_speed);
    fn("gimbal.stick_expo", g.stick_expo);
    fn("gimbal.smoothing_s", g.smoothing_s);
    fn("gimbal.tilt_min", g.tilt_min);
    fn("gimbal.tilt_max", g.tilt_max);
    fn("gimbal.roll_limit", g.roll_limit);
    fn("gimbal.follow_time_s", g.follow_time_s);
    fn("gimbal.ronin_drift", g.ronin_drift);
    fn("gimbal.drift_deg_per_g", g.drift_deg_per_g);
    fn("gimbal.drift_max_deg", g.drift_max_deg);
    fn("gimbal.drift_time_s", g.drift_time_s);
    fn("gimbal.focal_mm", g.focal_mm);
    fn("gimbal.sensor_width_mm", g.sensor_width_mm);
    fn("gimbal.zoom_speed", g.zoom_speed);

    auto& sv = s.second;
    fn("second.enabled", sv.enabled);
    fn("second.view", sv.view);
    fn("second.height", sv.height);
    fn("second.aspect", sv.aspect);
    fn("second.interval", sv.interval);
    fn("second.analog", sv.fx.analog);
    fn("second.noise", sv.fx.noise);
    fn("second.softness", sv.fx.softness);
    fn("second.scanlines", sv.fx.scanlines);
    fn("second.gamma", sv.fx.gamma);
    fn("second.auto_exposure", sv.fx.auto_exposure);
    fn("second.exposure", sv.fx.exposure);
    fn("second.sharpness", sv.fx.sharpness);
    fn("second.supersample", sv.fx.supersample);

    auto& c = s.collision;
    fn("collision.enabled", c.enabled);
    fn("collision.radius_m", c.radius_m);
    fn("collision.restitution", c.restitution);
    fn("collision.friction", c.friction);
    fn("collision.spin_on_impact", c.spin_on_impact);
    fn("collision.crash_enabled", c.crash_enabled);
    fn("collision.crash_speed_ms", c.crash_speed_ms);
    fn("collision.trace_channel", s.trace_channel);

    fn("quad.thrust_to_weight", d.thrust_to_weight);
    fn("quad.idle_throttle", d.idle_throttle);
    fn("quad.throttle_expo", d.throttle_expo);
    fn("quad.motor_time_constant", d.motor_time_constant);
    fn("quad.rate_time_constant", d.rate_time_constant);
    fn("quad.drag_quadratic", d.drag_quadratic);
    fn("quad.drag_linear", d.drag_linear);
    fn("quad.vertical_drag_mult", d.vertical_drag_mult);
    fn("quad.camera_tilt_deg", d.camera_tilt_deg);
    fn("quad.gravity", d.gravity);
    fn("quad.angle_mode", d.angle_mode);
    fn("quad.max_angle_deg", d.max_angle_deg);
    fn("quad.angle_gain", d.angle_gain);

    auto& in = s.input;
    fn("input.source", in.source);
    fn("input.xinput_user", in.xinput_user);
    fn("input.gamepad_throttle", in.gamepad_throttle);
    fn("input.stick_deadzone", in.stick_deadzone);
    fn("input.radio_deadzone", in.radio_deadzone);
    fn("input.joystick_id", in.joystick_id);
    static const char* ch_names[ChCount]{"throttle", "roll", "pitch", "yaw"};
    static const char* axis_names[AxisCount]{"x", "y", "z", "r", "u", "v"};
    for (int c = 0; c < ChCount; ++c) {
        fn((std::string{"input.map."} + ch_names[c]).c_str(), in.axis_map[c]);
        fn((std::string{"input.invert."} + ch_names[c]).c_str(), in.invert[c]);
    }
    for (int a = 0; a < AxisCount; ++a) {
        fn((std::string{"input.cal.min."} + axis_names[a]).c_str(), in.axis_min[a]);
        fn((std::string{"input.cal.max."} + axis_names[a]).c_str(), in.axis_max[a]);
    }

    fn("camera.override_fov", s.override_fov);
    fn("camera.fov_deg", s.fov_deg);
    fn("world.units_per_meter", s.units_per_meter);
    fn("game.block_gamepad", s.block_game_gamepad);
    fn("game.freeze_player", s.freeze_player);
    fn("game.show_osd", s.show_osd);
    fn("game.show_crosshair", s.show_crosshair);
    fn("game.hide_ui", s.hide_game_ui);
    fn("input.arm.mode", in.arm_mode);
    fn("input.arm.source", in.arm_source);
    fn("input.arm.button", in.arm_button);
    fn("input.arm.axis", in.arm_axis);
    fn("input.arm.threshold", in.arm_threshold);
    fn("input.arm.invert", in.arm_invert);
}

template <typename T>
void parse_into(const std::string& text, T& out) {
    std::istringstream ss{text};
    if constexpr (std::is_enum_v<T>) {
        int v{};
        if (ss >> v) out = static_cast<T>(v);
    } else if constexpr (std::is_same_v<T, bool>) {
        out = (text == "1" || text == "true" || text == "yes");
    } else if constexpr (std::is_same_v<T, float>) {
        double v{};
        if (ss >> v) out = static_cast<float>(v);
    } else {
        T v{};
        if (ss >> v) out = v;
    }
}

template <typename T>
std::string to_text(const T& v) {
    std::ostringstream ss;
    if constexpr (std::is_enum_v<T>) {
        ss << static_cast<int>(v);
    } else if constexpr (std::is_same_v<T, bool>) {
        ss << (v ? "true" : "false");
    } else {
        ss << v;
    }
    return ss.str();
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    const auto e = s.find_last_not_of(" \t\r\n");
    return b == std::string::npos ? std::string{} : s.substr(b, e - b + 1);
}

} // namespace

bool Settings::load(const std::filesystem::path& path) {
    std::ifstream file{path};
    if (!file) {
        return false;
    }

    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        values[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }

    visit_fields(*this, [&](const char* key, auto& field) {
        const auto it = values.find(key);
        if (it != values.end()) {
            parse_into(it->second, field);
        }
    });

    // Files written before v0.5.5 (no config.version): new defaults for the OSD, the crosshair and the
    // gimbal (Assetto Corsa mod angles / speeds, Ronin drift off).
    int version = 1;
    if (auto it = values.find("config.version"); it != values.end()) parse_into(it->second, version);
    if (version < 5 && drone.wing.camera_tilt_deg == 5.0) drone.wing.camera_tilt_deg = 0.0; // v0.7.4 default
    if (version < 4) second.fx.analog = false; // v0.7.1: clean feed by default
    if (version < 3) cinema.gimbal.tilt_min = Settings{}.cinema.gimbal.tilt_min;
    if (version < 2) {
        const Settings d{};
        show_osd = d.show_osd;
        show_crosshair = d.show_crosshair;
        auto& g = cinema.gimbal;
        const auto& dg = d.cinema.gimbal;
        g.max_pan_speed = dg.max_pan_speed;
        g.max_tilt_speed = dg.max_tilt_speed;
        g.max_roll_speed = dg.max_roll_speed;
        g.stick_expo = dg.stick_expo;
        g.tilt_min = dg.tilt_min;
        g.tilt_max = dg.tilt_max;
        g.roll_limit = dg.roll_limit;
        g.ronin_drift = dg.ronin_drift;
    }

    // Files from v0.1 / v0.2 had one Betaflight rate set shared by roll and pitch.
    if (values.count("rates.rp.rc_rate") != 0 && values.count("rates.betaflight.roll.rc_rate") == 0) {
        auto legacy = [&](const char* key, double& out) {
            if (auto it = values.find(key); it != values.end()) parse_into(it->second, out);
        };
        for (int a : {AxisRoll, AxisPitch}) {
            legacy("rates.rp.rc_rate", drone.rates[a].betaflight.rc_rate);
            legacy("rates.rp.super_rate", drone.rates[a].betaflight.super_rate);
            legacy("rates.rp.expo", drone.rates[a].betaflight.expo);
        }
        legacy("rates.yaw.rc_rate", drone.rates[AxisYaw].betaflight.rc_rate);
        legacy("rates.yaw.super_rate", drone.rates[AxisYaw].betaflight.super_rate);
        legacy("rates.yaw.expo", drone.rates[AxisYaw].betaflight.expo);
        drone.rates_type = RatesType::Betaflight;
    }
    return true;
}

bool Settings::save(const std::filesystem::path& path) const {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    std::ofstream file{path, std::ios::trunc};
    if (!file) {
        return false;
    }

    file << "# UEVR FPV Drone settings (per game)\n";
    file << "config.version = 5\n";
    Settings copy = *this; // visit_fields needs mutable refs
    visit_fields(copy, [&](const char* key, auto& field) {
        file << key << " = " << to_text(field) << "\n";
    });
    return static_cast<bool>(file);
}

} // namespace fpv
