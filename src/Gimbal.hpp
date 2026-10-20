// Gimbal.hpp - engine-independent 3-axis camera gimbal (DJI Ronin / Inspire style).
//
// The camera is stabilized in the world (horizon level whatever the drone does). The operator
// commands angular speeds with inertia (like a Ronin's smoothing), the pilot-only mode follows
// the drone's heading. Ronin roll drift: under lateral g the horizon swings a few degrees and
// recovers, like a real RS3 Pro roll motor. Optics: focal length on a configurable sensor.
#pragma once

#include "MathTypes.hpp"

namespace fpv {

struct GimbalParams {
    // Operator control (deg/s at full stick)
    // Defaults match the Assetto Corsa mod: 120 deg/s on every axis, linear sticks.
    double max_pan_speed{120.0};
    double max_tilt_speed{120.0};
    double max_roll_speed{120.0};
    double stick_expo{0.0};
    double smoothing_s{0.35};     // Ronin-like inertia of the commanded speeds
    double tilt_min{-110.0};      // past -90: looks backward, like an RS 4 Pro hung under a drone
    double tilt_max{85.0};
    double roll_limit{180.0};     // 180 = free roll (wraps around)

    // Pilot-only mode: the pan follows the drone's heading
    double follow_time_s{0.8};

    // Ronin roll drift
    bool ronin_drift{false};
    double drift_deg_per_g{6.0};
    double drift_max_deg{10.0};
    double drift_time_s{0.45};

    // Optics
    double focal_mm{24.0};
    double sensor_width_mm{36.0}; // full frame; Super 35 = 24.9
    double zoom_speed{1.0};       // focal factor per second at full trigger (x2 / s at 1.0)
    double focal_min{6.0};
    double focal_max{200.0};
};

struct GimbalInput {
    double pan{0.0};    // -1..1, + = right
    double tilt{0.0};   // -1..1, + = up
    double roll{0.0};   // -1..1, + = clockwise (right side down)
    double zoom{0.0};   // -1..1, + = zoom in (longer focal)
    bool has_tilt_absolute{false};
    double tilt_absolute{0.0}; // -1..1 mapped to tilt_min..tilt_max (e.g. a pot on the radio)
    bool has_zoom_absolute{false};
    double zoom_absolute{0.0};   // -1..1 mapped to focal_min..focal_max (log scale), e.g. a throttle stick
};

class Gimbal {
public:
    void reset(double yaw_deg, double pitch_deg, double focal_mm);

    // follow_heading: pilot-only mode (pan tracks heading_deg, pan input adds an offset).
    // camera_pos_m: camera position this frame, used for the Ronin drift (lateral g).
    void update(const GimbalInput& in, const GimbalParams& p, bool follow_heading, double heading_deg,
                const Vec3& camera_pos_m, double dt);

    Rotator orientation() const; // Unreal rotator of the camera
    double focal_mm() const { return m_focal; }
    void set_focal(double focal_mm) { m_focal = focal_mm; }
    double drift_deg() const { return m_drift; }
    static double hfov_deg(double focal_mm, double sensor_width_mm);

private:
    double m_yaw{0.0}, m_pitch{0.0}, m_roll{0.0};
    double m_pan_speed{0.0}, m_tilt_speed{0.0}, m_roll_speed{0.0};
    double m_follow_offset{0.0};
    double m_focal{24.0};

    // Drift
    bool m_has_prev{false};
    Vec3 m_prev_pos{};
    Vec3 m_prev_vel{};
    Vec3 m_accel{};
    double m_drift{0.0};
};

} // namespace fpv
