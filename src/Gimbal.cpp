// Gimbal.cpp - see Gimbal.hpp
#include "Gimbal.hpp"

namespace fpv {

namespace {

double expo(double x, double e) {
    x = clampv(x, -1.0, 1.0);
    return x * (1.0 - e) + x * x * x * e;
}

double alpha(double dt, double tau) {
    return 1.0 - std::exp(-dt / std::max(tau, 1e-3));
}

double wrap180(double a) {
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0.0) a += 360.0;
    return a - 180.0;
}

} // namespace

double Gimbal::hfov_deg(double focal_mm, double sensor_width_mm) {
    return 2.0 * std::atan(sensor_width_mm / (2.0 * std::max(focal_mm, 0.5))) * kRad2Deg;
}

void Gimbal::reset(double yaw_deg, double pitch_deg, double focal_mm) {
    m_yaw = yaw_deg;
    m_pitch = pitch_deg;
    m_roll = 0.0;
    m_pan_speed = m_tilt_speed = m_roll_speed = 0.0;
    m_follow_offset = 0.0;
    m_focal = focal_mm;
    m_has_prev = false;
    m_accel = {};
    m_drift = 0.0;
}

void Gimbal::update(const GimbalInput& in, const GimbalParams& p, bool follow_heading, double heading_deg,
                    const Vec3& camera_pos_m, double dt) {
    if (!(dt > 0.0)) return;

    // --- Operator speeds with Ronin-like inertia --------------------------------------------
    const double a = alpha(dt, p.smoothing_s);
    m_pan_speed += (expo(in.pan, p.stick_expo) * p.max_pan_speed - m_pan_speed) * a;
    m_tilt_speed += (expo(in.tilt, p.stick_expo) * p.max_tilt_speed - m_tilt_speed) * a;
    m_roll_speed += (expo(in.roll, p.stick_expo) * p.max_roll_speed - m_roll_speed) * a;

    // Pan
    if (follow_heading) {
        m_follow_offset = wrap180(m_follow_offset + m_pan_speed * dt);
        const double target = heading_deg + m_follow_offset;
        m_yaw += wrap180(target - m_yaw) * alpha(dt, p.follow_time_s);
    } else {
        m_yaw += m_pan_speed * dt;
    }
    m_yaw = wrap180(m_yaw);

    // Tilt
    if (in.has_tilt_absolute) {
        const double t = (clampv(in.tilt_absolute, -1.0, 1.0) + 1.0) * 0.5;
        const double target = p.tilt_min + (p.tilt_max - p.tilt_min) * t;
        m_pitch += (target - m_pitch) * alpha(dt, p.smoothing_s);
    } else {
        m_pitch += m_tilt_speed * dt;
    }
    m_pitch = clampv(m_pitch, p.tilt_min, p.tilt_max);

    // Roll (operator)
    if (p.roll_limit >= 180.0) {
        m_roll = std::remainder(m_roll + m_roll_speed * dt, 360.0);
    } else {
        m_roll = clampv(m_roll + m_roll_speed * dt, -p.roll_limit, p.roll_limit);
    }

    // --- Ronin roll drift ---------------------------------------------------------------------
    if (m_has_prev) {
        const Vec3 vel = (camera_pos_m - m_prev_pos) / dt;
        const Vec3 acc = (vel - m_prev_vel) / dt;
        m_accel += (acc - m_accel) * alpha(dt, 0.12); // filter numeric noise
        m_prev_vel = vel;
    } else {
        m_prev_vel = {};
        m_has_prev = true;
    }
    m_prev_pos = camera_pos_m;

    double drift_target = 0.0;
    if (p.ronin_drift) {
        // Lateral acceleration relative to the camera (positive = to the camera's right).
        const double yr = m_yaw * kDeg2Rad;
        const Vec3 cam_right{-std::sin(yr), std::cos(yr), 0.0};
        const double lateral_g = dot(m_accel, cam_right) / 9.81;
        // The payload swings like a pendulum toward the apparent gravity: accelerating to the
        // right rolls the camera to the left (horizon tilts the other way), then recovers.
        drift_target = clampv(-lateral_g * p.drift_deg_per_g, -p.drift_max_deg, p.drift_max_deg);
    }
    m_drift += (drift_target - m_drift) * alpha(dt, p.drift_time_s);

    // --- Zoom -----------------------------------------------------------------------------------
    if (in.has_zoom_absolute) {
        // Focal length follows the stick / pot, smoothed in log space (a zoom ring feel).
        const double t = (clampv(in.zoom_absolute, -1.0, 1.0) + 1.0) * 0.5;
        const double lmin = std::log(std::max(p.focal_min, 0.5)), lmax = std::log(std::max(p.focal_max, p.focal_min + 0.5));
        const double target = lmin + (lmax - lmin) * t;
        const double cur = std::log(std::max(m_focal, 0.5));
        m_focal = std::exp(cur + (target - cur) * alpha(dt, 0.12));
    } else if (std::abs(in.zoom) > 0.02) {
        m_focal *= std::pow(2.0, clampv(in.zoom, -1.0, 1.0) * p.zoom_speed * dt);
    }
    m_focal = clampv(m_focal, p.focal_min, p.focal_max);
}

Rotator Gimbal::orientation() const {
    // Tilting down past -90 deg (inverted gimbal) is a valid orientation; go through a basis so the
    // engine always gets a canonical rotator (pitch within +-90).
    return Basis::from_rotator({m_pitch, m_yaw, m_roll + m_drift}).to_rotator();
}

} // namespace fpv
