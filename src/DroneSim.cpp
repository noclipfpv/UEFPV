// DroneSim.cpp - see DroneSim.hpp
#include "DroneSim.hpp"

namespace fpv {

namespace {
constexpr double kSubstep = 1.0 / 500.0;  // 500 Hz physics
constexpr int kMaxSubsteps = 100;          // avoid spiral of death after hitches
constexpr double kMaxRateDps = 1998.0;     // Betaflight SETPOINT_RATE_LIMIT
constexpr double kArmThreshold = 0.05;     // throttle must be below this to arm
constexpr double kRestSpeed = 0.5;         // m/s, "at rest" for re-arming after a crash

double pow3(double x) { return x * x * x; }
double pow5(double x) { return x * x * x * x * x; }

// Formulas from Betaflight src/main/fc/rc.c (applyBetaflightRates / applyKissRates /
// applyActualRates), with the configurator's units instead of the raw CLI integers.
double betaflight_rate(double x, double ax, const BetaflightRates& r) {
    if (r.expo > 0.0) {
        x = x * pow3(ax) * r.expo + x * (1.0 - r.expo);
    }
    double rc_rate = r.rc_rate;
    if (rc_rate > 2.0) {
        rc_rate += 14.54 * (rc_rate - 2.0);
    }
    double angle_rate = 200.0 * rc_rate * x;
    if (r.super_rate > 0.0) {
        angle_rate *= 1.0 / clampv(1.0 - ax * r.super_rate, 0.01, 1.0);
    }
    return angle_rate;
}

double kiss_rate(double x, double ax, const KissRates& r) {
    const double use_rates = 1.0 / clampv(1.0 - ax * r.rate, 0.01, 1.0);
    const double cmd = (pow3(x) * r.rc_curve + x * (1.0 - r.rc_curve)) * (r.rc_rate / 10.0);
    return 2000.0 * use_rates * cmd;
}

double actual_rate(double x, double ax, const ActualRates& r) {
    const double expo = ax * (pow5(x) * r.expo + x * (1.0 - r.expo));
    const double center = r.center_sensitivity;
    const double stick_movement = std::max(0.0, r.max_rate - center);
    return x * center + stick_movement * expo;
}

} // namespace

DroneParams::DroneParams() {
    // Betaflight configurator defaults: yaw is usually a bit slower.
    rates[AxisYaw].betaflight.super_rate = 0.6;
}

double DroneSim::rate_curve(double stick, RatesType type, const AxisRates& rates) {
    const double x = clampv(stick, -1.0, 1.0);
    const double ax = std::abs(x);
    double rate = 0.0;
    switch (type) {
    case RatesType::Kiss: rate = kiss_rate(x, ax, rates.kiss); break;
    case RatesType::Actual: rate = actual_rate(x, ax, rates.actual); break;
    case RatesType::Betaflight:
    default: rate = betaflight_rate(x, ax, rates.betaflight); break;
    }
    return clampv(rate, -kMaxRateDps, kMaxRateDps);
}

void DroneSim::reset(double yaw_deg) {
    m_pos = {};
    m_vel = {};
    m_body = Basis::from_rotator({0.0, yaw_deg, 0.0});
    m_rates_dps = {};
    m_cine_yaw_rate = 0.0;
    m_cine_accel = {};
    m_thrust_frac = 0.0;
    m_last_impact = 0.0;
    m_arm_state = ArmState::WaitingForThrottleLow;
    m_throttle_blocks = false;
    m_switch_seen_off = false;
    m_wing_launched = false;
    m_aoa_deg = 0.0;
}

Basis DroneSim::camera(const DroneParams& params) const {
    Basis cam = m_body;
    cam.pitch_up((params.flight_mode == FlightMode::Wing ? params.wing.camera_tilt_deg : params.camera_tilt_deg) * kDeg2Rad);
    return cam;
}

DroneTelemetry DroneSim::telemetry() const {
    DroneTelemetry t;
    t.speed_ms = m_vel.length();
    t.vertical_speed_ms = m_vel.z;
    t.altitude_m = m_pos.z;
    t.distance_m = std::sqrt(m_pos.x * m_pos.x + m_pos.y * m_pos.y);
    t.thrust_frac = m_thrust_frac;
    t.last_impact_ms = m_last_impact;
    t.arm_state = m_arm_state;
    t.throttle_blocks_arming = m_throttle_blocks;
    t.wing_waiting_launch = m_wing_mode && !m_wing_launched && m_arm_state == ArmState::Armed;
    t.aoa_deg = m_aoa_deg;
    return t;
}

void DroneSim::update(const DroneInput& input, const DroneParams& params, double dt) {
    if (!(dt > 0.0)) {
        return; // paused or invalid
    }

    update_arming(input);
    if (m_arm_state == ArmState::WaitingForThrottleLow) {
        return; // hold position until armed
    }

    const bool wing = params.flight_mode == FlightMode::Wing;
    m_wing_mode = wing;
    if (wing && !m_wing_launched) {
        // Held in the hand until the throttle comes up, then thrown forward, slightly nose up.
        if (m_arm_state != ArmState::Armed || input.throttle < 0.25) {
            m_vel = {};
            m_rates_dps = {};
            return;
        }
        m_wing_launched = true;
        const double yaw = heading_deg();
        m_body = Basis::from_rotator({8.0, yaw, 0.0});
        m_vel = m_body.f * params.wing.launch_speed_ms + Vec3{0, 0, 1.5};
        m_thrust_frac = clampv(input.throttle, 0.0, 1.0);
    }

    int steps = static_cast<int>(std::ceil(dt / kSubstep));
    steps = clampv(steps, 1, kMaxSubsteps);
    const double h = dt / steps;

    const bool cinema = params.flight_mode == FlightMode::Cinema && m_arm_state == ArmState::Armed;
    for (int i = 0; i < steps; ++i) {
        if (cinema) {
            substep_cinema(input, params, h);
        } else if (wing) {
            substep_wing(input, params, h);
        } else {
            substep(input, params, h);
        }
    }
}

void DroneSim::arm(bool flip_if_upside_down) {
    // If the quad ended up on its back, put it back on its feet (keeps the heading).
    if (flip_if_upside_down && m_body.u.z < 0.3) {
        const double yaw = std::atan2(m_body.f.y, m_body.f.x) * kRad2Deg;
        m_body = Basis::from_rotator({0.0, yaw, 0.0});
        m_rates_dps = {};
    }
    m_arm_state = ArmState::Armed;
}

void DroneSim::update_arming(const DroneInput& in) {
    const bool throttle_low = in.throttle < kArmThreshold;
    const bool at_rest = m_vel.length() < kRestSpeed;
    m_throttle_blocks = false;

    if (in.arm_switch < 0) {
        // No arm switch: arm by lowering the throttle.
        switch (m_arm_state) {
        case ArmState::WaitingForThrottleLow:
            if (throttle_low) arm(false);
            break;
        case ArmState::Crashed:
        case ArmState::Disarmed:
            if (throttle_low && at_rest) arm(true);
            break;
        case ArmState::Armed:
            break;
        }
        return;
    }

    // Arm switch (Betaflight-like): arming needs the throttle low, switch off disarms.
    const bool on = in.arm_switch > 0;
    if (!on) m_switch_seen_off = true;

    switch (m_arm_state) {
    case ArmState::Armed:
        if (!on) m_arm_state = ArmState::Disarmed;
        break;
    case ArmState::WaitingForThrottleLow:
        if (on) {
            if (throttle_low) arm(false);
            else m_throttle_blocks = true;
        }
        break;
    case ArmState::Disarmed:
        if (on) {
            if (throttle_low) arm(at_rest);
            else m_throttle_blocks = true;
        }
        break;
    case ArmState::Crashed:
        if (on && m_switch_seen_off) {
            if (throttle_low) arm(at_rest);
            else m_throttle_blocks = true;
        }
        break;
    }
}

void DroneSim::substep(const DroneInput& in, const DroneParams& p, double dt) {
    const bool motors = (m_arm_state == ArmState::Armed);

    // --- Motors -----------------------------------------------------------
    double target_thrust = 0.0;
    if (motors) {
        double thr = clampv(in.throttle, 0.0, 1.0);
        if (p.throttle_expo > 0.0) {
            thr = thr * (1.0 - p.throttle_expo) + thr * thr * thr * p.throttle_expo;
        }
        target_thrust = p.idle_throttle + (1.0 - p.idle_throttle) * thr;
    }
    const double motor_alpha = 1.0 - std::exp(-dt / std::max(p.motor_time_constant, 1e-4));
    m_thrust_frac += (target_thrust - m_thrust_frac) * motor_alpha;

    // --- Rate setpoints (deg/s, body frame) ------------------------------
    Vec3 target{};
    if (motors) {
        target.x = rate_curve(in.roll, p.rates_type, p.rates[AxisRoll]);    // roll right
        target.y = -rate_curve(in.pitch, p.rates_type, p.rates[AxisPitch]); // stick forward -> nose down
        target.z = rate_curve(in.yaw, p.rates_type, p.rates[AxisYaw]);      // yaw right

        if (p.angle_mode) {
            const double cur_pitch = std::asin(clampv(m_body.f.z, -1.0, 1.0)) * kRad2Deg;
            const double cur_roll = std::atan2(-m_body.r.z, m_body.u.z) * kRad2Deg;
            const double want_pitch = -clampv(in.pitch, -1.0, 1.0) * p.max_angle_deg;
            const double want_roll = clampv(in.roll, -1.0, 1.0) * p.max_angle_deg;
            target.x = clampv((want_roll - cur_roll) * p.angle_gain, -kMaxRateDps, kMaxRateDps);
            target.y = clampv((want_pitch - cur_pitch) * p.angle_gain, -kMaxRateDps, kMaxRateDps);
        }

        const double rate_alpha = 1.0 - std::exp(-dt / std::max(p.rate_time_constant, 1e-4));
        m_rates_dps += (target - m_rates_dps) * rate_alpha;
    } else {
        // Motors off: no flight controller, the quad just tumbles and slowly loses spin.
        m_rates_dps *= std::exp(-dt / 0.5);
    }

    // --- Attitude ----------------------------------------------------------
    m_body.roll_right(m_rates_dps.x * kDeg2Rad * dt);
    m_body.pitch_up(m_rates_dps.y * kDeg2Rad * dt);
    m_body.yaw_right(m_rates_dps.z * kDeg2Rad * dt);
    m_body.orthonormalize();

    // --- Forces --------------------------------------------------------------
    const double max_accel = p.thrust_to_weight * p.gravity;
    Vec3 accel = m_body.u * (m_thrust_frac * max_accel);
    accel.z -= p.gravity;

    // Anisotropic drag computed in body frame.
    const Vec3 vb = m_body.to_body(m_vel);
    const double speed = m_vel.length();
    Vec3 drag_b{
        p.drag_quadratic * speed * vb.x + p.drag_linear * vb.x,
        p.drag_quadratic * speed * vb.y + p.drag_linear * vb.y,
        (p.drag_quadratic * speed * vb.z + p.drag_linear * vb.z) * p.vertical_drag_mult,
    };
    accel -= m_body.to_world(drag_b);

    // Semi-implicit Euler.
    m_vel += accel * dt;
    m_pos += m_vel * dt;
}

double DroneSim::heading_deg() const {
    // Use forward, or up when looking straight up/down (acro flips).
    Vec3 dir{m_body.f.x, m_body.f.y, 0.0};
    if (dir.length() < 0.2) dir = {-m_body.u.x, -m_body.u.y, 0.0};
    return std::atan2(dir.y, dir.x) * kRad2Deg;
}

namespace {
double stick_expo(double x, double expo) {
    x = clampv(x, -1.0, 1.0);
    return x * (1.0 - expo) + x * x * x * expo;
}
} // namespace

void DroneSim::substep_cinema(const DroneInput& in, const DroneParams& p, double dt) {
    const auto& c = p.cinema;

    // Heading
    const double yaw_target = stick_expo(in.yaw, c.stick_expo) * c.max_yaw_rate_dps;
    m_cine_yaw_rate += (yaw_target - m_cine_yaw_rate) * (1.0 - std::exp(-dt / std::max(c.yaw_response_s, 0.01)));
    const double heading = heading_deg() + m_cine_yaw_rate * dt;
    const double hr = heading * kDeg2Rad;
    const Vec3 fwd{std::cos(hr), std::sin(hr), 0.0};
    const Vec3 right{-std::sin(hr), std::cos(hr), 0.0};

    // Desired velocity: centered sticks = hold position.
    const double sf = stick_expo(in.pitch, c.stick_expo);
    const double sr = stick_expo(in.roll, c.stick_expo);
    double climb = (clampv(in.throttle, 0.0, 1.0) - 0.5) * 2.0;
    climb = std::abs(climb) < 0.06 ? 0.0 : (climb - std::copysign(0.06, climb)) / 0.94;
    const double vz = climb > 0.0 ? climb * c.max_climb_ms : climb * c.max_descent_ms;
    const Vec3 v_target = fwd * (sf * c.max_speed_ms) + right * (sr * c.max_speed_ms) + Vec3{0, 0, vz};

    // First-order velocity response with an acceleration limit (smooth starts and stops).
    Vec3 accel = (v_target - m_vel) / std::max(c.response_s, 0.05);
    Vec3 ah{accel.x, accel.y, 0.0};
    const double ah_len = ah.length();
    if (ah_len > c.max_accel_ms2) ah = ah * (c.max_accel_ms2 / ah_len);
    accel = {ah.x, ah.y, clampv(accel.z, -4.0, 4.0)};
    m_vel += accel * dt;
    m_pos += m_vel * dt;
    m_cine_accel += (accel - m_cine_accel) * (1.0 - std::exp(-dt / std::max(c.lean_response_s, 0.05)));

    // Body attitude (visual): the thrust vector leans into the acceleration and against drag.
    const Vec3 vh{m_vel.x, m_vel.y, 0.0};
    Vec3 lean = Vec3{m_cine_accel.x, m_cine_accel.y, 0.0} + vh * 0.25;
    const double max_lean = std::tan(c.max_tilt_deg * kDeg2Rad) * p.gravity;
    if (lean.length() > max_lean) lean = lean * (max_lean / lean.length());
    const Vec3 up = (lean + Vec3{0, 0, p.gravity}).normalized();
    // Build the body so that its forward axis keeps the heading exactly. (Projecting `fwd` on the
    // tilted disc changed the heading whenever the lean had a sideways part, and that
    // fed back into the next step: violent yaw spins when flying forward / backward.)
    // f = fwd + k * Z with f . up = 0: same horizontal direction as the heading, exactly.
    const Vec3 f = (fwd - Vec3{0, 0, 1} * (dot(fwd, up) / std::max(up.z, 0.2))).normalized();
    m_body.u = up;
    m_body.f = f;
    m_body.r = cross_lh(up, f).normalized();

    m_rates_dps = {};
    m_thrust_frac = 1.0 / std::max(p.thrust_to_weight, 1.0);
}

void DroneSim::substep_wing(const DroneInput& in, const DroneParams& p, double dt) {
    const auto& w = p.wing;
    const bool motors = (m_arm_state == ArmState::Armed);
    constexpr double kRho = 1.225;

    // --- Motor ---------------------------------------------------------------
    double thr = motors ? clampv(in.throttle, 0.0, 1.0) : 0.0;
    if (motors && p.throttle_expo > 0.0) thr = thr * (1.0 - p.throttle_expo) + thr * thr * thr * p.throttle_expo;
    m_thrust_frac += (thr - m_thrust_frac) * (1.0 - std::exp(-dt / 0.08));

    // --- Airflow in the body frame --------------------------------------------
    const double speed = m_vel.length();
    const Vec3 vb = m_body.to_body(m_vel);
    const double alpha = speed > 0.5 ? std::atan2(-vb.z, vb.x) : 0.0; // + = air from below
    const double beta = speed > 0.5 ? std::atan2(vb.y, std::max(std::abs(vb.x), 0.1)) : 0.0;
    m_aoa_deg = alpha * kRad2Deg;
    const double q = 0.5 * kRho * speed * speed;

    // --- Rotation: flight controller in rate mode + aerodynamic weathervane ---------------------
    const double authority = clampv(speed * speed / (w.control_speed_ms * w.control_speed_ms), 0.03, 1.0);
    Vec3 target{};
    if (motors) {
        target.x = rate_curve(in.roll, p.rates_type, p.rates[AxisRoll]) * w.roll_rate_scale * authority;
        target.y = -rate_curve(in.pitch, p.rates_type, p.rates[AxisPitch]) * w.pitch_rate_scale * authority;
        target.z = rate_curve(in.yaw, p.rates_type, p.rates[AxisYaw]) * w.yaw_rate_scale * authority;
    }
    // Weathervane (deg/s): the nose follows the airflow, toward a small trim angle of attack.
    const double v10 = std::min(speed, 40.0) / 10.0 * (motors ? 1.0 - clampv(w.gyro_hold, 0.0, 0.95) : 1.0);
    const double alpha_err = clampv(alpha * kRad2Deg - w.trim_aoa_deg, -60.0, 60.0);
    target.y -= alpha_err * w.pitch_stability * v10;
    target.z += clampv(beta * kRad2Deg, -60.0, 60.0) * w.yaw_stability * v10;
    // Airframe roll damping when nobody holds it (motors off).
    if (!motors) target.x = 0.0;
    const double rate_alpha = 1.0 - std::exp(-dt / (motors ? 0.04 : 0.15));
    m_rates_dps += (target - m_rates_dps) * rate_alpha;

    m_body.roll_right(m_rates_dps.x * kDeg2Rad * dt);
    m_body.pitch_up(m_rates_dps.y * kDeg2Rad * dt);
    m_body.yaw_right(m_rates_dps.z * kDeg2Rad * dt);
    m_body.orthonormalize();

    // --- Forces (N) ------------------------------------------------------------
    const double weight = w.mass_kg * p.gravity;
    Vec3 force{0, 0, -weight};

    // Pusher prop: thrust falls off as the airspeed approaches the prop's pitch speed.
    const double prop = std::max(0.0, 1.0 - std::max(vb.x, 0.0) / std::max(w.prop_speed_ms, 1.0));
    force += m_body.f * (m_thrust_frac * w.thrust_to_weight * weight * prop);

    if (speed > 0.5) {
        const Vec3 vdir = m_vel / speed;
        // Lift coefficient: linear up to the stall, then a flat plate.
        const double stall = w.stall_deg * kDeg2Rad;
        const double cl_lin = w.cl0 + w.cl_alpha * alpha;
        const double cl_plate = 0.9 * std::sin(2.0 * alpha);
        double cl;
        if (std::abs(alpha) <= stall) {
            cl = cl_lin;
        } else {
            const double clmax = w.cl0 + w.cl_alpha * std::copysign(stall, alpha);
            const double t = clampv((std::abs(alpha) - stall) / (10.0 * kDeg2Rad), 0.0, 1.0);
            cl = clmax * (1.0 - t) + cl_plate * 0.7 * t; // lift breaks down past the stall
        }
        const double sa = std::sin(alpha);
        const double cd = w.cd0 + w.induced_k * cl * cl + 1.1 * sa * sa;

        Vec3 lift_dir = m_body.u - vdir * dot(m_body.u, vdir);
        if (lift_dir.length() > 1e-6) force += lift_dir.normalized() * (q * w.wing_area_m2 * cl);
        force -= vdir * (q * w.wing_area_m2 * cd);
        force -= m_body.r * (q * w.wing_area_m2 * w.side_force * beta);
    }

    m_vel += force * (dt / std::max(w.mass_kg, 0.05));
    m_pos += m_vel * dt;
}

bool DroneSim::apply_contact(const Vec3& contact_pos_m, const Vec3& normal, const CollisionParams& p) {
    m_pos = contact_pos_m;

    const Vec3 n = normal.normalized();
    const double vn = dot(m_vel, n);
    if (vn >= 0.0) {
        return false; // already moving away from the surface
    }

    const double impact = -vn;
    m_last_impact = impact;

    const Vec3 v_normal = n * vn;
    Vec3 v_tangent = m_vel - v_normal;

    // Small impacts (resting on the ground, grazing) do not bounce: avoids endless jitter.
    const double restitution = impact > 1.0 ? clampv(p.restitution, 0.0, 1.0) : 0.0;
    v_tangent *= clampv(1.0 - p.friction, 0.0, 1.0);
    m_vel = v_tangent - v_normal * restitution;

    // Impacts kick the attitude: spin around the axis perpendicular to the normal and the
    // direction of travel (a prop clipping a wall flips the quad that way).
    if (p.spin_on_impact > 0.0 && impact > 1.0) {
        const Vec3 travel = (v_tangent + v_normal).normalized();
        const Vec3 axis_w = cross_lh(n, travel);
        const double kick = p.spin_on_impact * impact * 40.0; // deg/s
        m_rates_dps.x += dot(axis_w, m_body.f) * kick;
        m_rates_dps.y += dot(axis_w, m_body.r) * kick;
        m_rates_dps.z += dot(axis_w, m_body.u) * kick * 0.3;
    }

    if (p.crash_enabled && m_arm_state == ArmState::Armed && impact >= p.crash_speed_ms) {
        m_arm_state = ArmState::Crashed;
        m_switch_seen_off = false;
        return true;
    }
    return false;
}

} // namespace fpv
