// DroneSim.hpp - engine-independent FPV quadcopter flight model.
//
// Everything here is in SI units (meters, seconds, degrees for angles exposed to the user)
// and uses Unreal Engine world axes (see MathTypes.hpp). Positions are relative to the
// spawn point so that large world coordinates never hurt precision.
//
// Collisions are detected by the host (it owns the world); it reports contacts back with
// apply_contact() and the sim handles the response (slide, bounce, crash).
#pragma once

#include "MathTypes.hpp"

namespace fpv {

// Same three rate systems as the Betaflight configurator. Values use the configurator's units.
enum class RatesType : int {
    Betaflight = 0, // RC Rate / Super Rate / RC Expo
    Kiss = 1,       // RC Rate / Rate / RC Curve
    Actual = 2,     // Center Sensitivity (deg/s) / Max Rate (deg/s) / Expo
};

struct BetaflightRates {
    double rc_rate{1.0};
    double super_rate{0.7};
    double expo{0.0};
};

struct KissRates {
    double rc_rate{1.0};
    double rate{0.7};
    double rc_curve{0.0};
};

struct ActualRates {
    double center_sensitivity{70.0}; // deg/s
    double max_rate{670.0};          // deg/s
    double expo{0.0};
};

// Every rate system keeps its own values, so switching types does not lose a setup.
struct AxisRates {
    BetaflightRates betaflight{};
    KissRates kiss{};
    ActualRates actual{};
};

enum Axis : int { AxisRoll = 0, AxisPitch = 1, AxisYaw = 2, AxisCount3 = 3 };

enum class FlightMode : int {
    Acro = 0,   // FPV: sticks command body rates, throttle commands thrust
    Cinema = 1, // DJI Inspire-like: sticks command velocities, centered sticks hold position
    Wing = 2,   // flying wing (TBS Mojito-like): acro sticks, elevons, glides
};

// Flying wing, TBS Mojito class (~1.1 m span, ~1.1 kg, pusher prop). Sticks work like acro on a
// flight controller in rate mode, but the control surfaces lose authority at low airspeed and the
// airframe weathervanes into the airflow, so with centered sticks it glides along its flight path.
struct WingParams {
    double mass_kg{1.1};
    double wing_area_m2{0.21};
    double thrust_to_weight{1.4};
    double prop_speed_ms{34.0};     // airspeed where the prop stops pulling (pitch speed)
    double cl_alpha{3.8};           // lift slope (1/rad)
    double cl0{0.05};
    double stall_deg{13.0};
    double cd0{0.025};              // parasitic drag
    double induced_k{0.07};         // induced drag factor (CD = cd0 + k CL^2)
    double side_force{0.8};         // side force per rad of sideslip
    double trim_aoa_deg{4.0};       // angle of attack the airframe settles at
    double gyro_hold{0.6};          // armed: how much the flight controller resists the weathervane
    double pitch_stability{3.0};    // weathervane into the airflow (pitch)
    double yaw_stability{4.0};      // weathervane into the airflow (yaw)
    double control_speed_ms{12.0};  // airspeed for full control authority
    double roll_rate_scale{0.6};    // fraction of the FPV rates
    double pitch_rate_scale{0.35};
    double yaw_rate_scale{0.15};    // no rudder: a little yaw from differential elevons / thrust
    double launch_speed_ms{12.0};   // hand launch
    double camera_tilt_deg{0.0};
};

// Cinema (Inspire 3 style) flight. Sticks: pitch = forward, roll = sideways, throttle
// (centered = hold altitude) = climb / descent, yaw = heading rate.
struct CinemaParams {
    double max_speed_ms{18.0};      // horizontal (Inspire 3 normal mode ~ 18 m/s, sport ~ 26)
    double max_climb_ms{6.0};
    double max_descent_ms{5.0};
    double max_yaw_rate_dps{90.0};
    double max_accel_ms2{6.0};      // horizontal acceleration / braking limit
    double response_s{0.6};         // how softly the velocity follows the sticks
    double yaw_response_s{0.35};
    double stick_expo{0.35};
    double max_tilt_deg{35.0};      // visual body tilt limit
    double lean_response_s{0.35};   // how softly the body leans into accelerations (camera smoothness)
    double camera_tilt_deg{0.0};    // FPV camera: stabilized on tilt only (horizon-referenced), 0 = level
    double tilt_stab_lag_s{0.05};   // the tilt stabilizer reacts this late: short dip / bump on accel and braking (0 = perfect)
};

struct DroneParams {
    FlightMode flight_mode{FlightMode::Acro};
    CinemaParams cinema{};
    WingParams wing{};

    RatesType rates_type{RatesType::Betaflight};
    AxisRates rates[AxisCount3]{};

    double thrust_to_weight{5.0};     // max thrust / weight
    double idle_throttle{0.04};       // airmode idle thrust fraction when armed
    double throttle_expo{0.0};        // 0..1, softens the low end of the stick
    double motor_time_constant{0.03}; // s, first-order motor spool response
    double rate_time_constant{0.025}; // s, how fast body rates follow the setpoint

    double drag_quadratic{0.025};     // 1/m   (a = -k * |v| * v)
    double drag_linear{0.05};         // 1/s
    double vertical_drag_mult{1.6};   // extra drag along the prop disc normal (body up)

    double camera_tilt_deg{25.0};     // FPV camera uptilt
    double gravity{9.81};

    bool angle_mode{false};           // self-level mode
    double max_angle_deg{55.0};
    double angle_gain{8.0};           // deg/s of correction per degree of error

    DroneParams();
};

struct CollisionParams {
    bool enabled{true};
    double radius_m{0.12};        // collision sphere radius
    double restitution{0.25};     // 0 = no bounce, 1 = perfect bounce (normal component)
    double friction{0.15};        // fraction of tangential speed lost per contact
    double spin_on_impact{0.6};   // how much an impact kicks the attitude (0 = none)
    bool crash_enabled{false};    // off: the drone always bounces and stays armed
    double crash_speed_ms{12.0};  // impact speed (along the surface normal) that disarms
};

struct DroneInput {
    double throttle{0.0}; // 0..1
    double roll{0.0};     // -1..1, + = roll right
    double pitch{0.0};    // -1..1, + = stick forward (nose down)
    double yaw{0.0};      // -1..1, + = yaw right
    int arm_switch{-1};   // -1 = no arm switch (arm with throttle low), 0 = off, 1 = on
};

enum class ArmState {
    WaitingForThrottleLow, // hovering in place right after spawn, motors off, not simulated
    Armed,
    Disarmed,              // arm switch off: motors off, falling
    Crashed,               // motors off, falling; re-arm: throttle low (no switch) or switch off->on
};

struct DroneTelemetry {
    double speed_ms{};
    double vertical_speed_ms{};
    double altitude_m{};   // relative to spawn
    double distance_m{};   // horizontal distance from spawn
    double thrust_frac{};
    double last_impact_ms{};
    ArmState arm_state{ArmState::WaitingForThrottleLow};
    bool throttle_blocks_arming{false}; // arm switch is on but the throttle is not low
    bool wing_waiting_launch{false};    // flying wing armed in the hand: throttle up to launch
    double aoa_deg{};                   // flying wing angle of attack
};

class DroneSim {
public:
    // Spawn at the origin of the local frame, level, facing `yaw_deg`.
    void reset(double yaw_deg);

    // Advance the simulation by `dt` seconds (internally sub-stepped).
    void update(const DroneInput& input, const DroneParams& params, double dt);

    // Collision response. `contact_pos_m` is where the drone ends up (already pushed out of
    // the surface), `normal` the unit surface normal pointing away from the surface.
    // Returns true if this contact crashed the drone.
    bool apply_contact(const Vec3& contact_pos_m, const Vec3& normal, const CollisionParams& params);
    void set_position(const Vec3& pos_m) { m_pos = pos_m; }

    // Current heading (deg) of the body's forward axis projected on the ground.
    double heading_deg() const;

    // True while the drone moves on its own (armed or falling after a crash).
    bool is_simulating() const { return m_arm_state != ArmState::WaitingForThrottleLow; }

    const Vec3& position_m() const { return m_pos; }
    const Vec3& velocity_ms() const { return m_vel; }
    const Basis& body() const { return m_body; }
    Basis camera(const DroneParams& params) const;
    DroneTelemetry telemetry() const;

    // Rate curve in deg/s for a stick value in -1..1 (exposed for tests and UI previews).
    static double rate_curve(double stick, RatesType type, const AxisRates& rates);

private:
    void substep(const DroneInput& input, const DroneParams& params, double dt);
    void substep_cinema(const DroneInput& input, const DroneParams& params, double dt);
    void substep_wing(const DroneInput& input, const DroneParams& params, double dt);
    bool m_wing_launched{false};
    bool m_wing_mode{false};
    double m_aoa_deg{0.0};

    Vec3 m_pos{};
    Vec3 m_vel{};
    Basis m_body{};
    Vec3 m_rates_dps{}; // x = roll, y = pitch, z = yaw (deg/s, body frame)
    double m_cine_yaw_rate{0.0};    // deg/s (cinema mode)
    Vec3 m_cine_accel{};            // last commanded acceleration (cinema mode, for the tilt)
    double m_thrust_frac{0.0};
    void arm(bool flip_if_upside_down);
    void update_arming(const DroneInput& input);

    double m_last_impact{0.0};
    ArmState m_arm_state{ArmState::WaitingForThrottleLow};
    bool m_throttle_blocks{false};
    bool m_switch_seen_off{false}; // after a crash, the arm switch must go off before re-arming
};

} // namespace fpv
