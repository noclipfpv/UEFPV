// sim_tests.cpp - portable sanity tests for the flight model (no Unreal / UEVR needed).
// Build: g++ -std=c++20 -O2 -I../src sim_tests.cpp ../src/DroneSim.cpp -o sim_tests
#include <cstdio>
#include <cstdlib>

#include "DroneSim.hpp"

using namespace fpv;

static int g_failures = 0;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            ++g_failures;                                   \
            std::printf("FAIL: %s  ", #cond);               \
            std::printf(__VA_ARGS__);                       \
            std::printf("\n");                              \
        } else {                                            \
            std::printf("ok:   %s\n", #cond);               \
        }                                                   \
    } while (0)

static bool near(double a, double b, double eps) { return std::abs(a - b) <= eps; }

int main() {
    // Rotator <-> basis round trip.
    {
        const Rotator in{20.0, -135.0, 33.0};
        const Rotator out = Basis::from_rotator(in).to_rotator();
        CHECK(near(in.pitch, out.pitch, 1e-6) && near(in.yaw, out.yaw, 1e-6) && near(in.roll, out.roll, 1e-6),
              "got %f %f %f", out.pitch, out.yaw, out.roll);
    }

    // Body rotations match UE sign conventions.
    {
        Basis b;
        b.roll_right(30.0 * kDeg2Rad);
        CHECK(near(b.to_rotator().roll, 30.0, 1e-6), "roll %f", b.to_rotator().roll);
        Basis c;
        c.pitch_up(10.0 * kDeg2Rad);
        CHECK(near(c.to_rotator().pitch, 10.0, 1e-6), "pitch %f", c.to_rotator().pitch);
        Basis d;
        d.yaw_right(45.0 * kDeg2Rad);
        CHECK(near(d.to_rotator().yaw, 45.0, 1e-6), "yaw %f", d.to_rotator().yaw);
    }

    // Rate systems (Betaflight configurator reference values).
    {
        AxisRates r{};
        r.betaflight = {1.0, 0.7, 0.0};
        CHECK(near(DroneSim::rate_curve(1.0, RatesType::Betaflight, r), 666.67, 0.1), "bf %f", DroneSim::rate_curve(1.0, RatesType::Betaflight, r));
        CHECK(near(DroneSim::rate_curve(-1.0, RatesType::Betaflight, r), -666.67, 0.1), "bf symmetry");
        CHECK(near(DroneSim::rate_curve(0.0, RatesType::Betaflight, r), 0.0, 1e-9), "bf center");
        // super rate uses the raw stick, not the expo'd one: BF 1.0 / 0.7 / 0.5 at half stick
        r.betaflight = {1.0, 0.7, 0.5};
        const double half = 200.0 * (0.5 * 0.125 * 0.5 + 0.5 * 0.5) / (1.0 - 0.5 * 0.7);
        CHECK(near(DroneSim::rate_curve(0.5, RatesType::Betaflight, r), half, 1e-6), "bf expo %f vs %f", DroneSim::rate_curve(0.5, RatesType::Betaflight, r), half);

        r.kiss = {1.0, 0.7, 0.0};
        CHECK(near(DroneSim::rate_curve(1.0, RatesType::Kiss, r), 666.67, 0.1), "kiss %f", DroneSim::rate_curve(1.0, RatesType::Kiss, r));

        r.actual = {70.0, 670.0, 0.0};
        CHECK(near(DroneSim::rate_curve(1.0, RatesType::Actual, r), 670.0, 1e-6), "actual max %f", DroneSim::rate_curve(1.0, RatesType::Actual, r));
        const double c = (DroneSim::rate_curve(0.001, RatesType::Actual, r)) / 0.001;
        CHECK(near(c, 70.0, 1.0), "actual center sensitivity %f", c);
        r.actual = {200.0, 100.0, 0.0}; // max below center: linear at center sensitivity
        CHECK(near(DroneSim::rate_curve(1.0, RatesType::Actual, r), 200.0, 1e-6), "actual clamp");
    }

    // Different roll and pitch rates are honored.
    {
        DroneParams p;
        p.rates[AxisRoll].betaflight = {1.0, 0.0, 0.0};  // 200 deg/s
        p.rates[AxisPitch].betaflight = {2.0, 0.0, 0.0}; // 400 deg/s
        DroneSim a; a.reset(0.0); a.update({0, 0, 0, 0}, p, 0.001);
        for (int i = 0; i < 200; ++i) a.update({0.3, 0.2, 0, 0}, p, 0.001);
        DroneSim b; b.reset(0.0); b.update({0, 0, 0, 0}, p, 0.001);
        for (int i = 0; i < 200; ++i) b.update({0.3, 0, 0.2, 0}, p, 0.001);
        const double roll = a.body().to_rotator().roll, pitch = -b.body().to_rotator().pitch;
        CHECK(pitch > roll * 1.8, "roll %f pitch %f", roll, pitch);
    }

    DroneParams params;
    const double hover = (1.0 / params.thrust_to_weight - params.idle_throttle) / (1.0 - params.idle_throttle);

    // Holds position until throttle is low, then arms.
    {
        DroneSim sim;
        sim.reset(0.0);
        sim.update({0.8, 0, 0, 0}, params, 1.0);
        CHECK(sim.position_m().length() < 1e-9, "should not move before arming");
        sim.update({0.0, 0, 0, 0}, params, 0.01);
        CHECK(sim.telemetry().arm_state == ArmState::Armed, "should be armed");
    }

    // Hover throttle keeps altitude roughly constant.
    {
        DroneSim sim;
        sim.reset(0.0);
        sim.update({0.0, 0, 0, 0}, params, 0.001);
        // Let the motors spool up, then check that net vertical acceleration is ~0.
        for (int i = 0; i < 60; ++i) sim.update({hover, 0, 0, 0}, params, 1.0 / 60.0);
        const double vz0 = sim.telemetry().vertical_speed_ms;
        for (int i = 0; i < 60; ++i) sim.update({hover, 0, 0, 0}, params, 1.0 / 60.0);
        const double vz1 = sim.telemetry().vertical_speed_ms;
        CHECK(std::abs(vz1 - vz0) < 0.05 && std::abs(vz1) < 0.5, "vz %f -> %f (hover throttle %.3f)", vz0, vz1, hover);
    }

    // Full throttle climbs, pitched forward flight gains forward speed with a sane top speed.
    {
        DroneSim sim;
        sim.reset(0.0);
        sim.update({0.0, 0, 0, 0}, params, 0.001);
        for (int i = 0; i < 60; ++i) sim.update({1.0, 0, 0, 0}, params, 1.0 / 60.0);
        CHECK(sim.telemetry().altitude_m > 10.0, "alt after 1s full throttle %f", sim.telemetry().altitude_m);

        // pitch forward ~ 0.1 s at full stick, then hold
        for (int i = 0; i < 6; ++i) sim.update({1.0, 0, 1.0, 0}, params, 1.0 / 60.0);
        for (int i = 0; i < 600; ++i) sim.update({1.0, 0, 0, 0}, params, 1.0 / 60.0);
        const double fwd = dot(sim.velocity_ms(), Vec3{1, 0, 0});
        const double kmh = sim.telemetry().speed_ms * 3.6;
        CHECK(fwd > 5.0, "forward speed %f", fwd);
        CHECK(kmh > 60.0 && kmh < 250.0, "top speed %f km/h", kmh);
    }

    // Full roll stick for ~0.54 s at 666 deg/s ends upside-down-ish (sanity on rotation speed).
    {
        DroneSim sim;
        sim.reset(0.0);
        sim.update({0.0, 0, 0, 0}, params, 0.001);
        for (int i = 0; i < 1000; ++i) sim.update({0.3, 1.0, 0, 0}, params, 0.001);
        const Rotator r = sim.body().to_rotator();
        std::printf("      roll after 1 s full stick: %.1f deg (wraps), up.z = %.3f\n", r.roll, sim.body().u.z);
        CHECK(std::abs(sim.body().u.z - 1.0) > 1e-3, "should have rotated");
    }

    // Angle mode self-levels.
    {
        DroneParams ap = params;
        ap.angle_mode = true;
        DroneSim sim;
        sim.reset(0.0);
        sim.update({0.0, 0, 0, 0}, ap, 0.001);
        for (int i = 0; i < 120; ++i) sim.update({hover, 1.0, 0, 0}, ap, 1.0 / 60.0);
        const double roll_full = sim.body().to_rotator().roll;
        for (int i = 0; i < 120; ++i) sim.update({hover, 0.0, 0, 0}, ap, 1.0 / 60.0);
        const double roll_level = sim.body().to_rotator().roll;
        CHECK(near(roll_full, ap.max_angle_deg, 1.0), "roll at full stick %f", roll_full);
        CHECK(std::abs(roll_level) < 1.0, "roll after release %f", roll_level);
    }

    // Cinema (Inspire-like) flight.
    {
        DroneParams cp = params;
        cp.flight_mode = FlightMode::Cinema;
        DroneSim sim; sim.reset(90.0); // facing +Y
        sim.update({0.0, 0, 0, 0}, cp, 0.01); // arm (throttle low)
        for (int i = 0; i < 120; ++i) sim.update({0.5, 0, 0, 0}, cp, 1.0 / 60.0);
        const Vec3 p0 = sim.position_m();
        for (int i = 0; i < 120; ++i) sim.update({0.5, 0, 0, 0}, cp, 1.0 / 60.0);
        CHECK((sim.position_m() - p0).length() < 0.05, "centered sticks hold position (moved %f)", (sim.position_m() - p0).length());

        for (int i = 0; i < 600; ++i) sim.update({0.5, 0, 1.0, 0}, cp, 1.0 / 60.0); // full forward 10 s
        const Vec3 v = sim.velocity_ms();
        CHECK(near(v.y, cp.cinema.max_speed_ms, 0.3) && std::abs(v.x) < 0.3, "forward speed along heading: %f %f", v.x, v.y);
        CHECK(sim.body().to_rotator().pitch < -5.0, "nose down while flying forward: %f", sim.body().to_rotator().pitch);
        CHECK(near(sim.heading_deg(), 90.0, 0.5), "heading kept %f", sim.heading_deg());

        for (int i = 0; i < 360; ++i) sim.update({0.5, 0, 0, 0}, cp, 1.0 / 60.0); // release 6 s
        CHECK(sim.velocity_ms().length() < 0.1, "brakes to a stop (%f m/s)", sim.velocity_ms().length());
        CHECK(std::abs(sim.body().to_rotator().pitch) < 1.0, "level when stopped %f", sim.body().to_rotator().pitch);

        for (int i = 0; i < 300; ++i) sim.update({1.0, 0, 0, 0}, cp, 1.0 / 60.0); // full climb 5 s
        CHECK(near(sim.velocity_ms().z, cp.cinema.max_climb_ms, 0.2), "climb rate %f", sim.velocity_ms().z);

        for (int i = 0; i < 120; ++i) sim.update({0.5, 0, 0, 1.0}, cp, 1.0 / 60.0); // yaw right 2 s
        CHECK(sim.heading_deg() > 90.0 + 100.0 || sim.heading_deg() < -90.0, "yaws right: heading %f", sim.heading_deg());
    }

    // Arm switch (Betaflight-like).
    {
        DroneSim sim; sim.reset(0.0);
        DroneInput in{0.0, 0, 0, 0, 0};
        sim.update(in, params, 0.01);
        CHECK(sim.telemetry().arm_state == ArmState::WaitingForThrottleLow, "switch off: stays disarmed");
        in.arm_switch = 1; in.throttle = 0.5;
        sim.update(in, params, 0.01);
        CHECK(sim.telemetry().arm_state == ArmState::WaitingForThrottleLow && sim.telemetry().throttle_blocks_arming,
              "throttle high blocks arming");
        in.throttle = 0.0;
        sim.update(in, params, 0.01);
        CHECK(sim.telemetry().arm_state == ArmState::Armed, "arms with switch + throttle low");
        in.throttle = 0.6;
        for (int i = 0; i < 30; ++i) sim.update(in, params, 1.0 / 60.0);
        in.arm_switch = 0;
        sim.update(in, params, 1.0 / 60.0);
        CHECK(sim.telemetry().arm_state == ArmState::Disarmed, "switch off disarms in flight");
        const double vz0 = sim.velocity_ms().z;
        for (int i = 0; i < 30; ++i) sim.update(in, params, 1.0 / 60.0);
        CHECK(sim.velocity_ms().z < vz0 - 2.0, "falls when disarmed (vz %f -> %f)", vz0, sim.velocity_ms().z);
        in.arm_switch = 1; in.throttle = 0.0;
        sim.update(in, params, 1.0 / 60.0);
        CHECK(sim.telemetry().arm_state == ArmState::Armed, "re-arm in the air with throttle low");

        // After a crash the switch must be cycled.
        CollisionParams crash; crash.crash_enabled = true; crash.crash_speed_ms = 1.0;
        sim.apply_contact(sim.position_m(), -sim.velocity_ms().normalized(), crash); // head-on
        CHECK(sim.telemetry().arm_state == ArmState::Crashed, "crashed (impact %f)", sim.telemetry().last_impact_ms);
        sim.update(in, params, 1.0 / 60.0);
        CHECK(sim.telemetry().arm_state == ArmState::Crashed, "switch still on: stays crashed");
        in.arm_switch = 0; sim.update(in, params, 1.0 / 60.0);
        in.arm_switch = 1; sim.update(in, params, 1.0 / 60.0);
        CHECK(sim.telemetry().arm_state == ArmState::Armed, "switch cycled: armed again");
    }

    // Contacts: bounce off a wall, slide on the floor, optional crash.
    {
        CollisionParams cp;
        DroneSim sim; sim.reset(0.0); sim.update({0, 0, 0, 0}, params, 0.001);
        for (int i = 0; i < 6; ++i) sim.update({1.0, 0, 1.0, 0}, params, 1.0 / 60.0); // tilt forward
        for (int i = 0; i < 90; ++i) sim.update({0.8, 0, 0, 0}, params, 1.0 / 60.0); // accelerate
        const double vx = sim.velocity_ms().x;
        const bool crashed = sim.apply_contact(sim.position_m(), {-1, 0, 0}, cp); // wall facing -X
        CHECK(vx > 2.0 && !crashed, "vx %f crashed %d", vx, (int)crashed);
        CHECK(sim.velocity_ms().x < 0.0 && near(sim.velocity_ms().x, -vx * cp.restitution, 0.05 * vx), "bounce vx %f", sim.velocity_ms().x);
        CHECK(sim.telemetry().arm_state == ArmState::Armed, "still armed after bounce");

        DroneSim floor; floor.reset(0.0); floor.update({0, 0, 0, 0}, params, 0.001);
        for (int i = 0; i < 60; ++i) { floor.update({0, 0, 0, 0}, params, 1.0 / 60.0); }
        floor.apply_contact({0, 0, 0}, {0, 0, 1}, cp);
        CHECK(floor.velocity_ms().z >= 0.0, "floor stops the fall, vz %f", floor.velocity_ms().z);

        CollisionParams crash = cp; crash.crash_enabled = true; crash.crash_speed_ms = 5.0;
        DroneSim c; c.reset(0.0); c.update({0, 0, 0, 0}, params, 0.001);
        for (int i = 0; i < 90; ++i) c.update({0, 0, 0, 0}, params, 1.0 / 60.0); // free fall ~ 14 m/s
        CHECK(c.apply_contact(c.position_m(), {0, 0, 1}, crash), "hard landing crashes when enabled");
        for (int i = 0; i < 120; ++i) { c.update({0.0, 0, 0, 0}, params, 1.0 / 60.0); c.apply_contact({0, 0, 0}, {0, 0, 1}, crash); }
        CHECK(c.telemetry().arm_state == ArmState::Armed, "re-arms at rest with throttle low");
    }

    // Cinema: full forward / backward / diagonal sticks never change the heading (v0.6.0 yaw spin bug).
    {
        DroneParams cp2; cp2.flight_mode = FlightMode::Cinema;
        for (double pitch : {1.0, -1.0, 0.7}) {
            DroneSim c; c.reset(30.0);
            c.update({0, 0, 0, 0}, cp2, 1.0 / 60.0);
            for (int i = 0; i < 60 * 6; ++i) c.update({0.5, 0.4 * pitch, pitch, 0.0}, cp2, 1.0 / 60.0);
            CHECK(std::abs(c.heading_deg() - 30.0) < 0.5, "cinema heading holds (pitch %.1f): %f", pitch, c.heading_deg());
            CHECK(c.telemetry().speed_ms > 10.0, "cinema reaches speed %f", c.telemetry().speed_ms);
        }
        DroneSim y; y.reset(0.0); y.update({0, 0, 0, 0}, cp2, 1.0 / 60.0);
        for (int i = 0; i < 60; ++i) y.update({0.5, 0.0, 1.0, 1.0}, cp2, 1.0 / 60.0);
        CHECK(y.heading_deg() > 30.0, "cinema yaw stick still turns: %f", y.heading_deg());
    }

    // Flying wing: held until throttle up, then launches and glides.
    {
        DroneParams wp; wp.flight_mode = FlightMode::Wing;
        DroneSim w; w.reset(0.0);
        w.update({0, 0, 0, 0}, wp, 1.0 / 60.0); // arm
        CHECK(w.telemetry().wing_waiting_launch, "wing waits in the hand");
        for (int i = 0; i < 30; ++i) w.update({0.1, 0, 0, 0}, wp, 1.0 / 60.0);
        CHECK(w.position_m().length() < 1e-9, "no launch with low throttle");
        w.update({0.6, 0, 0, 0}, wp, 1.0 / 60.0);
        CHECK(!w.telemetry().wing_waiting_launch && w.velocity_ms().x > 8.0, "hand launch");
        for (int i = 0; i < 60 * 30; ++i) w.update({0.0, 0, 0, 0}, wp, 1.0 / 60.0); // 30 s glide, motor off
        const double drop = -w.position_m().z, dist = w.telemetry().distance_m;
        CHECK(drop > 5.0 && dist / drop > 5.0, "glides: dist %.0f m for %.0f m of height", dist, drop);
        CHECK(w.telemetry().speed_ms > 10.0 && w.telemetry().speed_ms < 30.0, "glide speed %f", w.telemetry().speed_ms);
        DroneSim c; c.reset(0.0); c.update({0, 0, 0, 0}, wp, 1.0 / 60.0); c.update({0.6, 0, 0, 0}, wp, 1.0 / 60.0);
        for (int i = 0; i < 60 * 20; ++i) c.update({0.6, 0, 0, 0}, wp, 1.0 / 60.0);
        CHECK(c.position_m().z > 20.0, "throttle climbs, alt %f", c.position_m().z);
    }

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
    return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
