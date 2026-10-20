// gimbal_tests.cpp - portable tests for the gimbal (no Unreal / UEVR needed).
#include <cstdio>
#include <cstdlib>

#include "FlightRecorder.hpp"
#include "Gimbal.hpp"

using namespace fpv;

static int g_failures = 0;
#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) { ++g_failures; std::printf("FAIL: %s  ", #cond); std::printf(__VA_ARGS__); std::printf("\n"); } \
        else { std::printf("ok:   %s\n", #cond); }                              \
    } while (0)

static bool near(double a, double b, double eps) { return std::abs(a - b) <= eps; }

int main() {
    const double dt = 1.0 / 60.0;
    GimbalParams p;

    // Optics: 24 mm on full frame = 73.7 deg horizontal.
    CHECK(near(Gimbal::hfov_deg(24.0, 36.0), 73.74, 0.05), "hfov %f", Gimbal::hfov_deg(24.0, 36.0));
    CHECK(near(Gimbal::hfov_deg(200.0, 36.0), 10.29, 0.05), "hfov 200mm %f", Gimbal::hfov_deg(200.0, 36.0));

    // Stabilized: no input, no motion -> nothing moves.
    {
        Gimbal g; g.reset(30.0, -10.0, 24.0);
        for (int i = 0; i < 120; ++i) g.update({}, p, false, 0.0, {0, 0, 0}, dt);
        const Rotator r = g.orientation();
        CHECK(near(r.yaw, 30.0, 1e-9) && near(r.pitch, -10.0, 1e-9) && near(r.roll, 0.0, 1e-9), "%f %f %f", r.pitch, r.yaw, r.roll);
    }

    // Pan with inertia: speed ramps up, then reaches the max speed.
    {
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        GimbalInput in; in.pan = 1.0;
        g.update(in, p, false, 0.0, {}, dt);
        const double first = g.orientation().yaw;
        CHECK(first > 0.0 && first < p.max_pan_speed * dt * 0.2, "eases in (%f)", first);
        for (int i = 0; i < 120; ++i) g.update(in, p, false, 0.0, {}, dt);
        const double y0 = g.orientation().yaw;
        g.update(in, p, false, 0.0, {}, dt);
        CHECK(near(g.orientation().yaw - y0, p.max_pan_speed * dt, 0.05), "full speed step %f", g.orientation().yaw - y0);
        // releases with inertia (keeps drifting a little)
        in.pan = 0.0;
        g.update(in, p, false, 0.0, {}, dt);
        CHECK(g.orientation().yaw - (y0 + p.max_pan_speed * dt) > 0.5, "eases out");
    }

    // Tilt limits.
    {
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        GimbalInput in; in.tilt = -1.0;
        for (int i = 0; i < 600; ++i) g.update(in, p, false, 0.0, {}, dt);
        const Basis tb = Basis::from_rotator(g.orientation());
        CHECK(near(tb.f.z, std::sin(p.tilt_min * kDeg2Rad), 1e-6), "tilt min f.z %f", tb.f.z);
        GimbalInput a; a.has_tilt_absolute = true; a.tilt_absolute = 1.0;
        for (int i = 0; i < 600; ++i) g.update(a, p, false, 0.0, {}, dt);
        CHECK(near(g.orientation().pitch, p.tilt_max, 0.01), "absolute tilt to max %f", g.orientation().pitch);
    }

    // Follow mode tracks the heading smoothly.
    {
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        g.update({}, p, true, 90.0, {}, dt);
        CHECK(g.orientation().yaw > 0.0 && g.orientation().yaw < 10.0, "smooth start %f", g.orientation().yaw);
        for (int i = 0; i < 600; ++i) g.update({}, p, true, 90.0, {}, dt);
        CHECK(near(g.orientation().yaw, 90.0, 0.1), "follows heading %f", g.orientation().yaw);
        for (int i = 0; i < 600; ++i) g.update({}, p, true, -170.0, {}, dt); // short way round
        CHECK(near(g.orientation().yaw, -170.0, 0.1), "wraps %f", g.orientation().yaw);
    }

    // Ronin drift: accelerating to the camera's right rolls the camera left, then recovers.
    {
        GimbalParams pd = p; pd.ronin_drift = true; // off by default
        Gimbal g; g.reset(0.0, 0.0, 24.0); // camera looks +X, right is +Y
        Vec3 pos{}, vel{};
        for (int i = 0; i < 60; ++i) { // 1 s at 0.5 g to the right
            vel.y += 0.5 * 9.81 * dt; pos += vel * dt;
            g.update({}, pd, false, 0.0, pos, dt);
        }
        const double d = g.drift_deg();
        CHECK(d < -1.5 && d > -pd.drift_max_deg - 1e-9, "drift under lateral g %f", d);
        for (int i = 0; i < 180; ++i) { pos += vel * dt; g.update({}, pd, false, 0.0, pos, dt); }
        CHECK(std::abs(g.drift_deg()) < 0.2, "recovers %f", g.drift_deg());
        GimbalParams off = p; off.ronin_drift = false;
        Gimbal h; h.reset(0.0, 0.0, 24.0); pos = {}; vel = {};
        for (int i = 0; i < 60; ++i) { vel.y += 0.5 * 9.81 * dt; pos += vel * dt; h.update({}, off, false, 0.0, pos, dt); }
        CHECK(std::abs(h.drift_deg()) < 1e-9, "no drift when disabled");
    }

    // Zoom: full trigger doubles the focal each second, clamped.
    {
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        GimbalInput in; in.zoom = 1.0;
        for (int i = 0; i < 60; ++i) g.update(in, p, false, 0.0, {}, dt);
        CHECK(near(g.focal_mm(), 48.0, 0.5), "zoom x2 in 1 s: %f", g.focal_mm());
        for (int i = 0; i < 600; ++i) g.update(in, p, false, 0.0, {}, dt);
        CHECK(near(g.focal_mm(), p.focal_max, 1e-9), "clamped %f", g.focal_mm());
    }

    // Flight recorder: record, replay with interpolation, loop, freeze after replay.
    {
        FlightRecorder r;
        Basis b;
        for (int i = 0; i <= 100; ++i) r.record(i == 0 ? 0.0 : 0.01, {i * 0.1, 0, 0}, b); // 1 s, 10 m
        CHECK(near(r.duration(), 1.0, 1e-9), "duration %f", r.duration());
        r.start_playback();
        r.advance(0.255, false);
        CHECK(near(r.position().x, 2.55, 1e-6), "interpolated %f", r.position().x);
        r.record(0.01, {99, 0, 0}, b);
        CHECK(near(r.duration(), 1.0, 1e-9), "no recording during replay");
        r.advance(1.0, true);
        CHECK(near(r.playback_time(), 0.255, 1e-6), "loops %f", r.playback_time());
        r.advance(5.0, false);
        CHECK(near(r.position().x, 10.0, 1e-6), "stops at the end %f", r.position().x);
        r.stop_playback();
        r.record(0.01, {99, 0, 0}, b);
        CHECK(r.frozen() && near(r.duration(), 1.0, 1e-9), "take frozen until cleared");
        r.clear();
        CHECK(r.empty() && !r.frozen(), "cleared");
    }

    // Inverted tilt past -90 (RS 4 Pro under a drone): looks backward and down, canonical rotator.
    {
        GimbalParams pi = p;
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        GimbalInput in{}; in.tilt = -1.0;
        for (int i = 0; i < 600; ++i) g.update(in, pi, false, 0.0, {}, dt);
        const Rotator r = g.orientation();
        const Basis b = Basis::from_rotator(r);
        CHECK(r.pitch >= -90.0 && r.pitch <= 90.0, "canonical pitch %f", r.pitch);
        CHECK(std::abs(b.f.z - std::sin(-110.0 * kDeg2Rad)) < 1e-3 && b.f.x < -0.3, "looks back-down f=(%f,%f)", b.f.x, b.f.z);
    }

    // Absolute zoom (operator radio throttle stick): stick position = focal length, log scale.
    {
        Gimbal g; g.reset(0.0, 0.0, 24.0);
        GimbalInput in{}; in.has_zoom_absolute = true;
        in.zoom_absolute = 1.0;
        for (int i = 0; i < 300; ++i) g.update(in, p, false, 0.0, {}, dt);
        CHECK(std::abs(g.focal_mm() - p.focal_max) < 0.5, "stick up = max focal %f", g.focal_mm());
        in.zoom_absolute = -1.0;
        for (int i = 0; i < 300; ++i) g.update(in, p, false, 0.0, {}, dt);
        CHECK(std::abs(g.focal_mm() - p.focal_min) < 0.1, "stick down = min focal %f", g.focal_mm());
        in.zoom_absolute = 0.0;
        for (int i = 0; i < 300; ++i) g.update(in, p, false, 0.0, {}, dt);
        CHECK(std::abs(g.focal_mm() - std::sqrt(p.focal_min * p.focal_max)) < 0.5, "center = geometric mid %f", g.focal_mm());
    }

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
    return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
