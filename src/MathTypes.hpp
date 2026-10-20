// MathTypes.hpp - minimal vector / basis math using Unreal Engine conventions.
//
// Unreal world axes: +X forward, +Y right, +Z up (left-handed).
// FRotator: Pitch (nose up +), Yaw (turn right +), Roll (right wing down +), all in degrees.
#pragma once

#include <algorithm>
#include <cmath>

namespace fpv {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

struct Vec3 {
    double x{}, y{}, z{};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        const double l = length();
        return l > 1e-12 ? *this / l : Vec3{};
    }
};

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Cross product with a left-handed sign convention such that cross(Forward, Right) == Up
// for the canonical UE axes (X, Y, Z).
inline Vec3 cross_lh(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Rotator {
    double pitch{}, yaw{}, roll{};
};

// Orientation stored as three orthonormal world-space axes.
struct Basis {
    Vec3 f{1, 0, 0}; // forward (body X)
    Vec3 r{0, 1, 0}; // right   (body Y)
    Vec3 u{0, 0, 1}; // up      (body Z)

    // Same math as UE's FRotationTranslationMatrix.
    static Basis from_rotator(const Rotator& rot) {
        const double sp = std::sin(rot.pitch * kDeg2Rad), cp = std::cos(rot.pitch * kDeg2Rad);
        const double sy = std::sin(rot.yaw * kDeg2Rad), cy = std::cos(rot.yaw * kDeg2Rad);
        const double sr = std::sin(rot.roll * kDeg2Rad), cr = std::cos(rot.roll * kDeg2Rad);

        Basis b;
        b.f = {cp * cy, cp * sy, sp};
        b.r = {sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp};
        b.u = {-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp};
        return b;
    }

    // Same math as UE's FMatrix::Rotator().
    Rotator to_rotator() const {
        Rotator out;
        out.pitch = std::atan2(f.z, std::sqrt(f.x * f.x + f.y * f.y)) * kRad2Deg;
        out.yaw = std::atan2(f.y, f.x) * kRad2Deg;

        const double yaw_rad = out.yaw * kDeg2Rad;
        const Vec3 sy_axis{-std::sin(yaw_rad), std::cos(yaw_rad), 0.0};
        out.roll = std::atan2(dot(u, sy_axis), dot(r, sy_axis)) * kRad2Deg;
        return out;
    }

    // Body-frame rotations (angles in radians).
    void yaw_right(double a) {
        const double c = std::cos(a), s = std::sin(a);
        const Vec3 nf = f * c + r * s;
        const Vec3 nr = r * c - f * s;
        f = nf; r = nr;
    }

    void pitch_up(double a) {
        const double c = std::cos(a), s = std::sin(a);
        const Vec3 nf = f * c + u * s;
        const Vec3 nu = u * c - f * s;
        f = nf; u = nu;
    }

    void roll_right(double a) {
        const double c = std::cos(a), s = std::sin(a);
        const Vec3 nr = r * c - u * s;
        const Vec3 nu = u * c + r * s;
        r = nr; u = nu;
    }

    // Gram-Schmidt re-orthonormalization to remove numerical drift.
    void orthonormalize() {
        f = f.normalized();
        r = (r - f * dot(r, f)).normalized();
        u = cross_lh(f, r).normalized();
    }

    // World vector -> body components (forward, right, up).
    Vec3 to_body(const Vec3& w) const { return {dot(w, f), dot(w, r), dot(w, u)}; }
    // Body components -> world vector.
    Vec3 to_world(const Vec3& b) const { return f * b.x + r * b.y + u * b.z; }
};

template <typename T>
inline T clampv(T v, T lo, T hi) { return std::min(std::max(v, lo), hi); }

} // namespace fpv
