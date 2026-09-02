#pragma once

#include <cmath>
#include <cstdint>

namespace lidar {

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;
inline constexpr float kDeg2Rad = kPi / 180.0f;
inline constexpr float kRad2Deg = 180.0f / kPi;

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    float length() const { return std::sqrt(x * x + y * y + z * z); }

    Vec3 normalized() const {
        const float len = length();
        if (len <= 1e-12f) {
            return {1.0f, 0.0f, 0.0f};
        }
        return {x / len, y / len, z / len};
    }
};

inline constexpr float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Right-handed FLU (Livox / ROS): X forward, Y left, Z up.
// Quaternion stored as (x, y, z, w).
struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    static Quat identity() { return {}; }

    static Quat fromYawPitchRoll(float yaw, float pitch, float roll) {
        const float cy = std::cos(yaw * 0.5f);
        const float sy = std::sin(yaw * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);
        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    Vec3 rotate(const Vec3& v) const {
        const Vec3 qv{x, y, z};
        const Vec3 t = cross(qv, v) * 2.0f;
        return v + t * w + cross(qv, t);
    }
};

struct Transform {
    Vec3 translation;
    Quat rotation = Quat::identity();

    Vec3 apply(const Vec3& p) const { return rotation.rotate(p) + translation; }
    Vec3 applyDir(const Vec3& d) const { return rotation.rotate(d); }

    Transform inverse() const {
        Quat inv;
        inv.x = -rotation.x;
        inv.y = -rotation.y;
        inv.z = -rotation.z;
        inv.w = rotation.w;
        Transform out;
        out.rotation = inv;
        out.translation = inv.rotate(translation * -1.0f);
        return out;
    }
};

// Matches Livox-SDK/livox_laser_simulation:
//   zenith_csv is degrees from +Z; they store elevation = zenith_rad - pi/2
//   then apply Euler(0, elevation, azimuth) to +X.
inline Vec3 directionFromLivoxAzimuthZenith(float azimuth_rad, float zenith_rad) {
    const float elev = zenith_rad - kPi * 0.5f;
    const float ce = std::cos(elev);
    const float se = std::sin(elev);
    const float ca = std::cos(azimuth_rad);
    const float sa = std::sin(azimuth_rad);
    return Vec3{ce * ca, ce * sa, -se}.normalized();
}

inline float wrapTwoPi(float a) {
    a = std::fmod(a, kTwoPi);
    if (a < 0.0f) {
        a += kTwoPi;
    }
    return a;
}

}  // namespace lidar
