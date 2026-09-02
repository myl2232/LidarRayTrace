#pragma once

#include "lidar_core/Math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace lidar {

enum class DeviceId {
    Mid360,
    Avia,
    Horizon,
    Mid40,
    Mid70,
    Tele15,
    Hap,
    Custom
};

struct DeviceSpec {
    DeviceId id = DeviceId::Mid360;
    std::string name = "MID-360";
    std::string scan_mode_file;  // optional Livox CSV in scan_mode/

    float horizontal_fov_deg = 360.0f;
    float vertical_min_deg = -7.0f;
    float vertical_max_deg = 52.0f;

    float min_range_m = 0.1f;
    float max_range_m = 70.0f;
    float max_range_10pct_reflectivity_m = 40.0f;

    float point_rate_hz = 200000.0f;
    float frame_rate_hz = 10.0f;

    float range_sigma_m = 0.02f;
    float angular_sigma_deg = 0.15f;
    float wavelength_nm = 905.0f;

    int beams = 4;
};

struct ScanSample {
    double time_s = 0.0;
    float azimuth_rad = 0.0f;
    float zenith_rad = 0.0f;  // Livox CSV convention (from +Z)
    uint16_t line_id = 0;
};

struct RayRequest {
    uint32_t index = 0;
    double timestamp_s = 0.0;
    float azimuth_rad = 0.0f;
    float elevation_rad = 0.0f;
    Vec3 origin;
    Vec3 direction{1.0f, 0.0f, 0.0f};
    float t_min = 0.1f;
    float t_max = 70.0f;
    uint16_t line_id = 0;
};

struct HitResult {
    bool hit = false;
    float t = 0.0f;
    Vec3 point;
    Vec3 normal{0.0f, 0.0f, 1.0f};
    float intensity = 0.0f;
    uint32_t primitive_id = 0;
};

struct LidarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    double timestamp_s = 0.0;
    uint16_t ring = 0;
    uint8_t tag = 0;
};

struct PointCloud {
    std::string frame_id = "livox_frame";
    double stamp_s = 0.0;
    std::string device_name;
    std::vector<LidarPoint> points;
    uint32_t missed_rays = 0;
};

enum class TraceBackendKind {
    CpuScene,       // standalone, no engine
    ChaosAsync,     // UE5 physics LineTrace batch
    HardwareRT      // UE5 DXR / Vulkan RT
};

}  // namespace lidar
