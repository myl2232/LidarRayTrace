#pragma once

#include "lidar_core/Types.h"

#include <array>
#include <optional>
#include <string_view>

namespace lidar {

inline DeviceSpec makeMid360() {
    DeviceSpec s;
    s.id = DeviceId::Mid360;
    s.name = "MID-360";
    s.scan_mode_file = "mid360.csv";
    s.horizontal_fov_deg = 360.0f;
    s.vertical_min_deg = -7.0f;
    s.vertical_max_deg = 52.0f;
    s.min_range_m = 0.1f;
    s.max_range_m = 70.0f;
    s.max_range_10pct_reflectivity_m = 40.0f;
    s.point_rate_hz = 200000.0f;
    s.frame_rate_hz = 10.0f;
    s.range_sigma_m = 0.02f;
    s.angular_sigma_deg = 0.15f;
    s.beams = 4;
    return s;
}

inline DeviceSpec makeAvia() {
    DeviceSpec s;
    s.id = DeviceId::Avia;
    s.name = "Avia";
    s.scan_mode_file = "avia.csv";
    s.horizontal_fov_deg = 70.4f;
    s.vertical_min_deg = -38.6f;
    s.vertical_max_deg = 38.6f;
    s.min_range_m = 0.05f;
    s.max_range_m = 450.0f;
    s.max_range_10pct_reflectivity_m = 260.0f;
    s.point_rate_hz = 240000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 6;
    return s;
}

inline DeviceSpec makeHorizon() {
    DeviceSpec s;
    s.id = DeviceId::Horizon;
    s.name = "Horizon";
    s.scan_mode_file = "horizon.csv";
    s.horizontal_fov_deg = 81.7f;
    s.vertical_min_deg = -12.55f;
    s.vertical_max_deg = 12.55f;
    s.min_range_m = 0.5f;
    s.max_range_m = 260.0f;
    s.max_range_10pct_reflectivity_m = 90.0f;
    s.point_rate_hz = 240000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 6;
    return s;
}

inline DeviceSpec makeMid40() {
    DeviceSpec s;
    s.id = DeviceId::Mid40;
    s.name = "Mid-40";
    s.scan_mode_file = "mid40.csv";
    s.horizontal_fov_deg = 38.4f;
    s.vertical_min_deg = -19.2f;
    s.vertical_max_deg = 19.2f;
    s.min_range_m = 0.5f;
    s.max_range_m = 260.0f;
    s.max_range_10pct_reflectivity_m = 90.0f;
    s.point_rate_hz = 100000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 1;
    return s;
}

inline DeviceSpec makeMid70() {
    DeviceSpec s;
    s.id = DeviceId::Mid70;
    s.name = "Mid-70";
    s.scan_mode_file = "mid70.csv";
    s.horizontal_fov_deg = 70.4f;
    s.vertical_min_deg = -35.2f;
    s.vertical_max_deg = 35.2f;
    s.min_range_m = 0.05f;
    s.max_range_m = 260.0f;
    s.max_range_10pct_reflectivity_m = 90.0f;
    s.point_rate_hz = 100000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 1;
    return s;
}

inline DeviceSpec makeTele15() {
    DeviceSpec s;
    s.id = DeviceId::Tele15;
    s.name = "Tele-15";
    s.scan_mode_file = "tele.csv";
    s.horizontal_fov_deg = 14.5f;
    s.vertical_min_deg = -8.1f;
    s.vertical_max_deg = 8.1f;
    s.min_range_m = 0.5f;
    s.max_range_m = 500.0f;
    s.max_range_10pct_reflectivity_m = 320.0f;
    s.point_rate_hz = 240000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 1;
    return s;
}

inline DeviceSpec makeHap() {
    DeviceSpec s;
    s.id = DeviceId::Hap;
    s.name = "HAP";
    s.scan_mode_file = "HAP.csv";
    s.horizontal_fov_deg = 120.0f;
    s.vertical_min_deg = -12.5f;
    s.vertical_max_deg = 12.5f;
    s.min_range_m = 0.1f;
    s.max_range_m = 150.0f;
    s.max_range_10pct_reflectivity_m = 80.0f;
    s.point_rate_hz = 452000.0f;
    s.frame_rate_hz = 10.0f;
    s.beams = 6;
    return s;
}

inline std::array<DeviceSpec, 7> allBuiltinDevices() {
    return {makeMid360(), makeAvia(), makeHorizon(), makeMid40(), makeMid70(), makeTele15(),
            makeHap()};
}

inline std::optional<DeviceSpec> findDevice(std::string_view name) {
    for (const DeviceSpec& spec : allBuiltinDevices()) {
        if (spec.name == name || spec.scan_mode_file == name) {
            return spec;
        }
    }
    if (name == "mid360" || name == "MID360" || name == "Mid360") {
        return makeMid360();
    }
    return std::nullopt;
}

inline DeviceSpec deviceFromId(DeviceId id) {
    switch (id) {
        case DeviceId::Avia:
            return makeAvia();
        case DeviceId::Horizon:
            return makeHorizon();
        case DeviceId::Mid40:
            return makeMid40();
        case DeviceId::Mid70:
            return makeMid70();
        case DeviceId::Tele15:
            return makeTele15();
        case DeviceId::Hap:
            return makeHap();
        case DeviceId::Mid360:
        case DeviceId::Custom:
        default:
            return makeMid360();
    }
}

}  // namespace lidar
