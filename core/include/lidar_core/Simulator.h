#pragma once

#include "lidar_core/IRayTraceBackend.h"
#include "lidar_core/ScanPattern.h"
#include "lidar_core/Types.h"

#include <cstdint>
#include <memory>

namespace lidar {

struct SimConfig {
    DeviceSpec device = {};
    bool apply_range_noise = true;
    bool apply_angular_noise = false;
    bool drop_out_of_range = true;
    uint32_t rng_seed = 1;
};

class Simulator {
public:
    Simulator(SimConfig config, ScanPattern pattern, IRayTraceBackend& backend);

    const DeviceSpec& device() const { return config_.device; }
    const ScanPattern& pattern() const { return pattern_; }
    IRayTraceBackend& backend() { return backend_; }
    std::size_t cursor() const { return cursor_; }
    void resetCursor() { cursor_ = 0; }

    std::size_t pointsPerFrame() const;

    PointCloud simulateFrame(const Transform& sensor_world, double frame_stamp_s);

private:
    SimConfig config_;
    ScanPattern pattern_;
    IRayTraceBackend& backend_;
    std::size_t cursor_ = 0;
};

}  // namespace lidar
