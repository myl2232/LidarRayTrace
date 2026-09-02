#pragma once

#include "lidar_core/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace lidar {

class ScanPattern {
public:
    static ScanPattern fromLivoxCsv(const std::string& path);
    static ScanPattern generateMid360Rose(std::size_t sample_count = 50000);
    static ScanPattern generateRepetitiveSpin(const DeviceSpec& spec, int columns, int rings);

    const std::vector<ScanSample>& samples() const { return samples_; }
    std::size_t size() const { return samples_.size(); }
    bool empty() const { return samples_.empty(); }
    const std::string& source() const { return source_; }

    // Livox non-repetitive sensors cycle the table. One frame uses
    // point_rate / frame_rate consecutive samples.
    std::vector<RayRequest> raysForFrame(const DeviceSpec& spec,
                                         std::size_t& cursor,
                                         double frame_stamp_s,
                                         const Transform& sensor_world) const;

private:
    std::vector<ScanSample> samples_;
    std::string source_;
};

}  // namespace lidar
