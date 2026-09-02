#include "lidar_core/ScanPattern.h"

#include "lidar_core/DeviceCatalog.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lidar {
namespace {

std::string trim(std::string s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool parseLivoxRow(const std::string& line, ScanSample& out, std::size_t index) {
    std::stringstream ss(line);
    std::string t, az, zen;
    if (!std::getline(ss, t, ',') || !std::getline(ss, az, ',') || !std::getline(ss, zen, ',')) {
        return false;
    }
    try {
        out.time_s = std::stod(trim(t));
        out.azimuth_rad = static_cast<float>(std::stod(trim(az)) * kDeg2Rad);
        out.zenith_rad = static_cast<float>(std::stod(trim(zen)) * kDeg2Rad);
        out.line_id = static_cast<uint16_t>(index % 4);
        return true;
    } catch (...) {
        return false;
    }
}

RayRequest sampleToRay(const ScanSample& sample,
                       const DeviceSpec& spec,
                       uint32_t index,
                       double stamp_s,
                       const Transform& sensor_world) {
    RayRequest ray;
    ray.index = index;
    ray.timestamp_s = stamp_s;
    ray.azimuth_rad = sample.azimuth_rad;
    ray.elevation_rad = sample.zenith_rad - kPi * 0.5f;
    ray.line_id = sample.line_id;
    ray.t_min = spec.min_range_m;
    ray.t_max = spec.max_range_m;
    const Vec3 local_dir = directionFromLivoxAzimuthZenith(sample.azimuth_rad, sample.zenith_rad);
    ray.origin = sensor_world.translation;
    ray.direction = sensor_world.applyDir(local_dir).normalized();
    return ray;
}

}  // namespace

ScanPattern ScanPattern::fromLivoxCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open Livox scan_mode CSV: " + path);
    }

    ScanPattern pattern;
    pattern.source_ = path;
    std::string line;
    std::size_t index = 0;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#' || line.find("Time") != std::string::npos ||
            line.find("Azimuth") != std::string::npos) {
            continue;
        }
        ScanSample sample;
        if (!parseLivoxRow(line, sample, index)) {
            continue;
        }
        pattern.samples_.push_back(sample);
        ++index;
    }
    if (pattern.samples_.empty()) {
        throw std::runtime_error("Livox scan_mode CSV contained no samples: " + path);
    }
    return pattern;
}

ScanPattern ScanPattern::generateMid360Rose(std::size_t sample_count) {
    // Four-beam non-repetitive rose. Matches Mid-360 FOV (-7° ~ 52°) and the
    // official CSV structure (groups of 4 simultaneous lasers). Not a bit-exact
    // replica of factory firmware, but dense enough for integration tests and
    // for running without the 50k-row official table.
    ScanPattern pattern;
    pattern.source_ = "procedural:mid360_rose";
    pattern.samples_.reserve(sample_count);

    constexpr int kBeams = 4;
    const float elev_centers_deg[kBeams] = {48.0f, 32.0f, 14.0f, -2.0f};
    const float elev_amp_deg[kBeams] = {6.0f, 10.0f, 12.0f, 5.0f};
    const float az_rate = 37.0f * kDeg2Rad;
    const float rose_rate = 11.0f * kDeg2Rad;
    const float precess = 0.17f * kDeg2Rad;

    for (std::size_t i = 0; i < sample_count; ++i) {
        const int beam = static_cast<int>(i % kBeams);
        const float t = static_cast<float>(i / kBeams);
        ScanSample s;
        s.time_s = static_cast<double>(i);
        s.line_id = static_cast<uint16_t>(beam);
        const float az = wrapTwoPi(az_rate * t + precess * t * t * 0.00002f +
                                   static_cast<float>(beam) * 0.08f);
        float elev_deg = elev_centers_deg[beam] +
                         elev_amp_deg[beam] * std::sin(rose_rate * t + static_cast<float>(beam));
        if (elev_deg < -7.0f) {
            elev_deg = -7.0f;
        }
        if (elev_deg > 52.0f) {
            elev_deg = 52.0f;
        }
        s.azimuth_rad = az;
        s.zenith_rad = (90.0f - elev_deg) * kDeg2Rad;
        pattern.samples_.push_back(s);
    }
    return pattern;
}

ScanPattern ScanPattern::generateRepetitiveSpin(const DeviceSpec& spec, int columns, int rings) {
    ScanPattern pattern;
    pattern.source_ = "procedural:spin/" + spec.name;
    if (columns < 2) {
        columns = 2;
    }
    if (rings < 1) {
        rings = 1;
    }
    const float az0 = -0.5f * spec.horizontal_fov_deg * kDeg2Rad;
    const float az_span = spec.horizontal_fov_deg * kDeg2Rad;
    const float el0 = spec.vertical_min_deg * kDeg2Rad;
    const float el_span = (spec.vertical_max_deg - spec.vertical_min_deg) * kDeg2Rad;
    std::size_t index = 0;
    for (int c = 0; c < columns; ++c) {
        const float az = az0 + az_span * (static_cast<float>(c) / static_cast<float>(columns));
        for (int r = 0; r < rings; ++r) {
            const float u = rings == 1 ? 0.5f : static_cast<float>(r) / static_cast<float>(rings - 1);
            const float elev = el0 + el_span * u;
            ScanSample s;
            s.time_s = static_cast<double>(index);
            s.azimuth_rad = wrapTwoPi(az);
            s.zenith_rad = (kPi * 0.5f) - elev;
            s.line_id = static_cast<uint16_t>(r);
            pattern.samples_.push_back(s);
            ++index;
        }
    }
    return pattern;
}

std::vector<RayRequest> ScanPattern::raysForFrame(const DeviceSpec& spec,
                                                  std::size_t& cursor,
                                                  double frame_stamp_s,
                                                  const Transform& sensor_world) const {
    std::vector<RayRequest> rays;
    if (samples_.empty()) {
        return rays;
    }
    const std::size_t count = static_cast<std::size_t>(
        std::max(1.0f, spec.point_rate_hz / std::max(spec.frame_rate_hz, 1.0f)));
    rays.reserve(count);
    const double dt = 1.0 / std::max(spec.point_rate_hz, 1.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const ScanSample& sample = samples_[(cursor + i) % samples_.size()];
        rays.push_back(sampleToRay(sample, spec, static_cast<uint32_t>(i),
                                   frame_stamp_s + static_cast<double>(i) * dt, sensor_world));
    }
    cursor = (cursor + count) % samples_.size();
    return rays;
}

}  // namespace lidar
