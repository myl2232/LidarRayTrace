#include "lidar_core/Simulator.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace lidar {
namespace {

float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace

Simulator::Simulator(SimConfig config, ScanPattern pattern, IRayTraceBackend& backend)
    : config_(std::move(config)), pattern_(std::move(pattern)), backend_(backend) {}

std::size_t Simulator::pointsPerFrame() const {
    return static_cast<std::size_t>(
        std::max(1.0f, config_.device.point_rate_hz / std::max(config_.device.frame_rate_hz, 1.0f)));
}

PointCloud Simulator::simulateFrame(const Transform& sensor_world, double frame_stamp_s) {
    PointCloud cloud;
    cloud.stamp_s = frame_stamp_s;
    cloud.device_name = config_.device.name;
    cloud.frame_id = "livox_frame";

    const std::vector<RayRequest> rays =
        pattern_.raysForFrame(config_.device, cursor_, frame_stamp_s, sensor_world);

    std::vector<HitResult> hits;
    backend_.traceBatch(rays, hits);

    std::mt19937 rng(config_.rng_seed + static_cast<uint32_t>(cursor_));
    std::normal_distribution<float> range_noise(0.0f, config_.device.range_sigma_m);

    const Transform world_to_sensor = sensor_world.inverse();
    cloud.points.reserve(rays.size());

    const std::size_t n = std::min(rays.size(), hits.size());
    for (std::size_t i = 0; i < n; ++i) {
        const RayRequest& ray = rays[i];
        const HitResult& hit = hits[i];
        if (!hit.hit) {
            ++cloud.missed_rays;
            continue;
        }

        float t = hit.t;
        if (config_.apply_range_noise) {
            t = std::max(ray.t_min, t + range_noise(rng));
        }
        if (config_.drop_out_of_range && (t < ray.t_min || t > ray.t_max)) {
            ++cloud.missed_rays;
            continue;
        }

        const Vec3 world_p = ray.origin + ray.direction * t;
        const Vec3 local = world_to_sensor.apply(world_p);

        LidarPoint pt;
        pt.x = local.x;
        pt.y = local.y;
        pt.z = local.z;
        pt.intensity = clampf(hit.intensity, 0.0f, 255.0f);
        pt.timestamp_s = ray.timestamp_s;
        pt.ring = ray.line_id;
        pt.tag = 0;
        cloud.points.push_back(pt);
    }
    return cloud;
}

}  // namespace lidar
