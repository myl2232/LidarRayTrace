#pragma once

#include "lidar_core/IRayTraceBackend.h"

#include <vector>

namespace lidar {

struct PlanePrimitive {
    Vec3 point;
    Vec3 normal{0.0f, 0.0f, 1.0f};
    float reflectivity = 0.8f;
};

struct SpherePrimitive {
    Vec3 center;
    float radius = 1.0f;
    float reflectivity = 0.6f;
};

struct AabbPrimitive {
    Vec3 min{-1.0f, -1.0f, -1.0f};
    Vec3 max{1.0f, 1.0f, 1.0f};
    float reflectivity = 0.4f;
};

// Minimal CPU scene used to develop and test the scan model without Unreal.
class CpuSceneBackend final : public IRayTraceBackend {
public:
    const char* name() const override { return "CpuScene"; }
    TraceBackendKind kind() const override { return TraceBackendKind::CpuScene; }

    void addPlane(const PlanePrimitive& p) { planes_.push_back(p); }
    void addSphere(const SpherePrimitive& s) { spheres_.push_back(s); }
    void addAabb(const AabbPrimitive& b) { aabbs_.push_back(b); }

    // Axis-aligned room: floor + ceiling + four walls.
    void addBoxRoom(float half_xy, float height, float reflectivity = 0.35f);

    void traceBatch(const std::vector<RayRequest>& rays, std::vector<HitResult>& hits) override;

    std::size_t primitiveCount() const { return planes_.size() + spheres_.size() + aabbs_.size(); }

private:
    std::vector<PlanePrimitive> planes_;
    std::vector<SpherePrimitive> spheres_;
    std::vector<AabbPrimitive> aabbs_;
};

}  // namespace lidar
