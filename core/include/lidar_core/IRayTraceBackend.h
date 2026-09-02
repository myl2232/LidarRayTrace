#pragma once

#include "lidar_core/Types.h"

namespace lidar {

// Engine-agnostic intersection port. UE5 implements this with Chaos
// LineTrace or a hardware RT dispatch. Standalone tests use CpuSceneBackend.
class IRayTraceBackend {
public:
    virtual ~IRayTraceBackend() = default;

    virtual const char* name() const = 0;
    virtual TraceBackendKind kind() const = 0;
    virtual bool isAsync() const { return false; }

    // Rays are in world space (same frame as the backend scene).
    // Implementations must write hits.size() == rays.size().
    virtual void traceBatch(const std::vector<RayRequest>& rays, std::vector<HitResult>& hits) = 0;
};

}  // namespace lidar
