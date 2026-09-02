#include "HardwareRayTraceBackend.h"

#include "Engine/World.h"
#include "RHI.h"

FHardwareRayTraceBackend::FHardwareRayTraceBackend(UWorld* World) : World_(World) {}

bool FHardwareRayTraceBackend::IsHardwareAvailable() const
{
#if RHI_RAYTRACING
	return GRHISupportsRayTracing;
#else
	return false;
#endif
}

bool FHardwareRayTraceBackend::DispatchHardwareRays(const std::vector<lidar::RayRequest>& Rays,
                                                    std::vector<lidar::HitResult>& Hits)
{
	// Intentionally not dispatching here: a correct GPU path must run on the
	// render thread after TLAS build. See docs/UE5_HARDWARE_RT.md and
	// Shaders/Private/LidarHardwareRT.usf. Returning false lets the caller
	// keep using the Chaos backend instead of producing an empty cloud.
	(void)Rays;
	(void)Hits;
	return false;
}

void FHardwareRayTraceBackend::traceBatch(const std::vector<lidar::RayRequest>& Rays,
                                          std::vector<lidar::HitResult>& Hits)
{
	Hits.assign(Rays.size(), {});
	if (IsHardwareAvailable() && DispatchHardwareRays(Rays, Hits))
	{
		return;
	}
}
