#pragma once

#include "CoreMinimal.h"
#include "lidar_core/IRayTraceBackend.h"

class UWorld;

/**
 * Hardware RT backend (DXR / Vulkan RT / inline RayQuery).
 *
 * This class is the UE-side adapter only. The actual dispatch lives on the
 * render thread:
 *   1. Enable r.RayTracing=1 and a hardware-RT capable RHI.
 *   2. After TLAS build (FSceneViewExtensionBase / PostTLASBuild),
 *      upload InRays as a structured buffer.
 *   3. Dispatch a 1D RayGen or inline compute shader (N = ray count).
 *      Do NOT use a 2D view-space dispatch — Livox patterns are not a grid.
 *   4. Read back hit t / normal / instance to fill HitResult.
 *
 * Independent of a specific game project, but NOT independent of UE5: the
 * scene TLAS belongs to the renderer. The scan table and point assembly
 * remain in lidar_core.
 *
 * Current implementation: safe CPU fallback that reports the missing GPU
 * path so the plugin still loads on machines without RT hardware. Replace
 * DispatchHardwareRays() with the RDG pass in docs/UE5_HARDWARE_RT.md.
 */
class LIDARRAYTRACE_API FHardwareRayTraceBackend final : public lidar::IRayTraceBackend
{
public:
	explicit FHardwareRayTraceBackend(UWorld* World);

	const char* name() const override { return "HardwareRT"; }
	lidar::TraceBackendKind kind() const override { return lidar::TraceBackendKind::HardwareRT; }
	bool isAsync() const override { return true; }

	void SetWorld(UWorld* World) { World_ = World; }
	bool IsHardwareAvailable() const;
	// True only after the RDG / TLAS dispatch in DispatchHardwareRays is wired.
	bool CanDispatch() const { return false; }

	void traceBatch(const std::vector<lidar::RayRequest>& Rays, std::vector<lidar::HitResult>& Hits) override;

private:
	bool DispatchHardwareRays(const std::vector<lidar::RayRequest>& Rays, std::vector<lidar::HitResult>& Hits);

	UWorld* World_ = nullptr;
};
