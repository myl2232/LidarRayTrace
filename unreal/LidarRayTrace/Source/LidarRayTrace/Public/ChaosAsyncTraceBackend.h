#pragma once

#include "CoreMinimal.h"
#include "lidar_core/IRayTraceBackend.h"

class UWorld;

/**
 * UE5 Chaos / physics LineTrace backend.
 *
 * Mid-360 at 10 Hz is 20,000 rays/frame. Individual AsyncLineTraceByChannel
 * calls have per-request overhead, so the default path is a ParallelFor of
 * LineTraceSingleByChannel against the physics scene. That is still "batch
 * async" from the game-thread point of view when bRunAsyncWorker is true:
 * the worker copies the ray list and traces off-thread, then the component
 * publishes the previous frame (one-frame sensor latency, which real lidars
 * also have).
 *
 * Accuracy: collision meshes, not Nanite/render meshes.
 */
class LIDARRAYTRACE_API FChaosAsyncTraceBackend final : public lidar::IRayTraceBackend
{
public:
	explicit FChaosAsyncTraceBackend(UWorld* World);

	const char* name() const override { return "ChaosAsync"; }
	lidar::TraceBackendKind kind() const override { return lidar::TraceBackendKind::ChaosAsync; }
	bool isAsync() const override { return true; }

	void SetWorld(UWorld* World) { World_ = World; }
	void SetTraceChannel(ECollisionChannel Channel) { Channel_ = Channel; }

	void traceBatch(const std::vector<lidar::RayRequest>& Rays, std::vector<lidar::HitResult>& Hits) override;

private:
	UWorld* World_ = nullptr;
	ECollisionChannel Channel_ = ECC_Visibility;
};
