#include "ChaosAsyncTraceBackend.h"

#include "Async/ParallelFor.h"
#include "CollisionQueryParams.h"
#include "Engine/World.h"

namespace
{
FVector ToUE(const lidar::Vec3& V)
{
	// Core is ROS/Livox FLU metres. Unreal is left-handed centimetres
	// with Y flipped: Xfwd, Yright, Zup.
	return FVector(V.x * 100.0f, -V.y * 100.0f, V.z * 100.0f);
}

lidar::Vec3 FromUE(const FVector& V)
{
	return lidar::Vec3(static_cast<float>(V.X / 100.0), static_cast<float>(-V.Y / 100.0),
	                   static_cast<float>(V.Z / 100.0));
}
}  // namespace

FChaosAsyncTraceBackend::FChaosAsyncTraceBackend(UWorld* World) : World_(World) {}

void FChaosAsyncTraceBackend::traceBatch(const std::vector<lidar::RayRequest>& Rays,
                                         std::vector<lidar::HitResult>& Hits)
{
	Hits.assign(Rays.size(), {});
	if (!World_)
	{
		return;
	}

	FCollisionQueryParams Params(SCENE_QUERY_STAT(LidarRayTrace), false);
	Params.bReturnPhysicalMaterial = false;

	// Mid-360 ~20k rays: ParallelFor over Chaos is the practical "batch"
	// path. Per-ray AsyncLineTraceByChannel creates 20k delegates and is
	// slower. Game-thread callers should invoke this from a worker if they
	// need a full frame of sensor latency.
	ParallelFor(static_cast<int32>(Rays.size()), [&](int32 Index)
	{
		const lidar::RayRequest& Ray = Rays[static_cast<size_t>(Index)];
		const FVector Start = ToUE(Ray.origin + Ray.direction * Ray.t_min);
		const FVector End = ToUE(Ray.origin + Ray.direction * Ray.t_max);

		FHitResult Hit;
		const bool bHit = World_->LineTraceSingleByChannel(Hit, Start, End, Channel_, Params);

		lidar::HitResult Out;
		if (bHit)
		{
			Out.hit = true;
			Out.t = static_cast<float>(Hit.Distance / 100.0) + Ray.t_min;
			Out.point = FromUE(Hit.ImpactPoint);
			Out.normal = FromUE(Hit.ImpactNormal).normalized();
			Out.intensity = 80.0f;
			if (Hit.GetActor())
			{
				Out.primitive_id = static_cast<uint32>(Hit.GetActor()->GetUniqueID());
			}
		}
		Hits[static_cast<size_t>(Index)] = Out;
	});
}
