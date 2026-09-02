#pragma once

#include "CoreMinimal.h"
#include "LidarTypes.generated.h"

UENUM(BlueprintType)
enum class ELivoxDevice : uint8
{
	Mid360 UMETA(DisplayName = "Livox Mid-360"),
	Avia UMETA(DisplayName = "Livox Avia"),
	Horizon UMETA(DisplayName = "Livox Horizon"),
	Mid40 UMETA(DisplayName = "Livox Mid-40"),
	Mid70 UMETA(DisplayName = "Livox Mid-70"),
	Tele15 UMETA(DisplayName = "Livox Tele-15"),
	Hap UMETA(DisplayName = "Livox HAP")
};

UENUM(BlueprintType)
enum class ELidarTraceBackend : uint8
{
	ChaosAsync UMETA(DisplayName = "Chaos Async LineTrace (recommended default)"),
	HardwareRT UMETA(DisplayName = "Hardware Ray Tracing (DXR / Vulkan RT)")
};

USTRUCT(BlueprintType)
struct LIDARRAYTRACE_API FLidarSimPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	FVector Location = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	float Intensity = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	float TimestampSeconds = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	int32 Ring = 0;
};

USTRUCT(BlueprintType)
struct LIDARRAYTRACE_API FLidarSimCloud
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	TArray<FLidarSimPoint> Points;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	int32 MissedRays = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	float StampSeconds = 0.0f;
};
