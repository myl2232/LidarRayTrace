#pragma once

#include "ChaosAsyncTraceBackend.h"
#include "Components/SceneComponent.h"
#include "HardwareRayTraceBackend.h"
#include "LidarTypes.h"
#include "lidar_core/Simulator.h"
#include "lidar_core/ScanPattern.h"

#include "LidarSensorComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnLidarCloudReady, const FLidarSimCloud&, Cloud);

/**
 * Place on any actor. Each tick (or at FrameRateHz) the standalone Livox
 * scan core emits world-space rays; the selected backend intersects the
 * Unreal scene and the core reassembles a sensor-frame point cloud.
 */
UCLASS(ClassGroup = (Sensors), meta = (BlueprintSpawnableComponent))
class LIDARRAYTRACE_API ULidarSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	ULidarSensorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UFUNCTION(BlueprintCallable, Category = "Lidar")
	FLidarSimCloud CaptureFrame();

	UFUNCTION(BlueprintPure, Category = "Lidar")
	int32 PointsPerFrame() const;

	UFUNCTION(BlueprintPure, Category = "Lidar")
	FString PatternSource() const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	ELivoxDevice Device = ELivoxDevice::Mid360;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	ELidarTraceBackend Backend = ELidarTraceBackend::ChaosAsync;

	/** Official Livox scan_mode CSV. Empty = procedural Mid-360 rose. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (FilePathFilter = "csv"))
	FFilePath ScanModeCsv;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (ClampMin = "1.0"))
	float FrameRateHz = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	bool bAutoCapture = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	UPROPERTY(BlueprintAssignable, Category = "Lidar")
	FOnLidarCloudReady OnCloudReady;

	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	FLidarSimCloud LatestCloud;

private:
	void RebuildSimulator();
	lidar::Transform MakeSensorWorld() const;
	static lidar::DeviceId ToCoreDevice(ELivoxDevice Device);

	TUniquePtr<FChaosAsyncTraceBackend> ChaosBackend;
	TUniquePtr<FHardwareRayTraceBackend> HardwareBackend;
	TUniquePtr<lidar::Simulator> Sim;
	lidar::ScanPattern Pattern;
	float AccumulatedSeconds = 0.0f;
	double SimTimeSeconds = 0.0;
};
