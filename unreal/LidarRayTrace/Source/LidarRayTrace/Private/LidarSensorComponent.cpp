#include "LidarSensorComponent.h"

#include "HardwareRayTraceBackend.h"
#include "lidar_core/DeviceCatalog.h"

#include "Engine/World.h"

ULidarSensorComponent::ULidarSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void ULidarSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	RebuildSimulator();
}

void ULidarSensorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Sim.Reset();
	ChaosBackend.Reset();
	HardwareBackend.Reset();
	Super::EndPlay(EndPlayReason);
}

void ULidarSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bAutoCapture || !Sim)
	{
		return;
	}
	AccumulatedSeconds += DeltaTime;
	const float Period = 1.0f / FMath::Max(FrameRateHz, 1.0f);
	if (AccumulatedSeconds + KINDA_SMALL_NUMBER >= Period)
	{
		AccumulatedSeconds = FMath::Fmod(AccumulatedSeconds, Period);
		CaptureFrame();
	}
}

int32 ULidarSensorComponent::PointsPerFrame() const
{
	return Sim ? static_cast<int32>(Sim->pointsPerFrame()) : 0;
}

FString ULidarSensorComponent::PatternSource() const
{
	return FString(UTF8_TO_TCHAR(Pattern.source().c_str()));
}

void ULidarSensorComponent::RebuildSimulator()
{
	UWorld* World = GetWorld();
	ChaosBackend = MakeUnique<FChaosAsyncTraceBackend>(World);
	ChaosBackend->SetTraceChannel(TraceChannel);
	HardwareBackend = MakeUnique<FHardwareRayTraceBackend>(World);

	lidar::DeviceSpec Spec = lidar::deviceFromId(ToCoreDevice(Device));
	Spec.frame_rate_hz = FrameRateHz;

	if (!ScanModeCsv.FilePath.IsEmpty())
	{
		Pattern = lidar::ScanPattern::fromLivoxCsv(TCHAR_TO_UTF8(*ScanModeCsv.FilePath));
	}
	else if (Device == ELivoxDevice::Mid360)
	{
		Pattern = lidar::ScanPattern::generateMid360Rose(50000);
	}
	else
	{
		Pattern = lidar::ScanPattern::generateRepetitiveSpin(Spec, 1800, Spec.beams);
	}

	lidar::SimConfig Cfg;
	Cfg.device = Spec;
	lidar::IRayTraceBackend* Chosen = ChaosBackend.Get();
	if (Backend == ELidarTraceBackend::HardwareRT && HardwareBackend->CanDispatch() &&
	    HardwareBackend->IsHardwareAvailable())
	{
		Chosen = HardwareBackend.Get();
	}
	Sim = MakeUnique<lidar::Simulator>(Cfg, Pattern, *Chosen);
}

lidar::Transform ULidarSensorComponent::MakeSensorWorld() const
{
	const FTransform T = GetComponentTransform();
	const FVector P = T.GetLocation();
	const FQuat Q = T.GetRotation();

	lidar::Transform Out;
	Out.translation = lidar::Vec3(static_cast<float>(P.X / 100.0), static_cast<float>(-P.Y / 100.0),
	                              static_cast<float>(P.Z / 100.0));
	// Unreal quat is left-handed; negate Y to move into ROS FLU.
	Out.rotation = lidar::Quat{static_cast<float>(Q.X), static_cast<float>(-Q.Y),
	                           static_cast<float>(Q.Z), static_cast<float>(Q.W)};
	return Out;
}

lidar::DeviceId ULidarSensorComponent::ToCoreDevice(ELivoxDevice InDevice)
{
	switch (InDevice)
	{
		case ELivoxDevice::Avia:
			return lidar::DeviceId::Avia;
		case ELivoxDevice::Horizon:
			return lidar::DeviceId::Horizon;
		case ELivoxDevice::Mid40:
			return lidar::DeviceId::Mid40;
		case ELivoxDevice::Mid70:
			return lidar::DeviceId::Mid70;
		case ELivoxDevice::Tele15:
			return lidar::DeviceId::Tele15;
		case ELivoxDevice::Hap:
			return lidar::DeviceId::Hap;
		case ELivoxDevice::Mid360:
		default:
			return lidar::DeviceId::Mid360;
	}
}

FLidarSimCloud ULidarSensorComponent::CaptureFrame()
{
	FLidarSimCloud Cloud;
	if (!Sim)
	{
		RebuildSimulator();
	}
	if (!Sim)
	{
		return Cloud;
	}

	const lidar::PointCloud Native = Sim->simulateFrame(MakeSensorWorld(), SimTimeSeconds);
	SimTimeSeconds += 1.0 / FMath::Max(FrameRateHz, 1.0f);

	Cloud.MissedRays = static_cast<int32>(Native.missed_rays);
	Cloud.StampSeconds = static_cast<float>(Native.stamp_s);
	Cloud.Points.Reserve(static_cast<int32>(Native.points.size()));
	for (const lidar::LidarPoint& P : Native.points)
	{
		FLidarSimPoint Out;
		Out.Location = FVector(P.x * 100.0f, -P.y * 100.0f, P.z * 100.0f);
		Out.Intensity = P.intensity;
		Out.TimestampSeconds = static_cast<float>(P.timestamp_s);
		Out.Ring = static_cast<int32>(P.ring);
		Cloud.Points.Add(Out);
	}
	LatestCloud = Cloud;
	OnCloudReady.Broadcast(Cloud);
	return Cloud;
}
