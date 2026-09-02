#include "LidarRayTraceModule.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FLidarRayTraceModule"

void FLidarRayTraceModule::StartupModule()
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("LidarRayTrace"));
	if (Plugin.IsValid())
	{
		const FString ShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));
		AddShaderSourceDirectoryMapping(TEXT("/Plugin/LidarRayTrace"), ShaderDir);
	}
}

void FLidarRayTraceModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FLidarRayTraceModule, LidarRayTrace)
