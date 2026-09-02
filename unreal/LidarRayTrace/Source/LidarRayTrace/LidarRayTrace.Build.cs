using System.IO;
using UnrealBuildTool;

public class LidarRayTrace : ModuleRules
{
	public LidarRayTrace(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = false;
		CppStandard = CppStandardVersion.Cpp17;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"RenderCore",
			"RHI",
			"Renderer"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"Projects"
		});

		// Engine-agnostic core lives outside the plugin so the same sources
		// build with CMake (no Unreal) and with UBT (inside UE5).
		string coreInclude = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../core/include"));
		string coreSrc = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../core/src"));
		PublicIncludePaths.Add(coreInclude);
		PrivateIncludePaths.Add(coreSrc);

		PublicDefinitions.Add("LIDAR_CORE_NO_UE=0");
	}
}
