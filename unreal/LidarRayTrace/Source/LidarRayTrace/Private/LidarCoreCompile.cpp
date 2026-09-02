// Compile the engine-agnostic core inside the UE module. The same files
// are built by CMake without any Unreal headers.
#include "ScanPattern.cpp"
#include "CpuSceneBackend.cpp"
#include "Simulator.cpp"
#include "PointCloudIo.cpp"
