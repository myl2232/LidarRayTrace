#pragma once

#include "Modules/ModuleManager.h"

class FLidarRayTraceModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
