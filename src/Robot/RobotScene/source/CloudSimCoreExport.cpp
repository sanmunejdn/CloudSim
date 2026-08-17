/// @file CloudSimCoreExport.cpp
/// @brief CloudSimCore导出

#include "robot_scene_global.h"

#include "CloudSimCoreVersion.h"
#include "IRobotService.h"
#include "NullCoreServices.h"

extern "C" ROBOT_SCENE_API cloudsim::core::IRobotService* cloudsimCreateRobotService(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::core::makeNullRobotService().release();
}
