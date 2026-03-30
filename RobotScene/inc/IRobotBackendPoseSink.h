#pragma once

#include "robot_scene_global.h"

#include <osg/Matrixd>

#include <string>

/// OSG backend root pose get/set (implemented by \ref OsgWidget in Widget).
class ROBOT_SCENE_API IRobotBackendPoseSink
{
public:
	virtual ~IRobotBackendPoseSink() = default;

	virtual bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const = 0;
	virtual void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) = 0;
};
