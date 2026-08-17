/// @file UrdfKinematicsWorkspace.cpp
/// @brief 线程局部 Workspace

#include "UrdfKinematicsWorkspace.h"

namespace UrdfRobotLoader
{
UrdfKinematicsWorkspace& threadLocalKinematicsWorkspace()
{
	thread_local UrdfKinematicsWorkspace ws;
	return ws;
}
} // namespace UrdfRobotLoader
