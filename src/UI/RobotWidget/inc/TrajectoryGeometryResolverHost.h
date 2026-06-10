#pragma once

#include "robotwidget_global.h"

class IRobotDocumentHost;
class IRobotOsgViewHost;

namespace trajectory_geometry_host
{

/// 将文档/OSG 几何烘焙桥注册到 RobotScene（预览/Apply 前调用）
ROBOTWIDGET_EXPORT void bindTrajectoryGeometryResolver(
	IRobotDocumentHost* document,
	IRobotOsgViewHost* osg);

ROBOTWIDGET_EXPORT void clearTrajectoryGeometryResolverBinding();

} // namespace trajectory_geometry_host
