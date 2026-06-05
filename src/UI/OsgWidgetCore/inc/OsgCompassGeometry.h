#pragma once

#include "osgwidgetcore_global.h"

#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/ref_ptr>

namespace osg_compass {

/// 与 updateCompassScale / updateTcpTeachCompassScale 除数一致
inline constexpr float kCompassAxisLength = 600.0f;
inline constexpr float kCompassGeomScale = kCompassAxisLength / 120.0f;
inline constexpr double kCompassModelDiagonalFactor = 0.40;
inline constexpr double kCompassMinAxisWorld = 100.0;

struct TransformCompassBranches
{
	osg::ref_ptr<osg::MatrixTransform> axis[3];
	osg::ref_ptr<osg::MatrixTransform> ring[3];
};

/// 对象选择与 TCP 示教共用罗盘网格（实心环 + 正半轴）
OSGWIDGETCORE_EXPORT osg::ref_ptr<osg::Node> buildTransformCompassNode(TransformCompassBranches* outBranches = nullptr);

} // namespace osg_compass
