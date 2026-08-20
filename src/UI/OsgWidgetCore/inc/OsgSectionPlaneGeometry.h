#ifndef OSGWIDGETCORE_OSGSECTIONPLANEGEOMETRY_H
#define OSGWIDGETCORE_OSGSECTIONPLANEGEOMETRY_H

/// @file OsgSectionPlaneGeometry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 半透明截面片（局部 Z 为法向，参与深度遮挡）

#include "osgwidgetcore_global.h"

#include "OsgCompassGeometry.h"

#include <osg/Node>
#include <osg/ref_ptr>

namespace osg_section_plane
{
/// 半透明截面片（局部 Z 为法向，参与深度遮挡）
OSGWIDGETCORE_EXPORT osg::ref_ptr<osg::Node> buildSectionPlaneQuadNode(float planeHalfSizeMm = 400.f);

/// 半透明截面片 + 罗盘（局部 Z 为法向）
OSGWIDGETCORE_EXPORT osg::ref_ptr<osg::Node>
buildSectionPlaneNode(float planeHalfSizeMm = 400.f, osg_compass::TransformCompassBranches* outBranches = nullptr);

} // namespace osg_section_plane

#endif // OSGWIDGETCORE_OSGSECTIONPLANEGEOMETRY_H
