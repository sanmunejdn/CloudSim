#ifndef BACKENDVISUAL_BACKENDGEOMETRYMETRICS_H
#define BACKENDVISUAL_BACKENDGEOMETRYMETRICS_H

/// @file BackendGeometryMetrics.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief BackendGeometryMetrics 接口

#include "backendvisual_global.h"

#include <vector>

#include <osg/Vec3f>

namespace backend_geometry_metrics
{
BACKENDVISUAL_EXPORT osg::Vec3f pointCloudCenterFromXyz(const std::vector<float>& xyz);
BACKENDVISUAL_EXPORT float pointCloudDiagonalFromXyz(const std::vector<float>& xyz);

BACKENDVISUAL_EXPORT osg::Vec3f meshCenterFromSoup(const std::vector<float>& soup);
BACKENDVISUAL_EXPORT float meshDiagonalFromSoup(const std::vector<float>& soup);

} // namespace backend_geometry_metrics

#endif // BACKENDVISUAL_BACKENDGEOMETRYMETRICS_H
