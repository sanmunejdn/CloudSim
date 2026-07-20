#ifndef BACKENDVISUAL_BACKENDGEOMETRYMETRICS_H
#define BACKENDVISUAL_BACKENDGEOMETRYMETRICS_H

/// @file BackendGeometryMetrics.h
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
