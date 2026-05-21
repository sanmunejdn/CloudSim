#pragma once

#include "backendvisual_global.h"

#include <osg/Vec3f>
#include <vector>

namespace backend_geometry_metrics {

BACKENDVISUAL_EXPORT osg::Vec3f pointCloudCenterFromXyz(const std::vector<float>& xyz);
BACKENDVISUAL_EXPORT float pointCloudDiagonalFromXyz(const std::vector<float>& xyz);

BACKENDVISUAL_EXPORT osg::Vec3f meshCenterFromSoup(const std::vector<float>& soup);
BACKENDVISUAL_EXPORT float meshDiagonalFromSoup(const std::vector<float>& soup);

} // namespace backend_geometry_metrics
