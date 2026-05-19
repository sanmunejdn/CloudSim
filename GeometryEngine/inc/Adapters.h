#pragma once

#include "geometry_engine_global.h"
#include "RigidTransform.h"

#include <array>
#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3f>

namespace engine
{

/// Column-major 16-double layout (compatible with BackendMat4::v).
using ColMajorMat4 = std::array<double, 16>;

GEOMETRY_ENGINE_API RigidTransform rigidTransformFromOsg(const osg::Matrixd& m);
GEOMETRY_ENGINE_API osg::Matrixd osgMatrixFromRigidTransform(const RigidTransform& t);

GEOMETRY_ENGINE_API RigidTransform rigidTransformFromColMajor(const ColMajorMat4& m);
GEOMETRY_ENGINE_API ColMajorMat4 colMajorFromRigidTransform(const RigidTransform& t);

GEOMETRY_ENGINE_API osg::Quat eulerDegToQuat(double exDeg, double eyDeg, double ezDeg);
GEOMETRY_ENGINE_API void quatToEulerDeg(const osg::Quat& q, double& exDeg, double& eyDeg, double& ezDeg);
GEOMETRY_ENGINE_API osg::Vec3f quatToEulerDegVec3f(const osg::Quat& q);

} // namespace engine
