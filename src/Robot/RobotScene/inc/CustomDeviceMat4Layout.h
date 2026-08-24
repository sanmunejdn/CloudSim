#ifndef ROBOTSCENE_CUSTOMDEVICEMAT4LAYOUT_H
#define ROBOTSCENE_CUSTOMDEVICEMAT4LAYOUT_H

#include "BackendFollowMath.h"
#include "robot_scene_global.h"

namespace CustomDeviceMat4Layout
{
/// Backend/OSG 打包矩阵（平移 v[3,7,11]）→ KinematicCore 列主序（平移 v[12..14]）
ROBOT_SCENE_API void osgBackendToKinematicCore(const double osgBackend[16], double kinematicCore[16]);

/// KinematicCore 列主序 → Backend/OSG 打包矩阵
ROBOT_SCENE_API void kinematicCoreToOsgBackend(const double kinematicCore[16], double osgBackend[16]);

ROBOT_SCENE_API void backendMat4ToKinematicCore(const BackendMat4& src, double kinematicCore[16]);

ROBOT_SCENE_API BackendMat4 kinematicCoreToBackendMat4(const double kinematicCore[16]);

ROBOT_SCENE_API bool kinematicCoreInvertRigid(const double kinematicCore[16], double outKinematicCore[16]);

} // namespace CustomDeviceMat4Layout

#endif // ROBOTSCENE_CUSTOMDEVICEMAT4LAYOUT_H
