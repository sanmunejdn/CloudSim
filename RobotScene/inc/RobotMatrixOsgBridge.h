#pragma once

#include "robot_scene_global.h"
#include "BackendFollowMath.h"

#include <osg/Matrixd>
#include <string>
#include <vector>

/// BackendMat4（列主序、列向量）与 osg::Matrixd（行向量 v'=v*M）之间的刚体变换桥接。
namespace RobotMatrixOsg
{

/// 列主序 BackendMat4 → OSG：转置映射，平移落在 osg 第 3 行 (m(3,0..2))，与 UrdfRobotLoader::mat4ToOsg 语义一致。
ROBOT_SCENE_API osg::Matrixd matrixFromBackendColMajor(const BackendMat4& m);

/// OSG → 列主序 BackendMat4（matrixFromBackendColMajor 的逆）。
ROBOT_SCENE_API BackendMat4 backendColMajorFromMatrix(const osg::Matrixd& m);

/// URDF linkWorld（行向量 v'=v*M）与工具系：T_base_target = T_base_flange_osg * T_flange_tool_osg，再落 BackendMat4。
/// 勿对 linkWorld 做 backend_mat4_multiply(backendColMajor(link), T_tool)，会与示教/场景不一致。
ROBOT_SCENE_API BackendMat4 targetInBaseFromFlangeLinkWorld(
	const osg::Matrixd& T_base_flange_osg,
	const BackendMat4& T_flange_tool);

/// T_base_flange_osg = T_base_target_osg * inv(T_flange_tool_osg)（行向量，与 targetInBaseFromFlangeLinkWorld 互逆）。
ROBOT_SCENE_API BackendMat4 flangeTargetFromToolOriginInBase(
	const BackendMat4& T_base_target,
	const BackendMat4& T_flange_tool);

/// T_base_target = T_base_flange * T_flange_tool（行向量；T_base_flange 为 Backend 刚体矩阵）。
ROBOT_SCENE_API BackendMat4 targetInBaseFromFlange(
	const BackendMat4& T_base_flange,
	const BackendMat4& T_flange_tool);

/// 与示教 capture / 工具系 overlay 相同：makeRotate(Rz*Ry*Rx) + setTrans，勿经 BackendMat4 转置。
ROBOT_SCENE_API osg::Matrixd matrixOsgFromPoseMmDeg(
	double px,
	double py,
	double pz,
	double exDeg,
	double eyDeg,
	double ezDeg);

/// 自检：失败信息写入 failures；全部通过返回 true。
ROBOT_SCENE_API bool runConventionSelfTest(std::vector<std::string>& failures);

} // namespace RobotMatrixOsg
