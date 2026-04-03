#pragma once

#include <QString>
#include <QStringList>
#include <QHash>
#include <QVector>
#include <vector>

#include <osg/Matrixd>

#include "MeshBackendData.h"

#include "robot_urdf_global.h"

/// Parses a URDF file, resolves mesh paths under the package root, and fills MeshHierarchyPart
/// with triangle soups in one world frame (revolute joints at zero angle).
///
/// Conventions (ROS URDF):
/// - Joint origin is the rigid transform from child link frame to parent link frame:
///   p_parent = R(rpy) * p_child + xyz, with R = Rz(yaw)*Ry(pitch)*Rx(roll) on column vectors.
/// - Link visual origin: p_link = R_vis * p_mesh (mesh vertices in visual / mesh file frame).
/// - Revolute / continuous: child pose includes origin * rotation about axis (axis in joint frame).
/// - After the kinematic chain, an optional fixed rotation maps ROS Z-up (REP-103) to the viewer Y-up
///   (see kApplyRosZUpToOsgYUp in UrdfRobotLoader.cpp). Disable if your meshes already match the viewer.
/// - By default, kUrdfBakeJointChainIntoMesh and kUrdfBakeVisualOriginIntoMesh are true: joint chain and \<visual\>\<origin\>
///   are baked into triangle soup so import agrees with URDF kinematics and axis control. For pre-assembled world-frame
///   meshes only, you may set those to false in UrdfRobotLoader.cpp (FK may then disagree with geometry).
///
/// 【中文】URDF 导入约定：解析 package 下 mesh、按零位角烘焙三角网到统一世界系。
/// - 关节 \<origin\>：子连杆相对父连杆，列向量下 p_parent = R(rpy)*p_child + xyz，R 为固定轴 rpy。
/// - \<visual\>\<origin\>：视觉几何相对连杆系；网格顶点应在「视觉/文件」坐标系中。
/// - 转动/连续关节：在 origin 之后绕 \<axis\> 旋转。
/// - 可选 ROS(Z 上)→视窗(Y 上) 固定旋转见 cpp 中 kApplyRosZUpToOsgYUp。
/// - 默认将关节链与 visual 烘焙进顶点；若 mesh 已是整机世界系装配，可在 cpp 中关闭 kUrdfBake*（FK 可能与几何不一致）。
namespace UrdfRobotLoader
{
	/// 【中文】加载 URDF，解析 mesh 路径，输出各连杆三角网（关节角为 0，顶点已乘变换链）。
	ROBOT_URDF_API bool loadMeshHierarchyParts(const QString& urdfFilePath, std::vector<MeshHierarchyPart>& outParts, QString* errorMessage);

	/// Revolute + continuous joints in BFS traversal order (matches jointAnglesRad indices in computeMeshWorldMatrices).
	/// 【中文】按 BFS 顺序列出转动/连续关节名，与 computeMeshWorldMatrices 的 jointAnglesRad 下标一致。
	ROBOT_URDF_API bool loadRevoluteJointNamesInOrder(const QString& urdfFilePath, QStringList& outJointNames, QString* errorMessage = nullptr);

	/// Same order as \ref computeMeshWorldMatrices joint vector. Limits from URDF <limit> (rad); if missing, revolute uses
	/// [-pi, pi], continuous [-2pi, 2pi].
	/// 【中文】同上顺序，并给出各关节弧度上下限；无 \<limit\> 时用上述默认范围。
	ROBOT_URDF_API bool loadRevoluteJointMeta(const QString& urdfFilePath,
		QStringList& outJointNames,
		QVector<double>& outLowerRad,
		QVector<double>& outUpperRad,
		QString* errorMessage = nullptr);

	/// Forward kinematics: OSG world matrix from mesh file frame to viewer frame. Uses full URDF (joint chain ×
	/// visual × optional ROS→OSG), matching the kinematic convention used for axis sliders.
	/// \a jointAnglesRad must align with loadRevoluteJointNamesInOrder; if shorter, missing entries are treated as 0.
	/// Uses an in-memory cache of the parsed URDF (keyed by canonical path + mtime); only the FK walk runs per call.
	/// 【中文】正解：每个带 mesh 的连杆输出 mesh 系到视窗世界系的 osg::Matrixd；关节向量须与 loadRevoluteJointNamesInOrder
	/// 对齐；偏短则缺省为 0。URDF 解析结果按路径+mtime 缓存，每次调用主要做 FK 遍历。
	ROBOT_URDF_API bool computeMeshWorldMatrices(
		const QString& urdfFilePath,
		const QVector<double>& jointAnglesRad,
		QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
		QString* errorMessage = nullptr);

	/// Drops cached parse trees (e.g. after replacing URDF on disk in edge cases). Normally unnecessary: cache keys include mtime.
	/// 【中文】清空已缓存的 URDF 解析树；一般依赖 mtime 自动失效即可。
	ROBOT_URDF_API void clearUrdfModelCache();
}
