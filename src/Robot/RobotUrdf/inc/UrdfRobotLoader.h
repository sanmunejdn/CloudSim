#ifndef ROBOTURDF_URDFROBOTLOADER_H
#define ROBOTURDF_URDFROBOTLOADER_H

/// @file UrdfRobotLoader.h
/// @brief URDF 层级场景：关节 MatrixTransform + 连杆几何，内部长度 mm（origin xyz 米→mm）

#include "robot_urdf_global.h"

#include "BackendDataBase.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>
#include <vector>

#include <RigidTransform.h>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Node>

/// URDF 层级场景：关节 MatrixTransform + 连杆几何，内部长度 mm（origin xyz 米→mm）
namespace UrdfRobotLoader
{
/// BFS 转动/连续关节名，与 computeMeshWorldMatrices 的 jointAnglesRad 下标一致
ROBOT_URDF_API bool loadRevoluteJointNamesInOrder(const QString& urdfFilePath, QStringList& outJointNames,
												  QString* errorMessage = nullptr);

/// 同上顺序 + 弧度限位；无 limit 时 revolute [-π,π]、continuous [-2π,2π]
ROBOT_URDF_API bool loadRevoluteJointMeta(const QString& urdfFilePath, QStringList& outJointNames,
										  QVector<double>& outLowerRad, QVector<double>& outUpperRad,
										  QString* errorMessage = nullptr);

/// 同上 BFS 顺序的转动关节子连杆名
ROBOT_URDF_API bool loadRevoluteJointChildLinksInOrder(const QString& urdfFilePath, QStringList& outChildLinkNames,
													   QString* errorMessage = nullptr);

/// 运动学树中最深叶连杆（终端连杆）
ROBOT_URDF_API bool loadPrimaryTerminalLinkName(const QString& urdfFilePath, QString& outLinkName,
												QString* errorMessage = nullptr);

/// 子连杆名 → 父连杆名（含 fixed/prismatic 等，用于按链分组 mesh 后端）
ROBOT_URDF_API bool loadLinkChildToParentMap(const QString& urdfFilePath,
											 QHash<QString, QString>& outChildLinkToParentLinkName,
											 QString* errorMessage = nullptr);

/// 给定关节角计算连杆世界矩阵；运行时优先 buildHierarchicalRobotScene + 改 Joint MT
ROBOT_URDF_API bool computeMeshWorldMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
											 QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
											 QString* errorMessage = nullptr,
											 bool meshVerticesAlreadyInLinkFrame = false);

/// 首 visual 的 mesh 文件系→连杆系 4×4 列主序（米→mm × 顶点单位），供烘焙顶点后 FK
ROBOT_URDF_API bool linkMeshFileToLinkColumnMajor16(const QString& urdfFilePath, const QString& linkName,
													double outColumnMajor16[16], QString* errorMessage = nullptr);

/// mesh 文件系→连杆系 OSG 行向量矩阵（与 computeMeshWorldMatrices / 连杆 outer 同约定）
ROBOT_URDF_API bool linkMeshFileToLinkOsgMatrix(const QString& urdfFilePath, const QString& linkName, osg::Matrixd& out,
												QString* errorMessage = nullptr);

/// 连杆坐标系（非 visual mesh 系）世界位姿
ROBOT_URDF_API bool computeLinkWorldMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
											 QHash<QString, osg::Matrixd>& outLinkNameToLinkWorld,
											 QString* errorMessage = nullptr);

/// 同上，输出 engine::RigidTransform（mm，四元数真值）
ROBOT_URDF_API bool computeLinkWorldRigidTransforms(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
													QHash<QString, engine::RigidTransform>& outLinkNameToLinkWorld,
													QString* errorMessage = nullptr);

/// 清空 URDF 解析缓存；正常依赖 mtime 键即可
ROBOT_URDF_API void clearUrdfModelCache();

/// 列出有 mesh visual 且路径可解析的连杆
ROBOT_URDF_API bool enumerateLinkVisualMeshes(const QString& urdfFilePath, QString& outRootLink,
											  QHash<QString, QString>& outLinkNameToAbsoluteMeshPath,
											  QString* errorMessage = nullptr);

/// 各连杆首 visual 的 URDF 材质色；无 color 的不输出
ROBOT_URDF_API bool loadLinkVisualMaterialColors(const QString& urdfFilePath,
												 QHash<QString, BackendColor>& outLinkNameToColor,
												 QString* errorMessage = nullptr);

/// 与场景 Joint MT 一致的关节矩阵：revolute/continuous 仅 R(q)，origin 由 JointN 承担
ROBOT_URDF_API bool computeJointTransformMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
												  QHash<QString, osg::Matrixd>& outJointMatrices,
												  QString* errorMessage = nullptr);

/// 构建三层分离 URDF 场景（几何 PAT + 连杆容器 Group + 关节 MT）
/// 返回 RobotAssembly 根（ref 计数 1，须挂场景或 ref_ptr）；多机时键加 backendId 前缀防碰撞
ROBOT_URDF_API osg::Group* buildHierarchicalRobotScene(const QString& urdfFilePath,
													   QHash<QString, osg::Node*>& outLinkToGeometry,
													   QHash<QString, osg::Group*>& outLinkToContainer,
													   QHash<QString, osg::MatrixTransform*>& outJointTransforms,
													   QString* errorMessage = nullptr);
} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFROBOTLOADER_H
