#pragma once

#include <QString>
#include <QStringList>
#include <QHash>
#include <QVector>
#include <vector>

#include <osg/Matrixd>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Node>

#include "robot_urdf_global.h"

/// 【中文】URDF 机器人层级场景图加载：解析 URDF，构建三层分离的动态层级场景图。
///
/// 新架构（动态层级法）：
/// - 停止修改顶点，改为移动节点
/// - osg::MatrixTransform 节点 = URDF 中的 <joint>
/// - osg::Geode/PAT = URDF 中的 Link 几何体
/// - 层级结构：Parent_Container -> Joint_MT -> Child_Container -> Geometry
///
/// 坐标约定 (ROS URDF)：
/// - 内部长度单位为毫米：URDF 文件中 origin 的 xyz 为米（REP-103），加载后换算为 mm；网格顶点默认按 mm 文件坐标。
/// - Joint origin 是子连杆系到父连杆系的刚体变换：p_parent = R(rpy) * p_child + xyz
/// - Link visual origin：p_link = R_vis * p_mesh
/// - Revolute / continuous：绕 axis 旋转（axis 定义在 joint 坐标系中）
/// - 可选 ROS Z-up → OSG Y-up 坐标系转换
namespace UrdfRobotLoader
{
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

	/// 【中文】给定关节角度，计算各连杆的世界变换矩阵（用于参考/调试）。
	/// 新架构推荐使用 buildHierarchicalRobotScene + 直接修改 Joint MatrixTransform，无需调用此函数。
	///
	/// 此函数保持向后兼容，用于需要显式获取连杆世界矩阵的场景。
	ROBOT_URDF_API bool computeMeshWorldMatrices(
		const QString& urdfFilePath,
		const QVector<double>& jointAnglesRad,
		QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
		QString* errorMessage = nullptr);

	/// Drops cached parse trees (e.g. after replacing URDF on disk in edge cases). Normally unnecessary: cache keys include mtime.
	/// 【中文】清空已缓存的 URDF 解析树；一般依赖 mtime 自动失效即可。
	ROBOT_URDF_API void clearUrdfModelCache();

	/// 【中文】计算给定关节角度下、与场景中可 setMatrix 的关节节点一致的矩阵。
	/// - revolute/continuous：输出仅 R(q)（\<origin\> 由场景中 JointN(T_origin) 节点承担）
	/// - 其他关节类型：输出完整 parent_T_child
	///
	/// @param urdfFilePath URDF 文件路径
	/// @param jointAnglesRad 关节角度（弧度），顺序与 loadRevoluteJointNamesInOrder 一致
	/// @param outJointMatrices 输出：JointName -> osg::Matrixd（含义见上）
	/// @param errorMessage 可选错误输出
	/// @return true 成功，false 失败
	ROBOT_URDF_API bool computeJointTransformMatrices(
		const QString& urdfFilePath,
		const QVector<double>& jointAnglesRad,
		QHash<QString, osg::Matrixd>& outJointMatrices,
		QString* errorMessage = nullptr);

	/// 【中文】构建层级化 URDF 机器人场景图（三层分离架构）。
	/// 执行完整的四阶段流程：
	/// 1. 资源预加载：解析 URDF，加载所有 Link 的 Mesh 文件到 OSG 节点
	/// 2. 构建容器化场景图：创建 Link 容器 (osg::Group, setCullingActive(false)) 和 Joint MatrixTransform
	/// 3. 组装根节点：创建 RobotAssembly 总根节点并连接层级
	/// 4. 状态重置：重置几何体 PAT 矩阵为单位阵，刷新包围盒
	///
	/// 架构说明（三层分离模型）：
	/// - 几何体层：原始 Mesh 数据，使用 PAT/Geode，矩阵固定为单位阵 (位置=0,0,0)
	/// - 视觉容器层：osg::Group 包裹几何体，setCullingActive(false) 防止裁剪问题
	/// - 运动学层：转动关节为 JointN(T_origin) -> JointContent(Group) -> { Axis_Visual_N, JointRotation(R(q)) } -> LinkN 壳；非转动关节为单节点 parent_T_child
	///
	/// 层级结构（转动关节示意，N 为第 N 个转动关节序号）：
	///   RobotAssembly (Group)
	///    └── Link_A_Container (Group)
	///         └── Link_A_Geometry (MatrixTransform, visual origin × mesh)
	///         └── JointN (MatrixTransform, T_origin)
	///              └── JointContent (Group：合并轴线与旋转子树包围球)
	///                   ├── Axis_Visual_N (Group：关节系 \<axis\> 黄线)
	///                   └── \<URDF关节名\> (MatrixTransform, R(q) — 滑条更新此节点)
	///                   └── LinkN (Group)
	///                        └── Link_B_Container (Group)
	///                             └── Link_B_Geometry (...)
	///
	/// 【English】Build hierarchical URDF robot scene graph with three-layer separation.
	/// Geometry layer (PAT with identity matrix), Visual Container layer (Group with culling off),
	/// Kinematic layer (MatrixTransform storing joint parent_T_child).
	///
	/// @param urdfFilePath URDF file absolute path
	/// @param outLinkToGeometry Output: LinkName -> Geometry layer root (MatrixTransform: visual origin × mesh)
	/// @param outLinkToContainer Output: LinkName -> Visual container layer Group node
	/// @param outJointTransforms Output: JointName -> Kinematic layer MatrixTransform node
	/// @param errorMessage Optional error output (non-empty on failure)
	/// @return RobotAssembly root node (osg::Group), nullptr on failure
	///
	/// Caller responsibilities:
	/// - Add returned RobotAssembly to m_objectsGroup for display
	/// - Preserve outLinkToGeometry/outJointTransforms for runtime joint updates
	/// - To update joint angles: modify outJointTransforms[jointName]->setMatrix(newMatrix)
	/// - Multiple robots: prefix keys when merging into a document (e.g. backendId + "::" + jointName) so names do not collide.
	///
	/// Ownership: returns a raw pointer with reference count 1 (transferred from an internal ref_ptr via release()).
	/// The caller must attach the node to the scene graph (e.g. addChild) or store an osg::ref_ptr immediately;
	/// do not use manual ref() before return on the caller side to "fix" lifetime.
	ROBOT_URDF_API osg::Group* buildHierarchicalRobotScene(
		const QString& urdfFilePath,
		QHash<QString, osg::Node*>& outLinkToGeometry,
		QHash<QString, osg::Group*>& outLinkToContainer,
		QHash<QString, osg::MatrixTransform*>& outJointTransforms,
		QString* errorMessage = nullptr);
}
