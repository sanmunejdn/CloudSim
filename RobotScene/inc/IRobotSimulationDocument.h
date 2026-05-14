#pragma once

#include "robot_scene_global.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>

class BackendDataManager;

/// Read-only robot simulation state exposed by a document (implemented by \ref DocumentPage in Widget).
/// 【中文】支持动态层级法：存储关节 MatrixTransform 节点，用于直接修改关节角度。
class ROBOT_SCENE_API IRobotSimulationDocument
{
public:
	virtual ~IRobotSimulationDocument() = default;

	virtual bool hasRobotSimulationContext() const = 0;
	virtual bool hasRobotKinematicsBind() const = 0;
	virtual const QString& robotUrdfAbsolutePath() const = 0;
	virtual const QStringList& robotRevoluteJointNames() const = 0;
	virtual const QHash<QString, QString>& robotLinkNameToBackendId() const = 0;

	/// 【中文】多机器人：文档内独立运动学实例数量（单台机器人时为 1）。默认实现按单 URDF 推断。
	virtual int robotKinematicInstanceCount() const
	{
		return robotUrdfAbsolutePath().isEmpty() ? 0 : 1;
	}
	/// 【中文】第 \a instanceIndex 台机器人对应的 URDF 绝对路径。
	virtual QString robotUrdfAbsolutePathForInstance(int instanceIndex) const
	{
		return instanceIndex == 0 ? robotUrdfAbsolutePath() : QString();
	}
	/// 【中文】该实例在「展平关节角向量」中占用的关节个数（与 loadRevoluteJointMeta 顺序一致）。
	virtual int robotRevoluteJointCountForInstance(int instanceIndex) const
	{
		return instanceIndex == 0 ? robotRevoluteJointNames().size() : 0;
	}
	/// 【中文】FK 返回的 URDF 关节名与 \ref robotJointMatrixTransform 键之间的前缀（如 "RobotScene_x_1::"）。
	virtual QString robotJointKeyPrefixForInstance(int instanceIndex) const
	{
		(void)instanceIndex;
		return QString();
	}

	/// 【中文】动态层级法：获取关节的 MatrixTransform 节点，用于直接设置关节角度。
	/// 【English】Dynamic hierarchy: get joint MatrixTransform node for direct angle manipulation.
	/// @param jointName Display key: optional instance prefix + URDF joint name (see \ref robotJointKeyPrefixForInstance).
	/// @return The MatrixTransform node representing the joint, nullptr if not found
	virtual osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const = 0;

	/// 【中文】传统烘焙法的绑定数据（可选保留用于向后兼容）。
	virtual const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const = 0;
	virtual const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const = 0;

	/// True when per-link mesh vertices were transformed from URDF mesh-file frame into link frame (must match FK with identity visual).
	virtual bool robotUrdfMeshVerticesInLinkFrame() const { return false; }

	/// When non-null, \ref RobotSceneKinematics can write per-link \ref MeshBackendData poses for URDF playback.
	virtual BackendDataManager* robotBackendManagerForKinematics() { return nullptr; }

	/// Called after FK has been applied to the scene (per-link backends or hierarchy). Used to propagate follow-attachment dirties.
	virtual void notifyRobotKinematicsAppliedToScene() {}
};
