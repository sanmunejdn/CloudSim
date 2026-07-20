#ifndef ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H
#define ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H

/// @file IRobotSimulationDocument.h
/// @brief 单台机器人 per-link FK 切片（每连杆一个 mesh 后端）

#include "robot_scene_global.h"

#include "CoreTypes.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/ref_ptr>

class BackendDataManager;

/// 单台机器人 per-link FK 切片（每连杆一个 mesh 后端）
struct RobotPerLinkKinematicsSlice
{
	QString urdfAbsolutePath;
	QString sceneRootBackendId;
	QHash<QString, QString> linkNameToBackendId;
	QHash<QString, osg::Matrixd> fkMeshWorldT0;
	QHash<QString, osg::Matrixd> outerWorldAtBindByBackendId;
	/// 左乘 FK 输出；来自机器人场景根位姿属性
	osg::Matrixd robotBasePlacementWorld;
	bool meshVerticesInLinkFrame = false;
};

/// DTO 版本（供 Core 接口使用，Widget 不依赖 osg）
namespace cloudsim::core
{
struct RobotPerLinkKinematicsSliceDto;
}

/// 机器人仿真文档只读视图（Widget DocumentPage 实现）；动态层级法存关节 MT 节点
class ROBOT_SCENE_API IRobotSimulationDocument
{
public:
	virtual ~IRobotSimulationDocument() = default;

	virtual bool hasRobotSimulationContext() const = 0;
	virtual bool hasRobotKinematicsBind() const = 0;
	virtual const QString& robotUrdfAbsolutePath() const = 0;
	virtual const QStringList& robotRevoluteJointNames() const = 0;
	virtual const QHash<QString, QString>& robotLinkNameToBackendId() const = 0;

	virtual int robotKinematicInstanceCount() const { return robotUrdfAbsolutePath().isEmpty() ? 0 : 1; }
	virtual QString robotUrdfAbsolutePathForInstance(int instanceIndex) const
	{
		return instanceIndex == 0 ? robotUrdfAbsolutePath() : QString();
	}
	virtual int robotRevoluteJointCountForInstance(int instanceIndex) const
	{
		return instanceIndex == 0 ? robotRevoluteJointNames().size() : 0;
	}
	/// FK 关节键前缀，如 "RobotScene_x_1::"
	virtual QString robotJointKeyPrefixForInstance(int instanceIndex) const
	{
		(void)instanceIndex;
		return QString();
	}

	virtual bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const
	{
		(void)instanceIndex;
		return !robotLinkNameToBackendId().isEmpty();
	}

	/// 层级-only 实例返回 false
	virtual bool robotPerLinkKinematicsForInstance(int instanceIndex, RobotPerLinkKinematicsSlice& out) const
	{
		(void)instanceIndex;
		(void)out;
		return false;
	}

	/// DTO 版本（Widget 优先调用，避免 osg 依赖）
	virtual bool robotPerLinkKinematicsDtoForInstance(int instanceIndex,
													  cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const
	{
		(void)instanceIndex;
		(void)out;
		return false;
	}

	/// 动态层级：直接改关节角的 MatrixTransform
	virtual osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const = 0;

	virtual const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const = 0;
	virtual const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const = 0;

	/// DTO 版本（Widget 优先调用，避免 osg 依赖）
	virtual QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0Dto() const { return {}; }
	virtual QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBindDto() const { return {}; }

	/// mesh 顶点已烘焙到连杆系，FK 须 identity visual
	virtual bool robotUrdfMeshVerticesInLinkFrame() const { return false; }

	virtual BackendDataManager* robotBackendManagerForKinematics() { return nullptr; }

	virtual void notifyRobotKinematicsAppliedToScene() {}
};

#endif // ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H
