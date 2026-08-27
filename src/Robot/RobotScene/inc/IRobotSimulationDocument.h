#ifndef ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H
#define ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H

/// @file IRobotSimulationDocument.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人仿真文档只读视图（无 osg；矩阵用 Core Mat4）

#include "robot_scene_global.h"

#include "CoreTypes.h"

#include "RobotExternalAxes.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <vector>

class BackendDataManager;

/// 机器人仿真文档只读视图（Widget DocumentPage 实现）
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

	/// per-link 实例 FK 切片（Core Mat4）；层级-only 返回 false
	virtual bool robotPerLinkKinematicsForInstance(int instanceIndex,
												   cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const
	{
		(void)instanceIndex;
		(void)out;
		return false;
	}

	/// 层级机器人是否持有可写关节局部矩阵
	virtual bool hasRobotJointLocalMatrix(const QString& jointName) const
	{
		(void)jointName;
		return false;
	}
	/// 读取关节节点世界矩阵（列主序）
	virtual bool robotJointWorldMatrix(const QString& jointName, cloudsim::core::Mat4& outWorld) const
	{
		(void)jointName;
		(void)outWorld;
		return false;
	}
	/// 写入单个关节局部矩阵（列主序）
	virtual bool applyRobotJointLocalMatrix(const QString& jointName, const cloudsim::core::Mat4& localColumnMajor)
	{
		(void)jointName;
		(void)localColumnMajor;
		return false;
	}
	/// 批量写入关节局部矩阵（键为带前缀关节名）
	virtual void applyRobotJointLocalMatrices(const QHash<QString, cloudsim::core::Mat4>& localByPrefixedJointKey)
	{
		for (auto it = localByPrefixedJointKey.constBegin(); it != localByPrefixedJointKey.constEnd(); ++it)
		{
			(void)applyRobotJointLocalMatrix(it.key(), it.value());
		}
	}

	virtual QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0() const = 0;
	virtual QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBind() const = 0;

	/// mesh 顶点已烘焙到连杆系，FK 须 identity visual
	virtual bool robotUrdfMeshVerticesInLinkFrame() const { return false; }

	virtual BackendDataManager* robotBackendManagerForKinematics() { return nullptr; }

	/// 外轴（无则默认空）
	virtual const RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) const
	{
		(void)instanceIndex;
		static const RobotExternal::RobotExternalAxisConfigSet kEmpty{};
		return kEmpty;
	}
	virtual std::vector<double> robotExternalAxisQ(int instanceIndex) const
	{
		(void)instanceIndex;
		return {};
	}
	virtual void setRobotExternalAxisQ(int instanceIndex, const std::vector<double>& qValues)
	{
		(void)instanceIndex;
		(void)qValues;
	}
	virtual cloudsim::core::Mat4 workpieceExternalBasePlacement(int instanceIndex, const QString& backendId) const
	{
		(void)instanceIndex;
		(void)backendId;
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	virtual void ensureWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
													  const cloudsim::core::Mat4& currentWorld)
	{
		(void)instanceIndex;
		(void)backendId;
		(void)currentWorld;
	}

	virtual void notifyRobotKinematicsAppliedToScene() {}

	/// FK 收口回写：本实例当前关节角（rad，非聚合向量）写入文档真源。
	/// 挂载设备 TCP 重算等读取方只依赖该真源；所有 FK 应用路径（手动/播放/预览/加载）必须在收口处调用，
	/// 否则读取方会拿到陈旧关节角（曾致挂载设备播放期间不跟随、停止后瞬移）。
	virtual void noteRobotJointAnglesAppliedForInstance(int instanceIndex, const QVector<double>& localJointRad)
	{
		(void)instanceIndex;
		(void)localJointRad;
	}
};

#endif // ROBOTSCENE_IROBOTSIMULATIONDOCUMENT_H
