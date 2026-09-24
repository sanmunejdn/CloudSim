#ifndef ROBOTSCENE_CONTROLCONTEXT_H
#define ROBOTSCENE_CONTROLCONTEXT_H

/// @file ControlContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器步进缓冲：目标角待应用、传感快照只读

#include "robot_scene_global.h"

#include "IRobotSimulationDocument.h"

#include <QMutex>
#include <QStringList>
#include <QVector>

class IRobotBackendPoseSink;

/// Webots 式 step 屏障：set 只入缓冲，apply 才写场景
class ROBOT_SCENE_API ControlContext
{
public:
	void reset();

	void setRobotInstanceIndex(int instanceIndex);
	int robotInstanceIndex() const;

	void setJointNames(const QStringList& names);
	QStringList jointNames() const;
	int jointCount() const;

	/// 写入本帧待应用目标（覆盖未应用的旧目标）
	void setPendingTargets(const QVector<double>& targetJointRad);
	bool hasPendingTargets() const;
	QVector<double> pendingTargets() const;
	void clearPendingTargets();

	void setSensorSnapshot(const QVector<double>& actualJointRad);
	QVector<double> sensorSnapshot() const;

	/// 采样文档侧关节名；失败返回 false
	bool sampleJointMetaFromDocument(IRobotSimulationDocument* doc);

	/// 应用 pending → FK；成功后写入 snapshot 并清空 pending
	bool applyPendingToScene(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
							 QVector<double>& aggregatedJointAnglesRad);

private:
	mutable QMutex m_mutex;
	int m_robotInstanceIndex = 0;
	QStringList m_jointNames;
	QVector<double> m_pendingTargets;
	bool m_hasPending = false;
	QVector<double> m_sensorSnapshot;
};

#endif // ROBOTSCENE_CONTROLCONTEXT_H
