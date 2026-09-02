#ifndef CLOUDSIMHOST_HEADLESSROBOTPLAYBACKBRIDGE_H
#define CLOUDSIMHOST_HEADLESSROBOTPLAYBACKBRIDGE_H

/// @file HeadlessRobotPlaybackBridge.h
/// @brief Web/Headless：RobotProgramExecutor 服务端回放

#include "cloudsim_host_global.h"

#include "RobotProgramExecutor.h"

#include <QJsonObject>
#include <QObject>
#include <QTimer>
#include <QVector>
#include <functional>
#include <vector>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessRobotPlaybackBridge : public QObject
{
	Q_OBJECT
public:
	explicit HeadlessRobotPlaybackBridge(DocumentHost& host, QObject* parent = nullptr);

	using EventPushFn = std::function<void(const QJsonObject&)>;
	void setEventPushFn(EventPushFn fn) { m_pushEvent = std::move(fn); }

	QJsonObject start(const QJsonObject& body);
	QJsonObject tickOnce();
	QJsonObject stop();
	QJsonObject statusJson() const;

private:
	enum class SeedPolicy
	{
		FromInstruction, // Chain
		FromCurrentPose  // Current：每段以实时关节为种子
	};

	void onTimerTick();
	void fillLazyPlans();
	bool buildPlanResults(const QString& sceneRootBackendId, int instIdx,
						  const std::vector<std::shared_ptr<RobotInstruction::Base>>& instructions,
						  std::vector<RobotInstruction::PlanResult>& outPlans, QString* err);
	bool readLiveJointSeed(const QString& sceneRootBackendId, int instIdx, QVector<double>& outSeed) const;

	DocumentHost& m_host;
	RobotProgramExecutor m_executor;
	QTimer m_timer;
	QString m_sceneRootId;
	int m_instanceIndex = -1;
	EventPushFn m_pushEvent;
	std::vector<const RobotInstruction::Base*> m_motions;
	QVector<double> m_rollingSeedQ;
	SeedPolicy m_seedPolicy = SeedPolicy::FromInstruction;
};

} // namespace cloudsim::host

#endif
