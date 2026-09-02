/// @file HeadlessRobotPlaybackBridge.cpp
/// @brief 服务端程序回放

#include "headless/HeadlessRobotPlaybackBridge.h"

#include "DocumentHost.h"
#include "HeadlessRobotContext.h"
#include "IRobotBackendPoseSink.h"
#include "RobotInstructionProgram.h"
#include "RobotPlanInstruction.h"
#include "RobotProgramStore.h"
#include "UrdfRobotLoader.h"
#include "io/IoSignalNetwork.h"

#include <QJsonArray>
#include <QMetaObject>

#include <algorithm>

namespace cloudsim::host
{
namespace
{
QJsonArray jointsToJson(const QVector<double>& q)
{
	QJsonArray arr;
	for (double v : q)
		arr.append(v);
	return arr;
}

QString defaultTcpLinkForUrdf(const QString& urdfPath)
{
	QString preferred;
	if (UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferred, nullptr) && !preferred.isEmpty())
		return preferred;
	QStringList childLinks;
	if (UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr) && !childLinks.isEmpty())
		return childLinks.back();
	return QString();
}
} // namespace

HeadlessRobotPlaybackBridge::HeadlessRobotPlaybackBridge(DocumentHost& host, QObject* parent)
	: QObject(parent), m_host(host)
{
	m_timer.setInterval(40);
	connect(&m_timer, &QTimer::timeout, this, &HeadlessRobotPlaybackBridge::onTimerTick);
}

bool HeadlessRobotPlaybackBridge::readLiveJointSeed(const QString& sceneRootBackendId, int instIdx,
													QVector<double>& outSeed) const
{
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc || instIdx < 0)
		return false;
	const int nj = hrc->robotRevoluteJointCountForInstance(instIdx);
	outSeed = QVector<double>(nj, 0.0);
	QVector<double> localQ;
	if (!m_host.robotLocalJointAnglesForSceneRoot(sceneRootBackendId, localQ) || nj <= 0)
		return nj > 0;
	for (int j = 0; j < nj && j < localQ.size(); ++j)
		outSeed[j] = localQ[j];
	return true;
}

bool HeadlessRobotPlaybackBridge::buildPlanResults(
	const QString& sceneRootBackendId, int instIdx,
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& instructions,
	std::vector<RobotInstruction::PlanResult>& outPlans, QString* err)
{
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("No headless robot context.");
		return false;
	}
	const auto motions = RobotInstruction::collectMotionInstructions(instructions);
	outPlans.clear();
	outPlans.reserve(motions.size());

	const QString urdfPath = hrc->robotUrdfAbsolutePathForInstance(instIdx);
	const std::string defaultTcp = defaultTcpLinkForUrdf(urdfPath).toStdString();
	const int nj = hrc->robotRevoluteJointCountForInstance(instIdx);
	QVector<double> rollingQ;
	if (!readLiveJointSeed(sceneRootBackendId, instIdx, rollingQ))
		rollingQ = QVector<double>(nj, 0.0);

	constexpr size_t kEager = 16;
	bool stoppedAfterFailure = false;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		RobotInstruction::PlanResult pr;
		if (stoppedAfterFailure)
		{
			pr.ok = false;
			pr.plannerName = "skippedAfterFailure";
			pr.summary = "Skipped after earlier planning failure";
			outPlans.push_back(pr);
			continue;
		}
		if (mi >= kEager)
		{
			pr.ok = false;
			pr.plannerName = "lazyPending";
			pr.summary = "Deferred until playback";
			outPlans.push_back(pr);
			continue;
		}
		const RobotInstruction::Base* motion = motions[mi];
		if (!motion)
		{
			if (err)
				*err = QStringLiteral("Invalid motion instruction.");
			return false;
		}
		// Current：每段重新取实时关节，不滚链式终点
		QVector<double> seedQ = rollingQ;
		if (m_seedPolicy == SeedPolicy::FromCurrentPose)
			(void)readLiveJointSeed(sceneRootBackendId, instIdx, seedQ);

		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motion);
		QString planErr;
		if (!planRobotInstruction(*hrc, *ins, seedQ, instIdx, urdfPath, defaultTcp, sceneRootBackendId, pr, &planErr))
		{
			pr.ok = false;
			pr.plannerName = "failed";
			if (pr.summary.empty())
				pr.summary = planErr.toStdString();
			outPlans.push_back(pr);
			stoppedAfterFailure = true;
			continue;
		}
		outPlans.push_back(pr);
		if (m_seedPolicy == SeedPolicy::FromInstruction && pr.ok && !pr.jointTargetsRad.empty())
		{
			for (size_t j = 0; j < pr.jointTargetsRad.size() && j < static_cast<size_t>(rollingQ.size()); ++j)
				rollingQ[static_cast<int>(j)] = pr.jointTargetsRad[j];
		}
	}
	return true;
}

QJsonObject HeadlessRobotPlaybackBridge::start(const QJsonObject& body)
{
	stop();
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc)
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("No robot context.")}};

	const QString seedPolicyRaw = body.value(QStringLiteral("seedPolicy")).toString();
	if (!seedPolicyRaw.isEmpty() &&
		seedPolicyRaw.compare(QStringLiteral("FromInstruction"), Qt::CaseInsensitive) != 0 &&
		seedPolicyRaw.compare(QStringLiteral("Chain"), Qt::CaseInsensitive) != 0 &&
		seedPolicyRaw.compare(QStringLiteral("FromCurrentPose"), Qt::CaseInsensitive) != 0 &&
		seedPolicyRaw.compare(QStringLiteral("Current"), Qt::CaseInsensitive) != 0)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"),
				 QStringLiteral("invalid seedPolicy; expected FromInstruction|Chain|FromCurrentPose|Current")}};
	}
	m_seedPolicy = (seedPolicyRaw.compare(QStringLiteral("FromCurrentPose"), Qt::CaseInsensitive) == 0 ||
					seedPolicyRaw.compare(QStringLiteral("Current"), Qt::CaseInsensitive) == 0)
					   ? SeedPolicy::FromCurrentPose
					   : SeedPolicy::FromInstruction;

	QString sceneRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	if (sceneRoot.isEmpty())
	{
		const auto inst = hrc->listInstances();
		if (!inst.isEmpty())
			sceneRoot = inst.front().sceneRootBackendId;
	}
	m_sceneRootId = sceneRoot;
	m_instanceIndex = hrc->robotInstanceIndexForBackendId(sceneRoot);
	if (m_instanceIndex < 0)
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("Robot instance not found.")}};

	const QString programId = body.value(QStringLiteral("programId")).toString();
	std::vector<std::shared_ptr<RobotInstruction::Base>> instructions;
	m_host.robotProgramStore().ensureRobotBackendId(sceneRoot);
	if (!programId.isEmpty())
	{
		auto* prog = m_host.robotProgramStore().catalogFor(sceneRoot).findProgram(programId.toStdString());
		if (prog)
			instructions = prog->steps;
	}
	if (instructions.empty())
		instructions = m_host.robotProgramStore().programFor(sceneRoot);
	if (instructions.empty())
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("Program empty.")}};

	for (auto& ins : instructions)
	{
		if (ins)
			ins->setControllerId(sceneRoot.toStdString());
	}

	std::vector<RobotInstruction::PlanResult> plans;
	QString err;
	if (!buildPlanResults(sceneRoot, m_instanceIndex, instructions, plans, &err))
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};

	const double rate = body.value(QStringLiteral("playbackRate")).toDouble(1.0);
	m_executor.setPlaybackRate(rate);

	QVector<double> initialAngles(hrc->robotRevoluteJointNames().size(), 0.0);
	QVector<double> localStart;
	if (m_host.robotLocalJointAnglesForSceneRoot(sceneRoot, localStart))
	{
		int off = 0;
		for (int i = 0; i < m_instanceIndex; ++i)
			off += hrc->robotRevoluteJointCountForInstance(i);
		for (int j = 0; j < localStart.size() && off + j < initialAngles.size(); ++j)
			initialAngles[off + j] = localStart[j];
	}

	IRobotBackendPoseSink* sink = hrc->urdfImportScenePoseSink();
	QString startErr;
	if (!m_executor.tryStart(hrc, sink, &m_host.ioSignalNetwork(), m_instanceIndex, instructions, plans,
							 initialAngles, &startErr))
	{
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), startErr}};
	}

	m_motions = RobotInstruction::collectMotionInstructions(instructions);
	{
		const int nj = hrc->robotRevoluteJointCountForInstance(m_instanceIndex);
		m_rollingSeedQ = QVector<double>(nj, 0.0);
		for (int j = 0; j < localStart.size() && j < nj; ++j)
			m_rollingSeedQ[j] = localStart[j];
		if (m_seedPolicy == SeedPolicy::FromInstruction)
		{
			for (const RobotInstruction::PlanResult& pr : plans)
			{
				if (!pr.ok || static_cast<int>(pr.jointTargetsRad.size()) != nj)
					continue;
				for (int j = 0; j < nj; ++j)
					m_rollingSeedQ[j] = pr.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
	}

	m_timer.start();
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("status"), QStringLiteral("running"));
	o.insert(QStringLiteral("seedPolicy"),
			 m_seedPolicy == SeedPolicy::FromCurrentPose ? QStringLiteral("FromCurrentPose")
														 : QStringLiteral("FromInstruction"));
	return o;
}

void HeadlessRobotPlaybackBridge::onTimerTick()
{
	const QJsonObject frame = tickOnce();
	if (m_pushEvent)
	{
		QJsonObject ev = frame;
		ev.insert(QStringLiteral("type"), QStringLiteral("PlaybackFrame"));
		m_pushEvent(ev);
	}
	if (!frame.value(QStringLiteral("running")).toBool())
		m_timer.stop();
}

// 播放中补算 lazyPending，失败立刻写 failed，避免执行器空等后走重规划绕过折圈
void HeadlessRobotPlaybackBridge::fillLazyPlans()
{
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc || m_motions.empty() || m_instanceIndex < 0)
		return;

	size_t currentMi = 0;
	if (const RobotInstruction::Base* active = m_executor.activeMotion())
	{
		for (size_t i = 0; i < m_motions.size(); ++i)
		{
			if (m_motions[i] == active)
			{
				currentMi = i;
				break;
			}
		}
	}

	const QString urdfPath = hrc->robotUrdfAbsolutePathForInstance(m_instanceIndex);
	const std::string defaultTcp = defaultTcpLinkForUrdf(urdfPath).toStdString();
	const int nj = hrc->robotRevoluteJointCountForInstance(m_instanceIndex);
	const size_t last = m_motions.size() - 1;
	const size_t needThrough = std::min(last, currentMi + 2);
	for (size_t mi = currentMi; mi <= needThrough; ++mi)
	{
		const RobotInstruction::Base* motion = m_motions[mi];
		if (!motion)
			continue;
		const RobotInstruction::PlanResult* existing = m_executor.motionPlanResult(motion);
		if (!existing || existing->plannerName != "lazyPending")
			continue;

		QVector<double> seed = m_rollingSeedQ;
		if (m_seedPolicy == SeedPolicy::FromCurrentPose)
		{
			(void)readLiveJointSeed(m_sceneRootId, m_instanceIndex, seed);
		}
		else if (mi > 0)
		{
			const RobotInstruction::PlanResult* prev = m_executor.motionPlanResult(m_motions[mi - 1]);
			if (prev && prev->ok && static_cast<int>(prev->jointTargetsRad.size()) == nj)
			{
				seed.resize(nj);
				for (int j = 0; j < nj; ++j)
					seed[j] = prev->jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		if (seed.size() != nj)
			continue;

		RobotInstruction::PlanResult pr;
		QString planErr;
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motion);
		const bool ok = planRobotInstruction(*hrc, *ins, seed, m_instanceIndex, urdfPath, defaultTcp, m_sceneRootId,
											 pr, &planErr);
		if (!ok || !pr.ok)
		{
			RobotInstruction::PlanResult failed{};
			failed.ok = false;
			failed.plannerName = "failed";
			failed.summary = pr.summary.empty() ? planErr.toStdString() : pr.summary;
			if (failed.summary.empty())
				failed.summary = "Instruction planning failed";
			(void)m_executor.updateMotionPlanResult(motion, failed);
			break;
		}
		(void)m_executor.updateMotionPlanResult(motion, pr);
		if (m_seedPolicy == SeedPolicy::FromInstruction && static_cast<int>(pr.jointTargetsRad.size()) == nj)
		{
			m_rollingSeedQ.resize(nj);
			for (int j = 0; j < nj; ++j)
				m_rollingSeedQ[j] = pr.jointTargetsRad[static_cast<size_t>(j)];
		}
	}
}

QJsonObject HeadlessRobotPlaybackBridge::tickOnce()
{
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	QJsonObject o;
	if (!m_executor.isRunning() || !hrc)
	{
		o.insert(QStringLiteral("ok"), true);
		o.insert(QStringLiteral("running"), false);
		return o;
	}
	fillLazyPlans();
	const auto result = m_executor.tick(hrc, hrc->urdfImportScenePoseSink());
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("running"), m_executor.isRunning());
	o.insert(QStringLiteral("tickResult"), static_cast<int>(result));
	o.insert(QStringLiteral("jointAnglesRad"), jointsToJson(m_executor.jointAnglesRad()));
	if (const auto* ins = m_executor.currentInstruction())
		o.insert(QStringLiteral("instructionId"), QString::fromStdString(ins->id()));
	o.insert(QStringLiteral("progress"), m_executor.motionSegmentProgress01());
	if (!m_executor.isRunning())
		o.insert(QStringLiteral("abortSummary"), QString::fromStdString(m_executor.lastAbortSummary()));
	m_host.flushVisualSync();
	QMetaObject::invokeMethod(&m_host, "visualSceneDirty", Qt::QueuedConnection);
	return o;
}

QJsonObject HeadlessRobotPlaybackBridge::stop()
{
	m_timer.stop();
	m_executor.stop();
	m_motions.clear();
	m_rollingSeedQ.clear();
	m_seedPolicy = SeedPolicy::FromInstruction;
	return {{QStringLiteral("ok"), true}, {QStringLiteral("status"), QStringLiteral("stopped")}};
}

QJsonObject HeadlessRobotPlaybackBridge::statusJson() const
{
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("running"), m_executor.isRunning());
	o.insert(QStringLiteral("sceneRootBackendId"), m_sceneRootId);
	o.insert(QStringLiteral("jointAnglesRad"), jointsToJson(m_executor.jointAnglesRad()));
	o.insert(QStringLiteral("seedPolicy"),
			 m_seedPolicy == SeedPolicy::FromCurrentPose ? QStringLiteral("FromCurrentPose")
														 : QStringLiteral("FromInstruction"));
	if (!m_executor.isRunning() && !m_executor.lastAbortSummary().empty())
		o.insert(QStringLiteral("abortSummary"), QString::fromStdString(m_executor.lastAbortSummary()));
	return o;
}

} // namespace cloudsim::host
