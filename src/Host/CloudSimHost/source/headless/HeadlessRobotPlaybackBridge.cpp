/// @file HeadlessRobotPlaybackBridge.cpp
/// @brief 服务端程序回放

#include "headless/HeadlessRobotPlaybackBridge.h"

#include "DocumentHost.h"
#include "HeadlessRobotContext.h"
#include "IRobotBackendPoseSink.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotPlanInstruction.h"
#include "RobotProgramStore.h"
#include "UrdfRobotLoader.h"
#include "io/IoSignalNetwork.h"

#include <QJsonArray>
#include <QMetaObject>

#include <json.hpp>

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
	int jointOffset = 0;
	for (int i = 0; i < instIdx; ++i)
		jointOffset += hrc->robotRevoluteJointCountForInstance(i);
	const int nj = hrc->robotRevoluteJointCountForInstance(instIdx);
	QVector<double> rollingQ(nj, 0.0);
	QVector<double> localQ;
	if (m_host.robotLocalJointAnglesForSceneRoot(sceneRootBackendId, localQ) && nj > 0)
	{
		for (int j = 0; j < nj && j < localQ.size(); ++j)
			rollingQ[j] = localQ[j];
	}

	constexpr size_t kEager = 32;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		RobotInstruction::PlanResult pr;
		if (mi >= kEager)
		{
			pr.ok = false;
			pr.plannerName = "lazyPending";
			pr.summary = "lazy pending";
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
		const nlohmann::json mj = RobotInstruction::toJson(*motion);
		std::string parseErr;
		std::shared_ptr<RobotInstruction::Base> ins = RobotInstruction::createFromJson(mj, &parseErr);
		if (!ins)
		{
			if (err)
				*err = QString::fromStdString(parseErr);
			return false;
		}
		if (!planRobotInstruction(*hrc, *ins, rollingQ, instIdx, urdfPath, defaultTcp, sceneRootBackendId, pr, err))
		{
			outPlans.push_back(pr);
			continue;
		}
		outPlans.push_back(pr);
		if (pr.ok && !pr.jointTargetsRad.empty())
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
	if (!programId.isEmpty())
	{
		auto* prog = m_host.robotProgramStore().activeCatalog().findProgram(programId.toStdString());
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

	m_timer.start();
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("status"), QStringLiteral("running"));
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
	return {{QStringLiteral("ok"), true}, {QStringLiteral("status"), QStringLiteral("stopped")}};
}

QJsonObject HeadlessRobotPlaybackBridge::statusJson() const
{
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("running"), m_executor.isRunning());
	o.insert(QStringLiteral("sceneRootBackendId"), m_sceneRootId);
	o.insert(QStringLiteral("jointAnglesRad"), jointsToJson(m_executor.jointAnglesRad()));
	return o;
}

} // namespace cloudsim::host
