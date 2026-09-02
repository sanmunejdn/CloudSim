/// @file HeadlessRobotCollisionBridge.cpp
/// @brief Headless 碰撞设置与起终点关节插值规划

#include "headless/HeadlessRobotCollisionBridge.h"

#include "DocumentHost.h"
#include "HeadlessRobotContext.h"
#include "RobotCollisionSettings.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionController.h"
#include "RobotInstructionProgram.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPlanInstruction.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"
#include "UrdfRobotLoader.h"

#include <BackendFollowMath.h>
#include <RigidTransform.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QVector>

#include <json.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace cloudsim::host
{
namespace
{
QJsonObject settingsToQJson(const RobotCollision::Settings& s)
{
	nlohmann::json j;
	RobotCollision::writeSettingsToJson(s, j);
	const QByteArray raw = QByteArray::fromStdString(j.dump());
	const QJsonDocument jd = QJsonDocument::fromJson(raw);
	return jd.isObject() ? jd.object() : QJsonObject{};
}

bool settingsFromQJson(const QJsonObject& o, RobotCollision::Settings& out)
{
	if (o.isEmpty())
		return false;
	const QByteArray raw = QJsonDocument(o).toJson(QJsonDocument::Compact);
	try
	{
		const nlohmann::json j = nlohmann::json::parse(raw.constData(), raw.constData() + raw.size());
		return RobotCollision::readSettingsFromJson(j, out);
	}
	catch (...)
	{
		return false;
	}
}

bool bodyHasSettingsFields(const QJsonObject& body)
{
	return body.contains(QStringLiteral("enabled")) || body.contains(QStringLiteral("securityMarginMm")) ||
		   body.contains(QStringLiteral("planningSpace")) || body.contains(QStringLiteral("plannerId"))
		   || body.contains(QStringLiteral("planningTimeSec")) ||
		   body.contains(QStringLiteral("whiteListBackendIds")) || body.contains(QStringLiteral("blackListBackendIds"));
}

/// plan 体可能只带部分设置字段，勿整表覆盖成默认值
void overlaySettingsFromBody(const QJsonObject& body, RobotCollision::Settings& s)
{
	if (body.contains(QStringLiteral("enabled")))
		s.enabled = body.value(QStringLiteral("enabled")).toBool();
	if (body.contains(QStringLiteral("securityMarginMm")))
	{
		s.securityMarginMm = body.value(QStringLiteral("securityMarginMm")).toDouble(s.securityMarginMm);
		if (s.securityMarginMm < 0.0)
			s.securityMarginMm = 0.0;
	}
	if (body.contains(QStringLiteral("planningSpace")))
	{
		const std::string ps = body.value(QStringLiteral("planningSpace")).toString().toStdString();
		if (ps == "Auto" || ps == "Joint" || ps == "Cartesian")
			s.planningSpace = ps;
	}
	if (body.contains(QStringLiteral("plannerId")))
	{
		const std::string pid = body.value(QStringLiteral("plannerId")).toString().toStdString();
		if (pid == "Auto" || pid == "BITstar" || pid == "InformedRRTstar" || pid == "RRTstar" || pid == "RRTConnect"
			|| pid == "Dijkstra")
			s.plannerId = pid;
	}
	if (body.contains(QStringLiteral("planningTimeSec")))
	{
		const double t = body.value(QStringLiteral("planningTimeSec")).toDouble(s.planningTimeSec);
		s.planningTimeSec = std::max(1.0, std::min(120.0, t));
	}
	auto readIds = [](const QJsonValue& v, std::vector<std::string>& out) {
		out.clear();
		if (!v.isArray())
			return;
		for (const QJsonValue& el : v.toArray())
		{
			const QString id = el.toString();
			if (!id.isEmpty())
				out.push_back(id.toStdString());
		}
	};
	if (body.contains(QStringLiteral("whiteListBackendIds")))
		readIds(body.value(QStringLiteral("whiteListBackendIds")), s.whiteListBackendIds);
	if (body.contains(QStringLiteral("blackListBackendIds")))
		readIds(body.value(QStringLiteral("blackListBackendIds")), s.blackListBackendIds);
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

std::shared_ptr<RobotInstruction::Base> findInstructionById(
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps, const std::string& id)
{
	for (const auto& ins : steps)
	{
		if (!ins)
			continue;
		if (ins->id() == id)
			return ins;
		if (auto nested = findInstructionById(ins->nestedSteps(), id))
			return nested;
		if (auto nested = findInstructionById(ins->elseSteps(), id))
			return nested;
	}
	return nullptr;
}

bool tcpPoseFromJoints(const QString& urdfPath, const QVector<double>& q, const QString& flangeLink,
					   const BackendMat4& T_flange_tool, RobotInstruction::TrajectoryPoint& out)
{
	QHash<QString, osg::Matrixd> linkWorld;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, q, linkWorld, &fkErr) || !linkWorld.contains(flangeLink))
		return false;
	const BackendMat4 TBaseTargetBm =
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(flangeLink), T_flange_tool);
	const engine::RigidTransform T = RobotCoordinate::rigidTransformFromBackendMat4(TBaseTargetBm);
	T.translationMm(out.poseMm.x, out.poseMm.y, out.poseMm.z);
	T.eulerDegForDisplay(out.eulerDeg.x, out.eulerDeg.y, out.eulerDeg.z);
	const Eigen::Quaterniond quat = T.rotation();
	out.quatXyzw[0] = quat.x();
	out.quatXyzw[1] = quat.y();
	out.quatXyzw[2] = quat.z();
	out.quatXyzw[3] = quat.w();
	out.hasQuat = true;
	out.reachable = true;
	out.speedMmPerSec = 200.0;
	out.jointRad.assign(q.begin(), q.end());
	return true;
}

RobotInstruction::RawTrajectory densifyJointLerpToRaw(const QString& urdfPath, const QVector<double>& startQ,
													  const QVector<double>& endQ, const QString& flangeLink,
													  const BackendMat4& T_flange_tool, QString* err)
{
	RobotInstruction::RawTrajectory raw;
	if (startQ.size() != endQ.size() || startQ.isEmpty())
	{
		if (err)
			*err = QStringLiteral("start/end joint dimension mismatch");
		return raw;
	}
	double maxAbs = 0.0;
	for (int i = 0; i < startQ.size(); ++i)
		maxAbs = std::max(maxAbs, std::abs(endQ[i] - startQ[i]));
	constexpr double kStepRad = 0.05;
	const int nSeg = std::max(1, static_cast<int>(std::ceil(maxAbs / kStepRad)));
	const int nPts = nSeg + 1;
	raw.points.reserve(static_cast<std::size_t>(nPts));
	for (int s = 0; s < nPts; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(nSeg);
		QVector<double> q(startQ.size());
		for (int i = 0; i < startQ.size(); ++i)
			q[i] = startQ[i] + t * (endQ[i] - startQ[i]);
		RobotInstruction::TrajectoryPoint pt;
		if (!tcpPoseFromJoints(urdfPath, q, flangeLink, T_flange_tool, pt))
		{
			if (err)
				*err = QStringLiteral("FK failed while densifying joint path");
			return RobotInstruction::RawTrajectory{};
		}
		raw.points.push_back(std::move(pt));
	}
	return raw;
}
} // namespace

HeadlessRobotCollisionBridge::HeadlessRobotCollisionBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessRobotCollisionBridge::getSettings(const QJsonObject& body)
{
	(void)body;
	QJsonObject o = settingsToQJson(m_host.robotCollisionSettings());
	o.insert(QStringLiteral("ok"), true);
	return o;
}

QJsonObject HeadlessRobotCollisionBridge::putSettings(const QJsonObject& body)
{
	RobotCollision::Settings s;
	if (!settingsFromQJson(body, s))
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("Invalid collision settings.")}};
	m_host.robotCollisionSettings() = s;
	return {{QStringLiteral("ok"), true}};
}

QJsonObject HeadlessRobotCollisionBridge::plan(const QJsonObject& body)
{
	m_hasCachedPlan = false;
	m_lastPlanRaw = RobotInstruction::RawTrajectory{};
	m_planSceneRootId.clear();
	m_planStartInstructionId.clear();
	m_planEndInstructionId.clear();

	if (bodyHasSettingsFields(body))
	{
		RobotCollision::Settings s = m_host.robotCollisionSettings();
		overlaySettingsFromBody(body, s);
		m_host.robotCollisionSettings() = s;
	}

	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc)
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("No robot context.")}};

	const QString startId = body.value(QStringLiteral("startInstructionId")).toString();
	const QString endId = body.value(QStringLiteral("endInstructionId")).toString();
	if (startId.isEmpty() || endId.isEmpty() || startId == endId)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("startInstructionId/endInstructionId required.")}};
	}

	QString sceneRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	if (sceneRoot.isEmpty())
	{
		const auto inst = hrc->listInstances();
		if (!inst.isEmpty())
			sceneRoot = inst.front().sceneRootBackendId;
	}
	const int instIdx = hrc->robotInstanceIndexForBackendId(sceneRoot);
	if (instIdx < 0)
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("Robot instance not found.")}};

	m_host.robotProgramStore().ensureRobotBackendId(sceneRoot);
	RobotInstruction::RobotProgramCatalog& catalog = m_host.robotProgramStore().catalogFor(sceneRoot);
	RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	std::vector<std::shared_ptr<RobotInstruction::Base>> steps;
	if (prog)
		steps = prog->steps;
	if (steps.empty())
		steps = m_host.robotProgramStore().programFor(sceneRoot);
	if (steps.empty())
		return {{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("Program empty.")}};

	auto startIns = findInstructionById(steps, startId.toStdString());
	auto endIns = findInstructionById(steps, endId.toStdString());
	if (!startIns || !endIns)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("start/end instruction not found.")}};
	}

	const QString urdfPath = hrc->robotUrdfAbsolutePathForInstance(instIdx);
	const std::string defaultTcp = defaultTcpLinkForUrdf(urdfPath).toStdString();
	const int nj = hrc->robotRevoluteJointCountForInstance(instIdx);
	QVector<double> seedQ(nj, 0.0);
	QVector<double> localQ;
	if (m_host.robotLocalJointAnglesForSceneRoot(sceneRoot, localQ) && nj > 0)
	{
		for (int j = 0; j < nj && j < localQ.size(); ++j)
			seedQ[j] = localQ[j];
	}

	RobotInstruction::PlanResult startPlan{};
	QString startErr;
	if (!planRobotInstruction(*hrc, *startIns, seedQ, instIdx, urdfPath, defaultTcp, sceneRoot, startPlan, &startErr) ||
		!startPlan.ok || static_cast<int>(startPlan.jointTargetsRad.size()) != nj)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"),
				 startErr.isEmpty() ? QStringLiteral("Failed to plan start instruction.") : startErr}};
	}
	QVector<double> startQ;
	startQ.reserve(nj);
	for (double v : startPlan.jointTargetsRad)
		startQ.push_back(v);

	RobotInstruction::PlanResult endPlan{};
	QString endErr;
	if (!planRobotInstruction(*hrc, *endIns, startQ, instIdx, urdfPath, defaultTcp, sceneRoot, endPlan, &endErr) ||
		!endPlan.ok || static_cast<int>(endPlan.jointTargetsRad.size()) != nj)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"),
				 endErr.isEmpty() ? QStringLiteral("Failed to plan end instruction.") : endErr}};
	}
	QVector<double> endQ;
	endQ.reserve(nj);
	for (double v : endPlan.jointTargetsRad)
		endQ.push_back(v);

	QString flangeLink = QString::fromStdString(defaultTcp);
	BackendMat4 T_flange_tool = BackendMat4::identity();
	{
		RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(instIdx);
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
		{
			T_flange_tool = RobotCoordinate::frameToMat4(tool->T_flange_tool);
			const QString eff =
				QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *tool));
			if (!eff.isEmpty())
				flangeLink = eff;
		}
		else if (!frames.flangeLinkName.empty())
		{
			flangeLink = QString::fromStdString(frames.flangeLinkName);
		}
	}

	QString densifyErr;
	RobotInstruction::RawTrajectory raw =
		densifyJointLerpToRaw(urdfPath, startQ, endQ, flangeLink, T_flange_tool, &densifyErr);
	if (raw.points.empty())
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"),
				 densifyErr.isEmpty() ? QStringLiteral("Joint lerp densify failed.") : densifyErr}};
	}

	m_lastPlanRaw = std::move(raw);
	m_planSceneRootId = sceneRoot;
	m_planStartInstructionId = startId;
	m_planEndInstructionId = endId;
	m_hasCachedPlan = true;

	return {{QStringLiteral("ok"), true},
			{QStringLiteral("plannerName"), QStringLiteral("JointLerp")},
			{QStringLiteral("pointCount"), static_cast<int>(m_lastPlanRaw.points.size())},
			{QStringLiteral("sceneRootBackendId"), sceneRoot},
			{QStringLiteral("startInstructionId"), startId},
			{QStringLiteral("endInstructionId"), endId}};
}

QJsonObject HeadlessRobotCollisionBridge::confirm(const QJsonObject& body)
{
	if (!m_hasCachedPlan || m_lastPlanRaw.points.empty())
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("No cached plan; call collision/plan first.")}};
	}
	if (m_planSceneRootId.isEmpty() || m_planStartInstructionId.isEmpty() || m_planEndInstructionId.isEmpty())
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("Cached plan missing scene/start/end ids.")}};
	}

	const QString bodyRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	if (!bodyRoot.isEmpty() && bodyRoot != m_planSceneRootId)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("sceneRootBackendId does not match cached plan.")}};
	}
	const QString bodyStart = body.value(QStringLiteral("startInstructionId")).toString();
	const QString bodyEnd = body.value(QStringLiteral("endInstructionId")).toString();
	if ((!bodyStart.isEmpty() && bodyStart != m_planStartInstructionId) ||
		(!bodyEnd.isEmpty() && bodyEnd != m_planEndInstructionId))
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("start/end instruction does not match cached plan.")}};
	}

	// 必须与 plan 同一 catalog，禁止用 activeCatalog（未登记实例时会落到静态空目录）
	m_host.robotProgramStore().ensureRobotBackendId(m_planSceneRootId);
	RobotInstruction::RobotProgramCatalog& catalog = m_host.robotProgramStore().catalogFor(m_planSceneRootId);
	RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
		prog = catalog.mainProgram();
	if (!prog)
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"), QStringLiteral("No active program.")}};
	}

	// 起终点已在程序中，只插入中间点
	RobotInstruction::RawTrajectory mid = m_lastPlanRaw;
	if (mid.points.size() >= 2)
	{
		mid.points.erase(mid.points.begin());
		mid.points.pop_back();
	}
	else
	{
		mid.points.clear();
	}

	{
		int iStart = -1;
		int iEnd = -1;
		const std::string sid = m_planStartInstructionId.toStdString();
		const std::string eid = m_planEndInstructionId.toStdString();
		for (int i = 0; i < static_cast<int>(prog->steps.size()); ++i)
		{
			const auto& ins = prog->steps[static_cast<std::size_t>(i)];
			if (!ins)
				continue;
			if (ins->id() == sid)
				iStart = i;
			if (ins->id() == eid)
				iEnd = i;
		}
		if (iStart >= 0 && iEnd >= 0 && iStart > iEnd)
			std::reverse(mid.points.begin(), mid.points.end());
	}

	std::string err;
	const int insertedCount = static_cast<int>(mid.points.size());
	if (!RobotInstruction::insertRawTrajectoryBetween(mid, *prog, m_planStartInstructionId.toStdString(),
													  m_planEndInstructionId.toStdString(), &err))
	{
		return {{QStringLiteral("ok"), false},
				{QStringLiteral("error"),
				 QString::fromStdString(err.empty() ? "insertRawTrajectoryBetween failed" : err)}};
	}

	m_hasCachedPlan = false;
	m_lastPlanRaw = RobotInstruction::RawTrajectory{};
	const QString plannedRoot = m_planSceneRootId;
	m_planSceneRootId.clear();
	m_planStartInstructionId.clear();
	m_planEndInstructionId.clear();

	return {{QStringLiteral("ok"), true},
			{QStringLiteral("insertedCount"), insertedCount},
			{QStringLiteral("sceneRootBackendId"), plannedRoot}};
}

} // namespace cloudsim::host
