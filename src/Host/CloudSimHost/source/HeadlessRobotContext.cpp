/// @file HeadlessRobotContext.cpp
/// @brief Web Headless 机器人导入/FK 上下文（无 OSG）

#include "HeadlessRobotContext.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "DocumentHost.h"
#include "IDataService.h"
#include "MeshBackendData.h"
#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotSceneKinematics.h"
#include "RobotTeachIk.h"
#include "UrdfRobotLoader.h"

#include <Adapters.h>
#include <RigidTransform.h>

#include <algorithm>
#include <cmath>
#include <osg/Matrixd>

namespace cloudsim::host
{
BackendDataPoseSink::BackendDataPoseSink(BackendDataManager& backend) : m_backend(backend) {}

bool BackendDataPoseSink::getBackendRootWorldMatrix(const std::string& backendId,
													cloudsim::core::Mat4& outWorld) const
{
	const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(m_backend.getData(backendId));
	if (!mesh)
	{
		return false;
	}
	const osg::Matrixd osgWorld = RobotMatrixOsg::matrixFromBackendColMajor(mesh->worldMatrix(&m_backend));
	outWorld = RobotSceneKinematics::coreMat4FromOsgMatrix(osgWorld);
	return true;
}

void BackendDataPoseSink::setBackendRootWorldMatrixFromWorld(const std::string& backendId,
															 const cloudsim::core::Mat4& worldColumnMajor)
{
	const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(m_backend.getData(backendId));
	if (!mesh)
	{
		return;
	}
	const osg::Matrixd osgWorld = RobotSceneKinematics::osgMatrixFromCoreMat4(worldColumnMajor);
	mesh->setWorldMatrix(RobotMatrixOsg::backendColMajorFromMatrix(osgWorld), &m_backend);
}

HeadlessRobotContext::HeadlessRobotContext(DocumentHost& host)
	: m_host(host), m_poseSink(std::make_unique<BackendDataPoseSink>(host.backend()))
{
}

HeadlessRobotContext::~HeadlessRobotContext() = default;

void HeadlessRobotContext::clearRobotSimulationContext()
{
	m_robots.clear();
	rebuildAggregates();
}

void HeadlessRobotContext::clearRobotSimulationIfContains(const QString& removedBackendId)
{
	if (removedBackendId.isEmpty())
	{
		return;
	}
	for (int i = 0; i < m_robots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_robots[i];
		if (ri.sceneBackendId == removedBackendId)
		{
			m_robots.removeAt(i);
			rebuildAggregates();
			return;
		}
		if (!ri.perLinkBackends)
		{
			continue;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == removedBackendId)
			{
				m_robots.removeAt(i);
				rebuildAggregates();
				return;
			}
		}
	}
}

QVector<HeadlessRobotContext::InstanceInfo> HeadlessRobotContext::listInstances() const
{
	QVector<InstanceInfo> out;
	out.reserve(m_robots.size());
	for (const HierarchicalRobotInstance& ri : m_robots)
	{
		InstanceInfo info;
		info.sceneRootBackendId = ri.sceneBackendId;
		info.urdfPath = ri.urdfAbsolutePath;
		info.jointCount = ri.revoluteJointNamesUnprefixed.size();
		const QString name = m_host.data().displayName(ri.sceneBackendId);
		info.label = name.isEmpty() ? ri.sceneBackendId : name;
		out.append(info);
	}
	return out;
}

bool HeadlessRobotContext::jointMetaForSceneRoot(const QString& sceneRootBackendId, QStringList& outNames,
												 QVector<double>& outLower, QVector<double>& outUpper,
												 QVector<double>& outAngles) const
{
	outNames.clear();
	outLower.clear();
	outUpper.clear();
	outAngles.clear();
	const int idx = robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (idx < 0)
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_robots[idx];
	outNames = ri.revoluteJointNamesUnprefixed;
	outLower = ri.jointLowerRad;
	outUpper = ri.jointUpperRad;
	outAngles = ri.lastLocalJointAnglesRad;
	if (outAngles.size() != outNames.size())
	{
		outAngles = QVector<double>(outNames.size(), 0.0);
	}
	return true;
}

void HeadlessRobotContext::recordJointAnglesForSceneRoot(const QString& sceneRootBackendId,
														 const QVector<double>& localAnglesRad)
{
	const int idx = robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (idx < 0)
	{
		return;
	}
	m_robots[idx].lastLocalJointAnglesRad = localAnglesRad;
}

int HeadlessRobotContext::robotInstanceIndexForBackendId(const QString& backendId, bool* outIsSceneRoot) const
{
	if (outIsSceneRoot)
	{
		*outIsSceneRoot = false;
	}
	if (backendId.isEmpty())
	{
		return -1;
	}
	for (int i = 0; i < m_robots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_robots[i];
		if (ri.sceneBackendId == backendId)
		{
			if (outIsSceneRoot)
			{
				*outIsSceneRoot = true;
			}
			return i;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == backendId)
			{
				return i;
			}
		}
	}
	return -1;
}

QString HeadlessRobotContext::robotGizmoAnchorBackendId(const QString& backendId) const
{
	bool isSceneRoot = false;
	const int idx = robotInstanceIndexForBackendId(backendId, &isSceneRoot);
	if (idx < 0 || !m_robots[idx].perLinkBackends)
	{
		return backendId;
	}
	const HierarchicalRobotInstance& ri = m_robots[idx];
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		if (m_host.backendParentId().value(it.value()) == ri.sceneBackendId)
		{
			return it.value();
		}
	}
	return ri.linkNameToBackendId.isEmpty() ? ri.sceneBackendId : ri.linkNameToBackendId.constBegin().value();
}

QString HeadlessRobotContext::robotFlangeBackendId(const QString& backendId) const
{
	const int idx = robotInstanceIndexForBackendId(backendId);
	if (idx < 0)
	{
		return QString();
	}
	const HierarchicalRobotInstance& ri = m_robots[idx];
	const QString flangeLink = QString::fromStdString(ri.coordinateFrames.flangeLinkName);
	if (!flangeLink.isEmpty())
	{
		const QString bid = ri.linkNameToBackendId.value(flangeLink);
		if (!bid.isEmpty())
		{
			return bid;
		}
	}
	// 无法兰名时取连杆名排序末项（多为末端）
	if (!ri.linkNameToBackendId.isEmpty())
	{
		QStringList names = ri.linkNameToBackendId.keys();
		std::sort(names.begin(), names.end());
		return ri.linkNameToBackendId.value(names.last());
	}
	return robotGizmoAnchorBackendId(backendId);
}

namespace
{
BackendMat4 threeJsColMajor16ToBackendMat4(const QVector<double>& threeJsColMajor16)
{
	BackendMat4 wm = BackendMat4::identity();
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			wm.v[r * 4 + c] = threeJsColMajor16[c * 4 + r];
		}
	}
	return wm;
}

QVector<double> clampJointsToLimits(const QVector<double>& q, const QVector<double>& lo, const QVector<double>& hi)
{
	QVector<double> out = q;
	const int n = out.size();
	for (int i = 0; i < n; ++i)
	{
		double v = out[i];
		if (i < lo.size())
		{
			v = std::max(lo[i], v);
		}
		if (i < hi.size())
		{
			v = std::min(hi[i], v);
		}
		out[i] = v;
	}
	return out;
}

/// 单步关节限幅，避免拖动时跳解
QVector<double> clampJointStep(const QVector<double>& q, const QVector<double>& prev, const double maxStepRad)
{
	if (prev.size() != q.size() || maxStepRad <= 0.0)
	{
		return q;
	}
	QVector<double> out = q;
	for (int i = 0; i < out.size(); ++i)
	{
		const double d = out[i] - prev[i];
		if (std::abs(d) > maxStepRad)
		{
			out[i] = prev[i] + std::copysign(maxStepRad, d);
		}
	}
	return out;
}
} // namespace

bool HeadlessRobotContext::applyFkFromGizmoAnchorThreeJsMatrix(const QString& anchorBackendId,
															   const QVector<double>& threeJsColMajor16,
															   QString* outError)
{
	if (threeJsColMajor16.size() < 16)
	{
		if (outError)
		{
			*outError = QStringLiteral("worldMatrix needs 16 elements");
		}
		return false;
	}
	const BackendMat4 wm = threeJsColMajor16ToBackendMat4(threeJsColMajor16);
	const cloudsim::core::Mat4 mat =
		RobotSceneKinematics::coreMat4FromOsgMatrix(RobotMatrixOsg::matrixFromBackendColMajor(wm));
	return applyFkFromGizmoAnchorWorld(anchorBackendId, mat, outError);
}

bool HeadlessRobotContext::applyIkFromFlangeThreeJsMatrix(const QString& flangeBackendId,
														  const QVector<double>& threeJsColMajor16,
														  QVector<double>* outJointAnglesRad, QString* outError)
{
	if (threeJsColMajor16.size() < 16)
	{
		if (outError)
		{
			*outError = QStringLiteral("worldMatrix needs 16 elements");
		}
		return false;
	}
	const int instanceIndex = robotInstanceIndexForBackendId(flangeBackendId);
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		if (outError)
		{
			*outError = QStringLiteral("not a robot backend");
		}
		return false;
	}
	HierarchicalRobotInstance& ri = m_robots[instanceIndex];
	if (!ri.perLinkBackends || ri.urdfAbsolutePath.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("robot is not per-link");
		}
		return false;
	}
	IRobotBackendPoseSink* sink = m_poseSink.get();
	if (!sink)
	{
		if (outError)
		{
			*outError = QStringLiteral("no pose sink");
		}
		return false;
	}

	QString flangeId = flangeBackendId;
	QString flangeLink = ri.linkNameToBackendId.key(flangeId);
	if (flangeLink.isEmpty())
	{
		flangeId = robotFlangeBackendId(ri.sceneBackendId);
		flangeLink = ri.linkNameToBackendId.key(flangeId);
	}
	if (flangeLink.isEmpty())
	{
		const QString named = QString::fromStdString(ri.coordinateFrames.flangeLinkName);
		if (!named.isEmpty() && ri.linkNameToBackendId.contains(named))
		{
			flangeLink = named;
			flangeId = ri.linkNameToBackendId.value(named);
		}
	}
	if (flangeLink.isEmpty())
	{
		QStringList revoluteChildren;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(ri.urdfAbsolutePath, revoluteChildren, nullptr);
		if (!revoluteChildren.isEmpty() && ri.linkNameToBackendId.contains(revoluteChildren.back()))
		{
			flangeLink = revoluteChildren.back();
			flangeId = ri.linkNameToBackendId.value(flangeLink);
		}
	}
	if (flangeLink.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("flange link not found");
		}
		return false;
	}

	cloudsim::core::RobotPerLinkKinematicsSliceDto dto;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, dto))
	{
		if (outError)
		{
			*outError = QStringLiteral("missing per-link kinematics");
		}
		return false;
	}
	const RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(dto);

	QVector<double> seedQ = ri.lastLocalJointAnglesRad;
	if (seedQ.size() != ri.revoluteJointNamesUnprefixed.size())
	{
		seedQ = QVector<double>(ri.revoluteJointNamesUnprefixed.size(), 0.0);
	}

	const BackendMat4 wm = threeJsColMajor16ToBackendMat4(threeJsColMajor16);
	// 网页拖拽罗盘挂在当前工具 TCP（与 overlays 同空间），不再把输入当法兰 mesh
	const osg::Matrixd tcpSceneDesired = RobotMatrixOsg::matrixFromBackendColMajor(wm);
	const osg::Matrixd P = slice.robotBasePlacementWorld;
	const osg::Matrixd tcpInBaseOsg = tcpSceneDesired * osg::Matrixd::inverse(P);

	BackendMat4 toolMat = BackendMat4::identity();
	QString flangeLinkForIk = flangeLink;
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(ri.coordinateFrames))
	{
		toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
		const QString eff =
			QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(ri.coordinateFrames, *tool));
		if (!eff.isEmpty() && ri.linkNameToBackendId.contains(eff))
		{
			flangeLinkForIk = eff;
			flangeId = ri.linkNameToBackendId.value(eff);
		}
	}

	RobotTeachIk::TeachIkContext ctx;
	ctx.urdfPath = ri.urdfAbsolutePath;
	ctx.ikLinkName = flangeLinkForIk;
	ctx.T_base_target = engine::rigidTransformFromOsg(tcpInBaseOsg);
	ctx.seedJointRad.reserve(static_cast<size_t>(seedQ.size()));
	for (double v : seedQ)
	{
		ctx.seedJointRad.push_back(v);
	}
	ctx.useOrientation = true;
	ctx.T_flange_tool = toolMat;
	ctx.maxIkIterations = 22;

	const RobotTeachIk::TeachIkResult ik = RobotTeachIk::solveTeachIk(ctx);
	if (!ik.ok || ik.jointRad.empty())
	{
		if (outError)
		{
			*outError = ik.error.empty() ? QStringLiteral("IK failed") : QString::fromStdString(ik.error);
		}
		return false;
	}

	QVector<double> qRad;
	qRad.reserve(static_cast<int>(ik.jointRad.size()));
	for (double v : ik.jointRad)
	{
		qRad.push_back(v);
	}
	if (qRad.size() != ri.revoluteJointNamesUnprefixed.size())
	{
		if (outError)
		{
			*outError = QStringLiteral("IK joint count mismatch");
		}
		return false;
	}
	qRad = clampJointsToLimits(qRad, ri.jointLowerRad, ri.jointUpperRad);
	static constexpr double kMaxJointStepRad = 0.12;
	qRad = clampJointStep(qRad, seedQ, kMaxJointStepRad);

	QVector<double> aggregated(robotRevoluteJointNames().size(), 0.0);
	if (!RobotSceneKinematics::applyJointAnglesForInstance(this, sink, instanceIndex, qRad, aggregated))
	{
		if (outError)
		{
			*outError = QStringLiteral("applyJointAnglesForInstance failed");
		}
		return false;
	}
	ri.lastLocalJointAnglesRad = qRad;
	rebuildAggregates();
	notifyRobotKinematicsAppliedToScene();
	if (outJointAnglesRad)
	{
		*outJointAnglesRad = qRad;
	}
	return true;
}

bool HeadlessRobotContext::captureTcpPose(const QString& sceneRootBackendId, TcpPoseCapture& out,
										  QString* outError) const
{
	out = {};
	const int instanceIndex = robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		if (outError)
		{
			*outError = QStringLiteral("unknown sceneRootBackendId");
		}
		return false;
	}
	const HierarchicalRobotInstance& ri = m_robots[instanceIndex];
	if (ri.urdfAbsolutePath.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("empty URDF path");
		}
		return false;
	}
	QVector<double> q = ri.lastLocalJointAnglesRad;
	if (q.size() != ri.revoluteJointNamesUnprefixed.size())
	{
		q = QVector<double>(ri.revoluteJointNamesUnprefixed.size(), 0.0);
	}
	QString flangeLink = QString::fromStdString(ri.coordinateFrames.flangeLinkName);
	if (flangeLink.isEmpty())
	{
		QStringList revoluteChildren;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(ri.urdfAbsolutePath, revoluteChildren, nullptr);
		if (!revoluteChildren.isEmpty())
		{
			flangeLink = revoluteChildren.back();
		}
	}
	if (flangeLink.isEmpty() && !ri.linkNameToBackendId.isEmpty())
	{
		QStringList names = ri.linkNameToBackendId.keys();
		std::sort(names.begin(), names.end());
		flangeLink = names.last();
	}
	if (flangeLink.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("flange link not found");
		}
		return false;
	}

	QHash<QString, osg::Matrixd> linkWorld;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(ri.urdfAbsolutePath, q, linkWorld, &fkErr) ||
		!linkWorld.contains(flangeLink))
	{
		if (outError)
		{
			*outError = fkErr.isEmpty() ? QStringLiteral("FK flange failed") : fkErr;
		}
		return false;
	}

	BackendMat4 toolMat = BackendMat4::identity();
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(ri.coordinateFrames))
	{
		toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
		const QString eff =
			QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(ri.coordinateFrames, *tool));
		if (!eff.isEmpty() && linkWorld.contains(eff))
		{
			flangeLink = eff;
		}
	}
	const BackendMat4 TBaseTargetBm =
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(flangeLink), toolMat);
	const engine::RigidTransform T = RobotCoordinate::rigidTransformFromBackendMat4(TBaseTargetBm);
	T.translationMm(out.positionMm[0], out.positionMm[1], out.positionMm[2]);
	T.eulerDegForDisplay(out.eulerDeg[0], out.eulerDeg[1], out.eulerDeg[2]);
	out.flangeLinkName = flangeLink;
	// 与 overlays 一致：FK 基系 TCP × 基座放置 P（OSG 后乘）
	cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
	cloudsim::core::Mat4 basePlacement = cloudsim::core::PlanContextDto::identityMat4();
	if (robotPerLinkKinematicsForInstance(instanceIndex, pl))
	{
		basePlacement = pl.robotBasePlacementWorld;
	}
	const osg::Matrixd tcpOsg = RobotMatrixOsg::matrixFromBackendColMajor(TBaseTargetBm);
	const osg::Matrixd P = RobotSceneKinematics::osgMatrixFromCoreMat4(basePlacement);
	out.worldMat = RobotMatrixOsg::backendColMajorFromMatrix(tcpOsg * P);
	QStringList parts;
	parts.reserve(q.size());
	for (double v : q)
	{
		parts.push_back(QString::number(v, 'g', 12));
	}
	out.jointRadCsv = parts.join(QLatin1Char(','));
	return true;
}

bool HeadlessRobotContext::applyFkFromGizmoAnchorWorld(const QString& anchorBackendId,
													   const cloudsim::core::Mat4& anchorWorldColumnMajor,
													   QString* outError)
{
	const int instanceIndex = robotInstanceIndexForBackendId(anchorBackendId);
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		if (outError)
		{
			*outError = QStringLiteral("not a robot backend");
		}
		return false;
	}
	HierarchicalRobotInstance& ri = m_robots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		if (outError)
		{
			*outError = QStringLiteral("robot is not per-link");
		}
		return false;
	}
	IRobotBackendPoseSink* sink = m_poseSink.get();
	if (!sink)
	{
		if (outError)
		{
			*outError = QStringLiteral("no pose sink");
		}
		return false;
	}

	QString anchorId = anchorBackendId;
	if (ri.linkNameToBackendId.key(anchorId).isEmpty() && ri.sceneBackendId == anchorId)
	{
		anchorId = robotGizmoAnchorBackendId(anchorId);
	}
	if (ri.linkNameToBackendId.key(anchorId).isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("anchor is not a robot link");
		}
		return false;
	}

	sink->setBackendRootWorldMatrixFromWorld(anchorId.toStdString(), anchorWorldColumnMajor);

	cloudsim::core::RobotPerLinkKinematicsSliceDto dto;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, dto))
	{
		if (outError)
		{
			*outError = QStringLiteral("missing per-link kinematics");
		}
		return false;
	}
	RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(dto);
	QVector<double> q = ri.lastLocalJointAnglesRad;
	if (q.size() != ri.revoluteJointNamesUnprefixed.size())
	{
		q = QVector<double>(ri.revoluteJointNamesUnprefixed.size(), 0.0);
	}

	osg::Matrixd anchorWorld = RobotSceneKinematics::osgMatrixFromCoreMat4(anchorWorldColumnMajor);
	osg::Matrixd placement;
	if (!RobotSceneKinematics::computeBasePlacementFromAnchorLinkWorld(slice, anchorId, q, anchorWorld, placement))
	{
		if (outError)
		{
			*outError = QStringLiteral("computeBasePlacementFromAnchorLinkWorld failed");
		}
		return false;
	}

	cloudsim::core::Mat4 pEff = RobotSceneKinematics::coreMat4FromOsgMatrix(placement);
	cloudsim::core::Mat4 p0{};
	const std::vector<double> qs =
		ri.externalAxisQ.empty() ? std::vector<double>{ri.externalAxisQMm} : ri.externalAxisQ;
	RobotExternal::unbakeBasePlacementExternalAxis(pEff.data(), ri.externalAxes, qs, p0.data());
	ri.basePlacementWorld = p0;
	RobotExternal::composeBasePlacementWithExternalAxis(p0.data(), ri.externalAxes, qs, pEff.data());
	placement = RobotSceneKinematics::osgMatrixFromCoreMat4(pEff);
	slice.robotBasePlacementWorld = placement;

	if (!RobotSceneKinematics::applyPerLinkRobotBasePlacement(sink, m_host.backend(), slice, q, placement))
	{
		if (outError)
		{
			*outError = QStringLiteral("applyPerLinkRobotBasePlacement failed");
		}
		return false;
	}
	rebuildAggregates();
	notifyRobotKinematicsAppliedToScene();
	return true;
}

BackendDataManager& HeadlessRobotContext::urdfImportBackend()
{
	return m_host.backend();
}

IRobotSimulationDocument* HeadlessRobotContext::urdfImportRobotSimulationDocument()
{
	return this;
}

IRobotBackendPoseSink* HeadlessRobotContext::urdfImportScenePoseSink()
{
	return m_poseSink.get();
}

bool HeadlessRobotContext::urdfImportLoadLinkMeshIntoScene(const MeshBackendData& mesh, QString* errorMessage)
{
	// Data 已 register；无 OSG 时跳过场景节点
	(void)mesh;
	(void)errorMessage;
	return true;
}

void HeadlessRobotContext::urdfImportSetBackendParent(const std::string& childBackendId,
													  const std::string& parentBackendId)
{
	m_host.syncSceneBackendParent(childBackendId, parentBackendId);
}

void HeadlessRobotContext::urdfImportClearStagingGeometry()
{
	m_host.clearStagingGeometry();
}

void HeadlessRobotContext::urdfImportFocusCameraOnBackend(const std::string& backendId)
{
	m_host.focusSceneCameraOnBackend(backendId);
}

QMap<QString, QString>& HeadlessRobotContext::urdfImportBackendSourcePath()
{
	return m_host.backendSourcePath();
}

QMap<QString, QString>& HeadlessRobotContext::urdfImportBackendSourceType()
{
	return m_host.backendSourceType();
}

QMap<QString, QString>& HeadlessRobotContext::urdfImportBackendParentId()
{
	return m_host.backendParentId();
}

void HeadlessRobotContext::appendHierarchicalRobotSimulationContext(
	const QString& urdfAbsolutePath, const QStringList& revoluteJointNamesUnprefixed,
	const QVector<double>& jointLowerRad, const QVector<double>& jointUpperRad,
	const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys, const QString& robotSceneBackendId,
	const QString& jointPrefixRootOverride)
{
	(void)jointTransformsPrefixedKeys;
	HierarchicalRobotInstance ri;
	ri.urdfAbsolutePath = urdfAbsolutePath;
	ri.sceneBackendId = robotSceneBackendId;
	ri.jointKeyPrefix = jointPrefixRootOverride.isEmpty() ? (robotSceneBackendId + QStringLiteral("::"))
														  : (jointPrefixRootOverride + QStringLiteral("::"));
	ri.revoluteJointNamesUnprefixed = revoluteJointNamesUnprefixed;
	ri.jointLowerRad = jointLowerRad;
	ri.jointUpperRad = jointUpperRad;
	ri.lastLocalJointAnglesRad = QVector<double>(revoluteJointNamesUnprefixed.size(), 0.0);
	ri.basePlacementWorld = cloudsim::core::PlanContextDto::identityMat4();
	m_robots.append(std::move(ri));
	rebuildAggregates();
}

void HeadlessRobotContext::setRobotPerLinkKinematicsBinding(
	const QString& importKey, const QHash<QString, QString>& linkNameToBackendId,
	const QHash<QString, cloudsim::core::Mat4>& fkMeshWorldT0,
	const QHash<QString, cloudsim::core::Mat4>& outerWorldAtBindByBackendId, bool meshVerticesInLinkFrame)
{
	QString jointPrefix = importKey;
	if (jointPrefix.endsWith(QStringLiteral("_ctx")))
	{
		jointPrefix.chop(4);
	}
	jointPrefix += QStringLiteral("::");

	HierarchicalRobotInstance* target = nullptr;
	for (int i = m_robots.size() - 1; i >= 0; --i)
	{
		if (m_robots[i].jointKeyPrefix == jointPrefix)
		{
			target = &m_robots[i];
			break;
		}
	}
	if (!target && !m_robots.isEmpty())
	{
		target = &m_robots.last();
	}
	if (!target)
	{
		return;
	}
	target->perLinkBackends = true;
	target->perLinkImportKey = importKey;
	target->linkNameToBackendId = linkNameToBackendId;
	target->fkMeshWorldT0 = fkMeshWorldT0;
	target->outerWorldAtBindByBackendId = outerWorldAtBindByBackendId;
	target->meshVerticesInLinkFrame = meshVerticesInLinkFrame;
	rebuildAggregates();
}

int HeadlessRobotContext::robotKinematicInstanceCount() const
{
	return m_robots.size();
}

int HeadlessRobotContext::robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const
{
	for (int i = 0; i < m_robots.size(); ++i)
	{
		if (m_robots[i].sceneBackendId == sceneBackendId)
		{
			return i;
		}
	}
	return -1;
}

RobotCoordinate::RobotCoordinateFrameSet& HeadlessRobotContext::robotCoordinateFramesForInstance(int instanceIndex)
{
	static RobotCoordinate::RobotCoordinateFrameSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		return kEmpty;
	}
	return m_robots[instanceIndex].coordinateFrames;
}

RobotExternal::RobotExternalAxisConfigSet& HeadlessRobotContext::robotExternalAxesForInstance(int instanceIndex)
{
	static RobotExternal::RobotExternalAxisConfigSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		return kEmpty;
	}
	return m_robots[instanceIndex].externalAxes;
}

void HeadlessRobotContext::setRobotBasePlacementWorldForInstance(int instanceIndex,
																 const cloudsim::core::Mat4& placementWorld)
{
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		return;
	}
	m_robots[instanceIndex].basePlacementWorld = placementWorld;
	rebuildAggregates();
}

bool HeadlessRobotContext::hasRobotSimulationContext() const
{
	return !m_robots.isEmpty() || !m_robotUrdfAbsolutePath.isEmpty() || !m_robotLinkNameToBackendId.isEmpty();
}

bool HeadlessRobotContext::hasRobotKinematicsBind() const
{
	for (const HierarchicalRobotInstance& ri : m_robots)
	{
		if (ri.perLinkBackends && !ri.fkMeshWorldT0.isEmpty() && !ri.outerWorldAtBindByBackendId.isEmpty())
		{
			return true;
		}
	}
	return !m_robotFkMeshWorldT0.isEmpty() && !m_robotOuterWorldAtBind.isEmpty();
}

const QString& HeadlessRobotContext::robotUrdfAbsolutePath() const
{
	return m_robotUrdfAbsolutePath;
}

const QStringList& HeadlessRobotContext::robotRevoluteJointNames() const
{
	return m_robotRevoluteJointNames;
}

const QHash<QString, QString>& HeadlessRobotContext::robotLinkNameToBackendId() const
{
	return m_robotLinkNameToBackendId;
}

QString HeadlessRobotContext::robotUrdfAbsolutePathForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_robots.size() ? m_robots[instanceIndex].urdfAbsolutePath : QString();
}

int HeadlessRobotContext::robotRevoluteJointCountForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_robots.size()
			   ? m_robots[instanceIndex].revoluteJointNamesUnprefixed.size()
			   : 0;
}

QString HeadlessRobotContext::robotJointKeyPrefixForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_robots.size() ? m_robots[instanceIndex].jointKeyPrefix : QString();
}

bool HeadlessRobotContext::robotUsesPerLinkBackendsForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_robots.size() && m_robots[instanceIndex].perLinkBackends;
}

bool HeadlessRobotContext::robotPerLinkKinematicsForInstance(int instanceIndex,
															 cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const
{
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_robots[instanceIndex];
	if (!ri.perLinkBackends || ri.linkNameToBackendId.isEmpty())
	{
		return false;
	}
	out.urdfAbsolutePath = ri.urdfAbsolutePath;
	out.sceneRootBackendId = ri.sceneBackendId;
	out.linkNameToBackendId = ri.linkNameToBackendId;
	out.fkMeshWorldT0 = ri.fkMeshWorldT0;
	out.outerWorldAtBindByBackendId = ri.outerWorldAtBindByBackendId;
	RobotExternal::composeBasePlacementWithExternalAxis(ri.basePlacementWorld.data(), ri.externalAxes,
														ri.externalAxisQ.empty()
															? std::vector<double>{ri.externalAxisQMm}
															: ri.externalAxisQ,
														out.robotBasePlacementWorld.data());
	out.meshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
	return true;
}

QHash<QString, cloudsim::core::Mat4> HeadlessRobotContext::robotFkMeshWorldT0() const
{
	return m_robotFkMeshWorldT0;
}

QHash<QString, cloudsim::core::Mat4> HeadlessRobotContext::robotOuterWorldAtBind() const
{
	return m_robotOuterWorldAtBind;
}

bool HeadlessRobotContext::robotUrdfMeshVerticesInLinkFrame() const
{
	return m_robotUrdfMeshVerticesInLinkFrame;
}

BackendDataManager* HeadlessRobotContext::robotBackendManagerForKinematics()
{
	return &m_host.backend();
}

void HeadlessRobotContext::notifyRobotKinematicsAppliedToScene()
{
	if (m_host.suppressRobotFollowDirtyNotify())
	{
		return;
	}
	for (const HierarchicalRobotInstance& ri : m_robots)
	{
		if (!ri.sceneBackendId.isEmpty())
		{
			m_host.markFollowAttachmentDirtyFromBackendMove(ri.sceneBackendId.toStdString());
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (!it.value().isEmpty())
			{
				m_host.markFollowAttachmentDirtyFromBackendMove(it.value().toStdString());
			}
		}
	}
	// Headless：无 OSG 时 Follow 写 BackendData，网页 syncObjectTransforms 拉 worldMatrix
	cloudsim::core::FollowSolveContextDto ctx;
	(void)m_host.data().runFollowSolveAndSync(ctx, nullptr);
}

void HeadlessRobotContext::rebuildAggregates()
{
	m_robotRevoluteJointNames.clear();
	m_robotLinkNameToBackendId.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotUrdfMeshVerticesInLinkFrame = false;
	m_robotUrdfAbsolutePath = m_robots.isEmpty() ? QString() : m_robots.first().urdfAbsolutePath;

	for (const HierarchicalRobotInstance& ri : m_robots)
	{
		for (const QString& jn : ri.revoluteJointNamesUnprefixed)
		{
			m_robotRevoluteJointNames.append(ri.jointKeyPrefix + jn);
		}
	}

	int perLinkCount = 0;
	for (const HierarchicalRobotInstance& ri : m_robots)
	{
		if (!ri.perLinkBackends)
		{
			continue;
		}
		++perLinkCount;
		if (perLinkCount == 1)
		{
			m_robotLinkNameToBackendId = ri.linkNameToBackendId;
			m_robotFkMeshWorldT0 = ri.fkMeshWorldT0;
			m_robotOuterWorldAtBind = ri.outerWorldAtBindByBackendId;
			m_robotUrdfMeshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
		}
	}
}

} // namespace cloudsim::host
