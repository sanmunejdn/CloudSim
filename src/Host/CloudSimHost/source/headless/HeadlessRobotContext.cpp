/// @file HeadlessRobotContext.cpp
/// @brief Web Headless 机器人导入/FK 上下文（无 OSG）

#include "HeadlessRobotContext.h"
#include "visual/KinematicsBatchScope.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "DocumentHost.h"
#include "io/CustomDeviceRobotMountOps.h"
#include "IDataService.h"
#include "MeshBackendData.h"
#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotSceneKinematics.h"
#include "RobotTeachIk.h"
#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include <Adapters.h>
#include <RigidTransform.h>

#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>

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
	const osg::Matrixd osgWorld = RobotMatrixOsg::matrixFromBackendColMajor(mesh->worldMatrix());
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
	mesh->setWorldMatrix(RobotMatrixOsg::backendColMajorFromMatrix(osgWorld));
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

namespace
{
/// 与设备库瓦片一致：优先 …/型号/urdf/*.urdf 的包名，否则 URDF 主文件名
QString robotModelLabelFromUrdfPath(const QString& urdfAbsolutePath)
{
	if (urdfAbsolutePath.isEmpty())
	{
		return {};
	}
	const QFileInfo fi(urdfAbsolutePath);
	const QDir urdfDir = fi.dir();
	if (urdfDir.dirName().compare(QStringLiteral("urdf"), Qt::CaseInsensitive) == 0)
	{
		QDir pkg = urdfDir;
		if (pkg.cdUp())
		{
			const QString pkgName = pkg.dirName();
			if (!pkgName.isEmpty() && pkgName != QStringLiteral(".") && pkgName != QStringLiteral(".."))
			{
				return pkgName;
			}
		}
	}
	return fi.completeBaseName();
}
} // namespace

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
		QString model = robotModelLabelFromUrdfPath(ri.urdfAbsolutePath);
		if (model.isEmpty() && ri.sceneBackendId.startsWith(QStringLiteral("RobotURDF_")))
		{
			model = ri.sceneBackendId.mid(QStringLiteral("RobotURDF_").size());
		}
		const QString name = m_host.data().displayName(ri.sceneBackendId);
		if (!model.isEmpty())
		{
			info.label = model;
		}
		else
		{
			info.label = name.isEmpty() ? ri.sceneBackendId : name;
		}
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
	m_robots[idx].tcpDragChaseValid = false;
	m_robots[idx].tcpDragOriLocked = false;
}

bool HeadlessRobotContext::getExternalAxesJson(const QString& sceneRootBackendId, QJsonObject& outConfigSet,
											   QString* outError) const
{
	outConfigSet = QJsonObject{};
	const int idx = robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (idx < 0)
	{
		if (outError)
			*outError = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	nlohmann::json eaJ;
	RobotExternal::writeExternalAxisConfigSetToJson(m_robots[idx].externalAxes, eaJ);
	const QByteArray raw = QByteArray::fromStdString(eaJ.dump());
	const QJsonDocument doc = QJsonDocument::fromJson(raw);
	if (!doc.isObject())
	{
		if (outError)
			*outError = QStringLiteral("externalAxes serialize failed");
		return false;
	}
	outConfigSet = doc.object();
	return true;
}

bool HeadlessRobotContext::setExternalAxesJson(const QString& sceneRootBackendId, const QJsonObject& axesOrConfigSet,
											   QString* outError)
{
	const int idx = robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (idx < 0)
	{
		if (outError)
			*outError = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	// 兼容 PUT `{axes:[]}` / 完整 `{axes:[]}` 配置集 / 嵌套 externalAxes
	QJsonObject configObj = axesOrConfigSet;
	if (configObj.contains(QStringLiteral("externalAxes")) && configObj.value(QStringLiteral("externalAxes")).isObject())
		configObj = configObj.value(QStringLiteral("externalAxes")).toObject();
	if (!configObj.contains(QStringLiteral("axes")) && axesOrConfigSet.contains(QStringLiteral("axes")))
		configObj = QJsonObject{{QStringLiteral("axes"), axesOrConfigSet.value(QStringLiteral("axes"))}};

	const QByteArray raw = QJsonDocument(configObj).toJson(QJsonDocument::Compact);
	try
	{
		const nlohmann::json eaJ = nlohmann::json::parse(raw.constData(), raw.constData() + raw.size());
		RobotExternal::RobotExternalAxisConfigSet axes;
		if (!RobotExternal::readExternalAxisConfigSetFromJson(eaJ, axes))
		{
			if (outError)
				*outError = QStringLiteral("invalid externalAxes JSON");
			return false;
		}
		std::string validateErr;
		if (!RobotExternal::validateExternalAxisConfigSet(axes, &validateErr))
		{
			if (outError)
				*outError = validateErr.empty() ? QStringLiteral("externalAxes validation failed")
												: QString::fromStdString(validateErr);
			return false;
		}
		m_robots[idx].externalAxes = std::move(axes);
		return true;
	}
	catch (...)
	{
		if (outError)
			*outError = QStringLiteral("externalAxes JSON parse failed");
		return false;
	}
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
	if (ri.linkNameToBackendId.isEmpty())
	{
		return ri.sceneBackendId;
	}

	// 与桌面 DocumentPage::robotFrameWorldReferenceBackendId 同策：勿用 QHash::begin 落到法兰
	const auto parentOf = [this](const QString& bid) -> QString
	{
		const QString sidecar = m_host.backendParentId().value(bid);
		if (!sidecar.isEmpty())
		{
			return sidecar;
		}
		const QVector<QString> parents = m_host.data().parentsOf(bid);
		return parents.isEmpty() ? QString() : parents.first();
	};

	QStringList underRootNames;
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		if (parentOf(it.value()) == ri.sceneBackendId)
		{
			underRootNames.append(it.key());
		}
	}

	const QString flangeLink = QString::fromStdString(ri.coordinateFrames.flangeLinkName);
	const auto pickPreferredName = [&](const QStringList& names) -> QString
	{
		static const QString kPreferred[] = {QStringLiteral("base_link"), QStringLiteral("base"),
											QStringLiteral("root")};
		for (const QString& want : kPreferred)
		{
			for (const QString& name : names)
			{
				if (name.compare(want, Qt::CaseInsensitive) == 0)
				{
					return name;
				}
			}
		}
		QStringList sorted = names;
		std::sort(sorted.begin(), sorted.end(),
				  [](const QString& a, const QString& b) { return a.compare(b, Qt::CaseInsensitive) < 0; });
		for (const QString& name : sorted)
		{
			if (!flangeLink.isEmpty() && name.compare(flangeLink, Qt::CaseInsensitive) == 0)
			{
				continue;
			}
			return name;
		}
		return sorted.isEmpty() ? QString() : sorted.first();
	};

	QString urdfRootLink;
	{
		QHash<QString, QString> meshes;
		(void)UrdfRobotLoader::enumerateLinkVisualMeshes(ri.urdfAbsolutePath, urdfRootLink, meshes, nullptr);
	}
	if (!urdfRootLink.isEmpty() && ri.linkNameToBackendId.contains(urdfRootLink))
	{
		return ri.linkNameToBackendId.value(urdfRootLink);
	}

	const QStringList namePool = underRootNames.isEmpty() ? QStringList(ri.linkNameToBackendId.keys()) : underRootNames;
	const QString chosen = pickPreferredName(namePool);
	if (!chosen.isEmpty())
	{
		return ri.linkNameToBackendId.value(chosen);
	}
	return ri.sceneBackendId;
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
	// 锚点多为基座而非末端法兰：挂载方只验"id 在册"，此处必须留痕，否则会静默挂到基座
	RunLogger::warn("[HeadlessRobotContext] robotFlangeBackendId: no flange link configured and link map empty, "
					"falling back to gizmo anchor (likely the BASE, not the end flange) for robot \"" +
					backendId.toStdString() + "\".");
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
														  QVector<double>* outJointAnglesRad, QString* outError,
														  bool* outIncomplete, const bool translateOnly)
{
	if (outIncomplete)
	{
		*outIncomplete = false;
	}
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
	if (!translateOnly)
	{
		ri.tcpDragOriLocked = false;
	}
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
	// 网页罗盘发的是工具 TCP 场景系，与 overlays/tcp-pose 一致
	osg::Matrixd tcpSceneDesired = RobotMatrixOsg::matrixFromBackendColMajor(wm);
	if (translateOnly)
	{
		// 锁当前 FK 姿态：与 seed 一致，避免纠姿大步导致拖不动/关节 lerp 拧 TCP
		if (!ri.tcpDragOriLocked)
		{
			osg::Quat q = tcpSceneDesired.getRotate();
			TcpPoseCapture curPose;
			if (captureTcpPose(ri.sceneBackendId, curPose, nullptr))
			{
				q = RobotMatrixOsg::matrixFromBackendColMajor(curPose.worldMat).getRotate();
			}
			ri.tcpDragOriQuatXyzw[0] = q.x();
			ri.tcpDragOriQuatXyzw[1] = q.y();
			ri.tcpDragOriQuatXyzw[2] = q.z();
			ri.tcpDragOriQuatXyzw[3] = q.w();
			ri.tcpDragOriLocked = true;
		}
		const osg::Vec3d tOnly = tcpSceneDesired.getTrans();
		tcpSceneDesired = osg::Matrixd::rotate(osg::Quat(ri.tcpDragOriQuatXyzw[0], ri.tcpDragOriQuatXyzw[1],
														 ri.tcpDragOriQuatXyzw[2], ri.tcpDragOriQuatXyzw[3]));
		tcpSceneDesired.setTrans(tOnly);
	}

	const osg::Vec3d tDesiredFull = tcpSceneDesired.getTrans();
	osg::Vec3d chaseFrom = tDesiredFull;
	bool haveChaseFrom = false;
	if (ri.tcpDragChaseValid)
	{
		chaseFrom.set(ri.tcpDragChaseTx, ri.tcpDragChaseTy, ri.tcpDragChaseTz);
		haveChaseFrom = true;
	}
	else
	{
		TcpPoseCapture curPose;
		if (captureTcpPose(ri.sceneBackendId, curPose, nullptr))
		{
			chaseFrom = RobotMatrixOsg::matrixFromBackendColMajor(curPose.worldMat).getTrans();
			haveChaseFrom = true;
		}
	}

	// 只截断平移、保留目标姿态；追赶基点用上一拍目标而非 FK
	static constexpr double kTcpDragMaxChaseMmPerIk = 120.0;
	static constexpr double kTcpDragMaxJointStepRad = 0.45;
	bool chaseClipped = false;
	osg::Vec3d tStep = tDesiredFull;
	if (haveChaseFrom)
	{
		const osg::Vec3d delta = tDesiredFull - chaseFrom;
		const double emitLen = delta.length();
		if (emitLen > kTcpDragMaxChaseMmPerIk)
		{
			chaseClipped = true;
			tStep = chaseFrom + delta * (kTcpDragMaxChaseMmPerIk / emitLen);
		}
	}
	tcpSceneDesired.setTrans(tStep);

	const osg::Matrixd P = slice.robotBasePlacementWorld;
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

	auto solveIkAtSceneTrans = [&](const osg::Vec3d& t, QVector<double>* outQ, QString* solveErr) -> bool {
		osg::Matrixd tcpScene = tcpSceneDesired;
		tcpScene.setTrans(t);
		const osg::Matrixd tcpInBaseOsg = tcpScene * osg::Matrixd::inverse(P);
		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = ri.urdfAbsolutePath;
		ctx.ikLinkName = flangeLinkForIk;
		ctx.T_base_target = engine::rigidTransformFromOsg(tcpInBaseOsg);
		ctx.seedJointRad.clear();
		ctx.seedJointRad.reserve(static_cast<size_t>(seedQ.size()));
		for (double v : seedQ)
		{
			ctx.seedJointRad.push_back(v);
		}
		ctx.useOrientation = true;
		ctx.T_flange_tool = toolMat;
		ctx.maxIkIterations = translateOnly ? 40 : 22;
		const RobotTeachIk::TeachIkResult ik = RobotTeachIk::solveTeachIk(ctx);
		if (!ik.ok || ik.jointRad.empty())
		{
			if (solveErr)
			{
				*solveErr = ik.error.empty() ? QStringLiteral("IK failed") : QString::fromStdString(ik.error);
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
			if (solveErr)
			{
				*solveErr = QStringLiteral("IK joint count mismatch");
			}
			return false;
		}
		*outQ = clampJointsToLimits(qRad, ri.jointLowerRad, ri.jointUpperRad);
		return true;
	};

	QVector<double> qRad;
	QString ikErr;
	bool jointStepClipped = false;
	if (translateOnly)
	{
		// 不用关节 lerp（会拧姿）；也不因步长门控冻住——笛卡尔追赶限幅即可
		if (!solveIkAtSceneTrans(tStep, &qRad, &ikErr))
		{
			// 整步失败则缩半再试，仍失败才停本拍（位置不动）
			bool recovered = false;
			for (int attempt = 0; attempt < 6; ++attempt)
			{
				tStep = chaseFrom + (tStep - chaseFrom) * 0.5;
				if ((tStep - chaseFrom).length() < 0.5)
				{
					break;
				}
				chaseClipped = true;
				if (solveIkAtSceneTrans(tStep, &qRad, &ikErr))
				{
					recovered = true;
					break;
				}
			}
			if (!recovered)
			{
				if (outError)
				{
					*outError = ikErr;
				}
				return false;
			}
		}
		tcpSceneDesired.setTrans(tStep);
		if ((tDesiredFull - tStep).length() > 0.5)
		{
			chaseClipped = true;
		}
	}
	else
	{
		if (!solveIkAtSceneTrans(tStep, &qRad, &ikErr))
		{
			if (outError)
			{
				*outError = ikErr;
			}
			return false;
		}
		const QVector<double> qBeforeStep = qRad;
		qRad = clampJointStep(qRad, seedQ, kTcpDragMaxJointStepRad);
		for (int i = 0; i < qRad.size(); ++i)
		{
			if (std::abs(qRad[i] - qBeforeStep[i]) > 1e-9)
			{
				jointStepClipped = true;
				break;
			}
		}
	}

	QVector<double> aggregated(robotRevoluteJointNames().size(), 0.0);
	cloudsim::host::KinematicsBatchScope batch(m_host);
	if (!RobotSceneKinematics::applyJointAnglesForInstance(this, sink, instanceIndex, qRad, aggregated))
	{
		if (outError)
		{
			*outError = QStringLiteral("applyJointAnglesForInstance failed");
		}
		return false;
	}
	ri.lastLocalJointAnglesRad = qRad;
	{
		const osg::Vec3d stepT = tcpSceneDesired.getTrans();
		ri.tcpDragChaseTx = stepT.x();
		ri.tcpDragChaseTy = stepT.y();
		ri.tcpDragChaseTz = stepT.z();
		ri.tcpDragChaseValid = true;
	}
	rebuildAggregates();
	notifyRobotKinematicsAppliedToScene();
	if (outJointAnglesRad)
	{
		*outJointAnglesRad = qRad;
	}
	if (outIncomplete)
	{
		*outIncomplete = chaseClipped || jointStepClipped;
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
	cloudsim::host::refreshCustomDevicesFollowingKinematicsTargets(m_host);
}

void HeadlessRobotContext::noteRobotJointAnglesAppliedForInstance(int instanceIndex, const QVector<double>& localJointRad)
{
	if (instanceIndex < 0 || instanceIndex >= m_robots.size())
	{
		return;
	}
	const QString sceneRoot = m_robots[instanceIndex].sceneBackendId;
	recordJointAnglesForSceneRoot(sceneRoot, localJointRad);
	// 播放桥起点种子读 host 侧缓存（HeadlessRobotPlaybackBridge），双写保持两端一致
	m_host.noteRobotLocalJointAnglesForSceneRoot(sceneRoot, localJointRad);
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
