/// @file HeadlessTrajectorySession.cpp
/// @brief Web 轨迹会话实现

#include "HeadlessTrajectorySession.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendSpatial.h"
#include "DocumentHost.h"
#include "FrameBackendData.h"
#include "GeometryRef.h"
#include "HeadlessRobotContext.h"
#include "ITrajectoryOp.h"
#include "MeshTrajectory.h"
#include "MeshTrajectoryIngress.h"
#include "RecipeBlueprint.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"
#include "ShapeQuery.h"
#include "TrajectoryOpBridge.h"
#include "TrajectoryPipelineEngine.h"
#include "UnifiedTrajectory.h"

#include <Adapters.h>
#include <Discretize.h>
#include <FeatureDiscretizerBridge.h>
#include <MeshBackendData.h>
#include <RigidTransform.h>

#include <Eigen/Core>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QStandardPaths>

#include <cmath>

namespace cloudsim::host
{
struct HeadlessTrajectorySession::EngineHolder
{
	RobotInstruction::TrajectoryPipelineEngine engine;
};

namespace
{
engine::RigidTransform rigidFromBackendMat4(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
		cm[static_cast<size_t>(i)] = m.v[i];
	return engine::rigidTransformFromColMajor(cm);
}

QString phaseStr(RobotInstruction::PathPlanPhase p)
{
	using P = RobotInstruction::PathPlanPhase;
	switch (p)
	{
	case P::RawReady:
		return QStringLiteral("raw_ready");
	case P::Applied:
		return QStringLiteral("applied");
	case P::Draft:
	default:
		return QStringLiteral("draft");
	}
}

QString paramTypeToken(trajectory_algo::TrajectoryParamType t)
{
	using T = trajectory_algo::TrajectoryParamType;
	switch (t)
	{
	case T::Int:
		return QStringLiteral("Int");
	case T::Bool:
		return QStringLiteral("Bool");
	case T::Enum:
		return QStringLiteral("Enum");
	case T::Vec3:
		return QStringLiteral("Vec3");
	case T::Message:
		return QStringLiteral("Message");
	case T::Double:
	default:
		return QStringLiteral("Double");
	}
}

QJsonObject fieldToJson(const trajectory_algo::TrajectoryOpParamField& f)
{
	QJsonObject o;
	o.insert(QStringLiteral("key"), QString::fromStdString(f.key));
	o.insert(QStringLiteral("type"), paramTypeToken(f.type));
	o.insert(QStringLiteral("labelZh"), QString::fromStdString(f.labelZh.empty() ? f.labelEn : f.labelZh));
	o.insert(QStringLiteral("labelEn"), QString::fromStdString(f.labelEn));
	o.insert(QStringLiteral("unit"), QString::fromStdString(f.unit));
	o.insert(QStringLiteral("group"), QString::fromStdString(f.group));
	o.insert(QStringLiteral("order"), f.order);
	o.insert(QStringLiteral("min"), f.minValue);
	o.insert(QStringLiteral("max"), f.maxValue);
	o.insert(QStringLiteral("step"), f.step);
	o.insert(QStringLiteral("minInt"), f.minInt);
	o.insert(QStringLiteral("maxInt"), f.maxInt);
	o.insert(QStringLiteral("defaultDouble"), f.defaultDouble);
	o.insert(QStringLiteral("defaultInt"), f.defaultInt);
	o.insert(QStringLiteral("defaultBool"), f.defaultBool);
	QJsonArray ev, el;
	for (const auto& v : f.enumValues)
		ev.append(QString::fromStdString(v));
	for (const auto& v : f.enumLabelsZh)
		el.append(QString::fromStdString(v));
	o.insert(QStringLiteral("enumValues"), ev);
	o.insert(QStringLiteral("enumLabelsZh"), el);
	o.insert(QStringLiteral("visibleWhenScopeKind"), QString::fromStdString(f.visibleWhenScopeKind));
	o.insert(QStringLiteral("visibleWhenFieldKey"), QString::fromStdString(f.visibleWhenFieldKey));
	o.insert(QStringLiteral("visibleWhenIntValue"), f.visibleWhenIntValue);
	o.insert(QStringLiteral("messageZh"), QString::fromStdString(f.messageZh.empty() ? f.messageEn : f.messageZh));
	o.insert(QStringLiteral("vec3SuffixX"), QString::fromStdString(f.vec3SuffixX));
	o.insert(QStringLiteral("vec3SuffixY"), QString::fromStdString(f.vec3SuffixY));
	o.insert(QStringLiteral("vec3SuffixZ"), QString::fromStdString(f.vec3SuffixZ));
	return o;
}

QJsonValue paramValueToJson(const trajectory_algo::TrajectoryParamValue& v)
{
	using K = trajectory_algo::TrajectoryParamValue::Kind;
	switch (v.kind)
	{
	case K::Bool:
		return v.asBool;
	case K::Int:
		return v.asInt;
	case K::String:
		return QString::fromStdString(v.asString);
	case K::Double:
	default:
		return v.asDouble;
	}
}
} // namespace

HeadlessTrajectorySession::HeadlessTrajectorySession(DocumentHost& host)
	: m_host(host), m_engine(std::make_unique<EngineHolder>())
{
}

HeadlessTrajectorySession::~HeadlessTrajectorySession() = default;

RobotInstruction::RobotProgramCatalog* HeadlessTrajectorySession::catalog()
{
	// 必须写入 scene 对应 catalog；activeCatalog 在未登记实例时会落到静态空目录，生成后指令树永远看不到
	if (!m_sceneBackendId.empty())
	{
		const QString sid = QString::fromStdString(m_sceneBackendId);
		auto& store = m_host.robotProgramStore();
		if (!store.robotBackendIds().contains(sid))
		{
			QStringList labels = store.robotLabels();
			QStringList ids = store.robotBackendIds();
			labels.append(sid);
			ids.append(sid);
			store.setRobotInstances(labels, ids);
		}
		store.setActiveRobotBackendId(sid);
		return &store.catalogFor(sid);
	}
	return &m_host.robotProgramStore().activeCatalog();
}

RobotInstruction::PathPlanInstruction* HeadlessTrajectorySession::boundPathPlan()
{
	if (m_boundPathPlanId.empty())
		return nullptr;
	auto* cat = catalog();
	return cat->findPathPlan(cat->activeProgramId(), m_boundPathPlanId);
}

bool HeadlessTrajectorySession::requireEdit(QString* err) const
{
	if (m_featureEditActive)
		return true;
	if (err)
		*err = QStringLiteral("请先「开始修改」");
	return false;
}

bool HeadlessTrajectorySession::requireBound(QString* err) const
{
	if (!m_boundPathPlanId.empty())
		return true;
	if (err)
		*err = QStringLiteral("未绑定 PathPlan");
	return false;
}

void HeadlessTrajectorySession::pushUndo()
{
	m_undo.push_back(DraftSnap{m_raw, m_ops});
	if (m_undo.size() > 64)
		m_undo.erase(m_undo.begin());
	m_redo.clear();
}

bool HeadlessTrajectorySession::beginEdit(QString* err)
{
	if (!requireBound(err))
		return false;
	// 对齐桌面 reloadBoundPathPlanFromStore：进入修改前从 PathPlan 重载
	reloadBoundFromStore();
	m_featureEditActive = true;
	m_undo.clear();
	m_redo.clear();
	return true;
}

void HeadlessTrajectorySession::cancelEdit()
{
	m_featureEditActive = false;
	// 放弃草稿，恢复已落盘内容（不回滚已写入 PathPlan 的历史提交）
	reloadBoundFromStore();
	m_undo.clear();
	m_redo.clear();
}

void HeadlessTrajectorySession::reloadBoundFromStore()
{
	if (m_boundPathPlanId.empty())
		return;
	auto* cat = catalog();
	if (auto* pp = boundPathPlan())
		m_ops = pp->pipeline();
	else
		m_ops.clear();
	RobotInstruction::RawTrajectory raw;
	if (cat->pathPlanRaws().load(m_boundPathPlanId, raw))
		m_raw = std::move(raw);
	else if (!boundPathPlan())
		m_raw.reset();
}

bool HeadlessTrajectorySession::createPathPlan(const QString& sceneBackendId, QString* outPathPlanId, QString* err)
{
	QString sceneRoot = sceneBackendId.trimmed();
	if (sceneRoot.isEmpty())
	{
		if (auto* robotCtx = m_host.headlessRobotContext())
		{
			const auto instances = robotCtx->listInstances();
			if (!instances.isEmpty())
				sceneRoot = instances.front().sceneRootBackendId;
		}
	}
	if (sceneRoot.isEmpty())
	{
		if (err)
			*err = QStringLiteral("需要机器人场景根：请先导入机器人后再创建 PathPlan（工件 alone 不够）");
		return false;
	}
	m_sceneBackendId = sceneRoot.toStdString();
	auto* cat = catalog();
	auto* prog = cat->mainProgram();
	if (!prog)
	{
		if (err)
			*err = QStringLiteral("无主程序");
		return false;
	}
	auto pp = std::make_shared<RobotInstruction::PathPlanInstruction>();
	pp->setName("path_plan");
	pp->setRawTrajectoryKey(pp->id());
	pp->setPhase(RobotInstruction::PathPlanPhase::Draft);
	prog->steps.push_back(pp);
	m_boundPathPlanId = pp->id();
	m_raw.reset();
	m_ops.clear();
	m_featureEditActive = false;
	m_emitDisabledAfterApply = false;
	if (outPathPlanId)
		*outPathPlanId = QString::fromStdString(pp->id());
	return true;
}

bool HeadlessTrajectorySession::bindPathPlan(const QString& pathPlanId, QString* err, const QString& sceneBackendId)
{
	if (!sceneBackendId.isEmpty())
		m_sceneBackendId = sceneBackendId.toStdString();
	auto* cat = catalog();
	auto* pp = cat->findPathPlan(cat->activeProgramId(), pathPlanId.toStdString());
	if (!pp)
	{
		if (err)
			*err = QStringLiteral("PathPlan 不存在");
		return false;
	}
	m_boundPathPlanId = pp->id();
	m_ops = pp->pipeline();
	RobotInstruction::RawTrajectory raw;
	if (cat->pathPlanRaws().load(m_boundPathPlanId, raw))
		m_raw = std::move(raw);
	else
		m_raw.reset();
	m_featureEditActive = false;
	return true;
}

void HeadlessTrajectorySession::clearBinding()
{
	m_boundPathPlanId.clear();
	m_raw.reset();
	m_ops.clear();
	m_featureEditActive = false;
	m_emitDisabledAfterApply = false;
}

QJsonObject HeadlessTrajectorySession::sessionSummaryJson() const
{
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("pathPlanId"), QString::fromStdString(m_boundPathPlanId));
	o.insert(QStringLiteral("featureEditActive"), m_featureEditActive);
	o.insert(QStringLiteral("hasRaw"), hasRaw());
	o.insert(QStringLiteral("rawPointCount"), m_raw ? static_cast<int>(m_raw->points.size()) : 0);
	o.insert(QStringLiteral("pipelineOpCount"), static_cast<int>(m_ops.size()));
	o.insert(QStringLiteral("canUndo"), !m_undo.empty());
	o.insert(QStringLiteral("canRedo"), !m_redo.empty());
	o.insert(QStringLiteral("emitDisabled"), m_emitDisabledAfterApply);
	QString rawStatus;
	if (!hasRaw())
		rawStatus = QStringLiteral("请先在轨迹生成页离散");
	else if (m_emitDisabledAfterApply)
		rawStatus = QStringLiteral("Raw %1 点（已应用，生成已禁用）").arg(o.value(QStringLiteral("rawPointCount")).toInt());
	else
		rawStatus = QStringLiteral("Raw %1 点").arg(o.value(QStringLiteral("rawPointCount")).toInt());
	o.insert(QStringLiteral("rawStatusText"), rawStatus);
	auto* self = const_cast<HeadlessTrajectorySession*>(this);
	if (const auto* pp = self->boundPathPlan())
	{
		o.insert(QStringLiteral("phase"), phaseStr(pp->phase()));
		const std::string& feat =
			(m_raw && !m_raw->sourceFeatureJson.empty()) ? m_raw->sourceFeatureJson : pp->sourceFeatureJson();
		if (!feat.empty())
			o.insert(QStringLiteral("sourceFeatureJson"), QString::fromStdString(feat));
	}
	else if (m_raw && !m_raw->sourceFeatureJson.empty())
	{
		o.insert(QStringLiteral("sourceFeatureJson"), QString::fromStdString(m_raw->sourceFeatureJson));
	}
	return o;
}

QJsonArray HeadlessTrajectorySession::listPathPlansJson(const QString& sceneBackendId) const
{
	auto* self = const_cast<HeadlessTrajectorySession*>(this);
	if (!sceneBackendId.isEmpty())
		self->m_sceneBackendId = sceneBackendId.toStdString();
	QJsonArray arr;
	auto* cat = self->catalog();
	auto* prog = cat->mainProgram();
	if (!prog)
		return arr;
	for (const auto& step : prog->steps)
	{
		auto* pp = dynamic_cast<RobotInstruction::PathPlanInstruction*>(step.get());
		if (!pp)
			continue;
		QJsonObject o;
		o.insert(QStringLiteral("id"), QString::fromStdString(pp->id()));
		o.insert(QStringLiteral("name"), QString::fromStdString(pp->name()));
		o.insert(QStringLiteral("phase"), phaseStr(pp->phase()));
		o.insert(QStringLiteral("bound"), pp->id() == m_boundPathPlanId);
		arr.append(o);
	}
	return arr;
}

bool HeadlessTrajectorySession::worldFromModelPoint(const std::string& backendId, double mx, double my, double mz,
													double& wx, double& wy, double& wz) const
{
	const auto data = m_host.backend().getData(backendId);
	if (!data)
		return false;
	// BackendMat4 与 OSG 同序（平移在 3/7/11），须走 Adapters，禁止当 Eigen 列向量读 v[12..]
	const BackendVec3 w = transformPointToWorld(*data, BackendVec3{mx, my, mz}, &m_host.backend());
	wx = w.x;
	wy = w.y;
	wz = w.z;
	return true;
}

bool HeadlessTrajectorySession::modelFromWorldPoint(const std::string& backendId, double wx, double wy, double wz,
													double& mx, double& my, double& mz) const
{
	const auto data = m_host.backend().getData(backendId);
	if (!data)
		return false;
	const BackendVec3 m = transformPointToStored(*data, BackendVec3{wx, wy, wz}, &m_host.backend());
	mx = m.x;
	my = m.y;
	mz = m.z;
	return true;
}

bool HeadlessTrajectorySession::modelFromWorldDir(const std::string& backendId, double wx, double wy, double wz,
												  double& mx, double& my, double& mz) const
{
	const auto data = m_host.backend().getData(backendId);
	if (!data)
		return false;
	const Eigen::Vector3d out =
		rigidFromBackendMat4(data->worldMatrix()).inverse().isometry().linear() *
		Eigen::Vector3d(wx, wy, wz);
	const double len = out.norm();
	if (len < 1e-12)
		return false;
	mx = out.x() / len;
	my = out.y() / len;
	mz = out.z() / len;
	return true;
}

bool HeadlessTrajectorySession::transformRawToWorld(const RobotInstruction::RawTrajectory& modelRaw,
													RobotInstruction::RawTrajectory& worldRaw, QString* err) const
{
	worldRaw = modelRaw;
	const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(modelRaw);
	if (backendId.empty())
	{
		// 已是世界或无工件：原样
		return true;
	}
	const auto data = m_host.backend().getData(backendId);
	if (!data)
	{
		if (err)
			*err = QStringLiteral("工件世界矩阵不可用");
		return false;
	}
	// 与桌面 FeaturePickTransform 同式：位置走 Backend 世界矩阵；姿态 OSG mFile*rot（勿用仅 Eigen 链替代）
	const engine::RigidTransform T_wm = rigidFromBackendMat4(data->worldMatrix());
	osg::Matrixd worldMat = engine::osgMatrixFromRigidTransform(T_wm);
	osg::Matrixd rot = worldMat;
	rot.setTrans(0.0, 0.0, 0.0);
	for (auto& pt : worldRaw.points)
	{
		double wx = 0, wy = 0, wz = 0;
		if (!worldFromModelPoint(backendId, pt.poseMm.x, pt.poseMm.y, pt.poseMm.z, wx, wy, wz))
		{
			if (err)
				*err = QStringLiteral("工件世界矩阵不可用");
			return false;
		}
		pt.poseMm.x = wx;
		pt.poseMm.y = wy;
		pt.poseMm.z = wz;

		const osg::Quat qFile = engine::eulerDegToQuat(pt.eulerDeg.x, pt.eulerDeg.y, pt.eulerDeg.z);
		const osg::Matrixd mFile = osg::Matrixd::rotate(qFile);
		const osg::Matrixd mWorld = mFile * rot;
		const osg::Vec3f eulerWorld = engine::quatToEulerDegVec3f(mWorld.getRotate());
		pt.eulerDeg.x = static_cast<double>(eulerWorld.x());
		pt.eulerDeg.y = static_cast<double>(eulerWorld.y());
		pt.eulerDeg.z = static_cast<double>(eulerWorld.z());
	}
	return true;
}

bool HeadlessTrajectorySession::pickMeshElement(const QByteArray& body, QJsonObject* out, QString* err)
{
	// 点击也带折线高亮，便于确认选中面/边
	return pickShapeRay(body, true, true, out, err);
}

bool HeadlessTrajectorySession::pickHover(const QByteArray& body, QJsonObject* out, QString* err)
{
	// 悬停也返回 BREP 面三角 soup（与桌面一致），不用显示 mesh 三角冒充
	return pickShapeRay(body, true, true, out, err);
}

bool HeadlessTrajectorySession::pickShapeRay(const QByteArray& body, bool requireEditGate, bool includeHighlight,
											 QJsonObject* out, QString* err)
{
	if (requireEditGate && !requireEdit(err))
		return false;
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON");
		return false;
	}
	const QJsonObject o = doc.object();
	const QString mode = o.value(QStringLiteral("mode")).toString(QStringLiteral("edge"));
	const QString workpiece = o.value(QStringLiteral("workpieceBackendId")).toString();
	if (workpiece.isEmpty())
	{
		if (err)
			*err = QStringLiteral("workpieceBackendId required");
		return false;
	}
	const QJsonArray origin = o.value(QStringLiteral("originMm")).toArray();
	const QJsonArray dir = o.value(QStringLiteral("dir")).toArray();
	if (origin.size() < 3 || dir.size() < 3)
	{
		if (err)
			*err = QStringLiteral("originMm[3] and dir[3] required");
		return false;
	}
	const double ox = origin[0].toDouble();
	const double oy = origin[1].toDouble();
	const double oz = origin[2].toDouble();
	double dx = dir[0].toDouble();
	double dy = dir[1].toDouble();
	double dz = dir[2].toDouble();
	const double dlen = std::sqrt(dx * dx + dy * dy + dz * dz);
	if (dlen < 1e-12)
	{
		if (err)
			*err = QStringLiteral("dir zero");
		return false;
	}
	dx /= dlen;
	dy /= dlen;
	dz /= dlen;

	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wpRef;
	std::string geoErr;
	if (geometry_backend_ops::resolveWorkpieceShape(workpiece.toStdString(), m_host.backend(), {}, shape, wpRef,
													&geoErr) == geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		if (err)
			*err = geoErr.empty() ? QStringLiteral("无法解析工件形状") : QString::fromStdString(geoErr);
		return false;
	}

	double mx = 0, my = 0, mz = 0, mdx = 0, mdy = 0, mdz = 0;
	if (!modelFromWorldPoint(workpiece.toStdString(), ox, oy, oz, mx, my, mz) ||
		!modelFromWorldDir(workpiece.toStdString(), dx, dy, dz, mdx, mdy, mdz))
	{
		if (err)
			*err = QStringLiteral("世界→模型坐标失败");
		return false;
	}

	geoalgo::Point3d originM{mx, my, mz};
	geoalgo::Point3d dirM{mdx, mdy, mdz};
	geoalgo::ShapeRayPickResult pick;
	std::string pickErr;
	const bool faceMode = mode.compare(QStringLiteral("face"), Qt::CaseInsensitive) == 0;
	bool ok = false;

	const QJsonArray hitWorld = o.value(QStringLiteral("hitPointWorldMm")).toArray();
	const QJsonArray hitNormal = o.value(QStringLiteral("hitNormalWorld")).toArray();
	double hitMx = 0, hitMy = 0, hitMz = 0;
	const bool haveHit =
		hitWorld.size() >= 3 &&
		modelFromWorldPoint(workpiece.toStdString(), hitWorld[0].toDouble(), hitWorld[1].toDouble(),
							hitWorld[2].toDouble(), hitMx, hitMy, hitMz);
	double nMx = 0, nMy = 0, nMz = 0;
	const bool haveNormal =
		haveHit && hitNormal.size() >= 3 &&
		modelFromWorldDir(workpiece.toStdString(), hitNormal[0].toDouble(), hitNormal[1].toDouble(),
						  hitNormal[2].toDouble(), nMx, nMy, nMz);

	if (faceMode)
	{
		// 用「显示 mesh 命中点 + 外法向」短射线打 BREP，与鼠标所见三角面对齐
		if (haveHit && haveNormal)
		{
			const geoalgo::Point3d originSurf{hitMx + nMx * 2.0, hitMy + nMy * 2.0, hitMz + nMz * 2.0};
			const geoalgo::Point3d dirInto{-nMx, -nMy, -nMz};
			ok = geoalgo::pickShapeFaceByModelRay(shape, originSurf, dirInto, pick, &pickErr);
		}
		if (!ok && haveHit)
		{
			const double vx = hitMx - originM.x;
			const double vy = hitMy - originM.y;
			const double vz = hitMz - originM.z;
			const double vlen = std::sqrt(vx * vx + vy * vy + vz * vz);
			if (vlen > 1e-9)
			{
				const geoalgo::Point3d throughHit{vx / vlen, vy / vlen, vz / vlen};
				ok = geoalgo::pickShapeFaceByModelRay(shape, originM, throughHit, pick, &pickErr);
			}
		}
		if (!ok)
			ok = geoalgo::pickShapeFaceByModelRay(shape, originM, dirM, pick, &pickErr);
		if (!ok && haveHit)
		{
			int faceIndex = -1;
			if (geoalgo::resolveFaceIndexFromModelPoint(shape, geoalgo::Point3d{hitMx, hitMy, hitMz}, faceIndex, 2.0,
														&pickErr) &&
				faceIndex >= 0)
			{
				pick.hit = true;
				pick.faceIndex = faceIndex;
				pick.hitPointModelMm = {hitMx, hitMy, hitMz};
				ok = true;
			}
		}
	}
	else
	{
		// 网页 mesh 命中点 → 最近 BRep 边（与桌面 OsgSceneBrepPick 兜底一致）
		if (haveHit)
			ok = geoalgo::pickShapeEdgeByModelPoint(shape, geoalgo::Point3d{hitMx, hitMy, hitMz}, 5.0, pick, &pickErr) &&
				 pick.hit;
		// 无三角面命中时才走射线找边，避免同路径二次踩坏边
		if (!ok && !haveHit)
			ok = geoalgo::pickShapeEdgeByModelRay(shape, originM, dirM, 5.0, pick, &pickErr);
	}
	if (!ok || !pick.hit)
	{
		if (err)
			*err = pickErr.empty() ? QStringLiteral("未命中") : QString::fromStdString(pickErr);
		return false;
	}

	double hitWx = 0, hitWy = 0, hitWz = 0;
	worldFromModelPoint(workpiece.toStdString(), pick.hitPointModelMm.x, pick.hitPointModelMm.y, pick.hitPointModelMm.z,
						hitWx, hitWy, hitWz);

	if (out)
	{
		(*out)[QStringLiteral("ok")] = true;
		(*out)[QStringLiteral("mode")] = faceMode ? QStringLiteral("face") : QStringLiteral("edge");
		(*out)[QStringLiteral("workpieceBackendId")] = workpiece;
		(*out)[QStringLiteral("faceIndex")] = pick.faceIndex;
		(*out)[QStringLiteral("edgeIndex")] = pick.edgeIndex;
		(*out)[QStringLiteral("hitPointModelMm")] =
			QJsonArray{pick.hitPointModelMm.x, pick.hitPointModelMm.y, pick.hitPointModelMm.z};
		(*out)[QStringLiteral("hitPointWorldMm")] = QJsonArray{hitWx, hitWy, hitWz};

		if (includeHighlight)
		{
			// 与桌面 OsgSceneBrepPick 同级 Medium，保证 BREP 整面高亮完整
			geoalgo::TessellateParams tess;
			tess.linearDeflectionMm = 0.01;
			tess.angularDeflectionDeg = 0.5;
			tess.linearDeflectionRelative = false;
			QJsonArray polysWorld;
			const std::string bid = workpiece.toStdString();
			auto appendPoly = [&](const geoalgo::Polyline3d& poly)
			{
				QJsonArray pts;
				for (size_t i = 0; i + 2 < poly.xyz.size(); i += 3)
				{
					double wx = 0, wy = 0, wz = 0;
					if (!worldFromModelPoint(bid, poly.xyz[i], poly.xyz[i + 1], poly.xyz[i + 2], wx, wy, wz))
					{
						wx = poly.xyz[i];
						wy = poly.xyz[i + 1];
						wz = poly.xyz[i + 2];
					}
					pts.append(QJsonArray{wx, wy, wz});
				}
				if (pts.size() >= 2)
					polysWorld.append(pts);
			};
			if (faceMode && pick.faceIndex >= 0)
			{
				const QString cacheKey =
					workpiece + QLatin1Char('#') + QString::number(pick.faceIndex);
				if (m_faceHighlightSoup.contains(cacheKey))
				{
					(*out)[QStringLiteral("soupWorldMm")] = m_faceHighlightSoup.value(cacheKey);
				}
				else
				{
					std::vector<float> soupModel;
					if (geoalgo::discretizeShapeFaceByIndex(shape, pick.faceIndex, tess, soupModel, nullptr) &&
						soupModel.size() >= 9U)
					{
						QJsonArray soupWorld;
						for (size_t i = 0; i + 2 < soupModel.size(); i += 3)
						{
							double wx = 0, wy = 0, wz = 0;
							if (!worldFromModelPoint(bid, soupModel[i], soupModel[i + 1], soupModel[i + 2], wx, wy,
													 wz))
							{
								wx = soupModel[i];
								wy = soupModel[i + 1];
								wz = soupModel[i + 2];
							}
							soupWorld.append(wx);
							soupWorld.append(wy);
							soupWorld.append(wz);
						}
						m_faceHighlightSoup.insert(cacheKey, soupWorld);
						(*out)[QStringLiteral("soupWorldMm")] = soupWorld;
					}
				}
			}
			else if (pick.edgeIndex >= 0)
			{
				geoalgo::Polyline3d edgePoly;
				if (geoalgo::discretizeShapeEdgeByIndex(shape, pick.edgeIndex, tess, edgePoly, nullptr))
					appendPoly(edgePoly);
			}
			(*out)[QStringLiteral("polylinesWorld")] = polysWorld;
		}
	}
	return true;
}

bool HeadlessTrajectorySession::opSchemaJson(const QString& kind, int opIndex, QJsonObject* out, QString* err)
{
	RobotInstruction::ensureTrajectoryOpConfigsLoaded(QCoreApplication::applicationDirPath().toStdString());
	RobotInstruction::TrajectoryOpKind opKind{};
	if (!RobotInstruction::trajectoryOpKindFromString(kind.toStdString(), opKind))
	{
		if (err)
			*err = QStringLiteral("未知算子 kind");
		return false;
	}
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(opKind);
	if (!algo)
	{
		if (err)
			*err = QStringLiteral("算子未注册");
		return false;
	}

	RobotInstruction::TrajectoryOpDescriptor desc;
	if (opIndex >= 0 && opIndex < static_cast<int>(m_ops.size()) && m_ops[static_cast<size_t>(opIndex)].kind == opKind)
		desc = m_ops[static_cast<size_t>(opIndex)];
	else
		desc = RobotInstruction::trajectoryOpDefaultUnified(opKind, RobotInstruction::OpScope{});

	const auto fields = RobotInstruction::trajectoryOpAllParamFields(*algo);
	QJsonArray fieldArr;
	QJsonObject values;
	for (const auto& f : fields)
	{
		fieldArr.append(fieldToJson(f));
		trajectory_algo::TrajectoryParamValue pv{};
		if (RobotInstruction::trajectoryOpParamRead(desc, f, pv))
			values.insert(QString::fromStdString(f.key), paramValueToJson(pv));
	}
	if (out)
	{
		(*out)[QStringLiteral("ok")] = true;
		(*out)[QStringLiteral("kind")] = kind;
		(*out)[QStringLiteral("displayNameZh")] = QString::fromUtf8(algo->displayName(true));
		(*out)[QStringLiteral("fields")] = fieldArr;
		(*out)[QStringLiteral("values")] = values;
		(*out)[QStringLiteral("enabled")] = desc.enabled;
	}
	return true;
}

bool HeadlessTrajectorySession::featureCatalogJson(const QString& workpieceBackendId, QByteArray* out, QString* err)
{
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wpRef;
	std::string geoErr;
	if (geometry_backend_ops::resolveWorkpieceShape(workpieceBackendId.toStdString(), m_host.backend(), {}, shape,
													wpRef, &geoErr) ==
		geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		if (err)
			*err = geoErr.empty() ? QStringLiteral("无法解析工件") : QString::fromStdString(geoErr);
		return false;
	}
	geoalgo::FeatureCatalog catalog;
	if (!geoalgo::enumerateFeatureCatalog(wpRef, shape, catalog, &geoErr))
	{
		if (err)
			*err = QString::fromStdString(geoErr);
		return false;
	}
	if (out)
		*out = QByteArray::fromStdString(geoalgo::featureCatalogToJson(catalog));
	return true;
}

namespace
{
QString affinityToken(geoalgo::GeometryAffinity a)
{
	switch (a)
	{
	case geoalgo::GeometryAffinity::Line:
		return QStringLiteral("Line");
	case geoalgo::GeometryAffinity::Face:
		return QStringLiteral("Face");
	case geoalgo::GeometryAffinity::Any:
	default:
		return QStringLiteral("Any");
	}
}

QString featureParamTypeToken(geoalgo::FeatureParamType t)
{
	switch (t)
	{
	case geoalgo::FeatureParamType::Int:
		return QStringLiteral("Int");
	case geoalgo::FeatureParamType::Bool:
		return QStringLiteral("Bool");
	case geoalgo::FeatureParamType::Enum:
		return QStringLiteral("Enum");
	case geoalgo::FeatureParamType::Vec3:
		return QStringLiteral("Vec3");
	case geoalgo::FeatureParamType::Message:
		return QStringLiteral("Message");
	case geoalgo::FeatureParamType::Double:
	default:
		return QStringLiteral("Double");
	}
}

QJsonObject featureFieldToJson(const geoalgo::FeatureDiscretizerParamField& f)
{
	QJsonObject o;
	o.insert(QStringLiteral("key"), QString::fromStdString(f.key));
	o.insert(QStringLiteral("type"), featureParamTypeToken(f.type));
	o.insert(QStringLiteral("labelZh"), QString::fromStdString(f.labelZh.empty() ? f.labelEn : f.labelZh));
	o.insert(QStringLiteral("labelEn"), QString::fromStdString(f.labelEn));
	o.insert(QStringLiteral("unit"), QString::fromStdString(f.unit));
	o.insert(QStringLiteral("group"), QString::fromStdString(f.group));
	o.insert(QStringLiteral("order"), f.order);
	o.insert(QStringLiteral("min"), f.minValue);
	o.insert(QStringLiteral("max"), f.maxValue);
	o.insert(QStringLiteral("step"), f.step);
	o.insert(QStringLiteral("minInt"), f.minInt);
	o.insert(QStringLiteral("maxInt"), f.maxInt);
	o.insert(QStringLiteral("defaultDouble"), f.defaultDouble);
	o.insert(QStringLiteral("defaultInt"), f.defaultInt);
	o.insert(QStringLiteral("defaultBool"), f.defaultBool);
	QJsonArray ev, el;
	for (const auto& v : f.enumValues)
		ev.append(QString::fromStdString(v));
	for (const auto& v : f.enumLabelsZh)
		el.append(QString::fromStdString(v));
	o.insert(QStringLiteral("enumValues"), ev);
	o.insert(QStringLiteral("enumLabelsZh"), el);
	o.insert(QStringLiteral("messageZh"), QString::fromStdString(f.messageZh.empty() ? f.messageEn : f.messageZh));
	return o;
}

void ensureFeatureDiscretizerRuntime()
{
	geometry_backend_ops::ensureFeatureDiscretizersRegistered();
	(void)geometry_backend_ops::ensureFeatureDiscretizerConfigsLoaded(
		QCoreApplication::applicationDirPath().toStdString(), nullptr);
}
} // namespace

bool HeadlessTrajectorySession::featureSchemaJson(const QString& strategyId, QJsonObject* out, QString* err)
{
	ensureFeatureDiscretizerRuntime();
	if (!out)
		return true;

	if (strategyId.isEmpty())
	{
		QJsonArray arr;
		for (const std::string& sid : geometry_backend_ops::featureDiscretizerListStrategyIds())
		{
			QJsonObject s;
			s.insert(QStringLiteral("strategyId"), QString::fromStdString(sid));
			s.insert(QStringLiteral("displayNameZh"),
					 QString::fromStdString(geometry_backend_ops::featureDiscretizerDisplayNameZh(sid)));
			s.insert(QStringLiteral("affinity"), affinityToken(geometry_backend_ops::featureDiscretizerAffinity(sid)));
			arr.append(s);
		}
		(*out)[QStringLiteral("ok")] = true;
		(*out)[QStringLiteral("strategies")] = arr;
		return true;
	}

	const std::string sid = strategyId.toStdString();
	if (geoalgo::featureDiscretizerGet(sid) == nullptr)
	{
		if (err)
			*err = QStringLiteral("未知策略: ") + strategyId;
		return false;
	}

	const auto fields = geometry_backend_ops::featureDiscretizerAllParamFields(sid);
	QJsonArray fieldArr;
	for (const auto& f : fields)
		fieldArr.append(featureFieldToJson(f));

	const nlohmann::json def = geometry_backend_ops::featureDiscretizerDefaultParams(sid);
	QJsonObject defaults =
		QJsonDocument::fromJson(QByteArray::fromStdString(def.is_object() ? def.dump() : "{}")).object();

	(*out)[QStringLiteral("ok")] = true;
	(*out)[QStringLiteral("strategyId")] = strategyId;
	(*out)[QStringLiteral("displayNameZh")] =
		QString::fromStdString(geometry_backend_ops::featureDiscretizerDisplayNameZh(sid));
	(*out)[QStringLiteral("affinity")] = affinityToken(geometry_backend_ops::featureDiscretizerAffinity(sid));
	(*out)[QStringLiteral("fields")] = fieldArr;
	(*out)[QStringLiteral("defaults")] = defaults;
	return true;
}

bool HeadlessTrajectorySession::persistRaw(QString* err)
{
	if (!requireBound(err) || !m_raw)
		return false;
	auto* cat = catalog();
	if (!cat->pathPlanRaws().save(m_boundPathPlanId, *m_raw))
	{
		if (err)
			*err = QStringLiteral("保存 raw 失败");
		return false;
	}
	if (auto* pp = boundPathPlan())
	{
		pp->setPhase(RobotInstruction::PathPlanPhase::RawReady);
		if (!m_raw->sourceFeatureJson.empty())
			pp->setSourceFeatureJson(m_raw->sourceFeatureJson);
	}
	return true;
}

bool HeadlessTrajectorySession::persistPipeline(QString* err)
{
	if (!requireBound(err))
		return false;
	if (auto* pp = boundPathPlan())
		pp->setPipeline(m_ops);
	return true;
}

bool HeadlessTrajectorySession::setFeaturesAndDiscretize(const QByteArray& featureListJson, QString* err)
{
	if (!requireEdit(err) || !requireBound(err))
		return false;
	geoalgo::FeatureListDocument doc;
	std::string jerr;
	if (!geoalgo::featureListFromJson(featureListJson.toStdString(), doc, &jerr))
	{
		if (err)
			*err = jerr.empty() ? QStringLiteral("FeatureList JSON 无效") : QString::fromStdString(jerr);
		return false;
	}
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wpRef;
	std::string geoErr;
	const std::string wpId = doc.workpiece.backendIdUtf8;
	if (geometry_backend_ops::resolveWorkpieceShape(wpId, m_host.backend(), doc.workpiece.stepPathUtf8, shape, wpRef,
													&geoErr) == geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		if (err)
			*err = geoErr.empty() ? QStringLiteral("无法解析工件形状") : QString::fromStdString(geoErr);
		return false;
	}
	geoalgo::RawPath path;
	if (!geoalgo::discretizeFeatureList(doc, shape, path, &geoErr))
	{
		if (err)
			*err = geoErr.empty() ? QStringLiteral("离散失败") : QString::fromStdString(geoErr);
		return false;
	}
	RobotInstruction::RawTrajectory traj;
	if (!RobotInstruction::importRawPathToTrajectory(path, RobotInstruction::FrameStrategy::SurfaceNormalZ, traj,
													 &geoErr))
	{
		if (err)
			*err = QString::fromStdString(geoErr);
		return false;
	}
	traj.sourceFeatureJson = geoalgo::featureListToJson(doc);
	pushUndo();
	m_raw = std::move(traj);
	m_emitDisabledAfterApply = false;
	return persistRaw(err);
}

bool HeadlessTrajectorySession::discretizeMeshSpec(const QByteArray& meshSpecJson, QString* err)
{
	if (!requireEdit(err) || !requireBound(err))
		return false;
	geoalgo::MeshTrajectorySpec spec;
	std::string jerr;
	if (!geoalgo::meshTrajectorySpecFromJson(meshSpecJson.toStdString(), spec, &jerr))
	{
		if (err)
			*err = jerr.empty() ? QStringLiteral("MeshSpec JSON 无效") : QString::fromStdString(jerr);
		return false;
	}
	const std::string meshId = spec.workpiece.backendIdUtf8;
	const auto meshData = std::dynamic_pointer_cast<MeshBackendData>(m_host.backend().getData(meshId));
	if (!meshData || meshData->triangleSoup().empty())
	{
		if (err)
			*err = QStringLiteral("Mesh 无三角面片");
		return false;
	}
	geoalgo::RawPath path;
	if (!geoalgo::generateMeshTrajectory(spec, meshData->triangleSoup(), path, &jerr))
	{
		if (err)
			*err = jerr.empty() ? QStringLiteral("Mesh 轨迹生成失败") : QString::fromStdString(jerr);
		return false;
	}
	RobotInstruction::MeshTrajectoryIngressParams params;
	RobotInstruction::RawTrajectory traj;
	if (!RobotInstruction::importMeshRawPathToRawTrajectory(path, meshSpecJson.toStdString(), params, traj, &jerr))
	{
		if (err)
			*err = QString::fromStdString(jerr);
		return false;
	}
	pushUndo();
	m_raw = std::move(traj);
	m_emitDisabledAfterApply = false;
	return persistRaw(err);
}

bool HeadlessTrajectorySession::setPipelineJson(const QByteArray& pipelineJson, QString* err)
{
	if (!requireEdit(err) || !requireBound(err))
		return false;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
	std::string jerr;
	const auto j = nlohmann::json::parse(pipelineJson.constData(), pipelineJson.constData() + pipelineJson.size(),
										 nullptr, false);
	if (j.is_discarded() ||
		!RobotInstruction::trajectoryPipelineFromJson(
			j.is_array() ? j : j.value("pipeline", nlohmann::json::array()), ops, &jerr))
	{
		if (err)
			*err = jerr.empty() ? QStringLiteral("pipeline JSON 无效") : QString::fromStdString(jerr);
		return false;
	}
	pushUndo();
	m_ops = std::move(ops);
	return persistPipeline(err);
}

bool HeadlessTrajectorySession::fillRecipe(const QString& recipeKind, QString* err)
{
	if (!requireEdit(err) || !requireBound(err))
		return false;
	RobotInstruction::RecipeKind kind = RobotInstruction::RecipeKind::Weld;
	if (recipeKind.compare(QStringLiteral("glue"), Qt::CaseInsensitive) == 0)
		kind = RobotInstruction::RecipeKind::Glue;
	else if (recipeKind.compare(QStringLiteral("grind"), Qt::CaseInsensitive) == 0)
		kind = RobotInstruction::RecipeKind::Grind;
	pushUndo();
	m_ops = RobotInstruction::buildRecipePreset(kind);
	return persistPipeline(err);
}

QByteArray HeadlessTrajectorySession::pipelineJson() const
{
	const nlohmann::json j = RobotInstruction::trajectoryPipelineToJson(m_ops);
	return QByteArray::fromStdString(j.dump());
}

void HeadlessTrajectorySession::injectWorkpieceReferenceOnEngine()
{
	auto& eng = m_engine->engine;
	HeadlessRobotContext* robotCtx = m_host.headlessRobotContext();
	if (!robotCtx)
	{
		eng.setWorkpieceReferenceInBase(nullptr);
		eng.setExternalTcpFrameResolver(nullptr);
		return;
	}

	QString sceneRoot = QString::fromStdString(m_sceneBackendId);
	if (sceneRoot.isEmpty())
	{
		const auto instances = robotCtx->listInstances();
		if (!instances.isEmpty())
			sceneRoot = instances.front().sceneRootBackendId;
	}

	HeadlessRobotContext::TcpPoseCapture tcp{};
	if (sceneRoot.isEmpty() || !robotCtx->captureTcpPose(sceneRoot, tcp, nullptr))
	{
		eng.setWorkpieceReferenceInBase(nullptr);
	}
	else
	{
		const engine::RigidTransform ref = engine::RigidTransform::fromTranslationEulerDeg(
			tcp.positionMm[0], tcp.positionMm[1], tcp.positionMm[2], tcp.eulerDeg[0], tcp.eulerDeg[1],
			tcp.eulerDeg[2]);
		eng.setWorkpieceReferenceInBase(&ref);
	}

	BackendDataManager* mgr = &m_host.backend();
	eng.setExternalTcpFrameResolver(
		[mgr](const std::string& backendId, engine::RigidTransform& out, std::string* errMsg) -> bool
		{
			const std::shared_ptr<BackendDataBase> data = mgr->getData(backendId);
			if (!data || !std::dynamic_pointer_cast<FrameBackendData>(data))
			{
				if (errMsg)
					*errMsg = "external TCP frame backend not found: " + backendId;
				return false;
			}
			out = rigidFromBackendMat4(data->worldMatrix());
			return true;
		});
}

bool HeadlessTrajectorySession::runPipelineOnWorldRaw(RobotInstruction::RawTrajectory& worldRawInOut, QString* err)
{
	if (m_ops.empty())
		return true;
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();
	auto& eng = m_engine->engine;
	eng.clear();
	eng.setUsingRaw(true);
	eng.setSourceRaw(worldRawInOut);
	eng.setRawRebuildFn(
		[](const RobotInstruction::RawTrajectory& sourceRaw, RobotInstruction::UnifiedTrajectory& outUnified,
		   std::string* errMsg) -> bool
		{ return RobotInstruction::unifiedTrajectoryFromRaw(sourceRaw, outUnified, errMsg); });
	if (auto* prog = catalog()->mainProgram())
		eng.setProgramContext(prog);
	eng.setOps(m_ops);
	// 须在 execute 前注入：转换工件型读当前 TCP / 场景 Frame
	injectWorkpieceReferenceOnEngine();
	std::string perr;
	if (!eng.executeFull(&perr))
	{
		if (err)
			*err = perr.empty() ? QStringLiteral("管线执行失败") : QString::fromStdString(perr);
		return false;
	}
	if (!RobotInstruction::unifiedTrajectoryToRaw(eng.result(), worldRawInOut, &perr))
	{
		worldRawInOut = eng.rawWorking();
	}
	return true;
}

namespace
{
void fillWorldPolylineJson(const RobotInstruction::RawTrajectory& worldRaw, QJsonObject* out, bool pipelineApplied)
{
	if (!out)
		return;
	QJsonArray pts;
	QJsonArray eulers;
	QJsonArray axesX;
	QJsonArray axesY;
	QJsonArray axesZ;
	QJsonArray segmentEnds;
	for (const auto& p : worldRaw.points)
	{
		pts.append(QJsonArray{p.poseMm.x, p.poseMm.y, p.poseMm.z});
		eulers.append(QJsonArray{p.eulerDeg.x, p.eulerDeg.y, p.eulerDeg.z});
		// 与桌面 appendWorldPoseMarker 同式：local * makeRotate(euler)，直接给前端画轴，避免再解欧拉
		osg::Matrixd m;
		m.makeRotate(engine::eulerDegToQuat(p.eulerDeg.x, p.eulerDeg.y, p.eulerDeg.z));
		const osg::Vec3d x = osg::Vec3d(1.0, 0.0, 0.0) * m;
		const osg::Vec3d y = osg::Vec3d(0.0, 1.0, 0.0) * m;
		const osg::Vec3d z = osg::Vec3d(0.0, 0.0, 1.0) * m;
		axesX.append(QJsonArray{x.x(), x.y(), x.z()});
		axesY.append(QJsonArray{y.x(), y.y(), y.z()});
		axesZ.append(QJsonArray{z.x(), z.y(), z.z()});
	}
	for (const std::size_t end : worldRaw.segmentEndExclusive)
		segmentEnds.append(static_cast<double>(end));
	(*out)[QStringLiteral("ok")] = true;
	(*out)[QStringLiteral("pointsMm")] = pts;
	(*out)[QStringLiteral("eulersDeg")] = eulers;
	(*out)[QStringLiteral("axesX")] = axesX;
	(*out)[QStringLiteral("axesY")] = axesY;
	(*out)[QStringLiteral("axesZ")] = axesZ;
	(*out)[QStringLiteral("segmentEndExclusive")] = segmentEnds;
	(*out)[QStringLiteral("pointCount")] = static_cast<int>(worldRaw.points.size());
	(*out)[QStringLiteral("pipelineApplied")] = pipelineApplied;
}
} // namespace

bool HeadlessTrajectorySession::previewRaw(QJsonObject* outPolylineWorld, QString* err)
{
	if (!requireEdit(err))
		return false;
	if (!hasRaw())
	{
		if (err)
			*err = QStringLiteral("无 Raw 轨迹");
		return false;
	}
	RobotInstruction::RawTrajectory worldRaw;
	if (!transformRawToWorld(*m_raw, worldRaw, err))
		return false;
	fillWorldPolylineJson(worldRaw, outPolylineWorld, false);
	return true;
}

bool HeadlessTrajectorySession::preview(QJsonObject* outPolylineWorld, QString* err)
{
	if (!requireEdit(err))
		return false;
	if (!hasRaw())
	{
		if (err)
			*err = QStringLiteral("无 Raw 轨迹");
		return false;
	}
	RobotInstruction::RawTrajectory worldRaw;
	if (!transformRawToWorld(*m_raw, worldRaw, err))
		return false;
	if (!runPipelineOnWorldRaw(worldRaw, err))
		return false;
	fillWorldPolylineJson(worldRaw, outPolylineWorld, true);
	return true;
}

bool HeadlessTrajectorySession::apply(QString* err)
{
	if (!requireEdit(err) || !requireBound(err) || !hasRaw())
	{
		if (err && err->isEmpty())
			*err = QStringLiteral("无 Raw 或未绑定");
		return false;
	}
	RobotInstruction::RawTrajectory worldRaw;
	if (!transformRawToWorld(*m_raw, worldRaw, err))
		return false;
	if (!runPipelineOnWorldRaw(worldRaw, err))
		return false;
	auto* cat = catalog();
	auto* prog = cat->mainProgram();
	if (!prog)
	{
		if (err)
			*err = QStringLiteral("无主程序");
		return false;
	}
	std::string emitErr;
	std::string outGroup;
	if (!RobotInstruction::emitRawTrajectoryToProgram(worldRaw, *prog, &emitErr, &outGroup, &m_boundPathPlanId))
	{
		if (err)
			*err = emitErr.empty() ? QStringLiteral("写出 LINE 失败") : QString::fromStdString(emitErr);
		return false;
	}
	if (auto* pp = boundPathPlan())
	{
		pp->setPhase(RobotInstruction::PathPlanPhase::Applied);
		pp->setPipeline(m_ops);
		pp->appliedHistoryMut() = m_ops;
		if (!outGroup.empty())
			pp->setOutputGroupId(outGroup);
	}
	persistRaw(nullptr);
	persistPipeline(nullptr);
	m_featureEditActive = false;
	m_emitDisabledAfterApply = true;
	return true;
}

bool HeadlessTrajectorySession::emitRawProgram(QString* err)
{
	if (m_emitDisabledAfterApply)
	{
		if (err)
			*err = QStringLiteral("已应用后请勿再生成程序，避免覆盖应用结果");
		return false;
	}
	if (!requireEdit(err) || !requireBound(err) || !hasRaw())
	{
		if (err && err->isEmpty())
			*err = QStringLiteral("无 Raw 或未绑定");
		return false;
	}
	RobotInstruction::RawTrajectory worldRaw;
	if (!transformRawToWorld(*m_raw, worldRaw, err))
		return false;
	auto* cat = catalog();
	auto* prog = cat->mainProgram();
	if (!prog)
	{
		if (err)
			*err = QStringLiteral("无主程序");
		return false;
	}
	std::string emitErr;
	std::string outGroup;
	if (!RobotInstruction::emitRawTrajectoryToProgram(worldRaw, *prog, &emitErr, &outGroup, &m_boundPathPlanId))
	{
		if (err)
			*err = emitErr.empty() ? QStringLiteral("写出 LINE 失败") : QString::fromStdString(emitErr);
		return false;
	}
	if (auto* pp = boundPathPlan())
	{
		pp->setPhase(RobotInstruction::PathPlanPhase::Applied);
		if (!outGroup.empty())
			pp->setOutputGroupId(outGroup);
	}
	persistRaw(nullptr);
	m_featureEditActive = false;
	m_emitDisabledAfterApply = true;
	return true;
}

bool HeadlessTrajectorySession::opPaletteJson(QJsonObject* out, QString* err)
{
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();
	RobotInstruction::ensureTrajectoryOpConfigsLoaded(QCoreApplication::applicationDirPath().toStdString());
	QJsonArray arr;
	for (const RobotInstruction::TrajectoryOpKind kind : RobotInstruction::trajectoryOpPaletteKinds())
	{
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(kind);
		QJsonObject o;
		o.insert(QStringLiteral("kind"), QString::fromStdString(RobotInstruction::trajectoryOpKindToString(kind)));
		o.insert(QStringLiteral("displayNameZh"),
				 algo ? QString::fromUtf8(algo->displayName(true))
					  : QString::fromStdString(RobotInstruction::trajectoryOpKindToString(kind)));
		arr.append(o);
	}
	if (out)
	{
		(*out)[QStringLiteral("ok")] = true;
		(*out)[QStringLiteral("ops")] = arr;
	}
	(void)err;
	return true;
}

bool HeadlessTrajectorySession::resetPipeline(QString* err)
{
	if (!requireEdit(err) || !requireBound(err))
		return false;
	pushUndo();
	m_ops.clear();
	return persistPipeline(err);
}

bool HeadlessTrajectorySession::undoDraft(QString* err)
{
	if (!requireEdit(err))
		return false;
	if (m_undo.empty())
	{
		if (err)
			*err = QStringLiteral("无可撤销");
		return false;
	}
	m_redo.push_back(DraftSnap{m_raw, m_ops});
	auto snap = m_undo.back();
	m_undo.pop_back();
	m_raw = snap.raw;
	m_ops = snap.ops;
	persistRaw(nullptr);
	persistPipeline(nullptr);
	return true;
}

bool HeadlessTrajectorySession::redoDraft(QString* err)
{
	if (!requireEdit(err))
		return false;
	if (m_redo.empty())
	{
		if (err)
			*err = QStringLiteral("无可重做");
		return false;
	}
	m_undo.push_back(DraftSnap{m_raw, m_ops});
	auto snap = m_redo.back();
	m_redo.pop_back();
	m_raw = snap.raw;
	m_ops = snap.ops;
	persistRaw(nullptr);
	persistPipeline(nullptr);
	return true;
}

const RobotInstruction::RawTrajectory* HeadlessTrajectorySession::raw() const
{
	return m_raw ? &*m_raw : nullptr;
}

QString HeadlessTrajectorySession::templatesDir(const QString& kind) const
{
	const QString root =
		QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QStringLiteral("/CloudSim/templates/") + kind;
	QDir().mkpath(root);
	return root;
}

bool HeadlessTrajectorySession::saveTemplate(const QString& kind, const QString& name, const QByteArray& payload,
											 QString* err)
{
	if (name.isEmpty())
	{
		if (err)
			*err = QStringLiteral("name required");
		return false;
	}
	QFile f(templatesDir(kind) + QLatin1Char('/') + name + QStringLiteral(".json"));
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (err)
			*err = QStringLiteral("写入模板失败");
		return false;
	}
	f.write(payload);
	return true;
}

bool HeadlessTrajectorySession::loadTemplate(const QString& kind, const QString& name, QByteArray* out, QString* err)
{
	QFile f(templatesDir(kind) + QLatin1Char('/') + name + QStringLiteral(".json"));
	if (!f.open(QIODevice::ReadOnly))
	{
		if (err)
			*err = QStringLiteral("模板不存在");
		return false;
	}
	if (out)
		*out = f.readAll();
	return true;
}

bool HeadlessTrajectorySession::deleteTemplate(const QString& kind, const QString& name, QString* err)
{
	const QString path = templatesDir(kind) + QLatin1Char('/') + name + QStringLiteral(".json");
	if (!QFile::exists(path))
	{
		if (err)
			*err = QStringLiteral("模板不存在");
		return false;
	}
	return QFile::remove(path);
}

QJsonArray HeadlessTrajectorySession::listTemplatesJson(const QString& kind) const
{
	QJsonArray arr;
	const QDir dir(templatesDir(kind));
	for (const QString& f : dir.entryList(QStringList{QStringLiteral("*.json")}, QDir::Files, QDir::Name))
	{
		QJsonObject o;
		o.insert(QStringLiteral("name"), QFileInfo(f).completeBaseName());
		arr.append(o);
	}
	return arr;
}

} // namespace cloudsim::host
