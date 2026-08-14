/// @file HeadlessPointCloudBridge.cpp
/// @brief Web 点云桥：同步调用 document_point_cloud_ops / point_cloud_backend_ops

#include "HeadlessPointCloudBridge.h"

#include "BackendFileImport.h"
#include "BackendDataManager.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "BrepImportArtifacts.h"
#include "DocumentHost.h"
#include "DocumentPointCloudOps.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendOps.h"
#include "PluginPointCloudTypes.h"

#include <GeometryBackendOps.h>
#include <MeshSurfaceReconstruction.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonObject>
#include <QTemporaryFile>

#include <algorithm>
#include <cmath>
#include <functional>

namespace cloudsim::host
{
/// 对齐桌面插件分阶段（含 Preprocess）；中间预览对象暂不注册以减负
struct HeadlessPointCloudBridge::SurfaceSession
{
	QString sessionId;
	QString meshBackendId;
	std::vector<float> rawSoup;
	std::vector<float> workingSoup;
	geoalgo::MeshSurfaceReconstructSessionPtr geoSession;
	int lastCompleted = 0;
};

namespace
{
QString backendIdFromJson(const QJsonObject& o, const char* primary = "backendId")
{
	QString id = o.value(QString::fromLatin1(primary)).toString();
	if (id.isEmpty())
		id = o.value(QStringLiteral("backend_id")).toString();
	return id;
}

QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}

QJsonObject okExtra(const QJsonObject& extra = QJsonObject{})
{
	QJsonObject o{{QStringLiteral("ok"), true}};
	for (auto it = extra.begin(); it != extra.end(); ++it)
		o.insert(it.key(), it.value());
	return o;
}

QJsonArray vec3ToJson(const PluginVec3& v)
{
	return QJsonArray{v.x, v.y, v.z};
}

QJsonObject boundsToJson(const PluginAxisAlignedBox& b)
{
	QJsonObject o;
	o.insert(QStringLiteral("valid"), b.valid);
	if (b.valid)
	{
		o.insert(QStringLiteral("minMm"), vec3ToJson(b.minMm));
		o.insert(QStringLiteral("maxMm"), vec3ToJson(b.maxMm));
	}
	return o;
}

QJsonObject infoToJson(const PluginPointCloudInfo& info)
{
	QJsonObject o;
	o.insert(QStringLiteral("pointCount"), static_cast<qint64>(info.pointCount));
	o.insert(QStringLiteral("hasPerVertexColors"), info.hasPerVertexColors);
	o.insert(QStringLiteral("hasPointNormals"), info.hasPointNormals);
	o.insert(QStringLiteral("bounds"), boundsToJson(info.bounds));
	o.insert(QStringLiteral("mixedRenderThreshold"), static_cast<qint64>(HeadlessPointCloudBridge::kMixedRenderThreshold));
	return o;
}

void fillMat4FromJson(const QJsonArray& arr, double out[16])
{
	for (int i = 0; i < 16; ++i)
		out[i] = (i < arr.size()) ? arr.at(i).toDouble(0.0) : 0.0;
	if (arr.isEmpty())
	{
		const double id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
		std::copy(id, id + 16, out);
	}
}

PluginAxisAlignedBox boxFromJson(const QJsonObject& o)
{
	PluginAxisAlignedBox box;
	box.valid = o.value(QStringLiteral("valid")).toBool(true);
	const QJsonArray minA = o.value(QStringLiteral("minMm")).toArray();
	const QJsonArray maxA = o.value(QStringLiteral("maxMm")).toArray();
	if (minA.size() >= 3 && maxA.size() >= 3)
	{
		box.minMm.x = minA.at(0).toDouble();
		box.minMm.y = minA.at(1).toDouble();
		box.minMm.z = minA.at(2).toDouble();
		box.maxMm.x = maxA.at(0).toDouble();
		box.maxMm.y = maxA.at(1).toDouble();
		box.maxMm.z = maxA.at(2).toDouble();
		box.valid = true;
	}
	return box;
}

PluginMeshCreateOptions meshOptionsFromJson(const QJsonObject& o)
{
	PluginMeshCreateOptions opt;
	opt.displayName = o.value(QStringLiteral("displayName")).toString();
	opt.sourcePath = o.value(QStringLiteral("sourcePath")).toString();
	opt.resetViewToHome = o.value(QStringLiteral("resetViewToHome")).toBool(false);
	opt.selectInTree = o.value(QStringLiteral("selectInTree")).toBool(false);
	const QJsonArray pose = o.value(QStringLiteral("poseMm")).toArray();
	const QJsonArray rot = o.value(QStringLiteral("rotationDeg")).toArray();
	if (pose.size() >= 3)
	{
		opt.poseMm.x = pose.at(0).toDouble();
		opt.poseMm.y = pose.at(1).toDouble();
		opt.poseMm.z = pose.at(2).toDouble();
	}
	if (rot.size() >= 3)
	{
		opt.rotationDeg.x = rot.at(0).toDouble();
		opt.rotationDeg.y = rot.at(1).toDouble();
		opt.rotationDeg.z = rot.at(2).toDouble();
	}
	return opt;
}

Eigen::AlignedBox3d eigenBoxFromPlugin(const PluginAxisAlignedBox& box)
{
	Eigen::AlignedBox3d b;
	if (!box.valid)
		return b;
	b.extend(Eigen::Vector3d(box.minMm.x, box.minMm.y, box.minMm.z));
	b.extend(Eigen::Vector3d(box.maxMm.x, box.maxMm.y, box.maxMm.z));
	return b;
}

void uniformSampleXyz(const std::vector<float>& worldXyz, std::size_t maxPoints, std::vector<float>& out)
{
	out.clear();
	const std::size_t n = worldXyz.size() / 3U;
	if (n == 0U)
		return;
	if (maxPoints == 0U || n <= maxPoints)
	{
		out = worldXyz;
		return;
	}
	out.reserve(maxPoints * 3U);
	const double stride = static_cast<double>(n) / static_cast<double>(maxPoints);
	for (std::size_t i = 0; i < maxPoints; ++i)
	{
		const std::size_t idx = static_cast<std::size_t>(std::floor(static_cast<double>(i) * stride));
		const std::size_t base = std::min(idx, n - 1U) * 3U;
		out.push_back(worldXyz[base]);
		out.push_back(worldXyz[base + 1U]);
		out.push_back(worldXyz[base + 2U]);
	}
}

bool mutatePointCloud(DocumentHost& host, const QString& backendId,
					  const std::function<bool(PointCloudBackendData&, std::string*)>& mutate, QJsonObject* out)
{
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(&host, backendId.toStdString(), &resolveErr);
	if (!pc)
	{
		if (out)
			*out = fail(QString::fromStdString(resolveErr));
		return false;
	}
	PointCloudBackendData working;
	working.setPointBuffers(pc->pointPositionsXyz(), pc->pointVertexRgba(), pc->pointNormalsNxNyNz());
	working.setWorldMatrix(pc->worldMatrix());
	std::string err;
	if (!mutate(working, &err))
	{
		if (out)
			*out = fail(QString::fromStdString(err.empty() ? "point cloud operation failed" : err));
		return false;
	}
	pc->setPointBuffers(working.pointPositionsXyz(), working.pointVertexRgba(), working.pointNormalsNxNyNz());
	document_point_cloud_ops::commitPointCloudVisual(&host, *pc);
	if (out)
	{
		*out = okExtra({{QStringLiteral("pointCountAfter"), static_cast<qint64>(pc->geometryElementCount())},
						{QStringLiteral("backendId"), backendId}});
	}
	return true;
}

point_cloud_backend_ops::MeshRepairRequest repairFromJson(const QJsonObject& o)
{
	point_cloud_backend_ops::MeshRepairRequest r;
	r.removeDegenerate = o.value(QStringLiteral("removeDegenerate")).toBool(true);
	r.removeDuplicate = o.value(QStringLiteral("removeDuplicate")).toBool(true);
	r.removeNonManifold = o.value(QStringLiteral("removeNonManifold")).toBool(true);
	r.fillHoles = o.value(QStringLiteral("fillHoles")).toBool(false);
	r.holeMaxEdgeCount = o.value(QStringLiteral("holeMaxEdgeCount")).toInt(30);
	return r;
}

geoalgo::MeshSurfaceReconstructParams surfaceParamsFromJson(const QJsonObject& o)
{
	geoalgo::MeshSurfaceReconstructParams p;
	p.normalSmoothIterations = o.value(QStringLiteral("normalSmoothIterations")).toInt(6);
	p.featureThresholdC0 = o.value(QStringLiteral("featureThresholdC0")).toDouble(0.8);
	p.runVcgRepairFirst = o.value(QStringLiteral("runVcgRepairFirst")).toBool(true);
	p.runIsotropicRemesh = o.value(QStringLiteral("runIsotropicRemesh")).toBool(false);
	p.remeshTargetEdgeLengthMm = o.value(QStringLiteral("remeshTargetEdgeLengthMm")).toDouble(2.0);
	p.remeshIterations = o.value(QStringLiteral("remeshIterations")).toInt(3);
	p.remeshFeatureAngleDeg = o.value(QStringLiteral("remeshFeatureAngleDeg")).toDouble(30.0);
	p.patchCountHint = o.value(QStringLiteral("patchCountHint")).toInt(0);
	return p;
}

QString makeUniqueBrepName(DocumentHost& host, const QString& base)
{
	QString candidate = base.isEmpty() ? QStringLiteral("ReconstructedBrep") : base;
	int suffix = 2;
	for (;;)
	{
		if (host.backend().findByName(candidate.toStdString()).empty())
			return candidate;
		candidate = base + QStringLiteral("_%1").arg(suffix++);
	}
}

int parseSurfaceStage(const QJsonObject& body)
{
	if (!body.contains(QStringLiteral("stage")))
		return -1;
	const QJsonValue v = body.value(QStringLiteral("stage"));
	if (v.isDouble())
	{
		const int n = v.toInt(-1);
		return (n >= 1 && n <= 8) ? n : -1;
	}
	const QString s = v.toString().trimmed();
	bool ok = false;
	const int n = s.toInt(&ok);
	if (ok && n >= 1 && n <= 8)
		return n;
	const QString lower = s.toLower();
	if (lower == QStringLiteral("preprocess"))
		return 1;
	if (lower == QStringLiteral("partition"))
		return 2;
	if (lower == QStringLiteral("sample"))
		return 3;
	if (lower == QStringLiteral("fit"))
		return 4;
	if (lower == QStringLiteral("boundaryblend") || lower == QStringLiteral("boundary"))
		return 5;
	if (lower == QStringLiteral("junctionblend") || lower == QStringLiteral("junction"))
		return 6;
	if (lower == QStringLiteral("fair"))
		return 7;
	if (lower == QStringLiteral("assemble"))
		return 8;
	return -1;
}

geoalgo::MeshSurfaceReconstructStage mapPluginStageToGeo(const int stage)
{
	switch (stage)
	{
	case 2:
		return geoalgo::MeshSurfaceReconstructStage::Partition;
	case 3:
		return geoalgo::MeshSurfaceReconstructStage::Sample;
	case 4:
		return geoalgo::MeshSurfaceReconstructStage::Fit;
	case 5:
		return geoalgo::MeshSurfaceReconstructStage::BoundaryBlend;
	case 6:
		return geoalgo::MeshSurfaceReconstructStage::JunctionBlend;
	case 7:
		return geoalgo::MeshSurfaceReconstructStage::Fair;
	case 8:
		return geoalgo::MeshSurfaceReconstructStage::Assemble;
	default:
		return geoalgo::MeshSurfaceReconstructStage::None;
	}
}
} // namespace

HeadlessPointCloudBridge::HeadlessPointCloudBridge(DocumentHost& host) : m_host(host) {}

HeadlessPointCloudBridge::~HeadlessPointCloudBridge() = default;

QJsonObject HeadlessPointCloudBridge::infoJson(const QString& backendId) const
{
	PluginPointCloudInfo info;
	if (!document_point_cloud_ops::queryPointCloudInfo(&m_host, backendId.toStdString(), info))
		return fail(QStringLiteral("not a point cloud or unknown id"));
	return okExtra({{QStringLiteral("info"), infoToJson(info)}, {QStringLiteral("backendId"), backendId}});
}

QJsonObject HeadlessPointCloudBridge::measureJson(const QString& backendId) const
{
	PluginPointCloudMeasure m;
	if (!document_point_cloud_ops::measurePointCloud(&m_host, backendId.toStdString(), m))
		return fail(QStringLiteral("measure failed"));
	QJsonObject meas;
	meas.insert(QStringLiteral("centroidMm"), vec3ToJson(m.centroidMm));
	meas.insert(QStringLiteral("averageSpacingMm"), m.averageSpacingMm);
	meas.insert(QStringLiteral("bounds"), boundsToJson(m.bounds));
	return okExtra({{QStringLiteral("measure"), meas}, {QStringLiteral("backendId"), backendId}});
}

bool HeadlessPointCloudBridge::previewSoup(const QString& backendId, std::size_t maxPoints, std::vector<float>& outXyz,
										 QString* err) const
{
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(&m_host, backendId.toStdString(), &resolveErr);
	if (!pc)
	{
		if (err)
			*err = QString::fromStdString(resolveErr);
		return false;
	}
	// 与 /api/mesh 一致：回传存储系坐标，前端用 worldMatrix 定位（避免再乘一遍）
	const std::vector<float>& local = pc->pointPositionsXyz();
	if (local.empty())
	{
		if (err)
			*err = QStringLiteral("empty point cloud buffers");
		return false;
	}
	if (maxPoints == 0U)
		maxPoints = kDefaultPreviewMaxPoints;
	uniformSampleXyz(local, maxPoints, outXyz);
	return !outXyz.empty();
}

bool HeadlessPointCloudBridge::chunkSoup(const QString& backendId, int lod, int index, std::size_t maxPoints,
										 std::vector<float>& outXyz, QJsonObject* outMeta, QString* err) const
{
	if (lod != 0)
	{
		if (err)
			*err = QStringLiteral("only lod 0 supported");
		return false;
	}
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(&m_host, backendId.toStdString(), &resolveErr);
	if (!pc)
	{
		if (err)
			*err = QString::fromStdString(resolveErr);
		return false;
	}
	const std::vector<float>& world = pc->pointPositionsXyz();
	const std::size_t n = world.size() / 3U;
	if (n == 0U)
	{
		if (err)
			*err = QStringLiteral("empty point cloud");
		return false;
	}
	const std::size_t chunkPts = maxPoints > 0U ? maxPoints : kChunkPointCount;
	const std::size_t chunkCount = (n + chunkPts - 1U) / chunkPts;
	if (index < 0 || static_cast<std::size_t>(index) >= chunkCount)
	{
		if (err)
			*err = QStringLiteral("chunk index out of range");
		return false;
	}
	const std::size_t begin = static_cast<std::size_t>(index) * chunkPts;
	const std::size_t end = std::min(n, begin + chunkPts);
	outXyz.clear();
	outXyz.reserve((end - begin) * 3U);
	for (std::size_t p = begin; p < end; ++p)
	{
		const std::size_t b = p * 3U;
		outXyz.push_back(world[b]);
		outXyz.push_back(world[b + 1U]);
		outXyz.push_back(world[b + 2U]);
	}
	if (outMeta)
	{
		outMeta->insert(QStringLiteral("lod"), lod);
		outMeta->insert(QStringLiteral("index"), index);
		outMeta->insert(QStringLiteral("chunkCount"), static_cast<qint64>(chunkCount));
		outMeta->insert(QStringLiteral("pointCount"), static_cast<qint64>(end - begin));
		outMeta->insert(QStringLiteral("totalPoints"), static_cast<qint64>(n));
	}
	return true;
}

QJsonObject HeadlessPointCloudBridge::downsample(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	const QString mode = body.value(QStringLiteral("mode")).toString(QStringLiteral("voxel"));
	QJsonObject out;
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	if (mode == QStringLiteral("random"))
	{
		const double frac = body.value(QStringLiteral("retainedFraction")).toDouble(0.5);
		if (!mutatePointCloud(
				m_host, id,
				[frac](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::downsamplePointCloudRandom(data, frac, err); }, &out))
			return out;
		return out;
	}
	const double voxel = body.value(QStringLiteral("voxelSizeMm")).toDouble(2.0);
	const unsigned int minCell = static_cast<unsigned int>(body.value(QStringLiteral("minPointsPerCell")).toInt(1));
	if (!mutatePointCloud(
			m_host, id,
			[voxel, minCell](PointCloudBackendData& data, std::string* err)
			{ return point_cloud_backend_ops::downsamplePointCloudVoxel(data, voxel, minCell, err); }, &out))
		return out;
	return out;
}

QJsonObject HeadlessPointCloudBridge::crop(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	const QString mode = body.value(QStringLiteral("mode")).toString();
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	QJsonObject out;
	if (mode == QStringLiteral("box"))
	{
		const PluginAxisAlignedBox box = boxFromJson(body.value(QStringLiteral("box")).toObject());
		if (!mutatePointCloud(
				m_host, id,
				[box](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::cropPointCloudByBox(data, eigenBoxFromPlugin(box), err); }, &out))
			return out;
		return out;
	}
	if (mode == QStringLiteral("sphere"))
	{
		const QJsonArray c = body.value(QStringLiteral("centerMm")).toArray();
		const double radius = body.value(QStringLiteral("radiusMm")).toDouble(10.0);
		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		if (c.size() >= 3)
			center = Eigen::Vector3d(c.at(0).toDouble(), c.at(1).toDouble(), c.at(2).toDouble());
		if (!mutatePointCloud(
				m_host, id,
				[center, radius](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::cropPointCloudBySphere(data, center, radius, err); }, &out))
			return out;
		return out;
	}
	if (mode == QStringLiteral("polyline"))
	{
		std::vector<float> xy;
		for (const QJsonValue& v : body.value(QStringLiteral("polylineScreenXy")).toArray())
			xy.push_back(static_cast<float>(v.toDouble()));
		double mvp[16];
		double mtw[16];
		fillMat4FromJson(body.value(QStringLiteral("mvpMatrix")).toArray(), mvp);
		fillMat4FromJson(body.value(QStringLiteral("modelToWorld")).toArray(), mtw);
		const int vw = body.value(QStringLiteral("viewportWidth")).toInt();
		const int vh = body.value(QStringLiteral("viewportHeight")).toInt();
		const bool keepInside = body.value(QStringLiteral("keepInside")).toBool(true);
		if (!mutatePointCloud(
				m_host, id,
				[&](PointCloudBackendData& data, std::string* err)
				{
					return point_cloud_backend_ops::cropPointCloudByPolyline2D(data, xy, mvp, mtw, vw, vh, keepInside,
																			 err);
				},
				&out))
			return out;
		return out;
	}
	return fail(QStringLiteral("crop mode must be box, sphere, or polyline"));
}

QJsonObject HeadlessPointCloudBridge::preprocess(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	const QString op = body.value(QStringLiteral("op")).toString();
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	QJsonObject out;
	if (op == QStringLiteral("outliers"))
	{
		const double pct = body.value(QStringLiteral("removalPercent")).toDouble(5.0);
		const unsigned int k = static_cast<unsigned int>(body.value(QStringLiteral("kNeighbors")).toInt(24));
		if (!mutatePointCloud(
				m_host, id,
				[pct, k](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::removePointCloudOutliers(data, pct, k, err); }, &out))
			return out;
		return out;
	}
	if (op == QStringLiteral("bilateral"))
	{
		if (!mutatePointCloud(
				m_host, id,
				[](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::smoothPointCloudBilateral(data, err); }, &out))
			return out;
		return out;
	}
	if (op == QStringLiteral("normalsPca"))
	{
		const unsigned int k = static_cast<unsigned int>(body.value(QStringLiteral("kNeighbors")).toInt(12));
		if (!mutatePointCloud(
				m_host, id,
				[k](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::estimatePointCloudNormalsPca(data, k, err); }, &out))
			return out;
		return out;
	}
	if (op == QStringLiteral("normalsMst"))
	{
		const unsigned int k = static_cast<unsigned int>(body.value(QStringLiteral("kNeighbors")).toInt(12));
		if (!mutatePointCloud(
				m_host, id,
				[k](PointCloudBackendData& data, std::string* err)
				{ return point_cloud_backend_ops::orientPointCloudNormalsMst(data, k, err); }, &out))
			return out;
		return out;
	}
	return fail(QStringLiteral("unknown preprocess op"));
}

QJsonObject HeadlessPointCloudBridge::registerCloud(const QJsonObject& body)
{
	const QString sourceId = backendIdFromJson(body, "sourceId");
	const QString targetId = body.value(QStringLiteral("targetId")).toString();
	const QString method = body.value(QStringLiteral("method")).toString(QStringLiteral("icp"));
	if (sourceId.isEmpty() || targetId.isEmpty())
		return fail(QStringLiteral("sourceId and targetId required"));

	std::string resolveErr;
	const auto source = document_point_cloud_ops::resolvePointCloud(&m_host, sourceId.toStdString(), &resolveErr);
	if (!source)
		return fail(QString::fromStdString(resolveErr));

	if (method == QStringLiteral("icp"))
	{
		const auto target = document_point_cloud_ops::resolvePointCloud(&m_host, targetId.toStdString(), &resolveErr);
		if (!target)
			return fail(QString::fromStdString(resolveErr));
		const int maxIter = body.value(QStringLiteral("maxIterations")).toInt(40);
		const double conv = body.value(QStringLiteral("convergenceTransMm")).toDouble(0.01);
		const double maxPair = body.value(QStringLiteral("maxPairDistanceMm")).toDouble(0.0);
		const std::size_t maxPts = static_cast<std::size_t>(body.value(QStringLiteral("icpMaxPoints")).toInt(4000));
		const bool apply = body.value(QStringLiteral("applyTransformToSource")).toBool(true);
		point_cloud_backend_ops::PointCloudIcpResult icp;
		std::string err;
		if (!point_cloud_backend_ops::rigidRegisterPointCloudsIcp(*source, *target, icp, maxIter, conv, maxPair,
																  maxPts, &err))
			return fail(QString::fromStdString(err));
		if (apply)
		{
			point_cloud_backend_ops::applyRigidTransformToPointCloud(*source, icp.sourceToTarget, &err);
			document_point_cloud_ops::commitPointCloudVisual(&m_host, *source);
		}
		return okExtra({{QStringLiteral("rmseMm"), icp.rmseMm},
						{QStringLiteral("pointCountAfter"), static_cast<qint64>(source->geometryElementCount())},
						{QStringLiteral("backendId"), sourceId}});
	}

	if (method == QStringLiteral("spare"))
	{
		const auto targetPc = document_point_cloud_ops::resolvePointCloud(&m_host, targetId.toStdString(), &resolveErr);
		const auto targetMesh = document_point_cloud_ops::resolveMesh(&m_host, targetId.toStdString(), nullptr);
		if (!targetPc && !targetMesh)
			return fail(QStringLiteral("SPARE target must be point cloud or mesh"));
		point_cloud_backend_ops::PointCloudSpareParams sp;
		sp.sampleRadiusRatio = body.value(QStringLiteral("sampleRadiusRatio")).toDouble(0.0);
		sp.wSmo = body.value(QStringLiteral("wSmo")).toDouble(0.01);
		sp.wRot = body.value(QStringLiteral("wRot")).toDouble(1e-4);
		sp.wArapCoarse = body.value(QStringLiteral("wArapCoarse")).toDouble(500.0);
		sp.wArapFine = body.value(QStringLiteral("wArapFine")).toDouble(200.0);
		sp.useSymmetricPointToPlane = body.value(QStringLiteral("useSymmetricPointToPlane")).toBool(true);
		sp.useCoarseReg = body.value(QStringLiteral("useCoarseReg")).toBool(true);
		sp.useFineReg = body.value(QStringLiteral("useFineReg")).toBool(true);
		sp.normalizeScale = body.value(QStringLiteral("normalizeScale")).toBool(true);
		sp.rigidPreAlign = body.value(QStringLiteral("rigidPreAlign")).toBool(false);
		sp.coarseGlobalAlign = body.value(QStringLiteral("coarseGlobalAlign")).toBool(false);
		sp.voxelPrefilterMm = body.value(QStringLiteral("voxelPrefilterMm")).toDouble(0.0);
		sp.maxOuterIters = body.value(QStringLiteral("maxOuterIters")).toInt(30);
		point_cloud_backend_ops::PointCloudSpareResult result;
		std::string err;
		const bool ok = targetPc ? point_cloud_backend_ops::nonRigidRegisterPointCloudsSpare(*source, *targetPc, result,
																							 sp, &err)
							   : point_cloud_backend_ops::nonRigidRegisterPointCloudToMeshSpare(*source, *targetMesh,
																								result, sp, &err);
		if (!ok)
			return fail(QString::fromStdString(err));
		document_point_cloud_ops::commitPointCloudVisual(&m_host, *source);
		return okExtra({{QStringLiteral("meanErrorMm"), result.meanErrorMm},
						{QStringLiteral("spareDeformationNodeCount"), result.deformationNodeCount},
						{QStringLiteral("pointCountAfter"), static_cast<qint64>(source->geometryElementCount())},
						{QStringLiteral("backendId"), sourceId}});
	}

	if (method == QStringLiteral("sdf"))
	{
		const auto targetPc = document_point_cloud_ops::resolvePointCloud(&m_host, targetId.toStdString(), &resolveErr);
		const auto targetMesh = document_point_cloud_ops::resolveMesh(&m_host, targetId.toStdString(), nullptr);
		if (!targetPc && !targetMesh)
			return fail(QStringLiteral("SDF target must be point cloud or mesh"));
		point_cloud_backend_ops::PointCloudSdfParams sp;
		sp.fieldMode = body.value(QStringLiteral("fieldMode")).toInt(1);
		sp.fieldVoxelMm = body.value(QStringLiteral("fieldVoxelMm")).toDouble(0.0);
		sp.fineDataTerm = body.value(QStringLiteral("fineDataTerm")).toInt(0);
		sp.useCoarseReg = body.value(QStringLiteral("useCoarseReg")).toBool(true);
		sp.useFineReg = body.value(QStringLiteral("useFineReg")).toBool(true);
		sp.sampleRadiusRatio = body.value(QStringLiteral("sampleRadiusRatio")).toDouble(0.0);
		sp.wSmo = body.value(QStringLiteral("wSmo")).toDouble(1.0);
		sp.wRot = body.value(QStringLiteral("wRot")).toDouble(1e-4);
		sp.wArapCoarse = body.value(QStringLiteral("wArapCoarse")).toDouble(500.0);
		sp.wArapFine = body.value(QStringLiteral("wArapFine")).toDouble(200.0);
		sp.normalizeScale = body.value(QStringLiteral("normalizeScale")).toBool(true);
		sp.rigidPreAlign = body.value(QStringLiteral("rigidPreAlign")).toBool(true);
		sp.voxelPrefilterMm = body.value(QStringLiteral("voxelPrefilterMm")).toDouble(0.0);
		sp.maxOuterIters = body.value(QStringLiteral("maxOuterIters")).toInt(30);
		point_cloud_backend_ops::PointCloudSdfResult result;
		std::string err;
		const bool ok = targetPc ? point_cloud_backend_ops::nonRigidRegisterPointCloudsSdf(*source, *targetPc, result,
																						   sp, &err)
							   : point_cloud_backend_ops::nonRigidRegisterPointCloudToMeshSdf(*source, *targetMesh,
																							  result, sp, &err);
		if (!ok)
			return fail(QString::fromStdString(err));
		document_point_cloud_ops::commitPointCloudVisual(&m_host, *source);
		QJsonObject extra{{QStringLiteral("meanErrorMm"), result.meanErrorMm},
						  {QStringLiteral("spareDeformationNodeCount"), result.deformationNodeCount},
						  {QStringLiteral("fieldVoxelMmUsed"), result.fieldVoxelMmUsed},
						  {QStringLiteral("pointCountAfter"), static_cast<qint64>(source->geometryElementCount())},
						  {QStringLiteral("backendId"), sourceId}};
		if (!result.debugSummary.empty())
			extra.insert(QStringLiteral("debugReport"), QString::fromStdString(result.debugSummary));
		return okExtra(extra);
	}

	return fail(QStringLiteral("register method must be icp, spare, or sdf"));
}

QJsonObject HeadlessPointCloudBridge::reconstruct(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	const QString method = body.value(QStringLiteral("method")).toString(QStringLiteral("poissonAuto"));
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(&m_host, id.toStdString(), &resolveErr);
	if (!pc)
		return fail(QString::fromStdString(resolveErr));

	MeshBackendData meshOut;
	std::string err;
	PluginMeshCreateOptions meshOpt = meshOptionsFromJson(body.value(QStringLiteral("meshOptions")).toObject());
	if (meshOpt.displayName.isEmpty())
		meshOpt.displayName = QStringLiteral("ReconstructedMesh");

	bool ok = false;
	if (method == QStringLiteral("scaleSpace"))
	{
		const std::size_t iters = static_cast<std::size_t>(body.value(QStringLiteral("smoothIterations")).toInt(4));
		const double radius = body.value(QStringLiteral("meshingRadiusMm")).toDouble(0.0);
		ok = point_cloud_backend_ops::reconstructMeshScaleSpace(*pc, meshOut, iters, radius, &err);
	}
	else
	{
		const double voxel = body.value(QStringLiteral("voxelPrefilterMm")).toDouble(1.0);
		ok = point_cloud_backend_ops::reconstructMeshFromPointCloudPoisson(*pc, meshOut, voxel, &err);
	}
	if (!ok)
		return fail(QString::fromStdString(err));

	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setTriangleSoup(meshOut.triangleSoup());
	const std::string newId =
		document_point_cloud_ops::registerReconstructedMesh(&m_host, nullptr, meshPtr, meshOpt, &err);
	if (newId.empty())
		return fail(QString::fromStdString(err));
	return okExtra({{QStringLiteral("newBackendId"), QString::fromStdString(newId)},
					{QStringLiteral("sourceBackendId"), id}});
}

QJsonObject HeadlessPointCloudBridge::meshPost(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	const QString op = body.value(QStringLiteral("op")).toString();
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(&m_host, id.toStdString(), &resolveErr);
	if (!mesh)
		return fail(QString::fromStdString(resolveErr));

	const std::vector<float> soupIn = mesh->triangleSoup();
	std::vector<float> soupOut;
	std::string err;
	PluginMeshCreateOptions meshOpt = meshOptionsFromJson(body.value(QStringLiteral("resultOptions")).toObject());
	if (meshOpt.displayName.isEmpty())
		meshOpt.displayName = QString::fromStdString(mesh->name()) + QStringLiteral("_post");

	point_cloud_backend_ops::MeshRepairStatistics repairStats;
	bool ok = false;
	if (op == QStringLiteral("simplify"))
	{
		ok = point_cloud_backend_ops::simplifyMesh(soupIn, soupOut, body.value(QStringLiteral("targetFaceCount")).toInt(0),
												   body.value(QStringLiteral("qualityThreshold")).toDouble(0.3), &err);
	}
	else if (op == QStringLiteral("smooth"))
	{
		point_cloud_backend_ops::MeshSmoothRequest req;
		req.iterations = body.value(QStringLiteral("iterations")).toInt(3);
		req.lambda = body.value(QStringLiteral("lambda")).toDouble(0.2);
		req.useTaubin = body.value(QStringLiteral("useTaubinSmooth")).toBool(false);
		req.preserveBoundary = body.value(QStringLiteral("preserveBoundary")).toBool(true);
		req.cotangentWeight = body.value(QStringLiteral("cotangentWeight")).toBool(true);
		req.repairBeforeSmooth = body.value(QStringLiteral("repairBeforeSmooth")).toBool(false);
		req.repairParams = repairFromJson(body.value(QStringLiteral("repairParams")).toObject());
		ok = point_cloud_backend_ops::smoothMesh(soupIn, soupOut, req, &repairStats, &err);
	}
	else if (op == QStringLiteral("repair"))
		ok = point_cloud_backend_ops::repairMesh(soupIn, soupOut, repairFromJson(body), &repairStats, &err);
	else if (op == QStringLiteral("remesh"))
		ok = point_cloud_backend_ops::remeshMeshIsotropic(soupIn, soupOut,
														  body.value(QStringLiteral("targetEdgeLengthMm")).toDouble(2.0),
														  body.value(QStringLiteral("iterations")).toInt(3), &err);
	else
		return fail(QStringLiteral("mesh op must be simplify, smooth, repair, or remesh"));

	if (!ok)
		return fail(QString::fromStdString(err));

	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setTriangleSoup(std::move(soupOut));
	const std::string newId =
		document_point_cloud_ops::registerReconstructedMesh(&m_host, nullptr, meshPtr, meshOpt, &err);
	if (newId.empty())
		return fail(QString::fromStdString(err));

	QJsonObject extra{{QStringLiteral("newBackendId"), QString::fromStdString(newId)},
					  {QStringLiteral("sourceBackendId"), id}};
	if (op == QStringLiteral("repair") || op == QStringLiteral("smooth"))
	{
		extra.insert(QStringLiteral("inputFaceCount"), repairStats.inputFaceCount);
		extra.insert(QStringLiteral("outputFaceCount"), repairStats.outputFaceCount);
	}
	return okExtra(extra);
}

QJsonObject HeadlessPointCloudBridge::meshExportPly(const QJsonObject& body)
{
	const QString id = backendIdFromJson(body);
	if (id.isEmpty())
		return fail(QStringLiteral("backendId required"));
	QString path = body.value(QStringLiteral("path")).toString();
	if (path.isEmpty())
	{
		QTemporaryFile tmp(QDir::tempPath() + QStringLiteral("/cloudsim_mesh_XXXXXX.ply"));
		tmp.setAutoRemove(false);
		if (!tmp.open())
			return fail(QStringLiteral("cannot create temp ply path"));
		path = tmp.fileName();
		tmp.close();
	}
	std::string err;
	if (!document_point_cloud_ops::exportMeshToPly(&m_host, id.toStdString(), QFile::encodeName(path).constData(), &err))
		return fail(QString::fromStdString(err));
	return okExtra({{QStringLiteral("path"), path}, {QStringLiteral("backendId"), id}});
}

QJsonObject HeadlessPointCloudBridge::surfaceRun(const QJsonObject& body)
{
	const QString mode = body.value(QStringLiteral("mode")).toString(QStringLiteral("full"));
	const geoalgo::MeshSurfaceReconstructParams geoParams =
		surfaceParamsFromJson(body.value(QStringLiteral("params")).toObject());

	if (mode == QStringLiteral("stage"))
	{
		const int stage = parseSurfaceStage(body);
		if (stage < 1 || stage > 8)
			return fail(QStringLiteral("stage required (1..8 or name)"));

		if (stage == 1)
		{
			const QString meshId = backendIdFromJson(body);
			if (meshId.isEmpty())
				return fail(QStringLiteral("backendId required to begin session"));
			std::string resolveErr;
			const auto mesh = document_point_cloud_ops::resolveMesh(&m_host, meshId.toStdString(), &resolveErr);
			if (!mesh)
				return fail(QString::fromStdString(resolveErr));
			m_surfaceSession = std::make_unique<SurfaceSession>();
			m_surfaceSession->sessionId = QStringLiteral("sr_web");
			m_surfaceSession->meshBackendId = meshId;
			m_surfaceSession->rawSoup = mesh->triangleSoup();
			m_surfaceSession->lastCompleted = 0;
		}

		if (!m_surfaceSession)
			return fail(QStringLiteral("无曲面重构会话；请先执行预处理"));
		if (stage != m_surfaceSession->lastCompleted + 1)
			return fail(QStringLiteral("请先完成上一阶段"));

		geoalgo::MeshSurfaceReconstructReport report;
		std::string err;
		if (stage == 1)
		{
			if (!geometry_backend_ops::preprocessMeshSoupForSurfaceReconstruct(m_surfaceSession->rawSoup, geoParams,
																			  m_surfaceSession->workingSoup, report,
																			  &err))
				return fail(QString::fromStdString(err));
			m_surfaceSession->geoSession =
				geometry_backend_ops::createMeshSurfaceReconstructSession(m_surfaceSession->workingSoup);
			if (!m_surfaceSession->geoSession)
				return fail(QStringLiteral("createMeshSurfaceReconstructSession failed"));
			m_surfaceSession->lastCompleted = 1;
			return okExtra({{QStringLiteral("sessionId"), m_surfaceSession->sessionId},
							{QStringLiteral("lastCompletedStage"), 1},
							{QStringLiteral("stage"), QStringLiteral("preprocess")},
							{QStringLiteral("sourceBackendId"), m_surfaceSession->meshBackendId}});
		}

		if (!m_surfaceSession->geoSession)
			return fail(QStringLiteral("surface reconstruction session not preprocessed"));

		const geoalgo::MeshSurfaceReconstructStage geoStage = mapPluginStageToGeo(stage);
		geoalgo::ShapeHandle shape;
		geoalgo::ShapeHandle* shapeOut = (stage == 8) ? &shape : nullptr;
		if (!geometry_backend_ops::runMeshSurfaceReconstructStage(*m_surfaceSession->geoSession, geoStage, geoParams,
																  shapeOut, report, &err))
			return fail(QString::fromStdString(err));

		m_surfaceSession->lastCompleted = stage;
		if (stage != 8)
		{
			return okExtra({{QStringLiteral("sessionId"), m_surfaceSession->sessionId},
							{QStringLiteral("lastCompletedStage"), stage},
							{QStringLiteral("patchCount"), report.patchCount},
							{QStringLiteral("sourceBackendId"), m_surfaceSession->meshBackendId}});
		}

		auto brep = std::make_shared<BrepBackendData>();
		if (!geometry_backend_ops::meshSurfaceReconstructShapeToBrep(shape, brep, &err))
			return fail(QString::fromStdString(err));

		std::string resolveErr;
		const auto mesh =
			document_point_cloud_ops::resolveMesh(&m_host, m_surfaceSession->meshBackendId.toStdString(), &resolveErr);
		if (mesh)
			brep->setColor(mesh->color());
		const QString displayName = makeUniqueBrepName(
			m_host, body.value(QStringLiteral("displayName"))
						.toString(mesh && !mesh->name().empty()
									  ? QString::fromStdString(mesh->name()) + QStringLiteral("_brep")
									  : QStringLiteral("ReconstructedBrep")));
		brep->setName(displayName.toStdString());
		geoalgo::clearBrepImportArtifactsCache();
		QString regErr;
		if (!registerAdoptedBrepAndLoadScene(m_host, brep, QStringLiteral("plugin://pointcloud/surface-reconstruct"),
											 QLatin1String(backend_type::kCatalogBrepModel), QString(), false, &regErr))
			return fail(regErr);
		std::string alignErr;
		(void)document_point_cloud_ops::inheritBrepVisualPoseFromSourceMesh(
			&m_host, m_surfaceSession->meshBackendId.toStdString(), brep->id(), *brep, &alignErr);
		const QString srcId = m_surfaceSession->meshBackendId;
		m_surfaceSession.reset();
		return okExtra({{QStringLiteral("newBackendId"), QString::fromStdString(brep->id())},
						{QStringLiteral("lastCompletedStage"), 8},
						{QStringLiteral("patchCount"), report.patchCount},
						{QStringLiteral("maxDeviationMm"), report.maxDeviationMm},
						{QStringLiteral("sourceBackendId"), srcId}});
	}

	const QString meshId = backendIdFromJson(body);
	if (meshId.isEmpty())
		return fail(QStringLiteral("backendId required"));

	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(&m_host, meshId.toStdString(), &resolveErr);
	if (!mesh)
		return fail(QString::fromStdString(resolveErr));

	auto brep = std::make_shared<BrepBackendData>();
	geoalgo::MeshSurfaceReconstructReport report;
	std::string err;
	if (!geometry_backend_ops::reconstructBrepFromMeshSoup(mesh->triangleSoup(), geoParams, brep, report, &err))
		return fail(QString::fromStdString(err));

	brep->setColor(mesh->color());
	const QString displayName = makeUniqueBrepName(
		m_host, body.value(QStringLiteral("displayName")).toString(
					mesh->name().empty() ? QStringLiteral("ReconstructedBrep")
										 : QString::fromStdString(mesh->name()) + QStringLiteral("_brep")));
	brep->setName(displayName.toStdString());
	geoalgo::clearBrepImportArtifactsCache();

	QString regErr;
	if (!registerAdoptedBrepAndLoadScene(m_host, brep, QStringLiteral("plugin://pointcloud/surface-reconstruct"),
										 QLatin1String(backend_type::kCatalogBrepModel), QString(), false, &regErr))
		return fail(regErr);

	std::string alignErr;
	if (!document_point_cloud_ops::inheritBrepVisualPoseFromSourceMesh(&m_host, meshId.toStdString(), brep->id(),
																	   *brep, &alignErr))
		return fail(QString::fromStdString(alignErr));

	return okExtra({{QStringLiteral("newBackendId"), QString::fromStdString(brep->id())},
					{QStringLiteral("patchCount"), report.patchCount},
					{QStringLiteral("maxDeviationMm"), report.maxDeviationMm},
					{QStringLiteral("sourceBackendId"), meshId}});
}

QJsonObject HeadlessPointCloudBridge::surfaceReset(const QJsonObject&)
{
	m_surfaceSession.reset();
	return okExtra();
}

QJsonObject HeadlessPointCloudBridge::deprecatedOp(const QJsonObject& body)
{
	const QString op = body.value(QStringLiteral("op")).toString();
	if (op == QStringLiteral("downsample") || op == QStringLiteral("voxel"))
	{
		QJsonObject b = body;
		if (!b.contains(QStringLiteral("mode")))
			b.insert(QStringLiteral("mode"), QStringLiteral("voxel"));
		return downsample(b);
	}
	if (op == QStringLiteral("crop"))
		return crop(body);
	if (op == QStringLiteral("normals") || op == QStringLiteral("preprocess"))
	{
		QJsonObject b = body;
		if (!b.contains(QStringLiteral("op")))
			b.insert(QStringLiteral("op"), QStringLiteral("normalsPca"));
		return preprocess(b);
	}
	if (op == QStringLiteral("icp"))
	{
		QJsonObject b = body;
		b.insert(QStringLiteral("method"), QStringLiteral("icp"));
		b.insert(QStringLiteral("sourceId"), backendIdFromJson(body));
		if (!b.contains(QStringLiteral("targetId")))
			b.insert(QStringLiteral("targetId"), body.value(QStringLiteral("targetBackendId")));
		return registerCloud(b);
	}
	return fail(QStringLiteral("unknown deprecated op; use typed /api/pointcloud/* routes"));
}

} // namespace cloudsim::host
