/// @file HeadlessGeometryBridge.cpp

#include "headless/HeadlessGeometryBridge.h"

#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentGeometryOps.h"
#include "DocumentHost.h"
#include "GeometryBackendOps.h"
#include "PluginGeometryTypes.h"

#include <QJsonArray>
#include <QJsonObject>

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}

QJsonObject okMeshResult(const PluginGeometryJobResult& result)
{
	QJsonObject o{{QStringLiteral("ok"), true}};
	if (!result.newBackendId.empty())
		o.insert(QStringLiteral("backendId"), QString::fromStdString(result.newBackendId));
	o.insert(QStringLiteral("triangleCount"), static_cast<qint64>(result.triangleCount));
	o.insert(QStringLiteral("avgEdgeLengthMm"), result.avgEdgeLengthMm);
	return o;
}

QJsonObject okIntersectResult(const PluginGeometryJobResult& result)
{
	QJsonObject o{{QStringLiteral("ok"), true}};
	o.insert(QStringLiteral("maxResidualMm"), result.maxResidualMm);
	QJsonArray points;
	for (const PluginPoint3d& p : result.intersectionPoints)
		points.append(QJsonArray{p.x, p.y, p.z});
	o.insert(QStringLiteral("intersectionPoints"), points);
	QJsonArray polylines;
	for (const std::vector<float>& pl : result.polylines)
	{
		QJsonArray flat;
		for (float v : pl)
			flat.append(static_cast<double>(v));
		polylines.append(flat);
	}
	o.insert(QStringLiteral("polylines"), polylines);
	return o;
}

std::string resolveStepPath(DocumentHost& host, const QJsonObject& body, QString* err)
{
	const QString backendId = body.value(QStringLiteral("backendId")).toString();
	if (!backendId.isEmpty())
	{
		const QString src = host.backendSourcePath().value(backendId);
		if (!src.isEmpty())
			return src.toUtf8().constData();
		if (err)
			*err = QStringLiteral("Backend has no source path.");
		return {};
	}
	const QString stepPath = body.value(QStringLiteral("stepPath")).toString();
	if (stepPath.isEmpty())
	{
		if (err)
			*err = QStringLiteral("backendId or stepPath required.");
		return {};
	}
	return stepPath.toUtf8().constData();
}

PluginMeshDiscretizeParams meshParamsFromJson(const QJsonObject& body)
{
	PluginMeshDiscretizeParams p;
	if (body.contains(QStringLiteral("targetEdgeLengthMm")))
	{
		p.densityControl = PluginMeshDensityControl::TargetEdgeLength;
		p.targetEdgeLengthMm = body.value(QStringLiteral("targetEdgeLengthMm")).toDouble(2.0);
	}
	if (body.contains(QStringLiteral("targetTriangleCount")))
	{
		p.densityControl = PluginMeshDensityControl::TargetTriangleCount;
		p.targetTriangleCount =
			static_cast<std::size_t>(body.value(QStringLiteral("targetTriangleCount")).toInt(5000));
	}
	return p;
}

PluginGeometryIntersectionParams ixParamsFromJson(const QJsonObject& body)
{
	PluginGeometryIntersectionParams p;
	p.toleranceMm = body.value(QStringLiteral("toleranceMm")).toDouble(0.01);
	p.discretizeCurves = body.value(QStringLiteral("discretizeCurves")).toBool(true);
	return p;
}

} // namespace

HeadlessGeometryBridge::HeadlessGeometryBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessGeometryBridge::discretize(const QJsonObject& body)
{
	QString pathErr;
	const std::string stepPath = resolveStepPath(m_host, body, &pathErr);
	if (stepPath.empty())
		return fail(pathErr);

	const PluginMeshDiscretizeParams params = meshParamsFromJson(body);
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(params);
	std::vector<float> soup;
	geoalgo::MeshDiscretizeReport report;
	std::string geoErr;
	if (!geometry_backend_ops::discretizeStepToMesh(stepPath, geoParams, soup, report, &geoErr))
		return fail(QString::fromStdString(geoErr));

	PluginMeshCreateOptions opt;
	opt.displayName = body.value(QStringLiteral("displayName")).toString(QStringLiteral("GeometryMesh"));
	opt.selectInTree = false;
	opt.resetViewToHome = false;
	std::string regErr;
	const std::string backendId =
		document_geometry_ops::registerMeshSoup(&m_host, nullptr, std::move(soup), opt, &regErr);
	if (backendId.empty())
		return fail(QString::fromStdString(regErr));

	return okMeshResult(document_geometry_ops::toPluginGeometryResult(report, backendId));
}

QJsonObject HeadlessGeometryBridge::intersect(const QJsonObject& body)
{
	const QString kind = body.value(QStringLiteral("kind")).toString(QStringLiteral("edgeFace"));
	QString pathErr;
	const std::string stepPath = resolveStepPath(m_host, body, &pathErr);
	if (stepPath.empty())
		return fail(pathErr);

	const PluginGeometryIntersectionParams params = ixParamsFromJson(body);
	const geoalgo::IntersectionParams geoParams = document_geometry_ops::toGeoIntersectionParams(params);
	geoalgo::IntersectionResult ix;
	std::string geoErr;
	bool ok = false;

	if (kind == QStringLiteral("faceFace"))
	{
		const int faceA = body.value(QStringLiteral("faceA")).toInt(0);
		const int faceB = body.value(QStringLiteral("faceB")).toInt(0);
		ok = geometry_backend_ops::intersectStepFaces(stepPath, faceA, faceB, geoParams, ix, &geoErr);
	}
	else if (kind == QStringLiteral("backends"))
	{
		QString toolErr;
		const std::string toolPath = body.value(QStringLiteral("toolStepPath")).toString().toUtf8().constData();
		if (toolPath.empty())
			return fail(QStringLiteral("toolStepPath required."));
		ok = geometry_backend_ops::intersectStepFiles(stepPath, toolPath, geoParams, ix, &geoErr);
		(void)toolErr;
	}
	else
	{
		const int edgeIndex = body.value(QStringLiteral("edgeIndex")).toInt(0);
		const int faceIndex = body.value(QStringLiteral("faceIndex")).toInt(0);
		ok = geometry_backend_ops::intersectStepEdgeFace(stepPath, edgeIndex, faceIndex, geoParams, ix, &geoErr);
	}

	if (!ok)
		return fail(QString::fromStdString(geoErr));
	return okIntersectResult(document_geometry_ops::toPluginGeometryResult(ix));
}

QJsonObject HeadlessGeometryBridge::op(const QJsonObject& body)
{
	QString pathErr;
	const std::string targetPath = resolveStepPath(m_host, body, &pathErr);
	if (targetPath.empty())
		return fail(pathErr);

	const std::string toolPath = body.value(QStringLiteral("toolStepPath")).toString().toUtf8().constData();
	if (toolPath.empty())
		return fail(QStringLiteral("toolStepPath required."));

	const QString opName = body.value(QStringLiteral("op")).toString(QStringLiteral("fuse"));
	PluginBrepBooleanOp brepOp = PluginBrepBooleanOp::Fuse;
	if (opName == QStringLiteral("cut"))
		brepOp = PluginBrepBooleanOp::Cut;
	else if (opName == QStringLiteral("common"))
		brepOp = PluginBrepBooleanOp::Common;

	const PluginMeshDiscretizeParams meshParams = meshParamsFromJson(body);
	const geoalgo::MeshDiscretizeParams geoMesh = document_geometry_ops::toGeoMeshParams(meshParams);
	const geoalgo::BrepBooleanOp geoOp = document_geometry_ops::toGeoBrepBooleanOp(brepOp);

	std::vector<float> soup;
	std::string geoErr;
	if (!geometry_backend_ops::brepBooleanStepFilesToMesh(targetPath, toolPath, geoOp, geoMesh, soup, &geoErr))
		return fail(QString::fromStdString(geoErr));

	PluginMeshCreateOptions opt;
	opt.displayName = body.value(QStringLiteral("displayName")).toString(QStringLiteral("BooleanMesh"));
	opt.selectInTree = false;
	opt.resetViewToHome = false;
	std::string regErr;
	const std::string backendId =
		document_geometry_ops::registerMeshSoup(&m_host, nullptr, std::move(soup), opt, &regErr);
	if (backendId.empty())
		return fail(QString::fromStdString(regErr));

	geoalgo::MeshDiscretizeReport report;
	return okMeshResult(document_geometry_ops::toPluginGeometryResult(report, backendId));
}

} // namespace cloudsim::host
