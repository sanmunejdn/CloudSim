#include "DocumentGeometryOps.h"

#include "GeometryBackendOps.h"
#include "DocumentHost.h"
#include "DocumentImportFacade.h"
#include "IPluginMainWindowHost.h"
#include "MeshBackendData.h"

namespace document_geometry_ops
{
namespace
{

geoalgo::MeshDiscretizeMode toGeoMode(const PluginMeshDiscretizeMode mode)
{
	switch (mode)
	{
	case PluginMeshDiscretizeMode::UniformRelative:
		return geoalgo::MeshDiscretizeMode::UniformRelative;
	case PluginMeshDiscretizeMode::UVStructuredGrid:
		return geoalgo::MeshDiscretizeMode::UVStructuredGrid;
	case PluginMeshDiscretizeMode::WireTubeMesh:
		return geoalgo::MeshDiscretizeMode::WireTubeMesh;
	case PluginMeshDiscretizeMode::WireRibbonMesh:
		return geoalgo::MeshDiscretizeMode::WireRibbonMesh;
	case PluginMeshDiscretizeMode::AdaptiveTriangulation:
	default:
		return geoalgo::MeshDiscretizeMode::AdaptiveTriangulation;
	}
}

geoalgo::MeshQualityPreset toGeoQuality(const PluginMeshQualityPreset quality)
{
	switch (quality)
	{
	case PluginMeshQualityPreset::Coarse:
		return geoalgo::MeshQualityPreset::Coarse;
	case PluginMeshQualityPreset::Fine:
		return geoalgo::MeshQualityPreset::Fine;
	case PluginMeshQualityPreset::Custom:
		return geoalgo::MeshQualityPreset::Custom;
	case PluginMeshQualityPreset::Medium:
	default:
		return geoalgo::MeshQualityPreset::Medium;
	}
}

geoalgo::MeshDensityControl toGeoDensityControl(const PluginMeshDensityControl control)
{
	switch (control)
	{
	case PluginMeshDensityControl::TargetEdgeLength:
		return geoalgo::MeshDensityControl::TargetEdgeLength;
	case PluginMeshDensityControl::TargetTriangleCount:
		return geoalgo::MeshDensityControl::TargetTriangleCount;
	case PluginMeshDensityControl::QualityPreset:
	default:
		return geoalgo::MeshDensityControl::QualityPreset;
	}
}

} // namespace

geoalgo::MeshDiscretizeParams toGeoMeshParams(const PluginMeshDiscretizeParams& params)
{
	geoalgo::MeshDiscretizeParams out;
	out.mode = toGeoMode(params.mode);
	out.densityControl = toGeoDensityControl(params.densityControl);
	out.targetEdgeLengthMm = params.targetEdgeLengthMm;
	out.targetTriangleCount = params.targetTriangleCount;
	out.quality = toGeoQuality(params.quality);
	out.tessellate.linearDeflectionMm = params.linearDeflectionMm;
	out.tessellate.linearDeflectionRelative = params.linearDeflectionRelative;
	out.tessellate.angularDeflectionDeg = params.angularDeflectionDeg;
	out.uvGridCountU = params.uvGridCountU;
	out.uvGridCountV = params.uvGridCountV;
	out.tubeRadiusMm = params.tubeRadiusMm;
	out.tubeSides = params.tubeSides;
	out.ribbonWidthMm = params.ribbonWidthMm;
	if (out.densityControl != geoalgo::MeshDensityControl::QualityPreset)
	{
		out.quality = geoalgo::MeshQualityPreset::Custom;
	}
	else
	{
		geometry_backend_ops::applyQualityPreset(out);
	}
	return out;
}

geoalgo::IntersectionParams toGeoIntersectionParams(const PluginGeometryIntersectionParams& params)
{
	geoalgo::IntersectionParams out;
	out.toleranceMm = params.toleranceMm;
	out.discretizeCurves = params.discretizeCurves;
	out.curveDisc.linearDeflectionMm = params.curveLinearDeflectionMm;
	return out;
}

geoalgo::BrepBooleanOp toGeoBrepBooleanOp(const PluginBrepBooleanOp op)
{
	switch (op)
	{
	case PluginBrepBooleanOp::Common:
		return geoalgo::BrepBooleanOp::Common;
	case PluginBrepBooleanOp::Cut:
		return geoalgo::BrepBooleanOp::Cut;
	case PluginBrepBooleanOp::Fuse:
	default:
		return geoalgo::BrepBooleanOp::Fuse;
	}
}

PluginGeometryJobResult toPluginGeometryResult(const geoalgo::IntersectionResult& result)
{
	PluginGeometryJobResult out;
	out.maxResidualMm = result.maxResidualMm;
	out.intersectionPoints.reserve(result.points.size());
	for (const geoalgo::IntersectionHit& hit : result.points)
	{
		PluginPoint3d p;
		p.x = hit.positionMm.x;
		p.y = hit.positionMm.y;
		p.z = hit.positionMm.z;
		out.intersectionPoints.push_back(p);
	}
	out.polylines.reserve(result.curves.size());
	for (const geoalgo::Polyline3d& curve : result.curves)
	{
		out.polylines.push_back(curve.xyz);
	}
	return out;
}

PluginGeometryJobResult toPluginGeometryResult(
	const geoalgo::MeshDiscretizeReport& report,
	const std::string& backendId)
{
	PluginGeometryJobResult out;
	out.newBackendId = backendId;
	out.triangleCount = report.triangleCount;
	out.avgEdgeLengthMm = report.avgEdgeLengthMm;
	return out;
}

std::string registerMeshSoup(
	cloudsim::host::DocumentHost* page,
	IPluginMainWindowHost* mainWindowHost,
	std::vector<float> soup,
	const PluginMeshCreateOptions& options,
	std::string* outError)
{
	if (!page || soup.empty())
	{
		if (outError)
		{
			*outError = "invalid document or empty soup";
		}
		return std::string();
	}
	auto mesh = std::make_shared<MeshBackendData>();
	const QString displayName =
		options.displayName.isEmpty() ? QStringLiteral("GeometryMesh") : options.displayName;
	mesh->setName(displayName.toStdString());

	BackendVec3 pos;
	pos.x = options.poseMm.x;
	pos.y = options.poseMm.y;
	pos.z = options.poseMm.z;
	mesh->setPose(pos);

	BackendVec3 rot;
	rot.x = options.rotationDeg.x;
	rot.y = options.rotationDeg.y;
	rot.z = options.rotationDeg.z;
	mesh->setRotation(rot);
	mesh->setTriangleSoup(std::move(soup));

	cloudsim::host::AdoptMeshOptions adoptOpt;
	adoptOpt.sourcePath =
		options.sourcePath.isEmpty() ? QStringLiteral("plugin://geometry") : options.sourcePath;
	adoptOpt.catalogTypeName = QStringLiteral("Model");
	adoptOpt.resetViewToHome = options.resetViewToHome;
	QString regErr;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedMesh(*page, mesh, adoptOpt, &regErr);
	if (!adopted.ok)
	{
		if (outError)
		{
			*outError = regErr.toStdString();
		}
		return std::string();
	}
	if (options.selectInTree && mainWindowHost)
	{
		mainWindowHost->focusBackendInTreeAfterImport(adopted.backendId);
	}
	return adopted.backendId.toStdString();
}

} // namespace document_geometry_ops
