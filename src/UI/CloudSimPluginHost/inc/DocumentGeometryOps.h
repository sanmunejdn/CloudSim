#pragma once

#include "GeometryBackendOps.h"
#include "PluginGeometryTypes.h"

#include <memory>
#include <string>

namespace cloudsim::host {
class DocumentHost;
}

class IPluginMainWindowHost;

namespace document_geometry_ops
{

geoalgo::MeshDiscretizeParams toGeoMeshParams(const PluginMeshDiscretizeParams& params);
geoalgo::IntersectionParams toGeoIntersectionParams(const PluginGeometryIntersectionParams& params);
geoalgo::BrepBooleanOp toGeoBrepBooleanOp(PluginBrepBooleanOp op);

PluginGeometryJobResult toPluginGeometryResult(const geoalgo::IntersectionResult& result);
PluginGeometryJobResult toPluginGeometryResult(
	const geoalgo::MeshDiscretizeReport& report,
	const std::string& backendId);

std::string registerMeshSoup(
	cloudsim::host::DocumentHost* page,
	IPluginMainWindowHost* mainWindowHost,
	std::vector<float> soup,
	const PluginMeshCreateOptions& options,
	std::string* outError = nullptr);

} // namespace document_geometry_ops
