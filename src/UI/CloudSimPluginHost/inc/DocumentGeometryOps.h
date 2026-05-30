#pragma once

#include "GeometryBackendOps.h"
#include "PluginGeometryTypes.h"

#include <memory>
#include <string>

class DocumentPage;
class MainWindow;

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
	DocumentPage* page,
	MainWindow* mainWindow,
	std::vector<float> soup,
	const PluginMeshCreateOptions& options,
	std::string* outError = nullptr);

} // namespace document_geometry_ops
