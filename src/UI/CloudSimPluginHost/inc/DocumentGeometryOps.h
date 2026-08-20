#ifndef CLOUDSIMPLUGINHOST_DOCUMENTGEOMETRYOPS_H
#define CLOUDSIMPLUGINHOST_DOCUMENTGEOMETRYOPS_H

/// @file DocumentGeometryOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief DocumentGeometryOps 接口

#include "GeometryBackendOps.h"
#include "PluginGeometryTypes.h"

#include <memory>
#include <string>

namespace cloudsim::host
{
class DocumentHost;
}

class IPluginMainWindowHost;

namespace document_geometry_ops
{
geoalgo::MeshDiscretizeParams toGeoMeshParams(const PluginMeshDiscretizeParams& params);
geoalgo::IntersectionParams toGeoIntersectionParams(const PluginGeometryIntersectionParams& params);
geoalgo::BrepBooleanOp toGeoBrepBooleanOp(PluginBrepBooleanOp op);

PluginGeometryJobResult toPluginGeometryResult(const geoalgo::IntersectionResult& result);
PluginGeometryJobResult toPluginGeometryResult(const geoalgo::MeshDiscretizeReport& report,
											   const std::string& backendId);

std::string registerMeshSoup(cloudsim::host::DocumentHost* page, IPluginMainWindowHost* mainWindowHost,
							 std::vector<float> soup, const PluginMeshCreateOptions& options,
							 std::string* outError = nullptr);

} // namespace document_geometry_ops

#endif // CLOUDSIMPLUGINHOST_DOCUMENTGEOMETRYOPS_H
