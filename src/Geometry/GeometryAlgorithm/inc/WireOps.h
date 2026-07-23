#ifndef GEOMETRYALGORITHM_WIREOPS_H
#define GEOMETRYALGORITHM_WIREOPS_H

/// @file WireOps.h
/// @brief 多条 Wire 合并为单 Wire 或离散折线

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
/**
 * 多 Wire 拓扑合并（BRepBuilderAPI_MakeWire）
 * @return false：wires 为空或 MakeWire 失败
 */
GEOMETRY_ALGORITHM_API bool fuseWires(const std::vector<TopoDS_Wire>& wires, TopoDS_Wire& outWire,
									  std::string* errMsg = nullptr);

/**
 * fuseWires 后再按 disc 离散为一条折线
 * @return false：合并或离散失败
 */
GEOMETRY_ALGORITHM_API bool fuseWiresToPolyline(const std::vector<TopoDS_Wire>& wires, const TessellateParams& disc,
												Polyline3d& out, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_WIREOPS_H
