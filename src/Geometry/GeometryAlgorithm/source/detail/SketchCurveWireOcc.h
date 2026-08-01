#ifndef GEOMETRYALGORITHM_DETAIL_SKETCHCURVEWIREOCC_H
#define GEOMETRYALGORITHM_DETAIL_SKETCHCURVEWIREOCC_H

/// @file SketchCurveWireOcc.h
/// @brief OCC wire/face 构建（仅 GeometryAlgorithm 内部）

#include "SketchCurveWire.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <string>
#include <vector>

namespace geoalgo
{
bool makeClosedWireFromPolylineMm(const std::vector<float>& xyzMm, double planeNx, double planeNy, double planeNz,
								  TopoDS_Wire& outWire, std::string* errMsg = nullptr);

bool makeClosedFaceFromPolylineMm(const std::vector<float>& xyzMm, double planeNx, double planeNy, double planeNz,
								  TopoDS_Face& outFace, std::string* errMsg = nullptr);

bool makeClosedWireFromSegments(const std::vector<SketchCurveSegment>& segs, double planeNx, double planeNy,
								double planeNz, TopoDS_Wire& outWire, std::string* errMsg = nullptr);

bool makeClosedFaceFromSegments(const std::vector<SketchCurveSegment>& segs, double planeNx, double planeNy,
								double planeNz, TopoDS_Face& outFace, std::string* errMsg = nullptr);

bool makeFaceFromProfileAndHolePolylinesMm(const std::vector<float>& outerXyzMm,
										  const std::vector<std::vector<float>>& holePolylinesXyzMm, double planeNx,
										  double planeNy, double planeNz, TopoDS_Face& outFace,
										  std::string* errMsg = nullptr);

bool estimatePolylinePlaneNormal(const std::vector<float>& xyzMm, double& nx, double& ny, double& nz);

} // namespace geoalgo

#endif
