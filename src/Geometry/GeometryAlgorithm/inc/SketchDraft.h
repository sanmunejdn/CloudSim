#ifndef GEOMETRYALGORITHM_SKETCHDRAFT_H

#define GEOMETRYALGORITHM_SKETCHDRAFT_H



/// @file SketchDraft.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com

/// @brief 选定面拔模（OCC DraftAngle）



#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"



#include <string>

#include <vector>



namespace geoalgo

{

GEOMETRY_ALGORITHM_API bool draftFacesToHandle(const ShapeHandle& base, const std::vector<int>& faceIndices,

											   double angleDeg, double neutralNx, double neutralNy, double neutralNz,

											   double ox, double oy, double oz, ShapeHandle& outShape,

											   std::string* errMsg = nullptr);



} // namespace geoalgo



#endif

