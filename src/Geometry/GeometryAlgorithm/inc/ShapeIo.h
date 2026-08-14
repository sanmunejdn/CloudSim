#ifndef GEOMETRYALGORITHM_SHAPEIO_H
#define GEOMETRYALGORITHM_SHAPEIO_H

/// @file ShapeIo.h
/// @brief STEP/BREP 读写与 ShapeHandle 刚体变换

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <string>

#include <Eigen/Geometry>

class TopoDS_Shape;

namespace geoalgo
{
/**
 * 读 STEP → TopoDS_Shape
 * @param pathLocal UTF-8 或系统窄路径（如 QFile::encodeName）；流式读取以支持中文路径
 * @return false：打开失败 / OCCT 读/transfer 失败或空 shape
 */
GEOMETRY_ALGORITHM_API bool readStepShape(const std::string& pathLocal, TopoDS_Shape& outShape, std::string* errMsg);

/** 读 STEP → ShapeHandle */
GEOMETRY_ALGORITHM_API bool readStepIntoHandle(const std::string& pathLocal, ShapeHandle& outShape,
											   std::string* errMsg);

/**
 * 读 BREP 文件
 * @return false：OCCT BREP read 失败
 */
GEOMETRY_ALGORITHM_API bool readBrepFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg);

/**
 * 写 BREP
 * @return false：null shape 或 OCCT write 失败
 */
GEOMETRY_ALGORITHM_API bool writeBrepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg);

/**
 * 写 STEP
 * @return false：null shape 或 OCCT transfer/write 失败
 */
GEOMETRY_ALGORITHM_API bool writeStepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg);

/** 对 shape 应用刚体变换，返回变换后的 ShapeHandle 副本 */
GEOMETRY_ALGORITHM_API ShapeHandle transformShape(const ShapeHandle& shape, const Eigen::Isometry3d& iso);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SHAPEIO_H
