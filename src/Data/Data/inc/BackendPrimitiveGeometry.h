#ifndef DATA_BACKENDPRIMITIVEGEOMETRY_H
#define DATA_BACKENDPRIMITIVEGEOMETRY_H

/// @file BackendPrimitiveGeometry.h
/// @brief 参数化基本体三角 soup（模型空间，9 float/三角）

#include "data_global.h"

#include <vector>

/// 参数化基本体三角 soup（模型空间，9 float/三角）
namespace BackendPrimitiveGeometry
{
enum class PrimitiveKind
{
	Box,
	Cylinder,
	Cone,
	Sphere
};

struct DATA_EXPORT PrimitiveMeshParams
{
	PrimitiveKind kind = PrimitiveKind::Box;
	double lengthMm = 100.0;
	double widthMm = 50.0;
	double heightMm = 100.0;
	double radiusMm = 30.0;
	double radiusTopMm = 0.0;
};

struct DATA_EXPORT PrimitiveMeshQuality
{
	int segments = 32;
	int rings = 16;
};

DATA_EXPORT std::vector<float> makePrimitiveTriangleSoup(const PrimitiveMeshParams& params,
														 const PrimitiveMeshQuality& quality = {});

DATA_EXPORT std::vector<float> makeBoxTriangleSoup(double lengthMm, double widthMm, double heightMm);
DATA_EXPORT std::vector<float> makeCylinderTriangleSoup(double radiusMm, double heightMm, int segments);
DATA_EXPORT std::vector<float> makeConeTriangleSoup(double radiusBottomMm, double radiusTopMm, double heightMm,
													int segments);
DATA_EXPORT std::vector<float> makeSphereTriangleSoup(double radiusMm, int segments, int rings);

} // namespace BackendPrimitiveGeometry

#endif // DATA_BACKENDPRIMITIVEGEOMETRY_H
