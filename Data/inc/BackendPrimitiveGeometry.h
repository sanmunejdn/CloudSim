#pragma once

#include "data_global.h"

#include <vector>

namespace BackendPrimitiveGeometry
{
enum class PrimitiveKind { Box, Cylinder, Cone, Sphere };

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

DATA_EXPORT std::vector<float> makePrimitiveTriangleSoup(
	const PrimitiveMeshParams& params,
	const PrimitiveMeshQuality& quality = {});

DATA_EXPORT std::vector<float> makeBoxTriangleSoup(double lengthMm, double widthMm, double heightMm);
DATA_EXPORT std::vector<float> makeCylinderTriangleSoup(double radiusMm, double heightMm, int segments);
DATA_EXPORT std::vector<float> makeConeTriangleSoup(double radiusBottomMm, double radiusTopMm, double heightMm, int segments);
DATA_EXPORT std::vector<float> makeSphereTriangleSoup(double radiusMm, int segments, int rings);

}
