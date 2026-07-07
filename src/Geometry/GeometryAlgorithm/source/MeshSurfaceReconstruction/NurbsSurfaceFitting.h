#pragma once

#include "MeshSurfaceReconstruction.h"

#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>

namespace geoalgo
{
namespace meshrecon
{

enum class NurbsFitMode : int
{
	Interpolate = 1,
	ApproxFixedCtrlpts = 2,
	ApproxCentripetal = 3,
	ApproxCentripetalFixedCtrlpts = 4,
};

struct AmrtoGridResolution
{
	int sampleNu = 0;
	int sampleNv = 0;
	int ctrlPtsU = 0;
	int ctrlPtsV = 0;
};

AmrtoGridResolution computeAmrtoGridResolution(
	double uSpanNorm,
	double vSpanNorm,
	const MeshSurfaceReconstructParams& params);

bool fitNurbsSurfaceFromGrid(
	const TColgp_Array2OfPnt& grid,
	int numCtrlU,
	int numCtrlV,
	NurbsFitMode mode,
	int degreeU,
	int degreeV,
	Handle(Geom_BSplineSurface)& outSurface);

NurbsFitMode nurbsFitModeFromMeshSurface(MeshSurfaceNurbsFitMode mode);

/// 由拟合格网每边点数推导控制点数（对齐 AMRTO controlPointDensityFactor 语义）
int resolveControlPointCountFromFitGrid(
	int gridPointsPerEdge,
	int degree,
	double controlPointDensityFactor,
	int minControlPointsPerDirection = 4);

} // namespace meshrecon
} // namespace geoalgo
