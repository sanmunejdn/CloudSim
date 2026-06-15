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

} // namespace meshrecon
} // namespace geoalgo
