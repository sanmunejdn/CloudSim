#include "NurbsSurfaceFitting.h"

#include "detail/OccIncludes.h"

#include <Eigen/Dense>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_HArray1OfReal.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

int clampInt(const int value, const int lo, const int hi)
{
	return std::max(lo, std::min(hi, value));
}

std::vector<double> buildClampedUniformKnots(const int numCtrlPts, const int degree)
{
	const int interiorCount = std::max(0, numCtrlPts - degree - 1);
	std::vector<double> uniqueKnots;
	uniqueKnots.reserve(static_cast<std::size_t>(interiorCount + 2));
	uniqueKnots.push_back(0.0);
	for (int i = 1; i <= interiorCount; ++i)
	{
		uniqueKnots.push_back(static_cast<double>(i) / static_cast<double>(interiorCount + 1));
	}
	uniqueKnots.push_back(1.0);
	return uniqueKnots;
}

void fillOccKnotArrays(
	const std::vector<double>& uniqueKnots,
	const int degree,
	TColStd_Array1OfReal& knots,
	TColStd_Array1OfInteger& mults)
{
	const int count = static_cast<int>(uniqueKnots.size());
	knots.Resize(1, count, Standard_False);
	mults.Resize(1, count, Standard_False);
	for (int i = 1; i <= count; ++i)
	{
		knots(i) = uniqueKnots[static_cast<std::size_t>(i - 1)];
		mults(i) = (i == 1 || i == count) ? degree + 1 : 1;
	}
}

std::vector<double> expandKnotVector(const std::vector<double>& uniqueKnots, const int degree)
{
	std::vector<double> expanded;
	for (std::size_t i = 0; i < uniqueKnots.size(); ++i)
	{
		const int mult = (i == 0 || i + 1 == uniqueKnots.size()) ? degree + 1 : 1;
		for (int j = 0; j < mult; ++j)
		{
			expanded.push_back(uniqueKnots[i]);
		}
	}
	return expanded;
}

double bsplineBasis(const int span, const int degree, const std::vector<double>& knots, const double u)
{
	if (degree == 0)
	{
		const double u0 = knots[static_cast<std::size_t>(span)];
		const double u1 = knots[static_cast<std::size_t>(span + 1)];
		const bool inUpper = (span + 1 == static_cast<int>(knots.size()) - 1) ? u <= u1 : u < u1;
		return (u >= u0 && inUpper) ? 1.0 : 0.0;
	}
	const double denomLeft = knots[static_cast<std::size_t>(span + degree)] - knots[static_cast<std::size_t>(span)];
	const double denomRight = knots[static_cast<std::size_t>(span + degree + 1)] - knots[static_cast<std::size_t>(span + 1)];
	double left = 0.0;
	double right = 0.0;
	if (denomLeft > 1e-15)
	{
		left = (u - knots[static_cast<std::size_t>(span)]) / denomLeft
			* bsplineBasis(span, degree - 1, knots, u);
	}
	if (denomRight > 1e-15)
	{
		right = (knots[static_cast<std::size_t>(span + degree + 1)] - u) / denomRight
			* bsplineBasis(span + 1, degree - 1, knots, u);
	}
	return left + right;
}

int findKnotSpan(const std::vector<double>& knots, const int degree, const double u)
{
	const int n = static_cast<int>(knots.size()) - degree - 2;
	int span = degree;
	for (int i = degree; i < n; ++i)
	{
		if (u >= knots[static_cast<std::size_t>(i)] && u < knots[static_cast<std::size_t>(i + 1)])
		{
			return i;
		}
	}
	return n;
}

std::vector<double> centripetalParamsFromPoints(const std::vector<Eigen::Vector3d>& points)
{
	std::vector<double> params(points.size(), 0.0);
	if (points.size() < 2)
	{
		return params;
	}
	double total = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i)
	{
		total += std::sqrt((points[i] - points[i - 1]).norm());
	}
	if (total < 1e-15)
	{
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			params[i] = static_cast<double>(i) / static_cast<double>(points.size() - 1);
		}
		return params;
	}
	double acc = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i)
	{
		acc += std::sqrt((points[i] - points[i - 1]).norm());
		params[i] = acc / total;
	}
	return params;
}

std::vector<double> isoParams(const int count)
{
	std::vector<double> params(static_cast<std::size_t>(count), 0.0);
	if (count <= 1)
	{
		return params;
	}
	for (int i = 0; i < count; ++i)
	{
		params[static_cast<std::size_t>(i)] = static_cast<double>(i) / static_cast<double>(count - 1);
	}
	return params;
}

bool fitCurveLeastSquares(
	const std::vector<double>& params,
	const std::vector<Eigen::Vector3d>& points,
	const int degree,
	const int numCtrl,
	std::vector<Eigen::Vector3d>& outCtrl)
{
	if (static_cast<int>(points.size()) < 2 || numCtrl < degree + 1)
	{
		return false;
	}
	const std::vector<double> uniqueKnots = buildClampedUniformKnots(numCtrl, degree);
	const std::vector<double> fullKnots = expandKnotVector(uniqueKnots, degree);
	const int sampleCount = static_cast<int>(points.size());
	Eigen::MatrixXd a = Eigen::MatrixXd::Zero(sampleCount, numCtrl);
	for (int k = 0; k < sampleCount; ++k)
	{
		const double u = std::min(1.0 - 1e-12, std::max(0.0, params[static_cast<std::size_t>(k)]));
		for (int i = 0; i < numCtrl; ++i)
		{
			a(k, i) = bsplineBasis(i, degree, fullKnots, u);
		}
	}
	Eigen::MatrixXd bx(sampleCount, 1);
	Eigen::MatrixXd by(sampleCount, 1);
	Eigen::MatrixXd bz(sampleCount, 1);
	for (int k = 0; k < sampleCount; ++k)
	{
		bx(k, 0) = points[static_cast<std::size_t>(k)].x();
		by(k, 0) = points[static_cast<std::size_t>(k)].y();
		bz(k, 0) = points[static_cast<std::size_t>(k)].z();
	}
	const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qrx(a);
	const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qry(a);
	const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qrz(a);
	if (qrx.rank() < std::min(degree + 1, numCtrl))
	{
		return false;
	}
	const Eigen::VectorXd px = qrx.solve(bx);
	const Eigen::VectorXd py = qry.solve(by);
	const Eigen::VectorXd pz = qrz.solve(bz);
	outCtrl.resize(static_cast<std::size_t>(numCtrl));
	for (int i = 0; i < numCtrl; ++i)
	{
		outCtrl[static_cast<std::size_t>(i)] = Eigen::Vector3d(px(i), py(i), pz(i));
	}
	return true;
}

bool fitSurfaceTwoStage(
	const TColgp_Array2OfPnt& grid,
	const int degreeU,
	const int degreeV,
	const int numCtrlU,
	const int numCtrlV,
	const bool centripetal,
	Handle(Geom_BSplineSurface)& outSurface)
{
	const int nu = grid.UpperRow() - grid.LowerRow();
	const int nv = grid.UpperCol() - grid.LowerCol();
	if (nu < 1 || nv < 1)
	{
		return false;
	}

	std::vector<double> uParams(static_cast<std::size_t>(nu + 1), 0.0);
	std::vector<double> vParams(static_cast<std::size_t>(nv + 1), 0.0);
	if (centripetal)
	{
		for (int i = 0; i <= nu; ++i)
		{
			std::vector<Eigen::Vector3d> row;
			row.reserve(static_cast<std::size_t>(nv + 1));
			for (int j = 0; j <= nv; ++j)
			{
				const gp_Pnt& p = grid.Value(grid.LowerRow() + i, grid.LowerCol() + j);
				row.emplace_back(p.X(), p.Y(), p.Z());
			}
			const std::vector<double> rowParams = centripetalParamsFromPoints(row);
			for (int j = 0; j <= nv; ++j)
			{
				uParams[static_cast<std::size_t>(i)] += rowParams[static_cast<std::size_t>(j)];
			}
			uParams[static_cast<std::size_t>(i)] /= static_cast<double>(nv + 1);
		}
		for (int j = 0; j <= nv; ++j)
		{
			std::vector<Eigen::Vector3d> col;
			col.reserve(static_cast<std::size_t>(nu + 1));
			for (int i = 0; i <= nu; ++i)
			{
				const gp_Pnt& p = grid.Value(grid.LowerRow() + i, grid.LowerCol() + j);
				col.emplace_back(p.X(), p.Y(), p.Z());
			}
			const std::vector<double> colParams = centripetalParamsFromPoints(col);
			for (int i = 0; i <= nu; ++i)
			{
				vParams[static_cast<std::size_t>(j)] += colParams[static_cast<std::size_t>(i)];
			}
			vParams[static_cast<std::size_t>(j)] /= static_cast<double>(nu + 1);
		}
	}
	else
	{
		uParams = isoParams(nu + 1);
		vParams = isoParams(nv + 1);
	}

	std::vector<std::vector<Eigen::Vector3d>> temp(
		static_cast<std::size_t>(numCtrlU),
		std::vector<Eigen::Vector3d>(static_cast<std::size_t>(nv + 1)));
	for (int j = 0; j <= nv; ++j)
	{
		std::vector<double> params;
		std::vector<Eigen::Vector3d> points;
		params.reserve(static_cast<std::size_t>(nu + 1));
		points.reserve(static_cast<std::size_t>(nu + 1));
		for (int i = 0; i <= nu; ++i)
		{
			params.push_back(uParams[static_cast<std::size_t>(i)]);
			const gp_Pnt& p = grid.Value(grid.LowerRow() + i, grid.LowerCol() + j);
			points.emplace_back(p.X(), p.Y(), p.Z());
		}
		std::vector<Eigen::Vector3d> colCtrl;
		if (!fitCurveLeastSquares(params, points, degreeU, numCtrlU, colCtrl))
		{
			return false;
		}
		for (int i = 0; i < numCtrlU; ++i)
		{
			temp[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = colCtrl[static_cast<std::size_t>(i)];
		}
	}

	TColgp_Array2OfPnt poles(1, numCtrlU, 1, numCtrlV);
	for (int i = 0; i < numCtrlU; ++i)
	{
		std::vector<double> params;
		std::vector<Eigen::Vector3d> points;
		params.reserve(static_cast<std::size_t>(nv + 1));
		points.reserve(static_cast<std::size_t>(nv + 1));
		for (int j = 0; j <= nv; ++j)
		{
			params.push_back(vParams[static_cast<std::size_t>(j)]);
			points.push_back(temp[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
		}
		std::vector<Eigen::Vector3d> rowCtrl;
		if (!fitCurveLeastSquares(params, points, degreeV, numCtrlV, rowCtrl))
		{
			return false;
		}
		for (int j = 0; j < numCtrlV; ++j)
		{
			const Eigen::Vector3d& p = rowCtrl[static_cast<std::size_t>(j)];
			poles.SetValue(i + 1, j + 1, gp_Pnt(p.x(), p.y(), p.z()));
		}
	}

	const std::vector<double> uUnique = buildClampedUniformKnots(numCtrlU, degreeU);
	const std::vector<double> vUnique = buildClampedUniformKnots(numCtrlV, degreeV);
	TColStd_Array1OfReal uKnotArr(1, static_cast<int>(uUnique.size()));
	TColStd_Array1OfReal vKnotArr(1, static_cast<int>(vUnique.size()));
	TColStd_Array1OfInteger uMult(1, static_cast<int>(uUnique.size()));
	TColStd_Array1OfInteger vMult(1, static_cast<int>(vUnique.size()));
	fillOccKnotArrays(uUnique, degreeU, uKnotArr, uMult);
	fillOccKnotArrays(vUnique, degreeV, vKnotArr, vMult);

	try
	{
		outSurface = new Geom_BSplineSurface(
			poles,
			uKnotArr,
			vKnotArr,
			uMult,
			vMult,
			degreeU,
			degreeV);
		return !outSurface.IsNull();
	}
	catch (...)
	{
		outSurface.Nullify();
		return false;
	}
}

bool fitWithGeomApi(
	const TColgp_Array2OfPnt& grid,
	const Approx_ParametrizationType paramType,
	const bool interpolate,
	const int degreeMin,
	const int degreeMax,
	Handle(Geom_BSplineSurface)& outSurface)
{
	try
	{
		if (interpolate)
		{
			GeomAPI_PointsToBSplineSurface api;
			api.Interpolate(grid, paramType, Standard_False);
			if (!api.IsDone() || api.Surface().IsNull())
			{
				return false;
			}
			outSurface = api.Surface();
			return true;
		}
		GeomAPI_PointsToBSplineSurface api(
			grid,
			paramType,
			degreeMin,
			degreeMax,
			GeomAbs_C2,
			std::max(1.0, 1.0e-3));
		if (!api.IsDone() || api.Surface().IsNull())
		{
			return false;
		}
		outSurface = api.Surface();
		return true;
	}
	catch (...)
	{
		outSurface.Nullify();
		return false;
	}
}

} // namespace

AmrtoGridResolution computeAmrtoGridResolution(
	const double uSpanNorm,
	const double vSpanNorm,
	const MeshSurfaceReconstructParams& params)
{
	AmrtoGridResolution out;
	const double uSpan = std::max(1e-6, uSpanNorm);
	const double vSpan = std::max(1e-6, vSpanNorm);
	int dU = static_cast<int>(std::lround(params.sampleRateFactor * 200.0 * vSpan));
	int dV = static_cast<int>(std::lround(params.sampleRateFactor * 200.0 * uSpan));
	dU = clampInt(dU, params.sampleGridMin, params.sampleGridMax);
	dV = clampInt(dV, params.sampleGridMin, params.sampleGridMax);

	const int degU = std::max(1, params.nurbsDegreeU);
	const int degV = std::max(1, params.nurbsDegreeV);
	int ctrlU = static_cast<int>(std::lround(params.controlPointDensityFactor * 200.0 * vSpan));
	int ctrlV = static_cast<int>(std::lround(params.controlPointDensityFactor * 200.0 * uSpan));
	ctrlU = std::min(ctrlU, static_cast<int>(0.8 * dU) - 1 - degU);
	ctrlV = std::min(ctrlV, static_cast<int>(0.8 * dV) - 1 - degV);
	ctrlU = clampInt(ctrlU, params.minControlPointsPerDirection, degU + 1);
	ctrlV = clampInt(ctrlV, params.minControlPointsPerDirection, degV + 1);

	out.sampleNu = dU;
	out.sampleNv = dV;
	out.ctrlPtsU = ctrlU;
	out.ctrlPtsV = ctrlV;
	return out;
}

bool fitNurbsSurfaceFromGrid(
	const TColgp_Array2OfPnt& grid,
	const int numCtrlU,
	const int numCtrlV,
	const NurbsFitMode mode,
	const int degreeU,
	const int degreeV,
	Handle(Geom_BSplineSurface)& outSurface)
{
	outSurface.Nullify();
	const int degU = std::max(1, degreeU);
	const int degV = std::max(1, degreeV);
	const int ctrlU = std::max(degU + 1, numCtrlU);
	const int ctrlV = std::max(degV + 1, numCtrlV);

	switch (mode)
	{
	case NurbsFitMode::Interpolate:
		return fitWithGeomApi(grid, Approx_Centripetal, true, degU, degU, outSurface);
	case NurbsFitMode::ApproxCentripetal:
		return fitWithGeomApi(grid, Approx_Centripetal, false, degU, degU, outSurface);
	case NurbsFitMode::ApproxFixedCtrlpts:
		return fitSurfaceTwoStage(grid, degU, degV, ctrlU, ctrlV, false, outSurface);
	case NurbsFitMode::ApproxCentripetalFixedCtrlpts:
		return fitSurfaceTwoStage(grid, degU, degV, ctrlU, ctrlV, true, outSurface);
	default:
		return false;
	}
}

} // namespace meshrecon
} // namespace geoalgo
