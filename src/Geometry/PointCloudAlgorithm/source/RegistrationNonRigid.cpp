/// @file RegistrationNonRigid.cpp
/// @brief RegistrationNonRigid 实现

#include "RegistrationNonRigid.h"

#include "PointCloudBuffer.h"

#include <cmath>
#include <limits>

#include <Eigen/LU>

namespace pclalgo
{
namespace
{
double tpsU(const double r)
{
	if (r <= 1e-12)
	{
		return 0.0;
	}
	return r * r * std::log(r);
}

Eigen::Vector3d pointAt(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

bool solveTpsWeightsFull(const std::vector<Eigen::Vector3d>& controlPoints,
						 const std::vector<Eigen::Vector3d>& displacements, const double lambda,
						 Eigen::VectorXd& weightsX, Eigen::VectorXd& weightsY, Eigen::VectorXd& weightsZ,
						 Eigen::Vector4d& affineX, Eigen::Vector4d& affineY, Eigen::Vector4d& affineZ)
{
	const std::size_t n = controlPoints.size();
	if (n < 3U || displacements.size() != n)
	{
		return false;
	}

	const std::size_t dim = n + 4U;
	Eigen::MatrixXd l = Eigen::MatrixXd::Zero(static_cast<int>(dim), static_cast<int>(dim));
	for (std::size_t i = 0; i < n; ++i)
	{
		for (std::size_t j = 0; j < n; ++j)
		{
			l(static_cast<int>(i), static_cast<int>(j)) = tpsU((controlPoints[i] - controlPoints[j]).norm());
		}
		l(static_cast<int>(i), static_cast<int>(i)) += lambda;
		l(static_cast<int>(i), static_cast<int>(n + 0U)) = 1.0;
		l(static_cast<int>(i), static_cast<int>(n + 1U)) = controlPoints[i].x();
		l(static_cast<int>(i), static_cast<int>(n + 2U)) = controlPoints[i].y();
		l(static_cast<int>(i), static_cast<int>(n + 3U)) = controlPoints[i].z();
		l(static_cast<int>(n + 0U), static_cast<int>(i)) = 1.0;
		l(static_cast<int>(n + 1U), static_cast<int>(i)) = controlPoints[i].x();
		l(static_cast<int>(n + 2U), static_cast<int>(i)) = controlPoints[i].y();
		l(static_cast<int>(n + 3U), static_cast<int>(i)) = controlPoints[i].z();
	}

	const Eigen::FullPivLU<Eigen::MatrixXd> lu(l);
	if (!lu.isInvertible())
	{
		return false;
	}

	auto solveAxis = [&](int axis, Eigen::VectorXd& w, Eigen::Vector4d& a)
	{
		Eigen::VectorXd rhs = Eigen::VectorXd::Zero(static_cast<int>(dim));
		for (std::size_t i = 0; i < n; ++i)
		{
			rhs(static_cast<int>(i)) = displacements[i][axis];
		}
		const Eigen::VectorXd sol = lu.solve(rhs);
		w = sol.topRows(static_cast<int>(n));
		a(0) = sol(static_cast<int>(n + 0U));
		a(1) = sol(static_cast<int>(n + 1U));
		a(2) = sol(static_cast<int>(n + 2U));
		a(3) = sol(static_cast<int>(n + 3U));
	};

	solveAxis(0, weightsX, affineX);
	solveAxis(1, weightsY, affineY);
	solveAxis(2, weightsZ, affineZ);
	return true;
}

Eigen::Vector3d evalTps(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& controlPoints,
						const Eigen::VectorXd& weightsX, const Eigen::VectorXd& weightsY,
						const Eigen::VectorXd& weightsZ, const Eigen::Vector4d& affineX, const Eigen::Vector4d& affineY,
						const Eigen::Vector4d& affineZ)
{
	auto evalAxis = [&](const Eigen::VectorXd& w, const Eigen::Vector4d& a)
	{
		double v = a(0) + a(1) * p.x() + a(2) * p.y() + a(3) * p.z();
		for (std::size_t i = 0; i < controlPoints.size(); ++i)
		{
			v += w(static_cast<int>(i)) * tpsU((p - controlPoints[i]).norm());
		}
		return v;
	};
	return Eigen::Vector3d(evalAxis(weightsX, affineX), evalAxis(weightsY, affineY), evalAxis(weightsZ, affineZ));
}

} // namespace

bool tpsDeformFromControls(std::vector<float>& xyzInOut, const std::vector<std::size_t>& controlPointIndices,
						   const double* controlDisplacementXyz, const std::size_t numControls,
						   const double regularizationLambda, std::string* errMsg)
{
	if (!validXyzLength(xyzInOut) || controlDisplacementXyz == nullptr || numControls < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "TPS needs valid xyz and at least 3 controls";
		}
		return false;
	}

	const std::size_t nPts = pointCountFromXyz(xyzInOut);
	std::vector<Eigen::Vector3d> controls;
	std::vector<Eigen::Vector3d> displacements;
	controls.reserve(numControls);
	displacements.reserve(numControls);

	for (std::size_t k = 0; k < numControls; ++k)
	{
		const std::size_t idx = controlPointIndices[k];
		if (idx >= nPts)
		{
			if (errMsg != nullptr)
			{
				*errMsg = "control index out of range";
			}
			return false;
		}
		controls.push_back(pointAt(xyzInOut, idx));
		const std::size_t b = k * 3U;
		displacements.emplace_back(controlDisplacementXyz[b], controlDisplacementXyz[b + 1U],
								   controlDisplacementXyz[b + 2U]);
	}

	Eigen::VectorXd weightsX;
	Eigen::VectorXd weightsY;
	Eigen::VectorXd weightsZ;
	Eigen::Vector4d affineX;
	Eigen::Vector4d affineY;
	Eigen::Vector4d affineZ;
	if (!solveTpsWeightsFull(controls, displacements, regularizationLambda, weightsX, weightsY, weightsZ, affineX,
							 affineY, affineZ))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "TPS linear system singular";
		}
		return false;
	}

	for (std::size_t i = 0; i < nPts; ++i)
	{
		const Eigen::Vector3d p0 = pointAt(xyzInOut, i);
		const Eigen::Vector3d delta = evalTps(p0, controls, weightsX, weightsY, weightsZ, affineX, affineY, affineZ);
		const Eigen::Vector3d p1 = p0 + delta;
		const std::size_t b = i * 3U;
		xyzInOut[b] = static_cast<float>(p1.x());
		xyzInOut[b + 1U] = static_cast<float>(p1.y());
		xyzInOut[b + 2U] = static_cast<float>(p1.z());
	}
	return true;
}

bool tpsFitAndDeform(const std::vector<float>& sourceXyz, const std::vector<float>& targetXyz,
					 const std::vector<std::size_t>& correspondenceIndices, std::vector<float>& sourceXyzDeformedOut,
					 const double regularizationLambda, std::string* errMsg)
{
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "invalid xyz buffers";
		}
		return false;
	}

	const std::size_t nSrc = pointCountFromXyz(sourceXyz);
	const std::size_t nTgt = pointCountFromXyz(targetXyz);
	if (correspondenceIndices.size() < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "need at least 3 correspondences";
		}
		return false;
	}

	std::vector<std::size_t> controlIndices;
	std::vector<double> disp;
	controlIndices.reserve(correspondenceIndices.size());
	disp.reserve(correspondenceIndices.size() * 3U);

	for (const std::size_t srcIdx : correspondenceIndices)
	{
		if (srcIdx >= nSrc)
		{
			continue;
		}
		const Eigen::Vector3d ps = pointAt(sourceXyz, srcIdx);
		double bestSq = std::numeric_limits<double>::max();
		std::size_t bestTgt = 0U;
		for (std::size_t j = 0; j < nTgt; ++j)
		{
			const double d2 = (ps - pointAt(targetXyz, j)).squaredNorm();
			if (d2 < bestSq)
			{
				bestSq = d2;
				bestTgt = j;
			}
		}
		const Eigen::Vector3d pt = pointAt(targetXyz, bestTgt);
		const Eigen::Vector3d delta = pt - ps;
		controlIndices.push_back(srcIdx);
		disp.push_back(delta.x());
		disp.push_back(delta.y());
		disp.push_back(delta.z());
	}

	if (controlIndices.size() < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "too few valid correspondences";
		}
		return false;
	}

	sourceXyzDeformedOut = sourceXyz;
	return tpsDeformFromControls(sourceXyzDeformedOut, controlIndices, disp.data(), controlIndices.size(),
								 regularizationLambda, errMsg);
}

} // namespace pclalgo
