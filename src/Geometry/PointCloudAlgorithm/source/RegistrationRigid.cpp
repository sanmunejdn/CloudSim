#include "RegistrationRigid.h"

#include "KdTreePointSet.h"
#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Transform.h"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

namespace pclalgo
{

namespace
{

Eigen::Vector3d pointAt(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

Eigen::Vector3d normalAt(const std::vector<float>& nrm, const std::size_t i)
{
	const std::size_t b = i * 3U;
	Eigen::Vector3d n(nrm[b], nrm[b + 1U], nrm[b + 2U]);
	const double len = n.norm();
	if (len > 1e-12)
	{
		n /= len;
	}
	return n;
}

bool normalsCompatible(
	const Eigen::Vector3d& srcNormal,
	const Eigen::Vector3d& tgtNormal,
	const double minNormalDot)
{
	if (minNormalDot <= 0.0)
	{
		return true;
	}
	if (srcNormal.norm() < 1e-9 || tgtNormal.norm() < 1e-9)
	{
		return true;
	}
	return srcNormal.normalized().dot(tgtNormal.normalized()) >= minNormalDot;
}

std::vector<std::size_t> subsampleIndices(const std::size_t count, const std::size_t maxPoints)
{
	std::vector<std::size_t> indices(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		indices[i] = i;
	}
	if (count <= maxPoints)
	{
		return indices;
	}
	std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
	indices.resize(maxPoints);
	return indices;
}

std::size_t nearestIndexWithNormalGate(
	const Eigen::Vector3d& query,
	const Eigen::Vector3d& queryNormal,
	const std::vector<float>& tgtXyz,
	const std::vector<float>* tgtNormals,
	const std::vector<std::size_t>& tgtIndices,
	const double maxDistSq,
	const double minNormalDot,
	Eigen::Vector3d& outNormal)
{
	std::size_t best = static_cast<std::size_t>(-1);
	double bestSq = std::numeric_limits<double>::max();
	for (const std::size_t j : tgtIndices)
	{
		const double d2 = (query - pointAt(tgtXyz, j)).squaredNorm();
		if (d2 >= bestSq || d2 > maxDistSq)
		{
			continue;
		}
		Eigen::Vector3d tn = Eigen::Vector3d::Zero();
		if (tgtNormals != nullptr && tgtNormals->size() == tgtXyz.size())
		{
			tn = normalAt(*tgtNormals, j);
			if (!normalsCompatible(queryNormal, tn, minNormalDot))
			{
				continue;
			}
		}
		bestSq = d2;
		best = j;
		outNormal = tn;
	}
	return best;
}

// 使用 KD-tree 加速的最近邻搜索（带法线门控）
std::size_t nearestIndexWithNormalGateKdTree(
	const Eigen::Vector3d& query,
	const Eigen::Vector3d& queryNormal,
	const KdTreePointSet& tree,
	const std::vector<float>& tgtXyz,
	const std::vector<float>* tgtNormals,
	const double maxDistSq,
	const double minNormalDot,
	Eigen::Vector3d& outNormal)
{
	double distSq = 0.0;
	const std::size_t best = tree.findNearest(query.x(), query.y(), query.z(), maxDistSq, distSq);

	if (best == static_cast<std::size_t>(-1))
	{
		return static_cast<std::size_t>(-1);
	}

	// 检查法线兼容性
	if (tgtNormals != nullptr && tgtNormals->size() == tgtXyz.size() && minNormalDot > 0.0)
	{
		const Eigen::Vector3d tn = normalAt(*tgtNormals, best);
		if (!normalsCompatible(queryNormal, tn, minNormalDot))
		{
			return static_cast<std::size_t>(-1);
		}
		outNormal = tn;
	}
	else
	{
		outNormal = Eigen::Vector3d::Zero();
	}

	return best;
}

std::size_t nearestIndex(
	const Eigen::Vector3d& query,
	const std::vector<float>& tgtXyz,
	const std::vector<std::size_t>& tgtIndices,
	const double maxDistSq)
{
	Eigen::Vector3d dummy = Eigen::Vector3d::Zero();
	return nearestIndexWithNormalGate(
		query,
		dummy,
		tgtXyz,
		nullptr,
		tgtIndices,
		maxDistSq,
		0.0,
		dummy);
}

bool kabsch(
	const std::vector<Eigen::Vector3d>& src,
	const std::vector<Eigen::Vector3d>& tgt,
	Eigen::Isometry3d& transform)
{
	if (src.size() != tgt.size() || src.empty())
	{
		return false;
	}

	Eigen::Vector3d srcCentroid = Eigen::Vector3d::Zero();
	Eigen::Vector3d tgtCentroid = Eigen::Vector3d::Zero();
	for (std::size_t i = 0; i < src.size(); ++i)
	{
		srcCentroid += src[i];
		tgtCentroid += tgt[i];
	}
	srcCentroid /= static_cast<double>(src.size());
	tgtCentroid /= static_cast<double>(tgt.size());

	Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
	for (std::size_t i = 0; i < src.size(); ++i)
	{
		h += (src[i] - srcCentroid) * (tgt[i] - tgtCentroid).transpose();
	}

	const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d r = svd.matrixV() * svd.matrixU().transpose();
	if (r.determinant() < 0.0)
	{
		Eigen::Matrix3d v = svd.matrixV();
		v.col(2) *= -1.0;
		r = v * svd.matrixU().transpose();
	}

	transform = Eigen::Isometry3d::Identity();
	transform.linear() = r;
	transform.translation() = tgtCentroid - r * srcCentroid;
	return true;
}

bool solvePointToPlaneStep(
	const std::vector<Eigen::Vector3d>& srcPts,
	const std::vector<Eigen::Vector3d>& tgtPts,
	const std::vector<Eigen::Vector3d>& tgtNormals,
	Eigen::Isometry3d& outStep)
{
	if (srcPts.size() < 3U || srcPts.size() != tgtPts.size() || srcPts.size() != tgtNormals.size())
	{
		return false;
	}

	Eigen::MatrixXd a(static_cast<int>(srcPts.size()), 6);
	Eigen::VectorXd b(static_cast<int>(srcPts.size()));
	for (std::size_t i = 0; i < srcPts.size(); ++i)
	{
		const Eigen::Vector3d& p = srcPts[i];
		const Eigen::Vector3d& q = tgtPts[i];
		Eigen::Vector3d n = tgtNormals[i];
		if (n.norm() < 1e-9)
		{
			n = (q - p).normalized();
		}
		else
		{
			n.normalize();
		}
		const Eigen::Vector3d cross = p.cross(n);
		a(static_cast<int>(i), 0) = cross.x();
		a(static_cast<int>(i), 1) = cross.y();
		a(static_cast<int>(i), 2) = cross.z();
		a(static_cast<int>(i), 3) = n.x();
		a(static_cast<int>(i), 4) = n.y();
		a(static_cast<int>(i), 5) = n.z();
		b(static_cast<int>(i)) = n.dot(q - p);
	}

	const Eigen::VectorXd x = a.colPivHouseholderQr().solve(b);
	if (!x.allFinite())
	{
		return false;
	}

	const Eigen::Vector3d omega(x(0), x(1), x(2));
	const Eigen::Vector3d trans(x(3), x(4), x(5));
	outStep = Eigen::Isometry3d::Identity();
	if (omega.norm() > 1e-12)
	{
		outStep.linear() = Eigen::AngleAxisd(omega.norm(), omega.normalized()).toRotationMatrix();
	}
	outStep.translation() = trans;
	return true;
}

std::size_t nearestIndexWithNormal(
	const Eigen::Vector3d& query,
	const std::vector<float>& tgtXyz,
	const std::vector<float>& tgtNormals,
	const std::vector<std::size_t>& tgtIndices,
	const double maxDistSq,
	Eigen::Vector3d& outNormal)
{
	return nearestIndexWithNormalGate(
		query,
		Eigen::Vector3d::Zero(),
		tgtXyz,
		&tgtNormals,
		tgtIndices,
		maxDistSq,
		0.0,
		outNormal);
}

} // namespace

bool rigidRegisterIcp(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& targetXyz,
	Eigen::Isometry3d& sourceToTarget,
	double* rmseMm,
	const int maxIterations,
	const double convergenceTransMm,
	double maxPairDistanceMm,
	const std::size_t icpMaxPoints,
	std::string* errMsg,
	const std::vector<float>* sourceNormalsNxNyNz,
	const std::vector<float>* targetNormalsNxNyNz,
	const double maxNormalAngleDeg)
{
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "invalid xyz buffer length";
		}
		return false;
	}

	const bool useNormalGate = maxNormalAngleDeg > 0.0
		&& sourceNormalsNxNyNz != nullptr
		&& targetNormalsNxNyNz != nullptr
		&& sourceNormalsNxNyNz->size() == sourceXyz.size()
		&& targetNormalsNxNyNz->size() == targetXyz.size();
	const double minNormalDot = useNormalGate
		? std::cos(maxNormalAngleDeg * 3.14159265358979323846 / 180.0)
		: 0.0;

	const std::size_t nSrc = pointCountFromXyz(sourceXyz);
	const std::size_t nTgt = pointCountFromXyz(targetXyz);
	if (nSrc < 3U || nTgt < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "need at least 3 points in source and target";
		}
		return false;
	}

	if (maxPairDistanceMm <= 0.0)
	{
		const Eigen::AlignedBox3d box = computeBoundingBox(targetXyz);
		maxPairDistanceMm = box.diagonal().norm() * 0.05;
		if (maxPairDistanceMm <= 0.0)
		{
			maxPairDistanceMm = 1.0;
		}
	}
	const double maxPairDistSq = maxPairDistanceMm * maxPairDistanceMm;

	const std::vector<std::size_t> srcIdx = subsampleIndices(nSrc, icpMaxPoints);
	const std::vector<std::size_t> tgtIdx = subsampleIndices(nTgt, icpMaxPoints);

	// 构建目标点云的 KD-tree 加速最近邻搜索
	const KdTreePointSet tgtTree(targetXyz, tgtIdx);

	sourceToTarget = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d cumulative = Eigen::Isometry3d::Identity();

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		std::vector<Eigen::Vector3d> pairedSrc;
		std::vector<Eigen::Vector3d> pairedTgt;
		pairedSrc.reserve(srcIdx.size());
		pairedTgt.reserve(srcIdx.size());

		const Eigen::Matrix3d rot = cumulative.linear();
		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = cumulative * pointAt(sourceXyz, i);
			Eigen::Vector3d sn = Eigen::Vector3d::Zero();
			if (useNormalGate)
			{
				sn = rot * normalAt(*sourceNormalsNxNyNz, i);
			}
			Eigen::Vector3d tn = Eigen::Vector3d::Zero();
			// 使用 KD-tree 加速最近邻搜索
			const std::size_t j = nearestIndexWithNormalGateKdTree(
				ps,
				sn,
				tgtTree,
				targetXyz,
				useNormalGate ? targetNormalsNxNyNz : nullptr,
				maxPairDistSq,
				minNormalDot,
				tn);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			pairedSrc.push_back(ps);
			pairedTgt.push_back(pointAt(targetXyz, j));
		}

		if (pairedSrc.size() < 3U)
		{
			if (errMsg != nullptr)
			{
				*errMsg = "too few ICP correspondences";
			}
			return false;
		}

		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		if (!kabsch(pairedSrc, pairedTgt, step))
		{
			if (errMsg != nullptr)
			{
				*errMsg = "kabsch failed";
			}
			return false;
		}

		const double delta = step.translation().norm();
		cumulative = step * cumulative;

		if (delta < convergenceTransMm)
		{
			break;
		}
	}

	sourceToTarget = cumulative;

	if (rmseMm != nullptr)
	{
		double sumSq = 0.0;
		std::size_t pairs = 0U;
		const Eigen::Matrix3d rot = sourceToTarget.linear();
		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = sourceToTarget * pointAt(sourceXyz, i);
			Eigen::Vector3d sn = Eigen::Vector3d::Zero();
			if (useNormalGate)
			{
				sn = rot * normalAt(*sourceNormalsNxNyNz, i);
			}
			Eigen::Vector3d tn = Eigen::Vector3d::Zero();
			// 使用 KD-tree 加速最近邻搜索
			const std::size_t j = nearestIndexWithNormalGateKdTree(
				ps,
				sn,
				tgtTree,
				targetXyz,
				useNormalGate ? targetNormalsNxNyNz : nullptr,
				maxPairDistSq,
				minNormalDot,
				tn);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			const double d = (ps - pointAt(targetXyz, j)).norm();
			sumSq += d * d;
			++pairs;
		}
		*rmseMm = pairs > 0U ? std::sqrt(sumSq / static_cast<double>(pairs)) : 0.0;
	}

	return true;
}

bool rigidRegisterPointToPlaneIcp(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& sourceNormalsNxNyNz,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormalsNxNyNz,
	Eigen::Isometry3d& sourceToTarget,
	double* rmseMm,
	const int maxIterations,
	const double convergenceTransMm,
	double maxPairDistanceMm,
	const std::size_t icpMaxPoints,
	std::string* errMsg,
	const double maxNormalAngleDeg)
{
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "invalid xyz buffer length";
		}
		return false;
	}
	if (sourceNormalsNxNyNz.size() != sourceXyz.size() || targetNormalsNxNyNz.size() != targetXyz.size())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "normal buffer length mismatch";
		}
		return false;
	}

	const double minNormalDot = maxNormalAngleDeg > 0.0
		? std::cos(maxNormalAngleDeg * 3.14159265358979323846 / 180.0)
		: -1.0;

	const std::size_t nSrc = pointCountFromXyz(sourceXyz);
	const std::size_t nTgt = pointCountFromXyz(targetXyz);
	if (nSrc < 3U || nTgt < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "need at least 3 points in source and target";
		}
		return false;
	}

	if (maxPairDistanceMm <= 0.0)
	{
		const Eigen::AlignedBox3d box = computeBoundingBox(targetXyz);
		maxPairDistanceMm = box.diagonal().norm() * 0.05;
		if (maxPairDistanceMm <= 0.0)
		{
		 maxPairDistanceMm = 1.0;
		}
	}
	const double maxPairDistSq = maxPairDistanceMm * maxPairDistanceMm;

	const std::vector<std::size_t> srcIdx = subsampleIndices(nSrc, icpMaxPoints);
	const std::vector<std::size_t> tgtIdx = subsampleIndices(nTgt, icpMaxPoints);

	// 构建目标点云的 KD-tree 加速最近邻搜索
	const KdTreePointSet tgtTree(targetXyz, tgtIdx);

	sourceToTarget = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d cumulative = Eigen::Isometry3d::Identity();

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		std::vector<Eigen::Vector3d> pairedSrc;
		std::vector<Eigen::Vector3d> pairedTgt;
		std::vector<Eigen::Vector3d> pairedNrm;
		pairedSrc.reserve(srcIdx.size());
		pairedTgt.reserve(srcIdx.size());
		pairedNrm.reserve(srcIdx.size());

		const Eigen::Matrix3d rot = cumulative.linear();
		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = cumulative * pointAt(sourceXyz, i);
			const Eigen::Vector3d sn = rot * normalAt(sourceNormalsNxNyNz, i);
			Eigen::Vector3d tn = Eigen::Vector3d::Zero();
			// 使用 KD-tree 加速最近邻搜索
			const std::size_t j =
				nearestIndexWithNormalGateKdTree(
					ps,
					sn,
					tgtTree,
					targetXyz,
					&targetNormalsNxNyNz,
					maxPairDistSq,
					minNormalDot,
					tn);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			if (tn.norm() < 1e-9)
			{
				tn = normalAt(targetNormalsNxNyNz, j);
			}
			pairedSrc.push_back(ps);
			pairedTgt.push_back(pointAt(targetXyz, j));
			pairedNrm.push_back(tn);
		}

		if (pairedSrc.size() < 3U)
		{
			if (errMsg != nullptr)
			{
				*errMsg = "too few point-to-plane correspondences";
			}
			return false;
		}

		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		if (!solvePointToPlaneStep(pairedSrc, pairedTgt, pairedNrm, step))
		{
			if (errMsg != nullptr)
			{
				*errMsg = "point-to-plane solve failed";
			}
			return false;
		}

		const double delta = step.translation().norm();
		cumulative = step * cumulative;

		if (delta < convergenceTransMm)
		{
			break;
		}
	}

	sourceToTarget = cumulative;

	if (rmseMm != nullptr)
	{
		double sumSq = 0.0;
		std::size_t pairs = 0U;
		const Eigen::Matrix3d rot = sourceToTarget.linear();
		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = sourceToTarget * pointAt(sourceXyz, i);
			const Eigen::Vector3d sn = rot * normalAt(sourceNormalsNxNyNz, i);
			Eigen::Vector3d tn = Eigen::Vector3d::Zero();
			// 使用 KD-tree 加速最近邻搜索
			const std::size_t j =
				nearestIndexWithNormalGateKdTree(
					ps,
					sn,
					tgtTree,
					targetXyz,
					&targetNormalsNxNyNz,
					maxPairDistSq,
					minNormalDot,
					tn);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			if (tn.norm() < 1e-9)
			{
				tn = normalAt(targetNormalsNxNyNz, j);
			}
			tn.normalize();
			const double d = std::abs(tn.dot(pointAt(targetXyz, j) - ps));
			sumSq += d * d;
			++pairs;
		}
		*rmseMm = pairs > 0U ? std::sqrt(sumSq / static_cast<double>(pairs)) : 0.0;
	}

	return true;
}

} // namespace pclalgo
