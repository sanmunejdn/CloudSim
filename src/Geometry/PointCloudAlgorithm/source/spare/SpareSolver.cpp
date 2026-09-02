/// @file SpareSolver.cpp
/// @brief SpareSolver 实现

#include "spare/SpareSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pclalgo
{
namespace spare
{
namespace
{
constexpr std::size_t kInvalidIndex = static_cast<std::size_t>(-1);

} // namespace

void SpareSolver::surfaceToFloatXyz(const SpareSurface& surface, std::vector<float>& xyzOut)
{
	const std::size_t n = surface.vertexCount();
	xyzOut.resize(n * 3U);
	for (std::size_t i = 0; i < n; ++i)
	{
		xyzOut[i * 3U] = static_cast<float>(surface.positions[i].x());
		xyzOut[i * 3U + 1U] = static_cast<float>(surface.positions[i].y());
		xyzOut[i * 3U + 2U] = static_cast<float>(surface.positions[i].z());
	}
}

void SpareSolver::buildSrcKnnIndices(const Matrix3X& srcPoints, const int knnCount, Eigen::MatrixXi& outIndices)
{
	std::vector<float> xyz;
	xyz.reserve(static_cast<std::size_t>(srcPoints.cols()) * 3U);
	for (int i = 0; i < srcPoints.cols(); ++i)
	{
		xyz.push_back(static_cast<float>(srcPoints(0, i)));
		xyz.push_back(static_cast<float>(srcPoints(1, i)));
		xyz.push_back(static_cast<float>(srcPoints(2, i)));
	}

	const KdTreePointSet tree(xyz);
	outIndices.resize(knnCount, srcPoints.cols());

#pragma omp parallel for
	for (int i = 0; i < srcPoints.cols(); ++i)
	{
		std::vector<std::size_t> indices;
		std::vector<double> distSq;
		tree.findKNearest(srcPoints(0, i), srcPoints(1, i), srcPoints(2, i), static_cast<unsigned int>(knnCount + 1),
						  indices, distSq);

		int written = 0;
		for (std::size_t j = 0; j < indices.size() && written < knnCount; ++j)
		{
			if (indices[j] != static_cast<std::size_t>(i))
			{
				outIndices(written, i) = static_cast<int>(indices[j]);
				++written;
			}
		}
		while (written < knnCount)
		{
			outIndices(written, i) = i;
			++written;
		}
	}
}

void SpareSolver::buildTargetKdTree()
{
	std::vector<float> xyz;
	surfaceToFloatXyz(*target_, xyz);
	targetTree_ = std::make_unique<KdTreePointSet>(xyz);
}

bool SpareSolver::init(SpareSurface& source, SpareSurface& target, SpareInternalParams& params)
{
	source_ = &source;
	target_ = &target;
	params_ = &params;

	nSrcVertex_ = static_cast<int>(source.vertexCount());
	nTarVertex_ = static_cast<int>(target.vertexCount());
	if (nSrcVertex_ <= 0 || nTarVertex_ <= 0)
	{
		return false;
	}

	useGeodesicMode_ = params.useGeodesicDist && source.hasFaces();
	alignSampleCount_ = params.alignSampleCount;
	if (alignSampleCount_ > static_cast<std::size_t>(nSrcVertex_))
	{
		alignSampleCount_ = static_cast<std::size_t>(nSrcVertex_);
	}

	srcPoints_.resize(3, nSrcVertex_);
	srcNormals_.resize(3, nSrcVertex_);
	corresU0_.resize(nSrcVertex_ * 3);

#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		srcPoints_.col(i) = source.positions[static_cast<std::size_t>(i)];
		srcNormals_.col(i) = source.normals[static_cast<std::size_t>(i)];
	}

	deformedNormals_ = srcNormals_;
	deformedPoints_ = Eigen::Map<VectorX>(srcPoints_.data(), nSrcVertex_ * 3);

	if (!useGeodesicMode_)
	{
		buildSrcKnnIndices(srcPoints_, knnNeighborCount_, srcKnnIndices_);
	}

	tarPoints_.resize(3, nTarVertex_);
	targetNormals_.resize(3, nTarVertex_);
	for (int i = 0; i < nTarVertex_; ++i)
	{
		tarPoints_.col(i) = target.positions[static_cast<std::size_t>(i)];
		targetNormals_.col(i) = target.normals[static_cast<std::size_t>(i)];
	}
	buildTargetKdTree();

	Scalar sampleRadius = 0.0;
	if (params.useCoarseReg)
	{
		if (useGeodesicMode_)
		{
			sampleRadius = nodeSampler_.sampleAndConstructMesh(source, srcPoints_, params.uniSampleRatio);
		}
		else
		{
			sampleRadius = nodeSampler_.sampleAndConstructPointCloudFps(source, srcPoints_, srcKnnIndices_,
																		params.uniSampleRatio, 4, 8);
		}
		(void)sampleRadius;
	}

	if (params.useCoarseReg)
	{
		numSampleNodes_ = static_cast<int>(nodeSampler_.nodeSize());
		params.numSampleNodes = static_cast<std::size_t>(numSampleNodes_);

		X_.resize(numSampleNodes_ * 12);
		X_.setZero();
		alignCoeffPv0_.resize(nSrcVertex_ * 3, numSampleNodes_ * 12);
		nodesP_.resize(nSrcVertex_ * 3);
		nodesR_.resize(numSampleNodes_ * 9);
		nodesR_.setZero();
		rigidCoeffL_.resize(numSampleNodes_ * 12, numSampleNodes_ * 12);
		rigidCoeffJ_.resize(numSampleNodes_ * 12, numSampleNodes_ * 9);

		std::vector<Triplet> coeffL(static_cast<std::size_t>(numSampleNodes_) * 9U);
		std::vector<Triplet> coeffJ(static_cast<std::size_t>(numSampleNodes_) * 9U);
		for (int i = 0; i < numSampleNodes_; ++i)
		{
			X_[i * 12] = 1.0;
			X_[i * 12 + 4] = 1.0;
			X_[i * 12 + 8] = 1.0;

			nodesR_[i * 9] = 1.0;
			nodesR_[i * 9 + 4] = 1.0;
			nodesR_[i * 9 + 8] = 1.0;

			for (int j = 0; j < 9; ++j)
			{
				coeffL[static_cast<std::size_t>(i) * 9U + static_cast<std::size_t>(j)] =
					Triplet(i * 12 + j, i * 12 + j, 1.0);
				coeffJ[static_cast<std::size_t>(i) * 9U + static_cast<std::size_t>(j)] =
					Triplet(i * 12 + j, i * 9 + j, 1.0);
			}
		}
		rigidCoeffL_.setFromTriplets(coeffL.begin(), coeffL.end());
		rigidCoeffJ_.setFromTriplets(coeffJ.begin(), coeffJ.end());

		nodeSampler_.initWeight(source, alignCoeffPv0_, nodesP_, regCoeffB_, regRightD_, regCwiseWeights_);
	}

	fullInArapCoeff();
	initRotations();

	samplingIndices_.clear();
	const std::size_t startIndex = 0;
	samplingIndices_.push_back(startIndex);

	VectorX minDistances = VectorX::Constant(nSrcVertex_, std::numeric_limits<Scalar>::max());
	minDistances[static_cast<Eigen::Index>(startIndex)] = 0.0;

	vertexSampleIndices_.assign(static_cast<std::size_t>(nSrcVertex_), -1);
	vertexSampleIndices_[startIndex] = 0;

	std::size_t curStart = startIndex;
	while (samplingIndices_.size() < alignSampleCount_)
	{
#pragma omp parallel for
		for (int i = 0; i < nSrcVertex_; ++i)
		{
			if (static_cast<std::size_t>(i) == curStart)
			{
				continue;
			}
			const Scalar dist = (srcPoints_.col(static_cast<Eigen::Index>(curStart)) - srcPoints_.col(i)).norm();
			if (dist < minDistances[i])
			{
				minDistances[i] = dist;
			}
		}

		int maxDistanceIndex = 0;
		minDistances.maxCoeff(&maxDistanceIndex);
		minDistances[maxDistanceIndex] = 0.0;
		curStart = static_cast<std::size_t>(maxDistanceIndex);
		samplingIndices_.push_back(curStart);
		vertexSampleIndices_[curStart] = static_cast<int>(samplingIndices_.size() - 1U);
	}

	initWelschParam();
	return true;
}

void SpareSolver::initWelschParam()
{
	weightD_.resize(nSrcVertex_ * 3);
	weightD_.setOnes();

	initCorrespondence(correspondencePairs_);

	VectorX initNus(static_cast<Eigen::Index>(correspondencePairs_.size()));
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(correspondencePairs_.size()); ++i)
	{
		const Vector3 closest = correspondencePairs_[static_cast<std::size_t>(i)].position;
		const int srcIdx = correspondencePairs_[static_cast<std::size_t>(i)].srcIdx;
		initNus[i] = (srcPoints_.col(srcIdx) - closest).norm();
	}

	Scalar medianNu = params_->dataNu;
	if (median(initNus, medianNu))
	{
		params_->dataNu = medianNu;
	}
}

void SpareSolver::fullInArapCoeff()
{
	arapLaplaceWeights_.resize(nSrcVertex_);

	if (useGeodesicMode_)
	{
		for (int i = 0; i < nSrcVertex_; ++i)
		{
			const int nn = static_cast<int>(source_->vertexNeighbors[static_cast<std::size_t>(i)].size());
			arapLaplaceWeights_[i] = nn > 0 ? 1.0 / static_cast<Scalar>(nn) : 0.0;
		}

		std::vector<Triplet> coeffs;
		std::vector<Triplet> coeffsFine;
		const int numHalfEdges = static_cast<int>(source_->halfEdges.size());

		for (int i = 0; i < numHalfEdges; ++i)
		{
			const int srcIdx = source_->halfEdges[static_cast<std::size_t>(i)].first;
			const int tarIdx = source_->halfEdges[static_cast<std::size_t>(i)].second;
			const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

			if (params_->useCoarseReg)
			{
				for (int k = 0; k < 3; ++k)
				{
					for (RowMajorSparseMatrix::InnerIterator it(alignCoeffPv0_, srcIdx * 3 + k); it; ++it)
					{
						coeffs.emplace_back(i * 3 + k, static_cast<int>(it.col()), w * it.value());
					}
					for (RowMajorSparseMatrix::InnerIterator it(alignCoeffPv0_, tarIdx * 3 + k); it; ++it)
					{
						coeffs.emplace_back(i * 3 + k, static_cast<int>(it.col()), -w * it.value());
					}
				}
			}

			coeffsFine.emplace_back(i * 3, srcIdx * 3, w);
			coeffsFine.emplace_back(i * 3 + 1, srcIdx * 3 + 1, w);
			coeffsFine.emplace_back(i * 3 + 2, srcIdx * 3 + 2, w);
			coeffsFine.emplace_back(i * 3, tarIdx * 3, -w);
			coeffsFine.emplace_back(i * 3 + 1, tarIdx * 3 + 1, -w);
			coeffsFine.emplace_back(i * 3 + 2, tarIdx * 3 + 2, -w);
		}

		if (params_->useCoarseReg)
		{
			arapCoeff_.resize(numHalfEdges * 3, numSampleNodes_ * 12);
			arapCoeff_.setFromTriplets(coeffs.begin(), coeffs.end());
			arapCoeffMul_ = arapCoeff_.transpose() * arapCoeff_;
		}

		arapCoeffFine_.resize(numHalfEdges * 3, nSrcVertex_ * 3);
		arapCoeffFine_.setFromTriplets(coeffsFine.begin(), coeffsFine.end());
		arapCoeffMulFine_ = arapCoeffFine_.transpose() * arapCoeffFine_;
		arapRight_.resize(numHalfEdges * 3);
		arapRightFine_.resize(numHalfEdges * 3);
	}
	else
	{
		const int nn = knnNeighborCount_;
		for (int i = 0; i < nSrcVertex_; ++i)
		{
			arapLaplaceWeights_[i] = 1.0 / static_cast<Scalar>(nn);
		}

		std::vector<Triplet> coeffs;
		std::vector<Triplet> coeffsFine;
		for (int srcIdx = 0; srcIdx < nSrcVertex_; ++srcIdx)
		{
			for (int j = 0; j < nn; ++j)
			{
				const int i = srcIdx * nn + j;
				const int tarIdx = srcKnnIndices_(j, srcIdx);
				const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

				if (params_->useCoarseReg)
				{
					for (int k = 0; k < 3; ++k)
					{
						for (RowMajorSparseMatrix::InnerIterator it(alignCoeffPv0_, srcIdx * 3 + k); it; ++it)
						{
							coeffs.emplace_back(i * 3 + k, static_cast<int>(it.col()), w * it.value());
						}
						for (RowMajorSparseMatrix::InnerIterator it(alignCoeffPv0_, tarIdx * 3 + k); it; ++it)
						{
							coeffs.emplace_back(i * 3 + k, static_cast<int>(it.col()), -w * it.value());
						}
					}
				}

				coeffsFine.emplace_back(i * 3, srcIdx * 3, w);
				coeffsFine.emplace_back(i * 3 + 1, srcIdx * 3 + 1, w);
				coeffsFine.emplace_back(i * 3 + 2, srcIdx * 3 + 2, w);
				coeffsFine.emplace_back(i * 3, tarIdx * 3, -w);
				coeffsFine.emplace_back(i * 3 + 1, tarIdx * 3 + 1, -w);
				coeffsFine.emplace_back(i * 3 + 2, tarIdx * 3 + 2, -w);
			}
		}

		if (params_->useCoarseReg)
		{
			arapCoeff_.resize(nSrcVertex_ * nn * 3, numSampleNodes_ * 12);
			arapCoeff_.setFromTriplets(coeffs.begin(), coeffs.end());
			arapCoeffMul_ = arapCoeff_.transpose() * arapCoeff_;
		}

		arapCoeffFine_.resize(nSrcVertex_ * nn * 3, nSrcVertex_ * 3);
		arapCoeffFine_.setFromTriplets(coeffsFine.begin(), coeffsFine.end());
		arapCoeffMulFine_ = arapCoeffFine_.transpose() * arapCoeffFine_;
		arapRight_.resize(nSrcVertex_ * nn * 3);
		arapRightFine_.resize(nSrcVertex_ * nn * 3);
	}
}

void SpareSolver::initRotations()
{
	localRotations_.resize(3, nSrcVertex_ * 3);
	localRotations_.setZero();
#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		localRotations_(0, i * 3) = 1.0;
		localRotations_(1, i * 3 + 1) = 1.0;
		localRotations_(2, i * 3 + 2) = 1.0;
	}
}

void SpareSolver::welschWeight(VectorX& r, const Scalar p) const
{
#pragma omp parallel for
	for (int i = 0; i < r.rows(); ++i)
	{
		if (r[i] >= 0.0)
		{
			r[i] = std::exp(-r[i] / (2.0 * p * p));
		}
		else
		{
			r[i] = 0.0;
		}
	}
}

void SpareSolver::calcNodeRotations()
{
#pragma omp parallel for
	for (int i = 0; i < numSampleNodes_; ++i)
	{
		Matrix33 rot;
		const Eigen::Map<Matrix33> block(X_.data() + i * 12, 3, 3);
		const Eigen::JacobiSVD<Matrix33> svd(block, Eigen::ComputeFullU | Eigen::ComputeFullV);
		if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
		{
			Vector3 s = Vector3::Ones();
			s(2) = -1.0;
			rot = svd.matrixU() * s.asDiagonal() * svd.matrixV().transpose();
		}
		else
		{
			rot = svd.matrixU() * svd.matrixV().transpose();
		}
		nodesR_.segment(i * 9, 9) = Eigen::Map<VectorX>(rot.data(), 9);
	}
}

void SpareSolver::calcArapRight()
{
	if (useGeodesicMode_)
	{
		const int numHalfEdges = static_cast<int>(source_->halfEdges.size());
#pragma omp parallel for
		for (int i = 0; i < numHalfEdges; ++i)
		{
			const int srcIdx = source_->halfEdges[static_cast<std::size_t>(i)].first;
			const int tarIdx = source_->halfEdges[static_cast<std::size_t>(i)].second;
			const Vector3 vij =
				localRotations_.block(0, srcIdx * 3, 3, 3) * (srcPoints_.col(srcIdx) - srcPoints_.col(tarIdx));
			const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

			arapRight_[i * 3] = w * (vij[0] - nodesP_[srcIdx * 3] + nodesP_[tarIdx * 3]);
			arapRight_[i * 3 + 1] = w * (vij[1] - nodesP_[srcIdx * 3 + 1] + nodesP_[tarIdx * 3 + 1]);
			arapRight_[i * 3 + 2] = w * (vij[2] - nodesP_[srcIdx * 3 + 2] + nodesP_[tarIdx * 3 + 2]);
		}
	}
	else
	{
		const int nn = knnNeighborCount_;
#pragma omp parallel for
		for (int srcIdx = 0; srcIdx < nSrcVertex_; ++srcIdx)
		{
			for (int j = 0; j < nn; ++j)
			{
				const int i = srcIdx * nn + j;
				const int tarIdx = srcKnnIndices_(j, srcIdx);
				const Vector3 vij =
					localRotations_.block(0, srcIdx * 3, 3, 3) * (srcPoints_.col(srcIdx) - srcPoints_.col(tarIdx));
				const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

				arapRight_[i * 3] = w * (vij[0] - nodesP_[srcIdx * 3] + nodesP_[tarIdx * 3]);
				arapRight_[i * 3 + 1] = w * (vij[1] - nodesP_[srcIdx * 3 + 1] + nodesP_[tarIdx * 3 + 1]);
				arapRight_[i * 3 + 2] = w * (vij[2] - nodesP_[srcIdx * 3 + 2] + nodesP_[tarIdx * 3 + 2]);
			}
		}
	}
}

void SpareSolver::calcArapRightFine()
{
	if (useGeodesicMode_)
	{
		const int numHalfEdges = static_cast<int>(source_->halfEdges.size());
#pragma omp parallel for
		for (int i = 0; i < numHalfEdges; ++i)
		{
			const int srcIdx = source_->halfEdges[static_cast<std::size_t>(i)].first;
			const int tarIdx = source_->halfEdges[static_cast<std::size_t>(i)].second;
			const Vector3 vij =
				localRotations_.block(0, srcIdx * 3, 3, 3) * (srcPoints_.col(srcIdx) - srcPoints_.col(tarIdx));
			const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

			arapRightFine_[i * 3] = w * vij[0];
			arapRightFine_[i * 3 + 1] = w * vij[1];
			arapRightFine_[i * 3 + 2] = w * vij[2];
		}
	}
	else
	{
		const int nn = knnNeighborCount_;
#pragma omp parallel for
		for (int srcIdx = 0; srcIdx < nSrcVertex_; ++srcIdx)
		{
			for (int j = 0; j < nn; ++j)
			{
				const int tarIdx = srcKnnIndices_(j, srcIdx);
				const int i = srcIdx * nn + j;
				const Vector3 vij =
					localRotations_.block(0, srcIdx * 3, 3, 3) * (srcPoints_.col(srcIdx) - srcPoints_.col(tarIdx));
				const Scalar w = std::sqrt(arapLaplaceWeights_[srcIdx]);

				arapRightFine_[i * 3] = w * vij[0];
				arapRightFine_[i * 3 + 1] = w * vij[1];
				arapRightFine_[i * 3 + 2] = w * vij[2];
			}
		}
	}
}

void SpareSolver::calcLocalRotations(const bool isCoarseAlign)
{
#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		Matrix33 sum = Matrix33::Zero();
		int nn = 0;

		if (useGeodesicMode_)
		{
			for (const int neighborIdx : source_->vertexNeighbors[static_cast<std::size_t>(i)])
			{
				const Vector3 dv = srcPoints_.col(i) - srcPoints_.col(neighborIdx);
				const Vector3 newDv = deformedPoints_.segment(i * 3, 3) - deformedPoints_.segment(neighborIdx * 3, 3);
				sum += dv * newDv.transpose();
				++nn;
			}
		}
		else
		{
			nn = knnNeighborCount_;
			for (int j = 0; j < nn; ++j)
			{
				const int neighborIdx = srcKnnIndices_(j, i);
				const Vector3 dv = srcPoints_.col(i) - srcPoints_.col(neighborIdx);
				const Vector3 newDv = deformedPoints_.segment(i * 3, 3) - deformedPoints_.segment(neighborIdx * 3, 3);
				sum += dv * newDv.transpose();
			}
		}

		if (nn > 0)
		{
			sum *= optimizeWArap_ / static_cast<Scalar>(nn);
		}

		if (!isCoarseAlign)
		{
			const int tarIdx = correspondencePairs_[static_cast<std::size_t>(i)].tarIdx;
			const Vector3 d = deformedPoints_.segment(i * 3, 3) - tarPoints_.col(tarIdx);
			const Scalar c = (targetNormals_.col(tarIdx) + deformedNormals_.col(i)).dot(d);
			const Scalar dNorm2 = d.squaredNorm();
			Vector3 h = deformedNormals_.col(i);
			if (dNorm2 > 0.0)
			{
				h -= c * d / dNorm2;
			}
			const Scalar w = optimizeWAlign_ * dNorm2 * weightD_[i];
			sum += w * srcNormals_.col(i) * h.transpose();
		}
		else if (vertexSampleIndices_[static_cast<std::size_t>(i)] >= 0)
		{
			const int sampleIdx = vertexSampleIndices_[static_cast<std::size_t>(i)];
			const int tarIdx = correspondencePairs_[static_cast<std::size_t>(sampleIdx)].tarIdx;
			const Vector3 d = deformedPoints_.segment(i * 3, 3) - tarPoints_.col(tarIdx);
			const Scalar c = (targetNormals_.col(tarIdx) + deformedNormals_.col(i)).dot(d);
			const Scalar dNorm2 = d.squaredNorm();
			Vector3 h = deformedNormals_.col(i);
			if (dNorm2 > 0.0)
			{
				h -= c * d / dNorm2;
			}
			const Scalar w = optimizeWAlign_ * dNorm2 * weightD_[sampleIdx];
			sum += w * srcNormals_.col(i) * h.transpose();
		}

		const Eigen::JacobiSVD<Matrix33> svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Matrix33 rot;
		if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
		{
			Vector3 s = Vector3::Ones();
			s(2) = -1.0;
			rot = svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();
		}
		else
		{
			rot = svd.matrixV() * svd.matrixU().transpose();
		}

		for (int s = 0; s < 3; ++s)
		{
			for (int t = 0; t < 3; ++t)
			{
				localRotations_(s, i * 3 + t) = rot(s, t);
			}
		}
	}
}

void SpareSolver::calcDeformedNormals()
{
#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		deformedNormals_.col(i) = localRotations_.block(0, i * 3, 3, 3) * srcNormals_.col(i);
	}
}

void SpareSolver::initNormalsSum()
{
	std::vector<Triplet> coeffs(static_cast<std::size_t>(correspondencePairs_.size()) * 3U);
	normalsSum_.resize(static_cast<Eigen::Index>(correspondencePairs_.size()), nSrcVertex_ * 3);
	normalsSum_.setZero();

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(correspondencePairs_.size()); ++i)
	{
		const int sidx = correspondencePairs_[static_cast<std::size_t>(i)].srcIdx;
		const int tidx = correspondencePairs_[static_cast<std::size_t>(i)].tarIdx;
		coeffs[static_cast<std::size_t>(i) * 3U] =
			Triplet(i, sidx * 3, deformedNormals_(0, sidx) + targetNormals_(0, tidx));
		coeffs[static_cast<std::size_t>(i) * 3U + 1U] =
			Triplet(i, sidx * 3 + 1, deformedNormals_(1, sidx) + targetNormals_(1, tidx));
		coeffs[static_cast<std::size_t>(i) * 3U + 2U] =
			Triplet(i, sidx * 3 + 2, deformedNormals_(2, sidx) + targetNormals_(2, tidx));
	}
	normalsSum_.setFromTriplets(coeffs.begin(), coeffs.end());
}

void SpareSolver::calcNormalsSum()
{
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(correspondencePairs_.size()); ++i)
	{
		const int sidx = correspondencePairs_[static_cast<std::size_t>(i)].srcIdx;
		const int tidx = correspondencePairs_[static_cast<std::size_t>(i)].tarIdx;
		int j = 0;
		for (RowMajorSparseMatrix::InnerIterator it(normalsSum_, i); it; ++it)
		{
			it.valueRef() = deformedNormals_(j, sidx) + targetNormals_(j, tidx);
			++j;
		}
	}
}

void SpareSolver::initCorrespondence(std::vector<SpareCorrespondence>& corres)
{
	findClosestPoints(corres);
	simplePruning(corres);
}

void SpareSolver::findClosestPoints(std::vector<SpareCorrespondence>& corres)
{
	corres.resize(static_cast<std::size_t>(nSrcVertex_));
	const double maxDistSq = std::numeric_limits<double>::max();

#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		double distSq = 0.0;
		const std::size_t idx =
			targetTree_->findNearest(srcPoints_(0, i), srcPoints_(1, i), srcPoints_(2, i), maxDistSq, distSq);

		SpareCorrespondence c;
		c.srcIdx = i;
		if (idx != kInvalidIndex)
		{
			c.tarIdx = static_cast<int>(idx);
			c.position = tarPoints_.col(static_cast<Eigen::Index>(idx));
			c.normal = targetNormals_.col(static_cast<Eigen::Index>(idx));
			c.minDist2 = static_cast<Scalar>(distSq);
		}
		else
		{
			c.tarIdx = 0;
			c.position = tarPoints_.col(0);
			c.normal = targetNormals_.col(0);
			c.minDist2 = static_cast<Scalar>(maxDistSq);
		}
		corres[static_cast<std::size_t>(i)] = c;
	}
}

void SpareSolver::findClosestPoints(std::vector<SpareCorrespondence>& corres, const VectorX& deformedV)
{
	corres.resize(static_cast<std::size_t>(nSrcVertex_));
	const double maxDistSq = std::numeric_limits<double>::max();

#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		double distSq = 0.0;
		const std::size_t idx =
			targetTree_->findNearest(deformedV[i * 3], deformedV[i * 3 + 1], deformedV[i * 3 + 2], maxDistSq, distSq);

		SpareCorrespondence c;
		c.srcIdx = i;
		if (idx != kInvalidIndex)
		{
			c.tarIdx = static_cast<int>(idx);
			c.position = tarPoints_.col(static_cast<Eigen::Index>(idx));
			c.minDist2 = static_cast<Scalar>(distSq);
		}
		else
		{
			c.tarIdx = 0;
			c.position = tarPoints_.col(0);
			c.minDist2 = static_cast<Scalar>(maxDistSq);
		}
		corres[static_cast<std::size_t>(i)] = c;
	}
}

void SpareSolver::findClosestPoints(std::vector<SpareCorrespondence>& corres, const VectorX& deformedV,
									const std::vector<std::size_t>& sampleIndices)
{
	corres.resize(sampleIndices.size());
	const double maxDistSq = std::numeric_limits<double>::max();

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(sampleIndices.size()); ++i)
	{
		const int sidx = static_cast<int>(sampleIndices[static_cast<std::size_t>(i)]);
		double distSq = 0.0;
		const std::size_t idx = targetTree_->findNearest(deformedV[sidx * 3], deformedV[sidx * 3 + 1],
														 deformedV[sidx * 3 + 2], maxDistSq, distSq);

		SpareCorrespondence c;
		c.srcIdx = sidx;
		if (idx != kInvalidIndex)
		{
			c.tarIdx = static_cast<int>(idx);
			c.position = tarPoints_.col(static_cast<Eigen::Index>(idx));
			c.minDist2 = static_cast<Scalar>(distSq);
		}
		else
		{
			c.tarIdx = 0;
			c.position = tarPoints_.col(0);
			c.minDist2 = static_cast<Scalar>(maxDistSq);
		}
		corres[static_cast<std::size_t>(i)] = c;
	}
}

void SpareSolver::simplePruning(std::vector<SpareCorrespondence>& corres) const
{
	if (!params_->useDistanceReject && !params_->useNormalReject)
	{
		return;
	}

	std::vector<SpareCorrespondence> kept;
	kept.reserve(corres.size());
	for (const SpareCorrespondence& c : corres)
	{
		const Vector3 srcPos = source_->positions[static_cast<std::size_t>(c.srcIdx)];
		const Scalar dist = (srcPos - c.position).norm();
		const Vector3 srcNormal = source_->normals[static_cast<std::size_t>(c.srcIdx)];
		const Scalar denom = srcNormal.norm() * c.normal.norm();
		Scalar angle = 0.0;
		if (denom > 1e-12)
		{
			angle = std::acos(std::clamp(srcNormal.dot(c.normal) / denom, -1.0, 1.0));
		}

		const bool passDistance = !params_->useDistanceReject || dist < params_->distanceThreshold;
		const bool passNormal = !params_->useNormalReject || !source_->hasFaces() || angle < params_->normalThreshold;
		if (passDistance && passNormal)
		{
			kept.push_back(c);
		}
	}
	corres = std::move(kept);
}

Scalar SpareSolver::computeMeanCorrespondenceError(const std::vector<SpareCorrespondence>& corres) const
{
	if (corres.empty())
	{
		return 0.0;
	}
	Scalar sum = 0.0;
	for (const SpareCorrespondence& c : corres)
	{
		const Vector3 srcPos = deformedPoints_.segment(c.srcIdx * 3, 3);
		const Vector3 d = srcPos - c.position;
		const Scalar nLen = c.normal.norm();
		// 有目标法线时用法向间隙；否则退回欧氏距离
		sum += (nLen > static_cast<Scalar>(1e-12)) ? std::abs(d.dot(c.normal / nLen)) : d.norm();
	}
	return sum / static_cast<Scalar>(corres.size());
}

void SpareSolver::graphCoarseReg(const Scalar nu1)
{
	VectorX prevV = VectorX::Zero(nSrcVertex_ * 3);
	bool runOnce = true;

	optimizeWAlign_ = 1.0;
	optimizeWSmo_ =
		params_->wSmo / static_cast<Scalar>(regCoeffB_.rows()) * static_cast<Scalar>(samplingIndices_.size());
	optimizeWRot_ = params_->wRot / static_cast<Scalar>(numSampleNodes_) * static_cast<Scalar>(samplingIndices_.size());
	optimizeWArap_ =
		params_->wArapCoarse / static_cast<Scalar>(arapCoeff_.rows()) * static_cast<Scalar>(samplingIndices_.size());

	wAlign_ = optimizeWAlign_;
	wSmo_ = optimizeWSmo_;
	if (params_->dataUseRobustWeight)
	{
		wAlign_ = optimizeWAlign_ * (2.0 * nu1 * nu1);
	}

	const RowMajorSparseMatrix aFixedCoeff =
		optimizeWSmo_ * regCoeffB_.transpose() * regCwiseWeights_.asDiagonal() * regCoeffB_ +
		optimizeWRot_ * rigidCoeffL_ + optimizeWArap_ * arapCoeffMul_;

	int outIter = 0;
	while (outIter < params_->maxOuterIters)
	{
		correspondencePairs_.clear();
		findClosestPoints(correspondencePairs_, deformedPoints_, samplingIndices_);

		corresU0_.setZero();
		weightD_.resize(static_cast<Eigen::Index>(correspondencePairs_.size()));
		weightD_.setConstant(-1.0);

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(correspondencePairs_.size()); ++i)
		{
			const int srcIdx = correspondencePairs_[static_cast<std::size_t>(i)].srcIdx;
			corresU0_.segment(srcIdx * 3, 3) = correspondencePairs_[static_cast<std::size_t>(i)].position;
			weightD_[i] = correspondencePairs_[static_cast<std::size_t>(i)].minDist2;
			const int tarIdx = correspondencePairs_[static_cast<std::size_t>(i)].tarIdx;
			if (deformedNormals_.col(srcIdx).dot(targetNormals_.col(tarIdx)) < 0.0)
			{
				weightD_[i] = -1.0;
			}
		}

		if (params_->dataUseRobustWeight)
		{
			welschWeight(weightD_, nu1);
		}
		else
		{
			weightD_.setOnes();
		}

		if (params_->useSymmetricPointToPlane)
		{
			if (outIter == 0)
			{
				initNormalsSum();
			}
			else
			{
				calcNormalsSum();
			}

			diffUp_ = corresU0_ - nodesP_;
			const RowMajorSparseMatrix weightNpv = normalsSum_ * alignCoeffPv0_;
			matA0_ = optimizeWAlign_ * weightNpv.transpose() * weightD_.asDiagonal() * weightNpv + aFixedCoeff;
			calcArapRight();
			vecB_ = optimizeWAlign_ * weightNpv.transpose() * weightD_.asDiagonal() * normalsSum_ * diffUp_ +
					optimizeWSmo_ * regCoeffB_.transpose() * regCwiseWeights_.asDiagonal() * regRightD_ +
					optimizeWRot_ * rigidCoeffJ_ * nodesR_ + optimizeWArap_ * arapCoeff_.transpose() * arapRight_;
		}
		else
		{
			VectorX weightD3(nSrcVertex_ * 3);
			weightD3.setZero();
			for (int i = 0; i < nSrcVertex_; ++i)
			{
				const int idx = vertexSampleIndices_[static_cast<std::size_t>(i)];
				if (idx >= 0)
				{
					weightD3[i * 3] = weightD3[i * 3 + 1] = weightD3[i * 3 + 2] = weightD_[idx];
				}
			}

			diffUp_ = corresU0_ - nodesP_;
			matA0_ =
				optimizeWAlign_ * alignCoeffPv0_.transpose() * weightD3.asDiagonal() * alignCoeffPv0_ + aFixedCoeff;
			calcArapRight();
			vecB_ = optimizeWAlign_ * alignCoeffPv0_.transpose() * weightD3.asDiagonal() * diffUp_ +
					optimizeWSmo_ * regCoeffB_.transpose() * regCwiseWeights_.asDiagonal() * regRightD_ +
					optimizeWRot_ * rigidCoeffJ_ * nodesR_ + optimizeWArap_ * arapCoeff_.transpose() * arapRight_;
		}

		if (runOnce)
		{
			solver_.analyzePattern(matA0_);
			runOnce = false;
		}
		solver_.factorize(matA0_);
		X_ = solver_.solve(vecB_);

		deformedPoints_ = alignCoeffPv0_ * X_ + nodesP_;
		calcLocalRotations(true);
		calcNodeRotations();
		calcDeformedNormals();

		if ((deformedPoints_ - prevV).norm() / std::sqrt(static_cast<Scalar>(nSrcVertex_)) < params_->stopCoarse)
		{
			break;
		}
		prevV = deformedPoints_;
		++outIter;
	}
}

void SpareSolver::pointwiseFineReg(const Scalar nu1)
{
	VectorX prevV = VectorX::Zero(nSrcVertex_ * 3);
	bool runOnce = true;

	optimizeWAlign_ = 1.0;
	optimizeWArap_ = params_->wArapFine / static_cast<Scalar>(arapCoeffFine_.rows()) * static_cast<Scalar>(nSrcVertex_);

	wAlign_ = optimizeWAlign_;
	if (params_->dataUseRobustWeight)
	{
		wAlign_ = optimizeWAlign_ * (2.0 * nu1 * nu1);
	}

	int outIter = 0;
	while (outIter < params_->maxOuterIters)
	{
		findClosestPoints(correspondencePairs_, deformedPoints_);

		corresU0_.setZero();
		weightD_.resize(nSrcVertex_);

		for (int i = 0; i < nSrcVertex_; ++i)
		{
			corresU0_.segment(i * 3, 3) = correspondencePairs_[static_cast<std::size_t>(i)].position;
			weightD_[i] = correspondencePairs_[static_cast<std::size_t>(i)].minDist2;
			const int tarIdx = correspondencePairs_[static_cast<std::size_t>(i)].tarIdx;
			if (deformedNormals_.col(i).dot(targetNormals_.col(tarIdx)) < 0.0)
			{
				weightD_[i] = -1.0;
			}
		}

		if (params_->dataUseRobustWeight)
		{
			welschWeight(weightD_, nu1);
		}
		else
		{
			weightD_.setOnes();
		}

		if (params_->useSymmetricPointToPlane)
		{
			if (outIter == 0)
			{
				initNormalsSum();
			}
			else
			{
				calcNormalsSum();
			}

			const RowMajorSparseMatrix normalsSumMul = normalsSum_.transpose() * weightD_.asDiagonal() * normalsSum_;
			matA0_ = optimizeWAlign_ * normalsSumMul + optimizeWArap_ * arapCoeffMulFine_;
			calcArapRightFine();
			vecB_ = optimizeWAlign_ * normalsSumMul * corresU0_ +
					optimizeWArap_ * arapCoeffFine_.transpose() * arapRightFine_;
		}
		else
		{
			RowMajorSparseMatrix diagWeights;
			diagWeights.resize(nSrcVertex_ * 3, nSrcVertex_ * 3);
			std::vector<Triplet> coeffsDiag(static_cast<std::size_t>(nSrcVertex_) * 3U);
			VectorX weightCorresU(nSrcVertex_ * 3);

			for (int i = 0; i < nSrcVertex_; ++i)
			{
				coeffsDiag[static_cast<std::size_t>(i) * 3U] = Triplet(i * 3, i * 3, weightD_[i]);
				coeffsDiag[static_cast<std::size_t>(i) * 3U + 1U] = Triplet(i * 3 + 1, i * 3 + 1, weightD_[i]);
				coeffsDiag[static_cast<std::size_t>(i) * 3U + 2U] = Triplet(i * 3 + 2, i * 3 + 2, weightD_[i]);

				weightCorresU[i * 3] = weightD_[i] * corresU0_[i * 3];
				weightCorresU[i * 3 + 1] = weightD_[i] * corresU0_[i * 3 + 1];
				weightCorresU[i * 3 + 2] = weightD_[i] * corresU0_[i * 3 + 2];
			}
			diagWeights.setFromTriplets(coeffsDiag.begin(), coeffsDiag.end());

			matA0_ = optimizeWAlign_ * diagWeights + optimizeWArap_ * arapCoeffMulFine_;
			calcArapRightFine();
			vecB_ = optimizeWAlign_ * weightCorresU + optimizeWArap_ * arapCoeffFine_.transpose() * arapRightFine_;
		}

		if (runOnce)
		{
			solver_.analyzePattern(matA0_);
			runOnce = false;
		}
		solver_.factorize(matA0_);
		deformedPoints_ = solver_.solve(vecB_);

		calcLocalRotations(false);
		calcDeformedNormals();

		if ((deformedPoints_ - prevV).norm() / std::sqrt(static_cast<Scalar>(nSrcVertex_)) < params_->stopFine)
		{
			break;
		}
		prevV = deformedPoints_;
		++outIter;
	}
}

void SpareSolver::writeBackToSource()
{
#pragma omp parallel for
	for (int i = 0; i < nSrcVertex_; ++i)
	{
		source_->positions[static_cast<std::size_t>(i)] = deformedPoints_.segment(i * 3, 3);
		source_->normals[static_cast<std::size_t>(i)] = deformedNormals_.col(i);
		const double len = source_->normals[static_cast<std::size_t>(i)].norm();
		if (len > 1e-12)
		{
			source_->normals[static_cast<std::size_t>(i)] /= len;
		}
	}
}

bool SpareSolver::run()
{
	if (source_ == nullptr || target_ == nullptr || params_ == nullptr)
	{
		return false;
	}

	const Scalar nu1 = params_->dataInitK * params_->dataNu;

	if (params_->useCoarseReg)
	{
		graphCoarseReg(nu1);
	}

	if (params_->useFineReg)
	{
		pointwiseFineReg(nu1);
	}

	writeBackToSource();

	findClosestPoints(correspondencePairs_, deformedPoints_);
	lastMeanError_ = computeMeanCorrespondenceError(correspondencePairs_);
	return true;
}

} // namespace spare
} // namespace pclalgo
