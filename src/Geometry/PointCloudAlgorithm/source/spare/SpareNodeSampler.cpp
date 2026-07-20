/// @file SpareNodeSampler.cpp
/// @brief SpareNodeSampler 实现

#include "spare/SpareNodeSampler.h"

#include "KdTreePointSet.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include <Eigen/Eigenvalues>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pclalgo
{
namespace spare
{
namespace
{
Scalar edgeLength(const Matrix3X& points, int v0, int v1)
{
	return (points.col(v0) - points.col(v1)).norm();
}

// 网格上测地距离，用于节点覆盖半径内的权重分配
void dijkstraWithinRadius(const SpareSurface& surface, const int seed, const Scalar maxDist,
						  std::vector<std::pair<int, Scalar>>& outVerts)
{
	const std::size_t n = surface.vertexCount();
	std::vector<Scalar> dist(n, std::numeric_limits<Scalar>::max());
	using QueueEntry = std::pair<Scalar, int>;
	std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;

	dist[static_cast<std::size_t>(seed)] = 0.0;
	pq.push({0.0, seed});

	while (!pq.empty())
	{
		const Scalar d = pq.top().first;
		const int v = pq.top().second;
		pq.pop();
		if (d > dist[static_cast<std::size_t>(v)])
		{
			continue;
		}
		if (d > maxDist)
		{
			continue;
		}
		outVerts.emplace_back(v, d);

		for (const int nb : surface.vertexNeighbors[static_cast<std::size_t>(v)])
		{
			const Scalar nd =
				d + (surface.positions[static_cast<std::size_t>(v)] - surface.positions[static_cast<std::size_t>(nb)])
						.norm();
			if (nd < dist[static_cast<std::size_t>(nb)] && nd <= maxDist)
			{
				dist[static_cast<std::size_t>(nb)] = nd;
				pq.push({nd, nb});
			}
		}
	}
}

Scalar falloffWeight(const Scalar dist, const Scalar radius)
{
	const Scalar ratio = dist / radius;
	return std::pow(1.0 - ratio * ratio, 3);
}

std::vector<int> pcaSortVertices(const Matrix3X& srcPoints)
{
	const Vector3 means = srcPoints.rowwise().mean();
	const Matrix3X demeaned = srcPoints.colwise() - means;
	const Matrix33 covariance = demeaned * demeaned.transpose();

	Eigen::SelfAdjointEigenSolver<Matrix33> eigensolver(covariance);
	Vector3 eigenValues = eigensolver.eigenvalues();
	int maxIdx = 2;
	if (eigenValues[0] > eigenValues[maxIdx])
	{
		maxIdx = 0;
	}
	if (eigenValues[1] > eigenValues[maxIdx])
	{
		maxIdx = 1;
	}
	const Vector3 mainAxis = eigensolver.eigenvectors().col(maxIdx);
	const VectorX projection = demeaned.transpose() * mainAxis;

	std::vector<int> order(static_cast<std::size_t>(projection.size()));
	for (Eigen::Index i = 0; i < projection.size(); ++i)
	{
		order[static_cast<std::size_t>(i)] = static_cast<int>(i);
	}
	std::sort(order.begin(), order.end(),
			  [&projection](const int a, const int b) { return projection(a) < projection(b); });
	return order;
}

} // namespace

Scalar SpareNodeSampler::computeAverageEdgeLength(const Matrix3X& srcPoints, const Eigen::MatrixXi* knnIndices) const
{
	if (knnIndices == nullptr)
	{
		return 0.0;
	}

	Scalar sum = 0.0;
	const int knnRows = knnIndices->rows();
	const int cols = knnIndices->cols();
	const std::size_t count = static_cast<std::size_t>(cols) * static_cast<std::size_t>(knnRows);
	for (int i = 0; i < cols; ++i)
	{
		for (int j = 0; j < knnRows; ++j)
		{
			sum += edgeLength(srcPoints, i, (*knnIndices)(j, i));
		}
	}
	return count > 0U ? sum / static_cast<Scalar>(count) : 0.0;
}

void SpareNodeSampler::finalizeNodeGraph(const Matrix3X& srcPoints, const int numNodeNeighbors)
{
	(void)srcPoints;
	(void)numNodeNeighbors;
}

Scalar SpareNodeSampler::sampleAndConstructMesh(const SpareSurface& surface, const Matrix3X& srcPoints,
												const Scalar sampleRadiusRatio)
{
	nodeContainer_.clear();
	vertexGraph_.clear();
	nodeGraph_.clear();

	vertexCount_ = surface.vertexCount();
	Scalar avgEdgeLen = 0.0;
	std::size_t edgeCount = 0;
	for (const auto& he : surface.halfEdges)
	{
		avgEdgeLen += (surface.positions[static_cast<std::size_t>(he.first)] -
					   surface.positions[static_cast<std::size_t>(he.second)])
						  .norm();
		++edgeCount;
	}
	if (edgeCount > 0U)
	{
		avgEdgeLen /= static_cast<Scalar>(edgeCount);
	}
	sampleRadius_ = sampleRadiusRatio * avgEdgeLen;

	const std::vector<int> projectionOrder = pcaSortVertices(srcPoints);
	vertexGraph_.resize(vertexCount_);

	std::vector<int> vertexNodeIdx(vertexCount_, -1);
	VectorX weightSum = VectorX::Zero(static_cast<Eigen::Index>(vertexCount_));
	std::size_t curNodeIdx = 0;

	for (const int vertexIdx : projectionOrder)
	{
		if (vertexNodeIdx[static_cast<std::size_t>(vertexIdx)] >= 0 ||
			!vertexGraph_[static_cast<std::size_t>(vertexIdx)].empty())
		{
			continue;
		}

		nodeContainer_.emplace_back(curNodeIdx, static_cast<std::size_t>(vertexIdx));
		vertexNodeIdx[static_cast<std::size_t>(vertexIdx)] = static_cast<int>(curNodeIdx);

		std::vector<std::pair<int, Scalar>> neighborVerts;
		dijkstraWithinRadius(surface, vertexIdx, sampleRadius_, neighborVerts);
		for (const auto& entry : neighborVerts)
		{
			const int neighIdx = entry.first;
			const Scalar geodist = entry.second;
			if (geodist < sampleRadius_)
			{
				const Scalar weight = falloffWeight(geodist, sampleRadius_);
				vertexGraph_[static_cast<std::size_t>(neighIdx)].emplace(curNodeIdx, weight);
				weightSum[neighIdx] += weight;
			}
		}
		++curNodeIdx;
	}

	nodeGraph_.resize(curNodeIdx);
	for (const int vertexIdx : projectionOrder)
	{
		for (auto& nodeEntry : vertexGraph_[static_cast<std::size_t>(vertexIdx)])
		{
			const std::size_t nodeIdx = nodeEntry.first;
			for (const auto& neighNode : vertexGraph_[static_cast<std::size_t>(vertexIdx)])
			{
				const std::size_t neighNodeIdx = neighNode.first;
				if (nodeIdx != neighNodeIdx)
				{
					nodeGraph_[nodeIdx].emplace(neighNodeIdx, 1.0);
				}
			}
			if (weightSum[vertexIdx] > 0.0)
			{
				nodeEntry.second /= weightSum[vertexIdx];
			}
		}
	}
	return sampleRadius_;
}

Scalar SpareNodeSampler::sampleAndConstructPointCloudFps(const SpareSurface& surface, const Matrix3X& srcPoints,
														 const Eigen::MatrixXi& srcKnnIndices,
														 const Scalar sampleRadiusRatio, const int numVertexNodes,
														 const int numNodeNeighbors)
{
	(void)surface;
	nodeContainer_.clear();
	vertexGraph_.clear();
	nodeGraph_.clear();

	vertexCount_ = static_cast<std::size_t>(srcPoints.cols());
	const int knnRows = srcKnnIndices.rows();

	Scalar avgEdgeLen = 0.0;
	for (int i = 0; i < srcPoints.cols(); ++i)
	{
		for (int j = 0; j < knnRows; ++j)
		{
			avgEdgeLen += edgeLength(srcPoints, i, srcKnnIndices(j, i));
		}
	}
	const std::size_t edgeCount = static_cast<std::size_t>(srcPoints.cols()) * static_cast<std::size_t>(knnRows);
	avgEdgeLen /= static_cast<Scalar>(edgeCount);
	sampleRadius_ = sampleRadiusRatio * avgEdgeLen;

	std::size_t startIndex = 0;
	VectorX minDistances = VectorX::Constant(srcPoints.cols(), std::numeric_limits<Scalar>::max());
	minDistances[static_cast<Eigen::Index>(startIndex)] = 0.0;
	nodeContainer_.emplace_back(0, startIndex);

	Scalar minimalDist = 1e10;
	int curNodeIdx = 1;
	while (minimalDist > sampleRadius_)
	{
#pragma omp parallel for
		for (int i = 0; i < srcPoints.cols(); ++i)
		{
			if (static_cast<std::size_t>(i) == startIndex)
			{
				continue;
			}
			const Scalar dist = (srcPoints.col(static_cast<Eigen::Index>(startIndex)) - srcPoints.col(i)).norm();
			if (dist < minDistances[i])
			{
				minDistances[i] = dist;
			}
		}

		int maxDistanceIndex = 0;
		minimalDist = minDistances.maxCoeff(&maxDistanceIndex);
		minDistances[maxDistanceIndex] = 0.0;
		startIndex = static_cast<std::size_t>(maxDistanceIndex);
		nodeContainer_.emplace_back(static_cast<std::size_t>(curNodeIdx), startIndex);
		++curNodeIdx;
	}

	Matrix3X nodePositions(3, curNodeIdx);
	for (int i = 0; i < curNodeIdx; ++i)
	{
		const int vidx = getNodeVertexIdx(static_cast<std::size_t>(i));
		nodePositions.col(i) = srcPoints.col(vidx);
	}

	std::vector<float> nodeXyz;
	nodeXyz.reserve(static_cast<std::size_t>(curNodeIdx) * 3U);
	for (int i = 0; i < curNodeIdx; ++i)
	{
		nodeXyz.push_back(static_cast<float>(nodePositions(0, i)));
		nodeXyz.push_back(static_cast<float>(nodePositions(1, i)));
		nodeXyz.push_back(static_cast<float>(nodePositions(2, i)));
	}
	const KdTreePointSet nodeTree(nodeXyz);

	vertexGraph_.resize(vertexCount_);
	VectorX weightSum = VectorX::Zero(static_cast<Eigen::Index>(vertexCount_));

#pragma omp parallel for
	for (int vidx = 0; vidx < srcPoints.cols(); ++vidx)
	{
		std::vector<std::size_t> outIndices;
		std::vector<double> outDistSq;
		nodeTree.findKNearest(srcPoints(0, vidx), srcPoints(1, vidx), srcPoints(2, vidx),
							  static_cast<unsigned int>(numVertexNodes), outIndices, outDistSq);

		Scalar localSum = 0.0;
		std::map<std::size_t, Scalar> localWeights;
		for (std::size_t k = 0; k < outIndices.size(); ++k)
		{
			const Scalar dist = std::sqrt(outDistSq[k]);
			const Scalar weight = falloffWeight(dist, sampleRadius_);
			localWeights[outIndices[k]] = weight;
			localSum += weight;
		}
		vertexGraph_[static_cast<std::size_t>(vidx)] = std::move(localWeights);
		weightSum[vidx] = localSum;
	}

#pragma omp parallel for
	for (int vidx = 0; vidx < srcPoints.cols(); ++vidx)
	{
		if (weightSum[vidx] > 0.0)
		{
			for (auto& neighNode : vertexGraph_[static_cast<std::size_t>(vidx)])
			{
				neighNode.second /= weightSum[vidx];
			}
		}
	}

	nodeGraph_.resize(static_cast<std::size_t>(curNodeIdx));
#pragma omp parallel for
	for (int nidx = 0; nidx < curNodeIdx; ++nidx)
	{
		std::vector<std::size_t> outIndices;
		std::vector<double> outDistSq;
		const int vidx = getNodeVertexIdx(static_cast<std::size_t>(nidx));
		nodeTree.findKNearest(srcPoints(0, vidx), srcPoints(1, vidx), srcPoints(2, vidx),
							  static_cast<unsigned int>(numNodeNeighbors + 1), outIndices, outDistSq);

		for (int k = 1; k < static_cast<int>(outIndices.size()) && k <= numNodeNeighbors; ++k)
		{
			nodeGraph_[static_cast<std::size_t>(nidx)].emplace(outIndices[static_cast<std::size_t>(k)], 1.0);
		}
	}
	return sampleRadius_;
}

void SpareNodeSampler::initWeight(const SpareSurface& surface, RowMajorSparseMatrix& matPv, VectorX& matP,
								  RowMajorSparseMatrix& matB, VectorX& matD, VectorX& smoothWeights) const
{
	std::vector<Triplet> coeff;
	matP.setZero();
	matP.resize(static_cast<Eigen::Index>(vertexCount_) * 3);

	for (std::size_t vertexIdx = 0; vertexIdx < vertexCount_; ++vertexIdx)
	{
		const Vector3 vi = surface.positions[vertexIdx];
		for (const auto& eachNeighbor : vertexGraph_[vertexIdx])
		{
			const std::size_t nodeIdx = eachNeighbor.first;
			const Scalar weight = vertexGraph_[vertexIdx].at(nodeIdx);
			const Vector3 pj = surface.positions[static_cast<std::size_t>(getNodeVertexIdx(nodeIdx))];

			for (int k = 0; k < 3; ++k)
			{
				const Eigen::Index row = static_cast<Eigen::Index>(vertexIdx) * 3 + k;
				const Eigen::Index colBase = static_cast<Eigen::Index>(nodeIdx) * 12;
				coeff.emplace_back(row, colBase + k, weight * (vi[0] - pj[0]));
				coeff.emplace_back(row, colBase + k + 3, weight * (vi[1] - pj[1]));
				coeff.emplace_back(row, colBase + k + 6, weight * (vi[2] - pj[2]));
				coeff.emplace_back(row, colBase + k + 9, weight * 1.0);
			}

			matP[static_cast<Eigen::Index>(vertexIdx) * 3] += weight * pj[0];
			matP[static_cast<Eigen::Index>(vertexIdx) * 3 + 1] += weight * pj[1];
			matP[static_cast<Eigen::Index>(vertexIdx) * 3 + 2] += weight * pj[2];
		}
	}
	matPv.resize(static_cast<Eigen::Index>(vertexCount_) * 3, static_cast<Eigen::Index>(nodeContainer_.size()) * 12);
	matPv.setFromTriplets(coeff.begin(), coeff.end());

	coeff.clear();
	const int maxEdgeNum = static_cast<int>(nodeContainer_.size()) * (static_cast<int>(nodeContainer_.size()) - 1);
	matB.resize(maxEdgeNum * 3, static_cast<Eigen::Index>(nodeContainer_.size()) * 12);
	matD.resize(maxEdgeNum * 3);
	smoothWeights.resize(maxEdgeNum * 3);
	smoothWeights.setZero();
	matD.setZero();

	int edgeId = 0;
	for (std::size_t nodeIdx = 0; nodeIdx < nodeContainer_.size(); ++nodeIdx)
	{
		const std::size_t vIdx0 = getNodeVertexIdx(nodeIdx);
		const Vector3 v0 = surface.positions[vIdx0];

		for (const auto& eachNeighbor : nodeGraph_[nodeIdx])
		{
			const std::size_t neighborIdx = eachNeighbor.first;
			const Vector3 v1 = surface.positions[static_cast<std::size_t>(getNodeVertexIdx(neighborIdx))];
			const Vector3 dv = v0 - v1;
			const int k = edgeId;

			for (int t = 0; t < 3; ++t)
			{
				const Eigen::Index row = k * 3 + t;
				const Eigen::Index neighborCol = static_cast<Eigen::Index>(neighborIdx) * 12;
				const Eigen::Index nodeCol = static_cast<Eigen::Index>(nodeIdx) * 12;
				coeff.emplace_back(row, neighborCol + t, dv[0]);
				coeff.emplace_back(row, neighborCol + t + 3, dv[1]);
				coeff.emplace_back(row, neighborCol + t + 6, dv[2]);
				coeff.emplace_back(row, neighborCol + t + 9, 1.0);
				coeff.emplace_back(row, nodeCol + t + 9, -1.0);
			}

			const Scalar dist = dv.norm();
			if (dist > 0.0)
			{
				smoothWeights[k * 3] = smoothWeights[k * 3 + 1] = smoothWeights[k * 3 + 2] = 1.0 / dist;
			}
			matD[k * 3] = dv[0];
			matD[k * 3 + 1] = dv[1];
			matD[k * 3 + 2] = dv[2];
			++edgeId;
		}
	}

	matB.setFromTriplets(coeff.begin(), coeff.end());
	matD.conservativeResize(edgeId * 3);
	matB.conservativeResize(edgeId * 3, matPv.cols());
	smoothWeights.conservativeResize(edgeId * 3);
	if (smoothWeights.sum() > 0.0)
	{
		smoothWeights *= static_cast<Scalar>(edgeId) / (smoothWeights.sum() / 3.0);
	}
}

} // namespace spare
} // namespace pclalgo
