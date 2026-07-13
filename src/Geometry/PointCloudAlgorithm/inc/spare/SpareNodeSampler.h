#pragma once

#include "spare/SpareInternal.h"

namespace pclalgo
{
namespace spare
{

class SpareNodeSampler
{
public:
	Scalar sampleAndConstructMesh(
		const SpareSurface& surface,
		const Matrix3X& srcPoints,
		Scalar sampleRadiusRatio);

	Scalar sampleAndConstructPointCloudFps(
		const SpareSurface& surface,
		const Matrix3X& srcPoints,
		const Eigen::MatrixXi& srcKnnIndices,
		Scalar sampleRadiusRatio,
		int numVertexNodes = 4,
		int numNodeNeighbors = 8);

	void initWeight(
		const SpareSurface& surface,
		RowMajorSparseMatrix& matPv,
		VectorX& matP,
		RowMajorSparseMatrix& matB,
		VectorX& matD,
		VectorX& smoothWeights) const;

	std::size_t nodeSize() const { return nodeContainer_.size(); }
	int getNodeVertexIdx(const std::size_t nodeIdx) const
	{
		return static_cast<int>(nodeContainer_.at(nodeIdx).second);
	}

	Scalar sampleRadius() const { return sampleRadius_; }

private:
	void finalizeNodeGraph(const Matrix3X& srcPoints, int numNodeNeighbors);
	Scalar computeAverageEdgeLength(
		const Matrix3X& srcPoints,
		const Eigen::MatrixXi* knnIndices) const;

	std::vector<std::pair<std::size_t, std::size_t>> nodeContainer_;
	std::vector<std::map<std::size_t, Scalar>> vertexGraph_;
	std::vector<std::map<std::size_t, Scalar>> nodeGraph_;
	Scalar sampleRadius_ = 0.0;
	std::size_t vertexCount_ = 0;
};

} // namespace spare
} // namespace pclalgo
