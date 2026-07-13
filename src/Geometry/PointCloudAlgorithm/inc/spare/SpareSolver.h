#pragma once

#include "KdTreePointSet.h"
#include "spare/SpareInternal.h"
#include "spare/SpareNodeSampler.h"

#include <Eigen/SparseCholesky>

#include <cstddef>
#include <memory>
#include <vector>

namespace pclalgo
{
namespace spare
{

class SpareSolver
{
public:
	bool init(SpareSurface& source, SpareSurface& target, SpareInternalParams& params);
	bool run();
	Scalar meanError() const { return lastMeanError_; }

private:
	void initWelschParam();
	void fullInArapCoeff();
	void initRotations();

	void calcNodeRotations();
	void calcArapRight();
	void calcArapRightFine();
	void calcLocalRotations(bool isCoarseAlign);
	void calcDeformedNormals();
	void initNormalsSum();
	void calcNormalsSum();
	void welschWeight(VectorX& r, Scalar p) const;

	void graphCoarseReg(Scalar nu1);
	void pointwiseFineReg(Scalar nu1);

	void initCorrespondence(std::vector<SpareCorrespondence>& corres);
	void findClosestPoints(std::vector<SpareCorrespondence>& corres);
	void findClosestPoints(std::vector<SpareCorrespondence>& corres, const VectorX& deformedV);
	void findClosestPoints(
		std::vector<SpareCorrespondence>& corres,
		const VectorX& deformedV,
		const std::vector<std::size_t>& sampleIndices);
	void simplePruning(std::vector<SpareCorrespondence>& corres) const;

	Scalar computeMeanCorrespondenceError(const std::vector<SpareCorrespondence>& corres) const;
	void writeBackToSource();
	void buildTargetKdTree();

	static void surfaceToFloatXyz(const SpareSurface& surface, std::vector<float>& xyzOut);
	static void buildSrcKnnIndices(
		const Matrix3X& srcPoints,
		int knnCount,
		Eigen::MatrixXi& outIndices);

	SpareSurface* source_ = nullptr;
	SpareSurface* target_ = nullptr;
	SpareInternalParams* params_ = nullptr;

	int nSrcVertex_ = 0;
	int nTarVertex_ = 0;
	int numSampleNodes_ = 0;
	int knnNeighborCount_ = 6;
	std::size_t alignSampleCount_ = 3000;

	SpareNodeSampler nodeSampler_;
	std::unique_ptr<KdTreePointSet> targetTree_;

	Matrix3X srcPoints_;
	Matrix3X srcNormals_;
	Matrix3X deformedNormals_;
	Matrix3X targetNormals_;
	Matrix3X tarPoints_;
	VectorX deformedPoints_;

	Eigen::MatrixXi srcKnnIndices_;

	VectorX weightD_;
	VectorX corresU0_;
	VectorX nodesP_;
	VectorX nodesR_;
	VectorX diffUp_;
	VectorX regRightD_;
	VectorX regCwiseWeights_;
	VectorX arapRight_;
	VectorX arapRightFine_;
	VectorX arapLaplaceWeights_;
	VectorX X_;

	RowMajorSparseMatrix alignCoeffPv0_;
	RowMajorSparseMatrix regCoeffB_;
	RowMajorSparseMatrix rigidCoeffL_;
	RowMajorSparseMatrix rigidCoeffJ_;
	RowMajorSparseMatrix arapCoeff_;
	RowMajorSparseMatrix arapCoeffMul_;
	RowMajorSparseMatrix arapCoeffFine_;
	RowMajorSparseMatrix arapCoeffMulFine_;
	RowMajorSparseMatrix normalsSum_;
	RowMajorSparseMatrix matA0_;

	Matrix3X localRotations_;
	VectorX vecB_;

	std::vector<SpareCorrespondence> correspondencePairs_;
	std::vector<std::size_t> samplingIndices_;
	std::vector<int> vertexSampleIndices_;

	Scalar optimizeWAlign_ = 1.0;
	Scalar optimizeWSmo_ = 0.0;
	Scalar optimizeWRot_ = 0.0;
	Scalar optimizeWArap_ = 0.0;
	Scalar wAlign_ = 1.0;
	Scalar wSmo_ = 0.0;
	Scalar lastMeanError_ = 0.0;

	bool useGeodesicMode_ = false;

	Eigen::SimplicialLDLT<RowMajorSparseMatrix, Eigen::Lower> solver_;
};

} // namespace spare
} // namespace pclalgo
