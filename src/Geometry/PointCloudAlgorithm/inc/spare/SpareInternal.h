#ifndef POINTCLOUDALGORITHM_SPAREINTERNAL_H
#define POINTCLOUDALGORITHM_SPAREINTERNAL_H

/// @file SpareInternal.h
/// @brief SpareInternal 接口

// SPARE 非刚性配准内部类型（移植自 yaoyx689/spare，研究用途）
// 论文: Symmetrized Point-to-Plane Distance for Robust Non-Rigid 3D Registration

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace pclalgo
{
namespace spare
{
using Scalar = double;
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Matrix33 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
using MatrixXX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using RowMajorSparseMatrix = Eigen::SparseMatrix<Scalar, Eigen::RowMajor>;
using Triplet = Eigen::Triplet<Scalar>;

struct SpareSurface
{
	std::vector<Vector3> positions;
	std::vector<Vector3> normals;
	std::vector<std::array<int, 3>> faces;
	std::vector<std::vector<int>> vertexNeighbors;
	std::vector<std::pair<int, int>> halfEdges;
	std::vector<int> soupCornerToVertex;

	bool hasFaces() const { return !faces.empty(); }
	std::size_t vertexCount() const { return positions.size(); }
};

struct SpareInternalParams
{
	int maxOuterIters = 30;
	Scalar wSmo = 0.01;
	Scalar wRot = 1e-4;
	Scalar wArapCoarse = 500.0;
	Scalar wArapFine = 200.0;
	bool useNormalReject = false;
	bool useDistanceReject = false;
	Scalar normalThreshold = static_cast<Scalar>(3.14159265358979323846 / 3.0);
	Scalar distanceThreshold = 0.05;
	bool dataUseRobustWeight = true;
	bool useSymmetricPointToPlane = true;
	Scalar dataNu = 0.0;
	Scalar dataInitK = 1.0;
	Scalar dataEndK = 1.0 / std::sqrt(3.0);
	Scalar stopCoarse = 1e-3;
	Scalar stopFine = 1e-4;
	Scalar uniSampleRatio = 5.0;
	bool useGeodesicDist = true;
	bool useCoarseReg = true;
	bool useFineReg = true;
	Scalar meshScale = 1.0;
	std::size_t alignSampleCount = 3000;
	int numSampleNodes = 0;
};

struct SpareCorrespondence
{
	int srcIdx = 0;
	int tarIdx = 0;
	Vector3 position = Vector3::Zero();
	Vector3 normal = Vector3::Zero();
	Scalar minDist2 = 0.0;
};

template <typename DerivedV, typename MType>
bool median(const Eigen::MatrixBase<DerivedV>& values, MType& outMedian)
{
	if (values.size() == 0)
	{
		return false;
	}
	std::vector<typename DerivedV::Scalar> buffer(values.size());
	for (Eigen::Index i = 0; i < values.size(); ++i)
	{
		buffer[static_cast<std::size_t>(i)] = values(i);
	}
	const std::size_t mid = buffer.size() / 2U;
	std::nth_element(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(mid), buffer.end());
	if (buffer.size() % 2U == 0U)
	{
		std::nth_element(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(mid - 1U), buffer.end());
		outMedian = static_cast<MType>(0.5 * (buffer[mid] + buffer[mid - 1U]));
	}
	else
	{
		outMedian = static_cast<MType>(buffer[mid]);
	}
	return true;
}

} // namespace spare
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SPAREINTERNAL_H
