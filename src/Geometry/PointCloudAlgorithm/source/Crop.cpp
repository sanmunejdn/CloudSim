#include "Crop.h"

#include "PointCloudBuffer.h"

namespace pclalgo
{

namespace
{

template<typename Predicate>
void cropXyzImpl(
	const std::vector<float>& srcXyz,
	const std::vector<float>* srcRgba,
	Predicate keepPredicate,
	std::vector<float>& outXyz,
	std::vector<float>* outRgba,
	std::vector<std::size_t>* keptIndices)
{
	if (!validXyzLength(srcXyz))
	{
		outXyz.clear();
		if (outRgba != nullptr)
		{
			outRgba->clear();
		}
		return;
	}

	const std::size_t n = pointCountFromXyz(srcXyz);
	std::vector<std::size_t> kept;
	kept.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d p(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		if (keepPredicate(p))
		{
			kept.push_back(i);
		}
	}

	compactXyzByIndices(srcXyz, kept, outXyz);
	if (srcRgba != nullptr && outRgba != nullptr && validRgbaLength(*srcRgba, n))
	{
		compactRgbaByIndices(*srcRgba, kept, *outRgba);
	}
	else if (outRgba != nullptr)
	{
		outRgba->clear();
	}

	if (keptIndices != nullptr)
	{
		*keptIndices = std::move(kept);
	}
}

} // namespace

void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices)
{
	cropXyzImpl(
		srcXyz,
		nullptr,
		[&box](const Eigen::Vector3d& p) { return box.contains(p); },
		outXyz,
		nullptr,
		keptIndices);
}

void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcRgba,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::vector<std::size_t>* keptIndices)
{
	cropXyzImpl(
		srcXyz,
		&srcRgba,
		[&box](const Eigen::Vector3d& p) { return box.contains(p); },
		outXyz,
		&outRgba,
		keptIndices);
}

void cropXyzBySphere(
	const std::vector<float>& srcXyz,
	const Eigen::Vector3d& centerMm,
	const double radiusMm,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices)
{
	const double r2 = radiusMm * radiusMm;
	cropXyzImpl(
		srcXyz,
		nullptr,
		[&centerMm, r2](const Eigen::Vector3d& p) { return (p - centerMm).squaredNorm() <= r2; },
		outXyz,
		nullptr,
		keptIndices);
}

} // namespace pclalgo
