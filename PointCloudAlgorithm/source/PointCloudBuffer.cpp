#include "PointCloudBuffer.h"

namespace pclalgo
{

std::size_t pointCountFromXyz(const std::vector<float>& xyz)
{
	return xyz.size() / 3U;
}

std::size_t triangleCountFromSoup(const std::vector<float>& triangleSoup)
{
	return triangleSoup.size() / 9U;
}

bool validXyzLength(const std::vector<float>& xyz)
{
	return xyz.size() % 3U == 0U;
}

bool validRgbaLength(const std::vector<float>& rgba, const std::size_t pointCount)
{
	return rgba.empty() || rgba.size() == pointCount * 4U;
}

void compactXyzByIndices(
	const std::vector<float>& srcXyz,
	const std::vector<std::size_t>& keepIndices,
	std::vector<float>& outXyz)
{
	outXyz.clear();
	outXyz.reserve(keepIndices.size() * 3U);
	for (const std::size_t idx : keepIndices)
	{
		const std::size_t base = idx * 3U;
		if (base + 2U >= srcXyz.size())
		{
			continue;
		}
		outXyz.push_back(srcXyz[base]);
		outXyz.push_back(srcXyz[base + 1U]);
		outXyz.push_back(srcXyz[base + 2U]);
	}
}

void compactRgbaByIndices(
	const std::vector<float>& srcRgba,
	const std::vector<std::size_t>& keepIndices,
	std::vector<float>& outRgba)
{
	outRgba.clear();
	outRgba.reserve(keepIndices.size() * 4U);
	for (const std::size_t idx : keepIndices)
	{
		const std::size_t base = idx * 4U;
		if (base + 3U >= srcRgba.size())
		{
			continue;
		}
		outRgba.push_back(srcRgba[base]);
		outRgba.push_back(srcRgba[base + 1U]);
		outRgba.push_back(srcRgba[base + 2U]);
		outRgba.push_back(srcRgba[base + 3U]);
	}
}

} // namespace pclalgo
