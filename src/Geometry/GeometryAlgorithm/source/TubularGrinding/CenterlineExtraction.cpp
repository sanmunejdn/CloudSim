#include "CenterlineExtraction.h"

#include "TubularGrinding.h"

#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

bool runCenterlineExtraction(
	const IndexedMeshLite& mesh,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	int& outSectionFitFailCount,
	std::string* errMsg,
	TubularCenterlinePcaAxis* outPcaAxis)
{
	outSamples.clear();
	outSectionFitFailCount = 0;
	if (outPcaAxis)
	{
		*outPcaAxis = TubularCenterlinePcaAxis{};
	}

	if (!runLaplacianSkeletonCenterline(mesh, params, outSamples, outPcaAxis))
	{
		if (errMsg)
		{
			*errMsg = "laplacian skeleton centerline extraction failed";
		}
		return false;
	}

	std::vector<TubularCenterlineSample> framed;
	buildFrenetFrames(outSamples, framed);
	outSamples = std::move(framed);

	return true;
}

} // namespace tg
} // namespace geoalgo
