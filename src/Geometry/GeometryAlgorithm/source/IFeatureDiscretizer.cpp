#include "IFeatureDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"

namespace geoalgo
{

bool IFeatureDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (input.strategyId != strategyId())
	{
		if (errMsg)
		{
			*errMsg = "strategyId mismatch";
		}
		return false;
	}
	switch (affinity())
	{
	case GeometryAffinity::Line:
		if (input.geometry.edgeIndices.empty())
		{
			if (errMsg)
			{
				*errMsg = "Line strategy requires edgeIndices";
			}
			return false;
		}
		break;
	case GeometryAffinity::Face:
		if (input.geometry.faceIndices.empty())
		{
			if (errMsg)
			{
				*errMsg = "Face strategy requires faceIndices";
			}
			return false;
		}
		break;
	case GeometryAffinity::Any:
		break;
	}
	return true;
}

} // namespace geoalgo
