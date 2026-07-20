/// @file FeatureDiscretizerBridge.cpp
/// @brief FeatureDiscretizerBridge 实现

#include "FeatureDiscretizerBridge.h"

#include "FeatureDiscretizerRegistry.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "detail/FeatureDiscretizeInternal.h"

namespace geoalgo
{
FeatureDiscretizerRegistry& featureDiscretizerRegistry()
{
	return FeatureDiscretizerRegistry::instance();
}

FeatureDiscretizerConfigRegistry& featureDiscretizerConfigRegistry()
{
	return FeatureDiscretizerConfigRegistry::instance();
}

bool ensureFeatureDiscretizerConfigsLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerConfigRegistry::instance().ensureLoaded(resourceBaseDir, errMsg);
}

const IFeatureDiscretizer* featureDiscretizerGet(const std::string& strategyId)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerRegistry::instance().get(strategyId);
}

std::vector<std::string> featureDiscretizerListStrategyIds()
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerRegistry::instance().listStrategyIds();
}

std::vector<FeatureDiscretizerParamField> featureDiscretizerAllParamFields(const std::string& strategyId)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerConfigRegistry::instance().paramFieldsForStrategy(strategyId);
}

bool discretizeFeatureList(const FeatureListDocument& doc, const ShapeHandle& shapeHandle, RawPath& out,
						   std::string* errMsg)
{
	ensureFeatureDiscretizersRegistered();
	if (!validateFeatureListDocument(doc, errMsg))
	{
		return false;
	}
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	return discretizeFeatureListInternal(doc, shape, out, errMsg);
}

bool discretizeFeatureList(const FeatureListDocument& doc, RawPath& out, std::string* errMsg)
{
	if (!validateFeatureListDocument(doc, errMsg))
	{
		return false;
	}
	if (doc.workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "workpiece.stepPathUtf8 is empty";
		}
		return false;
	}
	ShapeHandle handle;
	if (!readStepIntoHandle(doc.workpiece.stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return discretizeFeatureList(doc, handle, out, errMsg);
}

} // namespace geoalgo
