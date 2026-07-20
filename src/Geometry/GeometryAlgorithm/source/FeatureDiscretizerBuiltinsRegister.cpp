/// @file FeatureDiscretizerBuiltinsRegister.cpp
/// @brief FeatureDiscretizerBuiltinsRegister 实现

#include "EdgeChainDiscretizer.h"
#include "FaceBoundaryDiscretizer.h"
#include "FaceIntersectionDiscretizer.h"
#include "FaceOffsetCurveDiscretizer.h"
#include "FaceParamSurfaceDiscretizer.h"
#include "FaceSectionDiscretizer.h"
#include "FeatureDiscretizerBridge.h"
#include "FeatureDiscretizerConfigImpl.h"
#include "FeatureDiscretizerRegistry.h"
#include "SyntheticPolylineDiscretizer.h"

namespace geoalgo
{
namespace
{
bool g_builtinsRegistered = false;

void registerDiscretizerConfigs()
{
	FeatureDiscretizerConfigRegistry& registry = FeatureDiscretizerConfigRegistry::instance();
	registry.registerConfig(makeFeatureDiscretizerConfig("EdgeChain", "discretizers/EdgeChain.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("FaceBoundary", "discretizers/FaceBoundary.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("FaceSection", "discretizers/FaceSection.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("FaceParamSurface", "discretizers/FaceParamSurface.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("FaceIntersection", "discretizers/FaceIntersection.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("FaceOffsetCurve", "discretizers/FaceOffsetCurve.json"));
	registry.registerConfig(makeFeatureDiscretizerConfig("SyntheticPolyline", "discretizers/SyntheticPolyline.json"));
}

void touchStaticDiscretizerRegistrations()
{
	(void)EdgeChainDiscretizer{};
	(void)FaceBoundaryDiscretizer{};
	(void)FaceSectionDiscretizer{};
	(void)FaceParamSurfaceDiscretizer{};
	(void)FaceIntersectionDiscretizer{};
	(void)FaceOffsetCurveDiscretizer{};
	(void)SyntheticPolylineDiscretizer{};
}

} // namespace

void ensureFeatureDiscretizersRegistered()
{
	if (g_builtinsRegistered)
	{
		return;
	}
	touchStaticDiscretizerRegistrations();
	registerDiscretizerConfigs();
	g_builtinsRegistered = true;
}

} // namespace geoalgo
