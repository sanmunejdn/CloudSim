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

#include <mutex>

namespace geoalgo
{
namespace
{
// 与 Data 层 BackendRegistryBuiltins 同一纪律：懒初始化用 call_once，不用裸 bool 检查-置位
std::once_flag g_builtinsOnce;

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
	std::call_once(g_builtinsOnce, []()
	{
		touchStaticDiscretizerRegistrations();
		registerDiscretizerConfigs();
	});
}

} // namespace geoalgo
