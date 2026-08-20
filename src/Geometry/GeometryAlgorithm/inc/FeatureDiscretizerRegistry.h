#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H

/// @file FeatureDiscretizerRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FeatureDiscretizerRegistry 接口

#include "geometry_algorithm_global.h"

#include "IFeatureDiscretizer.h"

#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{
class GEOMETRY_ALGORITHM_API FeatureDiscretizerRegistry
{
public:
	static FeatureDiscretizerRegistry& instance();

	void registerDiscretizer(std::unique_ptr<IFeatureDiscretizer> discretizer);
	const IFeatureDiscretizer* get(const std::string& strategyId) const;
	std::vector<std::string> listStrategyIds() const;

	FeatureDiscretizerRegistry(const FeatureDiscretizerRegistry&) = delete;
	FeatureDiscretizerRegistry& operator=(const FeatureDiscretizerRegistry&) = delete;
	FeatureDiscretizerRegistry(FeatureDiscretizerRegistry&&) = delete;
	FeatureDiscretizerRegistry& operator=(FeatureDiscretizerRegistry&&) = delete;
	~FeatureDiscretizerRegistry() = default;

private:
	FeatureDiscretizerRegistry() = default;
	std::vector<std::unique_ptr<IFeatureDiscretizer>> m_discretizers;
};

GEOMETRY_ALGORITHM_API void ensureFeatureDiscretizersRegistered();

#define REGISTER_FEATURE_DISCRETIZER(DiscretizerType)                                                             \
	static const bool DiscretizerType##_registered = []()                                                         \
	{                                                                                                             \
		geoalgo::FeatureDiscretizerRegistry::instance().registerDiscretizer(std::make_unique<DiscretizerType>()); \
		return true;                                                                                              \
	}()

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H
