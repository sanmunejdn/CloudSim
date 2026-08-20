#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H

/// @file FeatureDiscretizerConfigRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FeatureDiscretizerConfigRegistry 接口

#include "geometry_algorithm_global.h"

#include "FeatureListDocument.h"
#include "IFeatureDiscretizerConfig.h"

#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

namespace geoalgo
{
class GEOMETRY_ALGORITHM_API FeatureDiscretizerConfigRegistry
{
public:
	static FeatureDiscretizerConfigRegistry& instance();

	void registerConfig(std::unique_ptr<IFeatureDiscretizerConfig> config);

	bool ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg = nullptr);
	const std::string& resourceBaseDir() const { return m_resourceBaseDir; }

	std::vector<FeatureDiscretizerParamField> paramFieldsForStrategy(const std::string& strategyId) const;
	nlohmann::json defaultParamsForStrategy(const std::string& strategyId) const;

	FeatureDiscretizerConfigRegistry(const FeatureDiscretizerConfigRegistry&) = delete;
	FeatureDiscretizerConfigRegistry& operator=(const FeatureDiscretizerConfigRegistry&) = delete;
	FeatureDiscretizerConfigRegistry(FeatureDiscretizerConfigRegistry&&) = delete;
	FeatureDiscretizerConfigRegistry& operator=(FeatureDiscretizerConfigRegistry&&) = delete;
	~FeatureDiscretizerConfigRegistry() = default;

private:
	FeatureDiscretizerConfigRegistry() = default;

	const IFeatureDiscretizerConfig* configFor(const std::string& strategyId) const;

	std::string m_resourceBaseDir;
	bool m_loaded = false;
	std::vector<std::unique_ptr<IFeatureDiscretizerConfig>> m_configs;
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H
