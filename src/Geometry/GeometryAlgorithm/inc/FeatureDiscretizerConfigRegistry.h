#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H

/// @file FeatureDiscretizerConfigRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FeatureDiscretizerConfigRegistry 接口

// schema 进程一份（组合根注入）；resourceBaseDir 为线程加载态，由 ensureLoaded 写入 TLS
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
	FeatureDiscretizerConfigRegistry() = default;

	static void setProcessInstance(FeatureDiscretizerConfigRegistry* registry);
	/// 业务路径须先 setProcessInstance（组合根）
	static FeatureDiscretizerConfigRegistry& instance();

	void registerConfig(std::unique_ptr<IFeatureDiscretizerConfig> config);

	/// 写入当前线程加载态目录（多文档应在操作前用该文档的 baseDir 调用）
	bool ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg = nullptr);
	const std::string& resourceBaseDir() const;

	std::vector<FeatureDiscretizerParamField> paramFieldsForStrategy(const std::string& strategyId) const;
	nlohmann::json defaultParamsForStrategy(const std::string& strategyId) const;

	FeatureDiscretizerConfigRegistry(const FeatureDiscretizerConfigRegistry&) = delete;
	FeatureDiscretizerConfigRegistry& operator=(const FeatureDiscretizerConfigRegistry&) = delete;
	FeatureDiscretizerConfigRegistry(FeatureDiscretizerConfigRegistry&&) = delete;
	FeatureDiscretizerConfigRegistry& operator=(FeatureDiscretizerConfigRegistry&&) = delete;
	~FeatureDiscretizerConfigRegistry() = default;

private:
	const IFeatureDiscretizerConfig* configFor(const std::string& strategyId) const;

	std::vector<std::unique_ptr<IFeatureDiscretizerConfig>> m_configs;
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGREGISTRY_H
