#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGIMPL_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGIMPL_H

/// @file FeatureDiscretizerConfigImpl.h
/// @brief FeatureDiscretizerConfigImpl 接口

#include "IFeatureDiscretizerConfig.h"

#include <memory>
#include <string>

namespace geoalgo
{
class FeatureDiscretizerConfigImpl final : public IFeatureDiscretizerConfig
{
public:
	FeatureDiscretizerConfigImpl(std::string strategyId, std::string jsonRelativePath);

	std::string strategyId() const override { return m_strategyId; }
	std::string jsonRelativePath() const override { return m_jsonRelativePath; }
	std::vector<FeatureDiscretizerParamField> paramFields() const override;
	nlohmann::json defaultParams() const override;

private:
	std::string m_strategyId;
	std::string m_jsonRelativePath;
};

GEOMETRY_ALGORITHM_API std::unique_ptr<IFeatureDiscretizerConfig>
makeFeatureDiscretizerConfig(const char* strategyId, const char* jsonRelativePath);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERCONFIGIMPL_H
