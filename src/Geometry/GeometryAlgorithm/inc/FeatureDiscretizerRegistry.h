#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H

/// @file FeatureDiscretizerRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FeatureDiscretizerRegistry 接口

#include "geometry_algorithm_global.h"

#include "IFeatureDiscretizer.h"

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

namespace geoalgo
{
class GEOMETRY_ALGORITHM_API FeatureDiscretizerRegistry
{
public:
	// 公开构造供 ServiceRegistry 持有；全局唯一性不再由类强制
	FeatureDiscretizerRegistry() = default;

	/// 宿主注册 ServiceRegistry 时写入，使本 DLL instance() 与宿主同一对象
	static void setProcessInstance(FeatureDiscretizerRegistry* registry);

	/// 业务路径须先 setProcessInstance（组合根）
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
	// get 返回的裸指针在锁释放后仍有效：条目只增不删，unique_ptr 搬移不影响所指点对象
	mutable std::shared_mutex m_mutex;
	std::vector<std::unique_ptr<IFeatureDiscretizer>> m_discretizers;
};

GEOMETRY_ALGORITHM_API void ensureFeatureDiscretizersRegistered();

#define REGISTER_FEATURE_DISCRETIZER(DiscretizerType)                                                                 \
	static const bool DiscretizerType##_registered = []()                                                             \
	{                                                                                                                 \
		geoalgo::deferFeatureDiscretizerRegistration([]() { return std::make_unique<DiscretizerType>(); });             \
		return true;                                                                                                    \
	}()

/// 静态注册宏用：进程槽就绪前只入队，由 setProcessInstance / ensure 刷入
GEOMETRY_ALGORITHM_API void deferFeatureDiscretizerRegistration(
	std::function<std::unique_ptr<IFeatureDiscretizer>()> factory);

GEOMETRY_ALGORITHM_API void flushDeferredFeatureDiscretizerRegistrations();

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERREGISTRY_H
