#ifndef CLOUDSIMHOST_GEOMETRYFILEIMPORTERREGISTRY_H
#define CLOUDSIMHOST_GEOMETRYFILEIMPORTERREGISTRY_H

/// @file GeometryFileImporterRegistry.h
/// @brief 几何文件导入器后缀注册表

#include "cloudsim_host_global.h"

#include "IGeometryFileImporter.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace cloudsim::host
{
class CLOUDSIM_HOST_EXPORT GeometryFileImporterRegistry
{
public:
	// 公开构造/析构供 ServiceRegistry 持有；全局唯一性不再由类强制
	GeometryFileImporterRegistry() = default;
	~GeometryFileImporterRegistry();

	// 兼容期入口：优先 ServiceRegistry，上下文未就绪回退静态实例
	static GeometryFileImporterRegistry& instance();

	GeometryFileImporterRegistry(const GeometryFileImporterRegistry&) = delete;
	GeometryFileImporterRegistry& operator=(const GeometryFileImporterRegistry&) = delete;

	void add(std::unique_ptr<IGeometryFileImporter> importer);
	const IGeometryFileImporter* find(const std::string& extensionLower) const;
	std::vector<std::string> allExtensions() const;

	void ensureBuiltinsRegistered();

private:
	struct Impl;
	Impl* m_impl = nullptr;
};

CLOUDSIM_HOST_EXPORT void registerBuiltinGeometryImporters(GeometryFileImporterRegistry& registry);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_GEOMETRYFILEIMPORTERREGISTRY_H
