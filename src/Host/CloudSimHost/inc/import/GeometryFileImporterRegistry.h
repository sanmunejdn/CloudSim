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
	static GeometryFileImporterRegistry& instance();

	GeometryFileImporterRegistry(const GeometryFileImporterRegistry&) = delete;
	GeometryFileImporterRegistry& operator=(const GeometryFileImporterRegistry&) = delete;

	void add(std::unique_ptr<IGeometryFileImporter> importer);
	const IGeometryFileImporter* find(const std::string& extensionLower) const;
	std::vector<std::string> allExtensions() const;

	void ensureBuiltinsRegistered();

private:
	GeometryFileImporterRegistry() = default;
	~GeometryFileImporterRegistry();

	struct Impl;
	Impl* m_impl = nullptr;
};

CLOUDSIM_HOST_EXPORT void registerBuiltinGeometryImporters(GeometryFileImporterRegistry& registry);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_GEOMETRYFILEIMPORTERREGISTRY_H
