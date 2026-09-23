#ifndef CLOUDSIMHOST_IGEOMETRYFILEIMPORTER_H
#define CLOUDSIMHOST_IGEOMETRYFILEIMPORTER_H

/// @file IGeometryFileImporter.h
/// @brief 几何文件解析：纯虚 parse → ImportParseResult

#include "cloudsim_host_global.h"

#include "BrepBackendData.h"
#include "MeshBackendData.h"

#include <ShapeHandle.h>

#include <string>
#include <vector>

namespace cloudsim::host
{

enum class ImportParseKind
{
	MeshHierarchy = 0,
	MeshSingleSoup,
	BrepHierarchy,
	BrepSingle,
	OsgCapture
};

struct CLOUDSIM_HOST_EXPORT ImportParseOptions
{
	int meshImportQuality = 1;
};

struct CLOUDSIM_HOST_EXPORT ImportParseResult
{
	ImportParseKind kind = ImportParseKind::MeshSingleSoup;
	std::string displayNameHint;
	std::vector<MeshHierarchyPart> meshParts;
	std::vector<float> meshSoup;
	std::vector<BrepHierarchyPart> brepParts;
	geoalgo::ShapeHandle brepAssembly;
	geoalgo::ShapeHandle brepSingle;
	bool ok = false;
};

/// 按后缀选型的几何解析器；不碰 DocumentHost
class CLOUDSIM_HOST_EXPORT IGeometryFileImporter
{
public:
	virtual ~IGeometryFileImporter() = default;
	virtual std::vector<std::string> extensions() const = 0;
	virtual bool parse(const std::string& nativePath, const ImportParseOptions& opt, ImportParseResult& out,
					   std::string* errMsg) const = 0;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IGEOMETRYFILEIMPORTER_H
