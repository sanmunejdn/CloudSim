#ifndef DATA_BACKENDIMPORTERS_H
#define DATA_BACKENDIMPORTERS_H

/// @file BackendImporters.h
/// @brief 后端几何文件导入自由函数（CGAL/OCCT IO 与数据类解耦）

#include "data_global.h"

#include <string>
#include <vector>

class BrepBackendData;
class MeshBackendData;
class PointCloudBackendData;
struct BrepHierarchyPart;
struct MeshHierarchyPart;

namespace geoalgo
{
class ShapeHandle;
}

namespace backend_io
{
DATA_EXPORT bool loadPointCloudFromFile(PointCloudBackendData& data, const std::string& path,
										std::string* errMsg = nullptr);

DATA_EXPORT bool loadMeshFromFile(MeshBackendData& mesh, const std::string& path, std::string* errMsg = nullptr,
								  int meshImportQuality = 1);

DATA_EXPORT bool loadBrepFromStepFile(BrepBackendData& brep, const std::string& path, std::string* errMsg = nullptr);

DATA_EXPORT bool loadMeshStepHierarchy(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
									   std::string* errMsg = nullptr);

DATA_EXPORT bool loadBrepStepHierarchy(const std::string& path, std::vector<BrepHierarchyPart>& outParts,
									   std::string* errMsg = nullptr, geoalgo::ShapeHandle* outAssembly = nullptr);
} // namespace backend_io

#endif // DATA_BACKENDIMPORTERS_H
