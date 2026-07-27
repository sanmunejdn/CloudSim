#ifndef CLOUDSIMCORE_BACKENDTYPEIDS_H
#define CLOUDSIMCORE_BACKENDTYPEIDS_H

/// @file BackendTypeIds.h
/// @brief 后端类型三键与工程侧车键（契约层唯一真源）

#include <string>

/// className / catalog|sourceType / 显示名 / project.json 侧车键
namespace backend_type
{
inline constexpr const char* kClassPointCloud = "PointCloudBackendData";
inline constexpr const char* kClassModel = "Model";
inline constexpr const char* kClassBrepModel = "BrepModel";
inline constexpr const char* kClassParametricBrep = "ParametricBrepModel";
inline constexpr const char* kClassFrame = "FrameBackendData";

/// Visual 读路径兼容；禁止写入工程 JSON className
inline constexpr const char* kClassModelVisualAlias = "MeshBackendData";

inline constexpr const char* kCatalogPointCloud = "PointCloud";
inline constexpr const char* kCatalogModel = "Model";
inline constexpr const char* kCatalogBrepModel = "BrepModel";
inline constexpr const char* kCatalogParametricBrep = "ParametricBrepModel";
inline constexpr const char* kCatalogCoordinateFrame = "CoordinateFrame";

inline constexpr const char* kDisplayPointCloud = "PointCloud";
inline constexpr const char* kDisplayMesh = "Mesh";
inline constexpr const char* kDisplayBrepModel = "BrepModel";
inline constexpr const char* kDisplayParametricBrep = "ParametricBrep";
inline constexpr const char* kDisplayCoordinateFrame = "CoordinateFrame";

/// project.json 插件侧车根键
inline constexpr const char* kProjectKeyProcessFlow = "processFlow";
inline constexpr const char* kProjectKeyGeometricModeling = "geometricModeling";

inline bool isBrepWorkpieceClassName(const std::string& className)
{
	return className == kClassBrepModel || className == kClassParametricBrep;
}

inline bool isCoordinateFrameClassName(const std::string& className)
{
	return className == kClassFrame;
}

inline bool isPointCloudClassName(const std::string& className)
{
	return className == kClassPointCloud;
}

inline bool isMeshClassName(const std::string& className)
{
	return className == kClassModel || className == kClassModelVisualAlias;
}

inline bool isBuiltinClassName(const std::string& className)
{
	return className == kClassPointCloud || className == kClassModel || className == kClassBrepModel ||
		   className == kClassParametricBrep || className == kClassFrame;
}

/// className → catalog；未知回落 Model
inline std::string catalogTypeFromClassName(const std::string& className)
{
	if (className == kClassPointCloud)
	{
		return kCatalogPointCloud;
	}
	if (className == kClassBrepModel)
	{
		return kCatalogBrepModel;
	}
	if (className == kClassParametricBrep)
	{
		return kCatalogParametricBrep;
	}
	if (className == kClassFrame)
	{
		return kCatalogCoordinateFrame;
	}
	if (className == kClassModel || className == kClassModelVisualAlias)
	{
		return kCatalogModel;
	}
	return kCatalogModel;
}

} // namespace backend_type

#endif // CLOUDSIMCORE_BACKENDTYPEIDS_H
