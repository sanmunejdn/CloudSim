#ifndef DATA_BACKENDREGISTRYBUILTINS_H
#define DATA_BACKENDREGISTRYBUILTINS_H

/// @file BackendRegistryBuiltins.h
/// @brief 进程内一次性注册 Mesh / PointCloud 等内置类型

#include "BackendRegistry.h"
#include "BrepBackendData.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

/// 进程内一次性注册 Mesh / PointCloud 等内置类型
inline void ensureBackendBuiltinsRegistered()
{
	static bool once = false;
	if (once)
	{
		return;
	}
	once = true;

	BackendMeta pointCloudMeta;
	pointCloudMeta.className = "PointCloudBackendData";
	pointCloudMeta.displayName = "PointCloud";
	pointCloudMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<PointCloudBackendData>()); };
	pointCloudMeta.supportsTransform = true;
	pointCloudMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(pointCloudMeta);

	BackendMeta meshMeta;
	meshMeta.className = "Model";
	meshMeta.displayName = "Mesh";
	meshMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<MeshBackendData>()); };
	meshMeta.supportsTransform = true;
	meshMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(meshMeta);

	BackendMeta brepMeta;
	brepMeta.className = "BrepModel";
	brepMeta.displayName = "BrepModel";
	brepMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<BrepBackendData>()); };
	brepMeta.supportsTransform = true;
	brepMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(brepMeta);

	BackendMeta frameMeta;
	frameMeta.className = "FrameBackendData";
	frameMeta.displayName = "CoordinateFrame";
	frameMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<FrameBackendData>()); };
	frameMeta.supportsTransform = true;
	frameMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(frameMeta);
}

#endif // DATA_BACKENDREGISTRYBUILTINS_H
