#ifndef DATA_BACKENDREGISTRYBUILTINS_H
#define DATA_BACKENDREGISTRYBUILTINS_H

/// @file BackendRegistryBuiltins.h
/// @brief 进程内一次性注册 Mesh / PointCloud 等内置类型

#include "BackendRegistry.h"
#include "BackendTypeIdentity.h"
#include "BrepBackendData.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "ParametricBrepBackendData.h"
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
	pointCloudMeta.className = backend_type::kClassPointCloud;
	pointCloudMeta.displayName = backend_type::kDisplayPointCloud;
	pointCloudMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<PointCloudBackendData>()); };
	pointCloudMeta.supportsTransform = true;
	pointCloudMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(pointCloudMeta);

	BackendMeta meshMeta;
	meshMeta.className = backend_type::kClassModel;
	meshMeta.displayName = backend_type::kDisplayMesh;
	meshMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<MeshBackendData>()); };
	meshMeta.supportsTransform = true;
	meshMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(meshMeta);

	BackendMeta brepMeta;
	brepMeta.className = backend_type::kClassBrepModel;
	brepMeta.displayName = backend_type::kDisplayBrepModel;
	brepMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<BrepBackendData>()); };
	brepMeta.supportsTransform = true;
	brepMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(brepMeta);

	BackendMeta parametricMeta;
	parametricMeta.className = backend_type::kClassParametricBrep;
	parametricMeta.displayName = backend_type::kDisplayParametricBrep;
	parametricMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<ParametricBrepBackendData>()); };
	parametricMeta.supportsTransform = true;
	parametricMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(parametricMeta);

	BackendMeta frameMeta;
	frameMeta.className = backend_type::kClassFrame;
	frameMeta.displayName = backend_type::kDisplayCoordinateFrame;
	frameMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<FrameBackendData>()); };
	frameMeta.supportsTransform = true;
	frameMeta.supportsVisibility = true;
	BackendRegistry::instance().registerType(frameMeta);
}

#endif // DATA_BACKENDREGISTRYBUILTINS_H
