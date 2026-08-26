#ifndef DATA_BACKENDREGISTRYBUILTINS_H
#define DATA_BACKENDREGISTRYBUILTINS_H

/// @file BackendRegistryBuiltins.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 进程内一次性注册 Mesh / PointCloud 等内置类型

#include "BackendRegistry.h"
#include "BackendTypeIdentity.h"
#include "BrepBackendData.h"
#include "CustomDeviceBackendData.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "ParametricBrepBackendData.h"
#include "PointCloudBackendData.h"

/// 进程内一次性注册 Mesh / PointCloud 等内置类型
inline void ensureBackendBuiltinsRegistered()
{
	static std::once_flag once;
	std::call_once(once,
				   []()
				   {
	BackendMeta pointCloudMeta;
	pointCloudMeta.className = backend_type::kClassPointCloud;
	pointCloudMeta.displayName = backend_type::kDisplayPointCloud;
	pointCloudMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<PointCloudBackendData>()); };
	BackendRegistry::instance().registerType(pointCloudMeta);

	BackendMeta meshMeta;
	meshMeta.className = backend_type::kClassModel;
	meshMeta.displayName = backend_type::kDisplayMesh;
	meshMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<MeshBackendData>()); };
	BackendRegistry::instance().registerType(meshMeta);

	BackendMeta brepMeta;
	brepMeta.className = backend_type::kClassBrepModel;
	brepMeta.displayName = backend_type::kDisplayBrepModel;
	brepMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<BrepBackendData>()); };
	BackendRegistry::instance().registerType(brepMeta);

	BackendMeta parametricMeta;
	parametricMeta.className = backend_type::kClassParametricBrep;
	parametricMeta.displayName = backend_type::kDisplayParametricBrep;
	parametricMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<ParametricBrepBackendData>()); };
	BackendRegistry::instance().registerType(parametricMeta);

	BackendMeta frameMeta;
	frameMeta.className = backend_type::kClassFrame;
	frameMeta.displayName = backend_type::kDisplayCoordinateFrame;
	frameMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<FrameBackendData>()); };
	BackendRegistry::instance().registerType(frameMeta);

	BackendMeta customDeviceMeta;
	customDeviceMeta.className = backend_type::kClassCustomDevice;
	customDeviceMeta.displayName = backend_type::kDisplayCustomDevice;
	customDeviceMeta.factory = []()
	{ return std::static_pointer_cast<BackendDataBase>(std::make_shared<CustomDeviceBackendData>()); };
	BackendRegistry::instance().registerType(customDeviceMeta);
				   });
}

#endif // DATA_BACKENDREGISTRYBUILTINS_H
