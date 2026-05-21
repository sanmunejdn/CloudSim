#pragma once

#include "BackendRegistry.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

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
	pointCloudMeta.factory = []() { return std::static_pointer_cast<BackendDataBase>(std::make_shared<PointCloudBackendData>()); };
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
}
