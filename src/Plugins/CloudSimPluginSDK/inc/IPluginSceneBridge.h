#ifndef CLOUDSIMPLUGINSDK_IPLUGINSCENEBRIDGE_H
#define CLOUDSIMPLUGINSDK_IPLUGINSCENEBRIDGE_H

/// @file IPluginSceneBridge.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 无 OSG 头的场景操作（4×4 列主序同 osg::Matrixd）

#include "cloudsim_plugin_sdk_global.h"

#include <array>
#include <string>

/// 无 OSG 头的场景操作（4×4 列主序同 osg::Matrixd）
class IPluginSceneBridge
{
public:
	virtual ~IPluginSceneBridge() = default;

	virtual bool setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
													  const std::array<double, 16>& columnMajor4x4) = 0;
	virtual bool getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
													  std::array<double, 16>& outColumnMajor4x4) const = 0;

	virtual void setBackendObjectVisible(const std::string& backendId, bool visible) = 0;
	virtual void removeBackendObjectVisual(const std::string& backendId) = 0;
	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINSCENEBRIDGE_H
