#ifndef CLOUDSIMPLUGINSDK_IPLUGINPROJECTCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINPROJECTCONTEXT_H

/// @file IPluginProjectContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工程钩子窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include <QJsonObject>
#include <QString>
#include <functional>
#include <string>

/// 工程上下文：工程保存/加载钩子、文件导入活动文档
class IPluginProjectContext
{
public:
	virtual ~IPluginProjectContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// 工程保存/加载扩展钩子（插件写入 processFlow 等）
	virtual void onProjectAboutToSave(std::function<void(const QString& documentId, QJsonObject& root)> callback) = 0;
	virtual void onProjectLoaded(std::function<void(const QString& documentId, const QJsonObject& root)> callback) = 0;

	/// 导入到活动文档（mesh/point cloud 各走宿主路径）
	virtual std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
													 std::string* outError = nullptr) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINPROJECTCONTEXT_H
