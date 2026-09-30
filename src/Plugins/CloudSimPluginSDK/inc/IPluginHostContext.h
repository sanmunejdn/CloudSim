#ifndef CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H

/// @file IPluginHostContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件宿主聚合门面：基础能力 + 窄上下文 getter（域 API 只在 IPlugin*Context）

#include "cloudsim_plugin_sdk_global.h"

#include <QString>
#include <functional>

class IPluginAiContext;
class IPluginDocumentContext;
class IPluginGeometryContext;
class IPluginJobContext;
class IPluginProjectContext;
class IPluginRobotContext;
class IPluginUiContext;

/// 聚合门面：日志/语言/路径 + 按域取窄上下文；勿再往本接口追加域虚函数
class IPluginHostContext
{
public:
	virtual ~IPluginHostContext() = default;

	virtual unsigned int hostVersion() const = 0;
	virtual QString applicationDirPath() const = 0;

	virtual void logInfo(const QString& message) const = 0;
	virtual void logWarn(const QString& message) const = 0;
	virtual void logError(const QString& message) const = 0;

	/// 与主窗口 Settings → Language 一致（默认中文）
	virtual bool useChinese() const = 0;

	/// 语言切换时 UI 线程回调（Settings → Language）
	virtual void onLanguageChanged(std::function<void(bool useChinese)> callback) = 0;

	virtual IPluginDocumentContext* documentContext() = 0;
	virtual IPluginUiContext* uiContext() = 0;
	virtual IPluginGeometryContext* geometryContext() = 0;
	virtual IPluginAiContext* aiContext() = 0;
	virtual IPluginRobotContext* robotContext() = 0;
	virtual IPluginJobContext* jobContext() = 0;
	virtual IPluginProjectContext* projectContext() = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H
