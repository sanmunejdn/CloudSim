#ifndef CLOUDSIMPLUGINSDK_IPLUGINDOCUMENTCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINDOCUMENTCONTEXT_H

/// @file IPluginDocumentContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 文档管理窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include <QString>
#include <functional>

class IPluginDocument;
class IPluginLabelingHost;

/// 文档管理上下文：枚举/激活/关闭通知/未保存标记
class IPluginDocumentContext
{
public:
	virtual ~IPluginDocumentContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	virtual int documentCount() const = 0;
	virtual IPluginDocument* activeDocument() = 0;
	virtual const IPluginDocument* activeDocument() const = 0;
	virtual IPluginDocument* documentAt(int index) = 0;
	virtual const IPluginDocument* documentAt(int index) const = 0;

	/// 按 documentId 取文档；无则 null
	virtual IPluginDocument* documentById(const QString& documentId) = 0;
	virtual const IPluginDocument* documentById(const QString& documentId) const = 0;

	/// 活动文档切换时 UI 线程回调
	virtual void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) = 0;

	/// 文档 Tab 关闭（deleteLater 前）；默认 destroyOnClose
	virtual void onDocumentClosed(std::function<void(const QString& documentId)> callback) = 0;

	/// 活动文档未保存标记（页签 *）
	virtual void markActiveDocumentModified() = 0;
	virtual void clearActiveDocumentModified() = 0;
	virtual bool isActiveDocumentModified() const = 0;

	/// 分割标注宿主；宿主版本不足时可为 null
	virtual IPluginLabelingHost* labelingHost() = 0;
	virtual const IPluginLabelingHost* labelingHost() const = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINDOCUMENTCONTEXT_H
