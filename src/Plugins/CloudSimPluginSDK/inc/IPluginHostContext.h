#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginBackendMeta.h"
#include "PluginPrimitiveTypes.h"

#include <functional>
#include <memory>
#include <vector>

#include <QDockWidget>
#include <QString>
#include <QStringList>
#include <QWidget>

class QAction;
class ICloudSimPlugin;
class IAiAssistantHost;
class IPluginDocument;
class IPluginPointCloudHost;
class QMenu;

using PluginJobProgressFn = std::function<void(double fraction, const QString& message)>;

class IPluginHostContext
{
public:
	virtual ~IPluginHostContext() = default;

	virtual unsigned int hostVersion() const = 0;
	virtual QString applicationDirPath() const = 0;

	virtual void logInfo(const QString& message) const = 0;
	virtual void logWarn(const QString& message) const = 0;
	virtual void logError(const QString& message) const = 0;

	virtual int documentCount() const = 0;
	virtual IPluginDocument* activeDocument() = 0;
	virtual const IPluginDocument* activeDocument() const = 0;
	virtual IPluginDocument* documentAt(int index) = 0;
	virtual const IPluginDocument* documentAt(int index) const = 0;

	/// 活动文档切换时 UI 线程回调
	virtual void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) = 0;

	/// 投递 fn 到 UI 线程（已在则直跑）
	virtual void invokeOnUiThread(std::function<void()> fn) = 0;

	/// JobSystem 后台任务；onFinished 回 UI 线程
	virtual void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished) = 0;

	/// 浮动 dock（左/下）；右侧用 registerSidePanelTab
	virtual QDockWidget* registerDockWidget(const QString& title, QWidget* widget,
		Qt::DockWidgetArea area = Qt::LeftDockWidgetArea) = 0;

	/// 右侧面板 Tab 父 widget；UI 未就绪可为 null
	virtual QWidget* sidePanelTabParent() const = 0;

	/// 右侧 Workspace/AI 旁加 Tab，避开仿真 UI
	virtual int registerSidePanelTab(const char* titleUtf8, QWidget* widget) = 0;
	virtual void unregisterSidePanelTab(QWidget* widget) = 0;

	/// 创建菜单路径并返回叶菜单
	virtual QMenu* registerMenuPath(const QStringList& path) = 0;
	virtual QAction* registerAction(QMenu* menu, const QString& text,
		std::function<void()> handler) = 0;

	/// Phase1：宿主建 MeshBackendData+OSG（同 AI create-mesh）
	virtual bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params,
		const PluginPrimitiveMeshQuality& quality, const PluginMeshCreateOptions& options,
		QString* outError) = 0;

	/// Phase2：向 BackendRegistry 注册插件后端类型
	virtual bool registerBackendType(const PluginBackendMeta& meta, QString* outError) = 0;

	/// Phase2：三角 soup 注册 mesh（每三角 9 float，mm）
	virtual bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
		QString* outError) = 0;

	/// 导入到活动文档（mesh/point cloud 各走宿主路径）
	virtual std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
		std::string* outError = nullptr) = 0;

	/// 1.2.0+：点云算法宿主；宿主版本不足时可为 null
	virtual IPluginPointCloudHost* pointCloudHost() = 0;
	virtual const IPluginPointCloudHost* pointCloudHost() const = 0;

	/// 1.3.0+：AI 助手宿主（CloudSimAiSDK）；未链 AiSDK 时可为 null
	virtual IAiAssistantHost* aiAssistantHost() = 0;
	virtual const IAiAssistantHost* aiAssistantHost() const = 0;

	/// 与主窗口 Settings → Language 一致（默认中文）
	virtual bool useChinese() const = 0;

	/// 语言切换时 UI 线程回调（Settings → Language）
	virtual void onLanguageChanged(std::function<void(bool useChinese)> callback) = 0;

	/// 更新已注册侧栏 Tab 标题（UTF-8）
	virtual void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) = 0;
};
