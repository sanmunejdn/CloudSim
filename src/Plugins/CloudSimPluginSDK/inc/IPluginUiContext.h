#ifndef CLOUDSIMPLUGINSDK_IPLUGINUICONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINUICONTEXT_H

/// @file IPluginUiContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief UI 注册与工作区模式窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include <QString>
#include <QStringList>
#include <QWidget>
#include <functional>

class QAction;
class QDockWidget;
class QMenu;

/// UI 上下文：dock/侧栏/菜单注册、中央 alternate、视口嵌入、工作区模式
class IPluginUiContext
{
public:
	virtual ~IPluginUiContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// 浮动 dock（左/下）；右侧用 registerSidePanelTab
	virtual QDockWidget* registerDockWidget(const QString& title, QWidget* widget,
											Qt::DockWidgetArea area = Qt::LeftDockWidgetArea) = 0;

	/// 右侧面板 Tab 父 widget；UI 未就绪可为 null
	virtual QWidget* sidePanelTabParent() const = 0;

	/// 右侧 Workspace/AI 旁加 Tab，避开仿真 UI；≥0 成功（含按偏好隐藏、尚未插入页签）
	virtual int registerSidePanelTab(const char* titleUtf8, QWidget* widget) = 0;
	virtual void unregisterSidePanelTab(QWidget* widget) = 0;

	/// 更新已注册侧栏 Tab 标题（UTF-8）
	virtual void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) = 0;

	/// 创建菜单路径并返回叶菜单
	virtual QMenu* registerMenuPath(const QStringList& path) = 0;
	virtual QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler) = 0;

	/// 活动文档中央 alternate（流程画布等）
	virtual void setCentralAlternateWidget(QWidget* widget) = 0;
	virtual void showCentralScene3D() = 0;
	virtual void showCentralAlternate() = 0;
	virtual bool isShowingCentralAlternate() const = 0;

	/// 通用 alternate 侧栏（左=节点库+属性，右=报表）；旧名 enterProcessFlowSideUi 由聚合接口转发
	virtual void enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel) = 0;
	virtual void exitAlternateSideUi() = 0;

	/// 把活动文档 3D 视口嵌入建模页中区槽；退出时 restore
	virtual bool embedActiveRenderWidget(QWidget* slot, QString* outError = nullptr) = 0;
	virtual void restoreActiveRenderWidget() = 0;

	/// 菜单栏下方独占模式工具条（nullptr 清除）；几何建模等插件挂 Ribbon
	virtual void setModeToolBar(QWidget* toolBar) = 0;

	/// 互斥工作区模式。进入工艺流程/几何建模时 claim；其它插件在回调里只清本地状态
	virtual void claimWorkspaceMode(const QString& modeId) = 0;
	virtual void onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback) = 0;
	virtual QString currentWorkspaceMode() const = 0;

	/// 注册工作区模式供顶栏分段；主程序 modeId 为空串由宿主内建
	virtual void registerWorkspaceMode(const QString& modeId, const QString& titleZh, const QString& titleEn,
									   std::function<void()> enterFn) = 0;
	/// 退回主程序（清 Ribbon、中央 3D、退出 alternate 侧栏）
	virtual void returnToMainWorkspace() = 0;
	/// 按已注册 modeId 调用 enterFn（顶栏分段点击）
	virtual void enterWorkspaceMode(const QString& modeId) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINUICONTEXT_H
