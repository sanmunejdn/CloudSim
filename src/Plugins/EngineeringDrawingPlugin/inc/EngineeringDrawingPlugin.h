#ifndef ENGINEERINGDRAWINGPLUGIN_ENGINEERINGDRAWINGPLUGIN_H
#define ENGINEERINGDRAWINGPLUGIN_ENGINEERINGDRAWINGPLUGIN_H

/// @file EngineeringDrawingPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 三维 → 二维工程图独立插件

#include "DrawingSheetCanvasWidget.h"
#include "ICloudSimPlugin.h"

#include <QHash>
#include <QObject>
#include <QPointer>

class DrawingPageWidget;
class DrawingRibbonBar;
class DrawingSidePanel;
class DrawingInfoPanel;
class QAction;
class QJsonObject;
class QMenu;
struct PluginDrawingHlrResult;

class EngineeringDrawingPlugin : public QObject, public ICloudSimPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
	Q_INTERFACES(ICloudSimPlugin)

public:
	EngineeringDrawingPlugin() = default;
	~EngineeringDrawingPlugin() override;

	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

private:
	void registerMenus();
	void applyLanguage();
	void ensureRibbon();
	void enterDrawing();
	void exitDrawing();
	void softExitDrawing();
	void refreshBackendList();
	void generateViews();
	void refreshViewPreviews(const QString& backendId);
	void applyHlrResultToUi(DrawingPageWidget* page, const QString& backendId, bool thirdAngle, bool layoutSheet,
							const PluginDrawingHlrResult& result);
	DrawingPageWidget* ensurePageForDocument(const QString& documentId);
	DrawingPageWidget* ensurePageForActiveDocument();
	void bindPage(DrawingPageWidget* page);
	void onProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void onProjectLoaded(const QString& documentId, const QJsonObject& root);
	void applyToolToActiveCanvas(DrawingCanvasTool tool);

	IPluginHostContext* m_host = nullptr;
	QMenu* m_menu = nullptr;
	QAction* m_enterAction = nullptr;
	QAction* m_exitAction = nullptr;
	QPointer<DrawingSidePanel> m_side;
	QPointer<DrawingInfoPanel> m_info;
	QPointer<DrawingRibbonBar> m_ribbon;
	QHash<QString, QPointer<DrawingPageWidget>> m_pagesByDocId;
	bool m_inDrawing = false;
};

#endif
