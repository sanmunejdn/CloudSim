#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSIDEPANEL_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSIDEPANEL_H

/// @file DrawingSidePanel.h
/// @brief 左侧：模型列表 + 可拖拽视角预览

#include "DrawingSheetCanvasWidget.h"

#include <QPixmap>
#include <QSize>
#include <QString>
#include <QVector>
#include <QWidget>

class QLabel;
class QListWidget;

using DrawingViewTemplate = DrawingSheetCanvasWidget::ViewTemplate;

/// 拖放到图幅的 MIME
inline const char* drawingViewMimeType()
{
	return "application/x-cloudsim-drawing-view";
}

QPixmap renderDrawingViewThumbnail(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visible,
								   const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidden,
								   const QSize& size);

class DrawingSidePanel final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingSidePanel(QWidget* parent = nullptr);

	void setBackends(const QStringList& displayNames, const QStringList& backendIds);
	QString selectedBackendId() const;
	void setViewTemplates(const QVector<DrawingViewTemplate>& templates);
	const QVector<DrawingViewTemplate>& viewTemplates() const { return m_templates; }
	void applyLanguage(bool useChinese);

signals:
	void selectionChanged(const QString& backendId);
	void viewTemplateActivated(const QString& kind);

private:
	void rebuildViewList();

	QLabel* m_modelTitle = nullptr;
	QListWidget* m_modelList = nullptr;
	QLabel* m_viewTitle = nullptr;
	QListWidget* m_viewList = nullptr;
	QStringList m_backendIds;
	QVector<DrawingViewTemplate> m_templates;
	bool m_useChinese = true;
};

#endif
