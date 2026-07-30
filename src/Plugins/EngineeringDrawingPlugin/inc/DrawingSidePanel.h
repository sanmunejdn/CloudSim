#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSIDEPANEL_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSIDEPANEL_H

/// @file DrawingSidePanel.h
/// @brief 左侧：模型列表 + 可拖拽视角预览 + 图层

#include "DrawingSheetCanvasWidget.h"

#include <QPixmap>
#include <QPointer>
#include <QSize>
#include <QString>
#include <QVector>
#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QPushButton;

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
	void bindCanvas(DrawingSheetCanvasWidget* canvas);
	void applyLanguage(bool useChinese);

signals:
	void selectionChanged(const QString& backendId);
	void viewTemplateActivated(const QString& kind);

private:
	void rebuildViewList();
	void rebuildLayerList();
	void rebuildDetailList();
	void syncLayerStyleUi();
	QString selectedLayerId() const;

	QLabel* m_modelTitle = nullptr;
	QListWidget* m_modelList = nullptr;
	QLabel* m_viewTitle = nullptr;
	QListWidget* m_viewList = nullptr;
	QLabel* m_detailTitle = nullptr;
	QListWidget* m_detailList = nullptr;
	QPushButton* m_detailRenameBtn = nullptr;
	QPushButton* m_detailScaleBtn = nullptr;
	QPushButton* m_detailDeleteBtn = nullptr;
	QLabel* m_layerTitle = nullptr;
	QListWidget* m_layerList = nullptr;
	QPushButton* m_layerAddBtn = nullptr;
	QPushButton* m_layerRenameBtn = nullptr;
	QPushButton* m_layerDeleteBtn = nullptr;
	QPushButton* m_layerMoveBtn = nullptr;
	QPushButton* m_layerColorBtn = nullptr;
	QLabel* m_layerLineTypeLabel = nullptr;
	QComboBox* m_layerLineTypeCombo = nullptr;
	QLabel* m_layerWidthLabel = nullptr;
	QDoubleSpinBox* m_layerWidthSpin = nullptr;
	QStringList m_backendIds;
	QVector<DrawingViewTemplate> m_templates;
	QPointer<DrawingSheetCanvasWidget> m_canvas;
	bool m_useChinese = true;
	bool m_layerUiBusy = false;
};

#endif
