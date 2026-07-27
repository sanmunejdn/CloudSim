#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H

/// @file DrawingRibbonBar.h
/// @brief 工程图 Ribbon：视图 / 绘图 / 标注（对齐几何建模卡片分组）

#include "DrawingSheetCanvasWidget.h"

#include <QWidget>

class QButtonGroup;
class QLabel;
class QToolButton;

class DrawingRibbonBar final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingRibbonBar(QWidget* parent = nullptr);

	void setActiveTool(DrawingCanvasTool tool);

public slots:
	void applyTheme(bool dark);
	void applyLanguage(bool useChinese);

signals:
	void toolRequested(DrawingCanvasTool tool);

private:
	void rebuildIcons(bool dark);
	void setBtnText(QToolButton* btn, const QString& text);

	bool m_dark = false;
	bool m_useChinese = true;
	QLabel* m_lblView = nullptr;
	QLabel* m_lblSketch = nullptr;
	QLabel* m_lblMarks = nullptr;
	QButtonGroup* m_tools = nullptr;
	QToolButton* m_btnPan = nullptr;
	QToolButton* m_btnDetail = nullptr;
	QToolButton* m_btnLine = nullptr;
	QToolButton* m_btnArc = nullptr;
	QToolButton* m_btnCircle = nullptr;
	QToolButton* m_btnRect = nullptr;
	QToolButton* m_btnSpline = nullptr;
	QToolButton* m_btnSelect = nullptr;
	QToolButton* m_btnDimLinear = nullptr;
	QToolButton* m_btnDimRadius = nullptr;
	QToolButton* m_btnDimDiameter = nullptr;
};

#endif
