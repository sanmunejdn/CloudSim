#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGPAGEWIDGET_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGPAGEWIDGET_H

/// @file DrawingPageWidget.h
/// @brief 中央工程图页：仅画布（出图选项在 Ribbon）

#include "DrawingSheetCanvasWidget.h"

#include <QWidget>

class DrawingPageWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingPageWidget(QWidget* parent = nullptr);

	DrawingSheetCanvasWidget* canvas() const { return m_canvas; }
	void applyLanguage(bool useChinese);

private:
	DrawingSheetCanvasWidget* m_canvas = nullptr;
};

#endif
