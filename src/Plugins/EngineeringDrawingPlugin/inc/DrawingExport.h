#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGEXPORT_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGEXPORT_H

/// @file DrawingExport.h
/// @brief 图纸 SVG / 简易 DXF 导出

#include "DrawingSheetCanvasWidget.h"
#include "SheetSketchAdapter.h"

#include <QString>
#include <QVector>

namespace drawing_export
{
bool writeSvg(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<SheetSketchPolyline>& sketch);
bool writeDxf(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<SheetSketchPolyline>& sketch);
} // namespace drawing_export

#endif
