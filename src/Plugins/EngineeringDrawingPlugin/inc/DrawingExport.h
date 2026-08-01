#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGEXPORT_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGEXPORT_H

/// @file DrawingExport.h
/// @brief 图纸 SVG / 简易 DXF / PDF 导出

#include "DrawingSheetCanvasWidget.h"
#include "SheetSketchAdapter.h"

#include <QHash>
#include <QString>
#include <QVector>

namespace drawing_export
{
bool writeSvg(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
			  const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper,
			  DrawingProjectionMethod projection, const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
			  const QHash<int, QString>& sketchLayers);
struct DxfSketchCircle
{
	QPointF center;
	double radius = 0.0;
	int entityId = -1;
};

struct DxfSketchArc
{
	QPointF center;
	double radius = 0.0;
	double startDeg = 0.0;
	double endDeg = 0.0;
	int entityId = -1;
};

bool writeDxf(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
			  const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper,
			  DrawingProjectionMethod projection, const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
			  const QHash<int, QString>& sketchLayers,
			  const QVector<DrawingSheetCanvasWidget::SheetHatch>& hatches = {},
			  const QVector<DrawingSheetCanvasWidget::SheetBlockDef>& blockDefs = {},
			  const QVector<DrawingSheetCanvasWidget::SheetBlockRef>& blockRefs = {},
			  const QVector<DxfSketchCircle>& sketchCircles = {},
			  const QVector<DxfSketchArc>& sketchArcs = {});
} // namespace drawing_export

#endif
