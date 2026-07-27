#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H

/// @file DrawingSheetCanvasWidget.h
/// @brief 工程图图幅：多视图、草图绘制、标注、拖拽、局部放大

#include "SheetSketchAdapter.h"

#include <QJsonObject>
#include <QPixmap>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>
#include <cmath>
#include <limits>

class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

enum class DrawingCanvasTool
{
	PanSelect = 0,
	LinearDim,
	DetailRegion,
	SketchLine,
	SketchRect,
	SketchCircle,
	SketchArc,
	SketchSpline,
	DimRadius,
	DimDiameter,
	SelectEntity
};

enum class DrawingProjectionMethod
{
	FirstAngle = 0,
	ThirdAngle
};

class DrawingSheetCanvasWidget final : public QWidget
{
	Q_OBJECT

public:
	struct Polyline2d
	{
		QVector<QPointF> points;
	};

	struct DrawingView
	{
		QString id;
		QString title;
		QString kind; ///< front|top|right|iso|section|detail
		QRectF frame;
		QVector<Polyline2d> visible;
		QVector<Polyline2d> hidden;
		double contentScale = 1.0;
		QString parentViewId;
	};

	struct SheetDimension
	{
		enum class Kind
		{
			Linear = 0,
			Radius,
			Diameter
		};
		Kind kind = Kind::Linear;
		QString id;
		QPointF p1;
		QPointF p2;
		QPointF textOffset{0.0, -12.0};
		int sketchEntityId = -1;
		double overrideValue = std::numeric_limits<double>::quiet_NaN();
	};

	using LinearDimension = SheetDimension;

	struct ViewTemplate
	{
		QString kind;
		QString title;
		QVector<Polyline2d> visible;
		QVector<Polyline2d> hidden;
		QPixmap thumbnail;
	};

	explicit DrawingSheetCanvasWidget(QWidget* parent = nullptr);

	void clearSheet();
	void setViews(const QVector<DrawingView>& views);
	QVector<DrawingView>& viewsMutable() { return m_views; }
	const QVector<DrawingView>& views() const { return m_views; }

	void setDimensions(const QVector<SheetDimension>& dims);
	const QVector<SheetDimension>& dimensions() const { return m_dims; }

	SheetSketchAdapter& sketch() { return m_sketch; }
	const SheetSketchAdapter& sketch() const { return m_sketch; }

	void setProjectionMethod(DrawingProjectionMethod m);
	DrawingProjectionMethod projectionMethod() const { return m_projection; }

	void setTool(DrawingCanvasTool tool);
	DrawingCanvasTool tool() const { return m_tool; }

	void setGridVisible(bool visible);
	void fitToView();
	void zoomIn();
	void zoomOut();
	void resetView();

	bool exportSvg(const QString& filePath) const;
	bool exportDxf(const QString& filePath) const;

	QJsonObject toJson() const;
	bool fromJson(const QJsonObject& root);
	bool isEmpty() const { return m_views.isEmpty() && m_dims.isEmpty() && m_sketch.document().lines().empty() &&
								  m_sketch.document().arcs().empty() && m_sketch.document().circles().empty() &&
								  m_sketch.document().splines().empty(); }

	QString backendId() const { return m_backendId; }
	void setBackendId(const QString& id) { m_backendId = id; }

	bool addDetailView(const QString& parentViewId, const QRectF& regionScene, double scale);

	void setViewCatalog(const QVector<ViewTemplate>& catalog);
	bool addCatalogViewAt(const QString& kind, const QPointF& sceneTopLeft);

signals:
	void sheetChanged();
	void viewChanged(double zoom);
	void statusMessage(const QString& text);

protected:
	void paintEvent(QPaintEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void keyPressEvent(QKeyEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dragMoveEvent(QDragMoveEvent* event) override;
	void dropEvent(QDropEvent* event) override;

private:
	QPointF sceneToWidget(const QPointF& scene) const;
	QPointF widgetToScene(const QPointF& widget) const;
	void clampZoom();
	QRectF contentBounds() const;
	void drawGrid(class QPainter& p) const;
	void drawView(class QPainter& p, const DrawingView& view) const;
	void drawSketch(class QPainter& p) const;
	void drawDimension(class QPainter& p, const SheetDimension& dim) const;
	void drawDimArrow(class QPainter& p, const QPointF& tipWidget, const QPointF& dirScene) const;
	int hitViewIndex(const QPointF& scenePos) const;
	int hitDimensionIndex(const QPointF& scenePos) const;
	void moveViewBy(int index, const QPointF& deltaScene);
	QVector<QPointF> collectViewSnapPoints() const;
	double snapTolMm() const;
	bool isSketchTool(DrawingCanvasTool t) const;
	void syncSketchTool();
	bool resolveCircleDim(int entityId, QPointF& center, QPointF& rim, double& radius) const;
	double dimensionValue(const SheetDimension& dim) const;
	QString dimensionText(const SheetDimension& dim) const;

	QVector<DrawingView> m_views;
	QVector<SheetDimension> m_dims;
	SheetSketchAdapter m_sketch;
	QVector<ViewTemplate> m_viewCatalog;
	QString m_backendId;
	DrawingProjectionMethod m_projection = DrawingProjectionMethod::FirstAngle;
	DrawingCanvasTool m_tool = DrawingCanvasTool::PanSelect;
	double m_zoom = 1.0;
	QPointF m_panOffset{40.0, 40.0};
	bool m_gridVisible = true;
	bool m_needInitialFit = true;
	bool m_panning = false;
	bool m_draggingView = false;
	int m_dragViewIndex = -1;
	bool m_dimPicking = false;
	int m_dimPickStep = 0;
	QPointF m_dimP1;
	QPointF m_dimP2;
	int m_dimEntityId = -1;
	bool m_detailDragging = false;
	QPointF m_detailStart;
	QPointF m_detailCurrent;
	QPointF m_lastWidgetPos;
	int m_nextDimId = 1;
	int m_nextDetailId = 1;
	int m_nextCatalogViewId = 1;
	int m_selectedSketchId = -1;
	int m_selectedDimIndex = -1;
};

QVector<DrawingSheetCanvasWidget::DrawingView> layoutEngineeringViews(
	DrawingProjectionMethod method, bool hasIso, bool hasSection,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& frontVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& frontHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& topVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& topHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& rightVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& rightHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& isoVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& isoHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& sectionVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& sectionHid);

#endif
