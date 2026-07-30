#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H

/// @file DrawingSheetCanvasWidget.h
/// @brief 工程图图幅：多视图、草图、图框、标注、局部放大

#include "SheetSketchAdapter.h"

#include <QColor>
#include <QJsonObject>
#include <QPen>
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
	SelectEntity,
	DimAngle,
	NoteLeader
};

enum class DrawingProjectionMethod
{
	FirstAngle = 0,
	ThirdAngle
};

enum class DrawingPaperSize
{
	A4 = 0,
	A3,
	A2,
	A1,
	A0,
	Custom
};

enum class SheetLineType
{
	Continuous = 0,
	Dashed,
	Center,
	DashDot
};

class DrawingSheetCanvasWidget final : public QWidget
{
	Q_OBJECT

public:
	struct Polyline2d
	{
		QVector<QPointF> points;
	};

	struct SheetLayer
	{
		QString id;
		QString name;
		bool visible = true;
		bool locked = false;
		QColor color = QColor(30, 35, 45);
		SheetLineType lineType = SheetLineType::Continuous;
		double lineWidthMm = 0.35;
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
		QString layerId = QStringLiteral("L0");
	};

	struct SheetDimension
	{
		enum class Kind
		{
			Linear = 0,
			Radius,
			Diameter,
			Angle
		};
		Kind kind = Kind::Linear;
		QString id;
		QPointF p1;
		QPointF p2;
		QPointF p3; ///< Angle: 第二射线终点；Linear 未用
		QPointF textOffset{0.0, -12.0};
		QString anchorViewId;
		QString layerId = QStringLiteral("L0");
		int sketchEntityId = -1;
		double overrideValue = std::numeric_limits<double>::quiet_NaN();
	};

	struct SheetNote
	{
		QString id;
		QPointF anchor;
		QPointF textPos;
		QString text;
		QString anchorViewId;
		QString layerId = QStringLiteral("L0");
	};

	struct SheetPaper
	{
		DrawingPaperSize size = DrawingPaperSize::A4;
		bool landscape = true;
		double customWidthMm = 297.0;
		double customHeightMm = 210.0;
		/// 图面 mm / 模型 mm；0.5 表示比例 1:2
		double sheetScale = 1.0;
		QString title;
		QString scaleText = QStringLiteral("1:1");
		QString date;
		bool visible = true;
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

	void setNotes(const QVector<SheetNote>& notes);
	const QVector<SheetNote>& notes() const { return m_notes; }

	SheetPaper& paperMutable() { return m_paper; }
	const SheetPaper& paper() const { return m_paper; }
	void setPaper(const SheetPaper& paper);
	QSizeF paperSizeMm() const;
	QRectF paperRect() const;
	QRectF paperDrawableRect() const;

	/// scale：图面/模型；rescaleContent 时按新旧比例改写场景几何
	void setSheetScale(double scale, bool rescaleContent = true);
	double sheetScale() const { return m_paper.sheetScale; }
	void syncScaleTextFromSheetScale();
	/// 按当前图幅有效区缩放并落位全部视图/标注
	bool fitViewsToPaper();
	/// 仅平移落位到图框内（不改比例）
	void placeViewsInPaper();
	/// 生成后：按 sheetScale 缩放并落位（假设当前几何为模型 1:1）
	void applySheetScaleFromModel();

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
	bool exportPdf(const QString& filePath);

	QJsonObject toJson() const;
	bool fromJson(const QJsonObject& root);
	bool isEmpty() const;

	QString backendId() const { return m_backendId; }
	void setBackendId(const QString& id);

	bool addDetailView(const QString& parentViewId, const QRectF& regionScene, double scale);
	bool removeView(const QString& viewId);
	bool renameView(const QString& viewId, const QString& title);
	bool setDetailViewScale(const QString& viewId, double scale);
	QVector<DrawingView> detailViews() const;

	void setDetailScale(double scale);
	double detailScale() const { return m_detailScale; }

	void setViewCatalog(const QVector<ViewTemplate>& catalog);
	bool addCatalogViewAt(const QString& kind, const QPointF& sceneTopLeft);

	static QString defaultLayerId() { return QStringLiteral("L0"); }
	const QVector<SheetLayer>& layers() const { return m_layers; }
	QString currentLayerId() const { return m_currentLayerId; }
	bool setCurrentLayer(const QString& layerId);
	QString addLayer(const QString& name);
	bool renameLayer(const QString& layerId, const QString& name);
	bool removeLayer(const QString& layerId);
	bool setLayerVisible(const QString& layerId, bool visible);
	bool setLayerLocked(const QString& layerId, bool locked);
	bool setLayerColor(const QString& layerId, const QColor& color);
	bool setLayerLineType(const QString& layerId, SheetLineType lineType);
	bool setLayerLineWidth(const QString& layerId, double widthMm);
	bool reassignSelectionToCurrentLayer();
	const SheetLayer* layerById(const QString& layerId) const;
	bool isLayerDrawable(const QString& layerId) const;
	bool isLayerEditable(const QString& layerId) const;
	static QString lineTypeToString(SheetLineType t);
	static SheetLineType lineTypeFromString(const QString& s);

signals:
	void sheetChanged();
	void layersChanged();
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
	void paintSheet(class QPainter& p, bool forExport) const;
	void drawGrid(class QPainter& p) const;
	void drawPaper(class QPainter& p) const;
	void drawView(class QPainter& p, const DrawingView& view) const;
	void drawSketch(class QPainter& p, bool interactive) const;
	void drawDimension(class QPainter& p, const SheetDimension& dim) const;
	void drawNote(class QPainter& p, const SheetNote& note) const;
	void drawDimArrow(class QPainter& p, const QPointF& tipWidget, const QPointF& dirScene) const;
	/// forceDashed：HLR 隐藏线强制虚线，仍用图层色宽
	QPen penForLayer(const QString& layerId, bool forceDashed = false, bool selected = false) const;
	int hitViewIndex(const QPointF& scenePos) const;
	int hitDimensionIndex(const QPointF& scenePos) const;
	int hitNoteIndex(const QPointF& scenePos) const;
	void moveViewBy(int index, const QPointF& deltaScene);
	QString inferAnchorViewId(const QPointF& scenePos) const;
	QVector<QPointF> collectViewSnapPoints() const;
	double snapTolMm() const;
	bool isSketchTool(DrawingCanvasTool t) const;
	void syncSketchTool();
	bool resolveCircleDim(int entityId, QPointF& center, QPointF& rim, double& radius) const;
	bool resolveHlrCircleNear(const QPointF& scenePos, QPointF& center, QPointF& rim, double& radius) const;
	double dimensionValue(const SheetDimension& dim) const;
	QString dimensionText(const SheetDimension& dim) const;
	void ensureDefaultLayer();
	int layerIndex(const QString& layerId) const;
	QString uniqueLayerName(const QString& base) const;
	int hitSketchEntity(const QPointF& scenePos, bool requireEditable) const;
	void migrateLayerEntities(const QString& fromId, const QString& toId);
	void scaleSceneContent(double factor);
	QRectF viewsContentBounds() const;

	QVector<DrawingView> m_views;
	QVector<SheetDimension> m_dims;
	QVector<SheetNote> m_notes;
	QVector<SheetLayer> m_layers;
	QString m_currentLayerId = QStringLiteral("L0");
	int m_nextLayerSeq = 1;
	SheetPaper m_paper;
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
	QPointF m_dimP3;
	int m_dimEntityId = -1;
	bool m_notePicking = false;
	QPointF m_noteAnchor;
	bool m_detailDragging = false;
	QPointF m_detailStart;
	QPointF m_detailCurrent;
	QPointF m_lastWidgetPos;
	int m_nextDimId = 1;
	int m_nextNoteId = 1;
	int m_nextDetailId = 1;
	int m_nextCatalogViewId = 1;
	double m_detailScale = 2.0;
	int m_selectedSketchId = -1;
	int m_selectedDimIndex = -1;
	int m_selectedNoteIndex = -1;
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
