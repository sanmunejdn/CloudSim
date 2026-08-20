#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETCANVASWIDGET_H

/// @file DrawingSheetCanvasWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工程图图幅：多视图、草图、图框、标注、局部放大

#include "SheetSketchAdapter.h"
#include "SheetSnapEngine.h"

#include <QColor>
#include <QHash>
#include <QJsonObject>
#include <QLineF>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>
#include <cmath>
#include <limits>
#include <memory>

class DrawingSheetModel;

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
	NoteLeader,
	MatchProp,
	ModifyMove,
	ModifyCopy,
	ModifyRotate,
	ModifyMirror,
	ModifyErase,
	HatchPick,
	TextNote,
	ModifyTrim,
	ModifyOffset,
	ModifyScale,
	ModifyFillet,
	ModifyChamfer,
	ModifyArray,
	ModifyPolarArray,
	ModifyExtend,
	InsertBlock,
	ExplodeBlock,
	MText,
	ProjectionGuide,
	DimContinuous,
	DimBaseline,
	ModifyBreak,
	ModifyJoin,
	ModifyStretch,
	SymRoughness,
	SymGdt
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

enum class ViewAlignMode
{
	Left = 0,
	HCenter,
	Right,
	Top,
	VCenter,
	Bottom
};

/// AutoCAD 风格：ByLayer / ByBlock / 覆盖
struct SheetEntityStyle
{
	bool colorByLayer = true;
	bool lineTypeByLayer = true;
	bool lineWidthByLayer = true;
	bool colorByBlock = false;
	bool lineTypeByBlock = false;
	bool lineWidthByBlock = false;
	QColor color = QColor(30, 35, 45);
	SheetLineType lineType = SheetLineType::Continuous;
	double lineWidthMm = 0.35;
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
		bool frozen = false;
		bool plottable = true;
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
		/// 拖动视图时的绘制缓存，避免每帧逐点重建/圆拟合
		mutable QVector<QPolygonF> visCache;
		mutable QVector<QPolygonF> hidCache;
		double contentScale = 1.0;
		QString parentViewId;
		QString layerId = QStringLiteral("L0");
		SheetEntityStyle style;
		/// 剖视/局部字母（A、B…）；父视图上的剖切线或细节框对角点
		QString markLetter;
		QPointF markP1;
		QPointF markP2;
		bool hasMark = false;
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
		bool tolOverride = false;
		double tolPlus = 0.0;
		double tolMinus = 0.0;
		bool showTolerance = false;
		/// 弱关联：视图 id + 折线/点索引，重生后重绑 p1/p2
		QString anchorEdgeKey;
		/// 连续/基线尺寸链：首段 id；空表示独立或链头
		QString chainParentId;
		SheetEntityStyle style;
		QString styleId = QStringLiteral("Standard");
	};

	struct SheetNote
	{
		enum class Kind
		{
			Leader = 0,
			Roughness,
			Gdt
		};
		QString id;
		Kind kind = Kind::Leader;
		QPointF anchor;
		QPointF textPos;
		QString text;
		/// GD&T 代号：位置度/平行度/平面度…
		QString gdtCode;
		double roughnessRa = 3.2;
		QString anchorViewId;
		QString layerId = QStringLiteral("L0");
		SheetEntityStyle style;
		QString textStyleId = QStringLiteral("Standard");
	};

	struct SheetHatch
	{
		QString id;
		QVector<QPointF> boundary;
		QString pattern = QStringLiteral("SOLID");
		double scale = 1.0;
		double angleDeg = 0.0;
		QString layerId = QStringLiteral("L0");
		SheetEntityStyle style;
		QString anchorViewId; ///< 随视图平移（剖面填充）
	};

	/// 正视↔俯/右 投影引导线
	struct SheetProjectionGuide
	{
		enum class Axis
		{
			Horizontal = 0, ///< front↔top（锁 X）
			Vertical		///< front↔right（锁 Y）
		};
		QString id;
		QString fromViewId;
		QString toViewId;
		Axis axis = Axis::Horizontal;
		bool visible = true;
		QString layerId = QStringLiteral("L0");
		/// 用户拖过的端点（场景坐标）；未定制时按视图框自动算
		bool tipsCustom = false;
		QPointF tipA;
		QPointF tipB;
	};

	struct DimStyle
	{
		QString id = QStringLiteral("Standard");
		QString name = QStringLiteral("Standard");
		double textHeightMm = 3.5;
		double arrowSizeMm = 2.5;
		int precision = 2;
		double tolPlus = 0.0;
		double tolMinus = 0.0;
		bool showTolerance = false;
	};

	struct TextStyle
	{
		QString id = QStringLiteral("Standard");
		QString name = QStringLiteral("Standard");
		double heightMm = 3.5;
		QString fontFamily = QStringLiteral("Microsoft YaHei");
	};

	struct SheetBlockDef
	{
		QString id;
		QString name;
		QPointF base;
		QVector<Polyline2d> geometry;
		/// 与 geometry 平行；缺省按 ByBlock
		QVector<SheetEntityStyle> geometryStyles;
		struct AttrDef
		{
			QString tag;
			QString prompt;
			QString defaultValue;
			QPointF position; ///< 相对 base
		};
		QVector<AttrDef> attrDefs;
	};

	struct SheetBlockRef
	{
		QString id;
		QString defId;
		QPointF insert;
		double scale = 1.0;
		double rotationDeg = 0.0;
		QString layerId = QStringLiteral("L0");
		SheetEntityStyle style;
		QHash<QString, QString> attrValues;
	};

	struct CtbEntry
	{
		int aci = 7;
		double widthMm = 0.35;
	};

	struct SheetPaper
	{
		DrawingPaperSize size = DrawingPaperSize::A4;
		bool landscape = true;
		double customWidthMm = 297.0;
		double customHeightMm = 210.0;
		/// 图面 mm / 模型 mm；0.5 表示比例 1:2
		double sheetScale = 1.0;
		/// 线型比例（对齐 LTSCALE）
		double ltScale = 1.0;
		QString title;
		QString scaleText = QStringLiteral("1:1");
		QString date;
		QString drawingNo;
		QString material;
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
	~DrawingSheetCanvasWidget() override;

	void clearSheet();
	/// preserveLayout：按 kind 保留旧视图中心，并保留 detail 视图
	void setViews(const QVector<DrawingView>& views, bool preserveLayout = false);
	QVector<DrawingView>& viewsMutable() { return m_views; }
	const QVector<DrawingView>& views() const { return m_views; }

	void setDimensions(const QVector<SheetDimension>& dims);
	const QVector<SheetDimension>& dimensions() const { return m_dims; }

	void setNotes(const QVector<SheetNote>& notes);
	const QVector<SheetNote>& notes() const { return m_notes; }

	const QVector<SheetHatch>& hatches() const { return m_hatches; }
	const QVector<DimStyle>& dimStyles() const { return m_dimStyles; }
	const QVector<TextStyle>& textStyles() const { return m_textStyles; }
	const QVector<SheetBlockDef>& blockDefs() const { return m_blockDefs; }
	const QVector<SheetBlockRef>& blockRefs() const { return m_blockRefs; }
	const QVector<SheetProjectionGuide>& projectionGuides() const { return m_projectionGuides; }

	SheetPaper& paperMutable() { return m_paper; }
	const SheetPaper& paper() const { return m_paper; }
	void setPaper(const SheetPaper& paper);
	void setLtScale(double scale);
	double ltScale() const { return m_paper.ltScale; }
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
	bool importDxf(const QString& filePath);
	/// 打印预览位图（线宽按 ltScale/图层展开）
	QPixmap renderPrintPreview(const QSize& pixelSize) const;

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
	bool setLayerFrozen(const QString& layerId, bool frozen);
	bool setLayerPlottable(const QString& layerId, bool plottable);
	bool setLayerColor(const QString& layerId, const QColor& color);
	bool setLayerLineType(const QString& layerId, SheetLineType lineType);
	bool setLayerLineWidth(const QString& layerId, double widthMm);
	bool reassignSelectionToCurrentLayer();
	const SheetLayer* layerById(const QString& layerId) const;
	bool isLayerDrawable(const QString& layerId) const;
	bool isLayerEditable(const QString& layerId) const;
	static QString lineTypeToString(SheetLineType t);
	static SheetLineType lineTypeFromString(const QString& s);

	/// 当前选中实体的样式（无选中返回 false）
	bool selectionStyle(SheetEntityStyle& outStyle, QString& outLayerId) const;
	bool applyStyleToSelection(const SheetEntityStyle& style, const QString& layerId);
	bool matchPropFromSelection();
	void setSnapFlags(const SheetSnapFlags& flags);
	SheetSnapFlags snapFlags() const;
	int selectedViewIndex() const { return m_selectedViewIndex; }
	QVector<int> selectedViewIndices() const { return m_selectedViewIndices; }
	bool isViewSelected(int index) const;
	/// 多选时相对首个锚点对齐；仅选 1 个时相对图幅有效区对齐
	bool alignSelectedViews(ViewAlignMode mode);
	/// 正视图与俯/右视图按投影关系做中心对齐
	bool alignProjectionViews();
	void setProjectionDragLock(bool on);
	bool projectionDragLock() const { return m_projectionDragLock; }
	void setProjectionPinned(bool on);
	bool projectionPinned() const { return m_projectionPinned; }
	void rebuildProjectionGuides();
	bool removeProjectionGuideAt(const QPointF& scenePos);
	void setProjectionGuidesVisible(bool visible);
	bool beginGuideTipDrag(const QPointF& scenePos);
	void updateGuideTipDrag(const QPointF& scenePos);
	void endGuideTipDrag();
	bool beginSectionMarkDrag(const QPointF& scenePos);
	void updateSectionMarkDrag(const QPointF& scenePos);
	void endSectionMarkDrag();
	/// 正视中面：用剖切符号中点相对正视中心推切面原点偏移（sheet mm）
	bool sectionMarkOriginHint(double outOriginMm[3], double outNormal[3]) const;
	void setHalfSection(bool on);
	bool halfSection() const { return m_halfSection; }
	void applyHalfSectionClip();
	/// 清除 override，用几何重算；selectedOnly 时仅当前选中尺寸
	int recalculateDimensions(bool selectedOnly = false);
	bool setSelectedDimTolerance(bool show, double tolPlus, double tolMinus, bool useOverride);
	void rebindAssociatedDimensions();
	bool editCtbTable();
	void setCtbTable(const QVector<CtbEntry>& table);
	const QVector<CtbEntry>& ctbTable() const { return m_ctbTable; }
	bool syncTitleBlockAttrsFromPaper();
	int selectedDimIndex() const { return m_selectedDimIndex; }
	int selectedNoteIndex() const { return m_selectedNoteIndex; }
	int selectedSketchId() const { return m_selectedSketchId; }
	int selectedHatchIndex() const { return m_selectedHatchIndex; }
	int selectedBlockRefIndex() const { return m_selectedBlockRefIndex; }

	bool createBlockFromSelection(const QString& name);
	bool insertBlock(const QString& defId, const QPointF& scenePos);
	void setPendingInsertBlockId(const QString& defId) { m_pendingInsertBlockId = defId; }
	QString pendingInsertBlockId() const { return m_pendingInsertBlockId; }
	bool explodeSelectedBlock();
	bool addDimStyle(const DimStyle& style);
	bool updateDimStyle(const DimStyle& style);
	bool addTextStyle(const TextStyle& style);
	void setCurrentDimStyleId(const QString& id);
	QString currentDimStyleId() const { return m_currentDimStyleId; }
	bool editTitleBlockAttrs();
	/// CTB：按 ACI/灰度简化线宽倍率（打印预览用）
	void setCtbEnabled(bool on) { m_ctbEnabled = on; }
	bool ctbEnabled() const { return m_ctbEnabled; }
	bool isLayerPlottable(const QString& layerId) const;

signals:
	void sheetChanged();
	void layersChanged();
	void selectionChanged();
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
	void drawHatch(class QPainter& p, const SheetHatch& h) const;
	void drawBlockRef(class QPainter& p, const SheetBlockRef& r) const;
	void drawProjectionGuide(class QPainter& p, const SheetProjectionGuide& g) const;
	void drawViewMarks(class QPainter& p, const DrawingView& view) const;
	void drawDimArrow(class QPainter& p, const QPointF& tipWidget, const QPointF& dirScene) const;
	void drawSnapMarker(class QPainter& p, const QPointF& widgetPos, const QString& kind) const;
	void drawPickPointMarker(class QPainter& p, const QPointF& scenePos, const QString& label) const;
	void drawHoverEdge(class QPainter& p, const QLineF& seg) const;
	void drawDimToolPreview(class QPainter& p) const;
	void drawPickFeedback(class QPainter& p) const;
	void updatePickFeedback(const QPointF& scenePos);
	bool findNearestEdge(const QPointF& scenePos, QLineF& outSeg, int* outSketchId) const;
	bool isDimTool(DrawingCanvasTool t) const;
	bool isPickFeedbackTool(DrawingCanvasTool t) const;
	SheetSnapResult snapSceneResult(const QPointF& raw, const QPointF* orthoRef) const;
	QPen resolvePen(const QString& layerId, const SheetEntityStyle& style, bool forceDashed = false,
					bool selected = false, const SheetEntityStyle* blockCtx = nullptr,
					const QString* blockLayerId = nullptr) const;
	QPen penForLayer(const QString& layerId, bool forceDashed = false, bool selected = false) const;
	int hitViewIndex(const QPointF& scenePos) const;
	int hitDimensionIndex(const QPointF& scenePos) const;
	int hitNoteIndex(const QPointF& scenePos) const;
	int hitHatchIndex(const QPointF& scenePos) const;
	int hitBlockRefIndex(const QPointF& scenePos) const;
	void moveViewBy(int index, const QPointF& deltaScene);
	QPointF constrainViewDragDelta(int viewIndex, const QPointF& delta) const;
	void translateViewGeometry(DrawingView& v, const QPointF& delta);
	void ensureSectionHatchForView(DrawingView& sectionView);
	QString nextMarkLetter();
	int hitProjectionGuideIndex(const QPointF& scenePos) const;
	QLineF projectionGuideLine(const SheetProjectionGuide& g) const;
	QPointF snapGuideTipToAxis(const SheetProjectionGuide& g, const QPointF& raw) const;
	QString makeDimEdgeKey(const QString& viewId, int polyIndex, int ptIndex) const;
	bool resolveDimEdgeKey(const QString& key, QPointF& out) const;
	void assignDimEdgeKeys(SheetDimension& dim);
	double ctbWidthForColor(const QColor& color, double fallbackMm) const;
	void transformSelection(const QPointF& delta);
	void rotateSelection(const QPointF& pivot, double angleRad);
	void mirrorSelection(const QLineF& axis);
	void scaleSelection(const QPointF& pivot, double factor);
	void eraseSelection();
	bool trimSketchAt(const QPointF& scenePos);
	bool extendSketchAt(const QPointF& boundaryPos, const QPointF& linePos);
	bool offsetSketchAt(const QPointF& scenePos, double distMm);
	bool filletSketchAt(const QPointF& scenePos, double radiusMm);
	bool chamferSketchAt(const QPointF& scenePos, double distMm);
	bool breakSketchAt(const QPointF& scenePos);
	bool joinSketchAt(const QPointF& scenePos);
	bool stretchSketchWindow(const QRectF& win, const QPointF& delta);
	bool arraySelectionRect(int cols, int rows, double dx, double dy);
	bool arraySelectionPolar(int count, double angleDeg, const QPointF& pivot);
	bool refreshDetailViewFromMark(int viewIndex);
	void clearSelection();
	void emitSelectionChanged();
	bool duplicateSelection();
	void selectAtScene(const QPointF& scenePos);
	void setViewSelection(const QVector<int>& indices, int primary = -1);
	void toggleViewSelection(int index);
	int findViewIndexByKind(const QString& kind) const;
	QString inferAnchorViewId(const QPointF& scenePos) const;
	QVector<QPointF> collectViewSnapPoints() const;
	void rebuildSnapGeometry();
	QPointF snapScenePoint(const QPointF& raw, const QPointF* orthoRef) const;
	double snapTolMm() const;
	bool isSketchTool(DrawingCanvasTool t) const;
	bool isModifyTool(DrawingCanvasTool t) const;
	void syncSketchTool();
	bool resolveCircleDim(int entityId, QPointF& center, QPointF& rim, double& radius) const;
	bool resolveHlrCircleNear(const QPointF& scenePos, QPointF& center, QPointF& rim, double& radius) const;
	double dimensionValue(const SheetDimension& dim) const;
	QString dimensionText(const SheetDimension& dim) const;
	const DimStyle* dimStyleById(const QString& id) const;
	const TextStyle* textStyleById(const QString& id) const;
	void ensureDefaultStyles();
	void ensureDefaultLayer();
	int layerIndex(const QString& layerId) const;
	QString uniqueLayerName(const QString& base) const;
	int hitSketchEntity(const QPointF& scenePos, bool requireEditable) const;
	void migrateLayerEntities(const QString& fromId, const QString& toId);
	void scaleSceneContent(double factor);
	QRectF viewsContentBounds() const;
	static QJsonObject styleToJson(const SheetEntityStyle& s);
	static SheetEntityStyle styleFromJson(const QJsonObject& o);

	QVector<DrawingView> m_views;
	/// 场景 QPainterPath 缓存；缩放/平移只变换绘制
	std::unique_ptr<DrawingSheetModel> m_sheetModel;
	QVector<SheetDimension> m_dims;
	QVector<SheetNote> m_notes;
	QVector<SheetHatch> m_hatches;
	QVector<SheetBlockDef> m_blockDefs;
	QVector<SheetBlockRef> m_blockRefs;
	QVector<SheetProjectionGuide> m_projectionGuides;
	QVector<DimStyle> m_dimStyles;
	QVector<TextStyle> m_textStyles;
	QVector<SheetLayer> m_layers;
	QString m_currentLayerId = QStringLiteral("L0");
	int m_nextLayerSeq = 1;
	SheetPaper m_paper;
	SheetSketchAdapter m_sketch;
	SheetSnapEngine m_snapEngine;
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
	bool m_projectionDragLock = true;
	/// 整图纸场景缓存；视图/内容改动才失效，缩放/平移只重贴
	QPixmap m_sceneCache;
	bool m_sceneCacheValid = false;
	/// 滚轮缩放防抖：滚动中只贴旧缓存，停顿后再重绘
	QTimer* m_zoomDebounceTimer = nullptr;
	bool m_zoomRepaintPending = false;
	/// 拖动/平移期间用缓存位图平移，避免逐帧重绘全部视图几何
	QPixmap m_interactiveCache;
	bool m_interactiveCacheValid = false;
	bool m_projectionPinned = false;
	bool m_halfSection = false;
	bool m_dimPicking = false;
	int m_dimPickStep = 0;
	QPointF m_dimP1;
	QPointF m_dimP2;
	QPointF m_dimP3;
	int m_dimEntityId = -1;
	QString m_chainParentId;
	double m_chainBaseOffset = -12.0;
	int m_chainCount = 0;
	bool m_notePicking = false;
	QPointF m_noteAnchor;
	bool m_detailDragging = false;
	QPointF m_detailStart;
	QPointF m_detailCurrent;
	QPointF m_lastWidgetPos;
	bool m_extendPicking = false;
	QLineF m_extendBoundary;
	bool m_guideTipDragging = false;
	int m_guideTipIndex = -1;
	int m_guideTipWhich = 0; ///< 0=A 1=B
	bool m_sectionMarkDragging = false;
	int m_sectionMarkViewIndex = -1;
	int m_sectionMarkWhich = 0;
	int m_nextDimId = 1;
	int m_nextNoteId = 1;
	int m_nextDetailId = 1;
	int m_nextCatalogViewId = 1;
	int m_nextHatchId = 1;
	int m_nextBlockDefId = 1;
	int m_nextBlockRefId = 1;
	int m_nextGuideId = 1;
	int m_nextMarkLetterIdx = 0;
	QString m_currentDimStyleId = QStringLiteral("Standard");
	QString m_pendingInsertBlockId;
	bool m_ctbEnabled = false;
	QVector<CtbEntry> m_ctbTable;
	double m_detailScale = 2.0;
	int m_selectedSketchId = -1;
	int m_selectedDimIndex = -1;
	int m_selectedNoteIndex = -1;
	int m_selectedViewIndex = -1;
	QVector<int> m_selectedViewIndices;
	int m_selectedHatchIndex = -1;
	int m_selectedBlockRefIndex = -1;
	int m_selectedGuideIndex = -1;
	bool m_modifyPicking = false;
	int m_modifyStep = 0;
	QPointF m_modifyP1;
	QPointF m_modifyP2;
	SheetEntityStyle m_matchStyle;
	QString m_matchLayerId;
	bool m_hasMatchStyle = false;
	QVector<QPointF> m_hatchPickPts;
	QRectF m_stretchWindow;
	bool m_stretchHasWindow = false;

	/// 交互悬停：点/线/实体高亮，避免尺寸点选时无反馈
	struct PickHoverState
	{
		SheetSnapResult snap;
		int sketchId = -1;
		int dimIndex = -1;
		int noteIndex = -1;
		int viewIndex = -1;
		int hatchIndex = -1;
		int blockRefIndex = -1;
		bool hasEdge = false;
		QLineF edgeSeg;
		QString tip;
	};
	PickHoverState m_pickHover;
	QString m_lastPickTip;
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
