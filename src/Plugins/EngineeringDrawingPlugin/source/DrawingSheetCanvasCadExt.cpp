/// @file DrawingSheetCanvasCadExt.cpp
/// @brief AutoCAD 对齐扩展：样式解析、修改、捕捉、填充、图块、DXF 读入

#include "DrawingSheetCanvasWidget.h"

#include "DrawingSheetModel.h"

#include <QFile>
#include <QHash>
#include <QInputDialog>
#include <QLineEdit>
#include <QPainter>
#include <QPolygonF>
#include <QRegion>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QTextStream>
#include <QtMath>

#include <cmath>
#include <limits>

namespace
{
QPointF rotPt(const QPointF& p, const QPointF& pivot, double ang)
{
	const double c = std::cos(ang), s = std::sin(ang);
	const QPointF d = p - pivot;
	return pivot + QPointF(d.x() * c - d.y() * s, d.x() * s + d.y() * c);
}

QPointF mirrorPt(const QPointF& p, const QLineF& axis)
{
	QLineF n = axis.normalVector();
	n.setLength(1.0);
	QLineF a = axis;
	a.setLength(1.0);
	const QPointF d = p - axis.p1();
	const double along = d.x() * a.dx() + d.y() * a.dy();
	const double cross = d.x() * n.dx() + d.y() * n.dy();
	return axis.p1() + QPointF(a.dx(), a.dy()) * along - QPointF(n.dx(), n.dy()) * cross;
}
} // namespace

void DrawingSheetCanvasWidget::ensureDefaultStyles()
{
	if (m_dimStyles.isEmpty())
	{
		DimStyle d;
		m_dimStyles.push_back(d);
	}
	if (m_textStyles.isEmpty())
	{
		TextStyle t;
		m_textStyles.push_back(t);
	}
}

QJsonObject DrawingSheetCanvasWidget::styleToJson(const SheetEntityStyle& s)
{
	QJsonObject o;
	o.insert(QStringLiteral("colorByLayer"), s.colorByLayer);
	o.insert(QStringLiteral("lineTypeByLayer"), s.lineTypeByLayer);
	o.insert(QStringLiteral("lineWidthByLayer"), s.lineWidthByLayer);
	o.insert(QStringLiteral("colorByBlock"), s.colorByBlock);
	o.insert(QStringLiteral("lineTypeByBlock"), s.lineTypeByBlock);
	o.insert(QStringLiteral("lineWidthByBlock"), s.lineWidthByBlock);
	o.insert(QStringLiteral("color"), s.color.name(QColor::HexRgb));
	o.insert(QStringLiteral("lineType"), lineTypeToString(s.lineType));
	o.insert(QStringLiteral("lineWidthMm"), s.lineWidthMm);
	return o;
}

SheetEntityStyle DrawingSheetCanvasWidget::styleFromJson(const QJsonObject& o)
{
	SheetEntityStyle s;
	s.colorByLayer = o.value(QStringLiteral("colorByLayer")).toBool(true);
	s.lineTypeByLayer = o.value(QStringLiteral("lineTypeByLayer")).toBool(true);
	s.lineWidthByLayer = o.value(QStringLiteral("lineWidthByLayer")).toBool(true);
	s.colorByBlock = o.value(QStringLiteral("colorByBlock")).toBool(false);
	s.lineTypeByBlock = o.value(QStringLiteral("lineTypeByBlock")).toBool(false);
	s.lineWidthByBlock = o.value(QStringLiteral("lineWidthByBlock")).toBool(false);
	const QString c = o.value(QStringLiteral("color")).toString();
	if (!c.isEmpty())
		s.color = QColor(c);
	s.lineType = lineTypeFromString(o.value(QStringLiteral("lineType")).toString());
	s.lineWidthMm = o.value(QStringLiteral("lineWidthMm")).toDouble(0.35);
	return s;
}

QPen DrawingSheetCanvasWidget::resolvePen(const QString& layerId, const SheetEntityStyle& style, bool forceDashed,
										 bool selected, const SheetEntityStyle* blockCtx,
										 const QString* blockLayerId) const
{
	auto resolveOne = [&](const QString& lid, const SheetEntityStyle& st, bool allowBlock) -> SheetEntityStyle {
		SheetEntityStyle out = st;
		const SheetLayer* L = layerById(lid);
		if (allowBlock && blockCtx && st.colorByBlock)
		{
			const QString blid = blockLayerId ? *blockLayerId : lid;
			const SheetLayer* BL = layerById(blid);
			out.color = (blockCtx->colorByLayer && BL) ? BL->color : blockCtx->color;
			out.colorByLayer = false;
			out.colorByBlock = false;
		}
		else if (st.colorByLayer && L)
			out.color = L->color;
		if (allowBlock && blockCtx && st.lineTypeByBlock)
		{
			const QString blid = blockLayerId ? *blockLayerId : lid;
			const SheetLayer* BL = layerById(blid);
			out.lineType = (blockCtx->lineTypeByLayer && BL) ? BL->lineType : blockCtx->lineType;
			out.lineTypeByLayer = false;
			out.lineTypeByBlock = false;
		}
		else if (st.lineTypeByLayer && L)
			out.lineType = L->lineType;
		if (allowBlock && blockCtx && st.lineWidthByBlock)
		{
			const QString blid = blockLayerId ? *blockLayerId : lid;
			const SheetLayer* BL = layerById(blid);
			out.lineWidthMm = (blockCtx->lineWidthByLayer && BL) ? BL->lineWidthMm : blockCtx->lineWidthMm;
			out.lineWidthByLayer = false;
			out.lineWidthByBlock = false;
		}
		else if (st.lineWidthByLayer && L)
			out.lineWidthMm = L->lineWidthMm;
		return out;
	};

	SheetEntityStyle resolved = resolveOne(layerId, style, true);
	QColor color = resolved.color;
	SheetLineType lineTy = resolved.lineType;
	double widthMm = resolved.lineWidthMm;
	if (!(widthMm > 1e-6))
		widthMm = 0.35;
	if (forceDashed)
		lineTy = SheetLineType::Dashed;
	if (selected)
		color = QColor(142, 68, 173);

	Qt::PenStyle ps = Qt::SolidLine;
	switch (lineTy)
	{
	case SheetLineType::Dashed:
		ps = Qt::DashLine;
		break;
	case SheetLineType::Center:
		ps = Qt::DashDotLine;
		break;
	case SheetLineType::DashDot:
		ps = Qt::DashDotDotLine;
		break;
	default:
		ps = Qt::SolidLine;
		break;
	}

	widthMm = ctbWidthForColor(color, widthMm);
	// LTSCALE 只影响虚线观感；不应乘进线宽，否则薄件正视会糊成黑块
	const double px = qBound(0.55, widthMm * m_zoom * (selected ? 1.25 : 1.0), 1.6);
	QPen pen(color, px, ps);
	pen.setCapStyle(Qt::FlatCap);
	pen.setJoinStyle(Qt::MiterJoin);
	return pen;
}

QPen DrawingSheetCanvasWidget::penForLayer(const QString& layerId, bool forceDashed, bool selected) const
{
	SheetEntityStyle s;
	return resolvePen(layerId, s, forceDashed, selected);
}

void DrawingSheetCanvasWidget::setLtScale(double scale)
{
	if (!(scale > 1e-6))
		return;
	m_paper.ltScale = scale;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setSnapFlags(const SheetSnapFlags& flags)
{
	m_snapEngine.setFlags(flags);
	emit statusMessage(flags.ortho ? QStringLiteral("正交：开") : QStringLiteral("捕捉已更新"));
	update();
}

SheetSnapFlags DrawingSheetCanvasWidget::snapFlags() const
{
	return m_snapEngine.flags();
}

void DrawingSheetCanvasWidget::rebuildSnapGeometry()
{
	QVector<QLineF> segs;
	QVector<QPointF> centers;
	auto addPoly = [&](const QVector<Polyline2d>& polys) {
		for (const auto& poly : polys)
		{
			for (int i = 1; i < poly.points.size(); ++i)
				segs.push_back(QLineF(poly.points[i - 1], poly.points[i]));
			if (poly.points.size() >= 8)
			{
				QRectF box;
				for (const QPointF& p : poly.points)
					box = box.isNull() ? QRectF(p, QSizeF(0, 0)) : box.united(QRectF(p, QSizeF(0, 0)));
				if (box.isValid())
					centers.push_back(box.center());
			}
		}
	};
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		addPoly(v.visible);
		addPoly(v.hidden);
	}
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		for (int i = 1; i < poly.points.size(); ++i)
			segs.push_back(QLineF(poly.points[i - 1], poly.points[i]));
	}
	m_snapEngine.setSegments(segs);
	m_snapEngine.setCenters(centers);
}

bool DrawingSheetCanvasWidget::duplicateSelection()
{
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		DrawingView copy = m_views[m_selectedViewIndex];
		copy.id = QStringLiteral("view_%1").arg(m_nextCatalogViewId++);
		copy.title += QStringLiteral(" 副本");
		m_views.push_back(copy);
		const int ni = m_views.size() - 1;
		m_selectedViewIndex = ni;
		m_selectedViewIndices = {ni};
		return true;
	}
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		SheetDimension copy = m_dims[m_selectedDimIndex];
		copy.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
		m_dims.push_back(copy);
		m_selectedDimIndex = m_dims.size() - 1;
		return true;
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		SheetNote copy = m_notes[m_selectedNoteIndex];
		copy.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
		m_notes.push_back(copy);
		m_selectedNoteIndex = m_notes.size() - 1;
		return true;
	}
	if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
	{
		SheetHatch copy = m_hatches[m_selectedHatchIndex];
		copy.id = QStringLiteral("h_%1").arg(m_nextHatchId++);
		m_hatches.push_back(copy);
		m_selectedHatchIndex = m_hatches.size() - 1;
		return true;
	}
	if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
	{
		SheetBlockRef copy = m_blockRefs[m_selectedBlockRefIndex];
		copy.id = QStringLiteral("BR%1").arg(m_nextBlockRefId++);
		m_blockRefs.push_back(copy);
		m_selectedBlockRefIndex = m_blockRefs.size() - 1;
		return true;
	}
	return false;
}

void DrawingSheetCanvasWidget::selectAtScene(const QPointF& scenePos)
{
	clearSelection();
	m_selectedHatchIndex = hitHatchIndex(scenePos);
	if (m_selectedHatchIndex >= 0)
	{
		emitSelectionChanged();
		return;
	}
	m_selectedBlockRefIndex = hitBlockRefIndex(scenePos);
	if (m_selectedBlockRefIndex >= 0)
	{
		emitSelectionChanged();
		return;
	}
	m_selectedDimIndex = hitDimensionIndex(scenePos);
	if (m_selectedDimIndex >= 0)
	{
		emitSelectionChanged();
		return;
	}
	m_selectedNoteIndex = hitNoteIndex(scenePos);
	if (m_selectedNoteIndex >= 0)
	{
		emitSelectionChanged();
		return;
	}
	m_selectedSketchId = hitSketchEntity(scenePos, true);
	if (m_selectedSketchId >= 0)
	{
		emitSelectionChanged();
		return;
	}
	const int vi = hitViewIndex(scenePos);
	if (vi >= 0)
		setViewSelection({vi}, vi);
	else
		emitSelectionChanged();
}

SheetSnapResult DrawingSheetCanvasWidget::snapSceneResult(const QPointF& raw, const QPointF* orthoRef) const
{
	const_cast<DrawingSheetCanvasWidget*>(this)->rebuildSnapGeometry();
	return m_snapEngine.snap(raw, snapTolMm(), orthoRef);
}

QPointF DrawingSheetCanvasWidget::snapScenePoint(const QPointF& raw, const QPointF* orthoRef) const
{
	const SheetSnapResult r = snapSceneResult(raw, orthoRef);
	return r.snapped ? r.pos : raw;
}

bool DrawingSheetCanvasWidget::isDimTool(DrawingCanvasTool t) const
{
	return t == DrawingCanvasTool::LinearDim || t == DrawingCanvasTool::DimRadius ||
		   t == DrawingCanvasTool::DimDiameter || t == DrawingCanvasTool::DimAngle ||
		   t == DrawingCanvasTool::DimContinuous || t == DrawingCanvasTool::DimBaseline;
}

bool DrawingSheetCanvasWidget::isPickFeedbackTool(DrawingCanvasTool t) const
{
	return isDimTool(t) || isModifyTool(t) || t == DrawingCanvasTool::PanSelect ||
		   t == DrawingCanvasTool::SelectEntity || t == DrawingCanvasTool::HatchPick ||
		   t == DrawingCanvasTool::TextNote || t == DrawingCanvasTool::NoteLeader ||
		   t == DrawingCanvasTool::MText || t == DrawingCanvasTool::DetailRegion ||
		   t == DrawingCanvasTool::ProjectionGuide || t == DrawingCanvasTool::SymRoughness ||
		   t == DrawingCanvasTool::SymGdt || isSketchTool(t);
}

bool DrawingSheetCanvasWidget::findNearestEdge(const QPointF& scenePos, QLineF& outSeg, int* outSketchId) const
{
	double best = snapTolMm() * 2.5;
	bool found = false;
	if (outSketchId)
		*outSketchId = -1;

	auto considerSeg = [&](const QPointF& a, const QPointF& b, int sketchId) {
		const QLineF seg(a, b);
		if (seg.length() < 1e-9)
			return;
		const QPointF ab = b - a;
		const double len2 = QPointF::dotProduct(ab, ab);
		const double t = qBound(0.0, QPointF::dotProduct(scenePos - a, ab) / len2, 1.0);
		const QPointF foot = a + ab * t;
		const double d = QLineF(scenePos, foot).length();
		if (d < best)
		{
			best = d;
			outSeg = seg;
			found = true;
			if (outSketchId)
				*outSketchId = sketchId;
		}
	};

	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		if (!isLayerDrawable(m_sketch.layerOf(poly.entityId)))
			continue;
		for (int i = 1; i < poly.points.size(); ++i)
			considerSeg(poly.points[i - 1], poly.points[i], poly.entityId);
	}
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		for (const Polyline2d& poly : v.visible)
		{
			for (int i = 1; i < poly.points.size(); ++i)
				considerSeg(poly.points[i - 1], poly.points[i], -1);
		}
	}
	return found;
}

void DrawingSheetCanvasWidget::updatePickFeedback(const QPointF& scenePos)
{
	PickHoverState h;
	if (!isPickFeedbackTool(m_tool))
	{
		m_pickHover = h;
		return;
	}

	const QPointF* orthoRef = nullptr;
	if (m_dimPicking && m_dimPickStep >= 1)
		orthoRef = &m_dimP1;
	else if (m_modifyPicking && m_modifyStep >= 1)
		orthoRef = &m_modifyP1;
	else if (m_notePicking)
		orthoRef = &m_noteAnchor;

	h.snap = snapSceneResult(scenePos, orthoRef);

	int edgeSketch = -1;
	h.hasEdge = findNearestEdge(scenePos, h.edgeSeg, &edgeSketch);
	if (h.hasEdge && edgeSketch >= 0)
		h.sketchId = edgeSketch;
	else if (isDimTool(m_tool) || isModifyTool(m_tool) || m_tool == DrawingCanvasTool::PanSelect ||
			 m_tool == DrawingCanvasTool::SelectEntity)
	{
		h.sketchId = hitSketchEntity(scenePos, false);
	}

	if (m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter)
	{
		QPointF c, rim;
		double r = 0;
		const int ent = hitSketchEntity(scenePos, false);
		if (ent >= 0 && resolveCircleDim(ent, c, rim, r))
			h.sketchId = ent;
		else if (resolveHlrCircleNear(scenePos, c, rim, r))
		{
			h.hasEdge = true;
			h.edgeSeg = QLineF(c, rim);
			h.sketchId = -1;
		}
	}

	if (m_tool == DrawingCanvasTool::PanSelect || m_tool == DrawingCanvasTool::SelectEntity ||
		m_tool == DrawingCanvasTool::MatchProp || m_tool == DrawingCanvasTool::ModifyErase ||
		m_tool == DrawingCanvasTool::ModifyMove || m_tool == DrawingCanvasTool::ModifyCopy ||
		m_tool == DrawingCanvasTool::ModifyRotate || m_tool == DrawingCanvasTool::ModifyMirror ||
		m_tool == DrawingCanvasTool::ModifyScale || m_tool == DrawingCanvasTool::ModifyArray ||
		m_tool == DrawingCanvasTool::ModifyPolarArray || m_tool == DrawingCanvasTool::ExplodeBlock)
	{
		h.blockRefIndex = hitBlockRefIndex(scenePos);
		h.hatchIndex = hitHatchIndex(scenePos);
		h.dimIndex = hitDimensionIndex(scenePos);
		h.noteIndex = hitNoteIndex(scenePos);
		if (h.sketchId < 0)
			h.sketchId = hitSketchEntity(scenePos, false);
		h.viewIndex = hitViewIndex(scenePos);
	}

	QStringList tipParts;
	if (h.snap.snapped)
	{
		static const QHash<QString, QString> kindCn = {
			{QStringLiteral("end"), QStringLiteral("端点")},
			{QStringLiteral("mid"), QStringLiteral("中点")},
			{QStringLiteral("int"), QStringLiteral("交点")},
			{QStringLiteral("cen"), QStringLiteral("圆心")},
			{QStringLiteral("perp"), QStringLiteral("垂足")},
			{QStringLiteral("near"), QStringLiteral("最近点")},
			{QStringLiteral("ortho"), QStringLiteral("正交")},
			{QStringLiteral("polar"), QStringLiteral("极轴")},
		};
		tipParts << QStringLiteral("点:%1").arg(kindCn.value(h.snap.kind, h.snap.kind));
	}
	if (h.sketchId >= 0)
		tipParts << QStringLiteral("线:草图实体");
	else if (h.hasEdge)
		tipParts << QStringLiteral("线:视图轮廓");
	if (h.dimIndex >= 0)
		tipParts << QStringLiteral("尺寸");
	if (h.noteIndex >= 0)
		tipParts << QStringLiteral("注释");
	if (h.blockRefIndex >= 0)
		tipParts << QStringLiteral("块参照");
	if (h.hatchIndex >= 0)
		tipParts << QStringLiteral("填充");
	if (h.viewIndex >= 0 && tipParts.isEmpty())
		tipParts << QStringLiteral("视图框");
	h.tip = tipParts.join(QStringLiteral(" · "));

	m_pickHover = h;
	if (h.tip != m_lastPickTip)
	{
		m_lastPickTip = h.tip;
		const bool quiet = m_dimPicking || m_notePicking || m_modifyPicking || m_detailDragging ||
						   m_extendPicking || m_hatchPickPts.size() > 0;
		if (!h.tip.isEmpty() && !quiet)
			emit statusMessage(h.tip);
	}
}

void DrawingSheetCanvasWidget::drawSnapMarker(QPainter& p, const QPointF& widgetPos, const QString& kind) const
{
	p.save();
	const QColor accent(230, 126, 34);
	p.setPen(QPen(accent, 1.8));
	p.setBrush(Qt::NoBrush);
	const double s = 6.5;
	if (kind == QLatin1String("end"))
	{
		p.drawRect(QRectF(widgetPos.x() - s, widgetPos.y() - s, s * 2, s * 2));
	}
	else if (kind == QLatin1String("mid"))
	{
		QPolygonF tri;
		tri << QPointF(widgetPos.x(), widgetPos.y() - s) << QPointF(widgetPos.x() + s, widgetPos.y() + s)
			<< QPointF(widgetPos.x() - s, widgetPos.y() + s);
		p.drawPolygon(tri);
	}
	else if (kind == QLatin1String("int"))
	{
		p.drawLine(widgetPos + QPointF(-s, -s), widgetPos + QPointF(s, s));
		p.drawLine(widgetPos + QPointF(-s, s), widgetPos + QPointF(s, -s));
		p.drawRect(QRectF(widgetPos.x() - s * 0.55, widgetPos.y() - s * 0.55, s * 1.1, s * 1.1));
	}
	else if (kind == QLatin1String("cen"))
	{
		p.drawEllipse(widgetPos, s, s);
		p.drawLine(widgetPos + QPointF(-s - 2, 0), widgetPos + QPointF(s + 2, 0));
		p.drawLine(widgetPos + QPointF(0, -s - 2), widgetPos + QPointF(0, s + 2));
	}
	else if (kind == QLatin1String("perp"))
	{
		p.drawLine(widgetPos + QPointF(-s, s), widgetPos + QPointF(s, s));
		p.drawLine(widgetPos + QPointF(0, -s), widgetPos + QPointF(0, s));
	}
	else if (kind == QLatin1String("near"))
	{
		QPolygonF dia;
		dia << QPointF(widgetPos.x(), widgetPos.y() - s) << QPointF(widgetPos.x() + s, widgetPos.y())
			<< QPointF(widgetPos.x(), widgetPos.y() + s) << QPointF(widgetPos.x() - s, widgetPos.y());
		p.drawPolygon(dia);
	}
	else
	{
		p.drawLine(widgetPos + QPointF(-s, 0), widgetPos + QPointF(s, 0));
		p.drawLine(widgetPos + QPointF(0, -s), widgetPos + QPointF(0, s));
		p.drawEllipse(widgetPos, 3.0, 3.0);
	}
	p.setPen(QPen(QColor(40, 50, 60), 1.0));
	QFont f = p.font();
	f.setPointSizeF(8.0);
	f.setBold(true);
	p.setFont(f);
	static const QHash<QString, QString> kindLbl = {
		{QStringLiteral("end"), QStringLiteral("端")},
		{QStringLiteral("mid"), QStringLiteral("中")},
		{QStringLiteral("int"), QStringLiteral("交")},
		{QStringLiteral("cen"), QStringLiteral("心")},
		{QStringLiteral("perp"), QStringLiteral("垂")},
		{QStringLiteral("near"), QStringLiteral("近")},
		{QStringLiteral("ortho"), QStringLiteral("正")},
		{QStringLiteral("polar"), QStringLiteral("极")},
	};
	p.drawText(widgetPos + QPointF(s + 3, -3), kindLbl.value(kind, kind));
	p.restore();
}

void DrawingSheetCanvasWidget::drawPickPointMarker(QPainter& p, const QPointF& scenePos, const QString& label) const
{
	const QPointF w = sceneToWidget(scenePos);
	p.save();
	p.setPen(QPen(QColor(192, 57, 43), 1.5));
	p.setBrush(QColor(231, 76, 60, 220));
	p.drawEllipse(w, 4.5, 4.5);
	p.setBrush(Qt::NoBrush);
	p.setPen(QPen(QColor(192, 57, 43), 1.2));
	p.drawText(w + QPointF(6, -4), label);
	p.restore();
}

void DrawingSheetCanvasWidget::drawHoverEdge(QPainter& p, const QLineF& seg) const
{
	p.save();
	p.setPen(QPen(QColor(0, 188, 212), qMax(2.2, 2.4 * m_zoom), Qt::SolidLine, Qt::RoundCap));
	p.drawLine(sceneToWidget(seg.p1()), sceneToWidget(seg.p2()));
	p.restore();
}

void DrawingSheetCanvasWidget::drawDimToolPreview(QPainter& p) const
{
	if (!m_dimPicking)
		return;
	p.save();
	const QColor dimPreview(192, 57, 43);
	p.setPen(QPen(dimPreview, 1.15, Qt::DashLine));

	auto drawLinearPreview = [&](const QPointF& a, const QPointF& b, double offY) {
		QLineF base(a, b);
		if (base.length() < 1e-6)
			return;
		base.setLength(1.0);
		const QPointF dir = base.p2() - base.p1();
		const QPointF n(-dir.y(), dir.x());
		const QPointF offset = n * offY;
		const QPointF da = a + offset;
		const QPointF db = b + offset;
		p.drawLine(sceneToWidget(a), sceneToWidget(a + offset * 1.15));
		p.drawLine(sceneToWidget(b), sceneToWidget(b + offset * 1.15));
		p.drawLine(sceneToWidget(da), sceneToWidget(db));
		const double len = QLineF(a, b).length();
		p.setPen(QPen(dimPreview, 1.0));
		p.drawText(sceneToWidget((da + db) * 0.5) + QPointF(4, -4), QString::number(len, 'f', 2));
		p.setPen(QPen(dimPreview, 1.15, Qt::DashLine));
	};

	if (m_tool == DrawingCanvasTool::LinearDim && m_dimPickStep >= 1)
	{
		if (m_dimPickStep == 1)
			drawLinearPreview(m_dimP1, m_dimP2, -12.0);
		else
			drawLinearPreview(m_dimP1, m_dimP2, m_dimP2.y() != m_dimP1.y() ? -12.0 : -12.0);
	}
	else if ((m_tool == DrawingCanvasTool::DimContinuous || m_tool == DrawingCanvasTool::DimBaseline) &&
			 m_dimPickStep >= 1)
	{
		drawLinearPreview(m_dimP1, m_dimP2, m_chainBaseOffset);
	}
	else if (m_tool == DrawingCanvasTool::DimAngle && m_dimPickStep >= 1)
	{
		p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
		if (m_dimPickStep >= 2)
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP3));
	}
	else if ((m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter) &&
			 m_dimPickStep >= 1)
	{
		if (m_dimPickStep == 10)
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
		else
		{
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
			if (m_tool == DrawingCanvasTool::DimDiameter)
			{
				const QPointF other = m_dimP1 - (m_dimP2 - m_dimP1);
				p.drawLine(sceneToWidget(other), sceneToWidget(m_dimP2));
			}
			const double r = QLineF(m_dimP1, m_dimP2).length();
			p.setPen(QPen(dimPreview, 1.0));
			p.drawText(sceneToWidget(m_dimP2) + QPointF(6, -4),
					   m_tool == DrawingCanvasTool::DimDiameter
						   ? QStringLiteral("Ø%1").arg(r * 2.0, 0, 'f', 2)
						   : QStringLiteral("R%1").arg(r, 0, 'f', 2));
		}
	}

	if (m_dimPickStep >= 1)
		drawPickPointMarker(p, m_dimP1, QStringLiteral("1"));
	if (m_dimPickStep >= 1 &&
		(m_tool == DrawingCanvasTool::LinearDim || m_tool == DrawingCanvasTool::DimContinuous ||
		 m_tool == DrawingCanvasTool::DimBaseline || m_tool == DrawingCanvasTool::DimAngle ||
		 m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter) &&
		QLineF(m_dimP1, m_dimP2).length() > 1e-6)
		drawPickPointMarker(p, m_dimP2, QStringLiteral("2"));
	if (m_tool == DrawingCanvasTool::DimAngle && m_dimPickStep >= 2)
		drawPickPointMarker(p, m_dimP3, QStringLiteral("3"));
	p.restore();
}

void DrawingSheetCanvasWidget::drawPickFeedback(QPainter& p) const
{
	if (m_pickHover.hasEdge)
		drawHoverEdge(p, m_pickHover.edgeSeg);

	if (m_pickHover.sketchId >= 0)
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), qMax(2.4, 2.6 * m_zoom), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		for (const SheetSketchPolyline& poly : m_sketch.tessellate())
		{
			if (poly.entityId != m_pickHover.sketchId || poly.points.size() < 2)
				continue;
			QPolygonF w;
			for (const QPointF& pt : poly.points)
				w << sceneToWidget(pt);
			p.drawPolyline(w);
		}
		p.restore();
	}

	if (m_pickHover.dimIndex >= 0 && m_pickHover.dimIndex < m_dims.size())
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), 2.0));
		const SheetDimension& d = m_dims[m_pickHover.dimIndex];
		p.drawEllipse(sceneToWidget(d.p1), 5, 5);
		p.drawEllipse(sceneToWidget(d.p2), 5, 5);
		p.drawLine(sceneToWidget(d.p1), sceneToWidget(d.p2));
		p.restore();
	}
	if (m_pickHover.noteIndex >= 0 && m_pickHover.noteIndex < m_notes.size())
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), 1.8, Qt::DashLine));
		const SheetNote& n = m_notes[m_pickHover.noteIndex];
		p.drawLine(sceneToWidget(n.anchor), sceneToWidget(n.textPos));
		p.drawEllipse(sceneToWidget(n.textPos), 6, 6);
		p.restore();
	}
	if (m_pickHover.viewIndex >= 0 && m_pickHover.viewIndex < m_views.size() &&
		m_pickHover.sketchId < 0 && !m_pickHover.hasEdge && m_pickHover.dimIndex < 0)
	{
		p.save();
		const QRectF fr = m_views[m_pickHover.viewIndex].frame;
		const QRectF w = QRectF(sceneToWidget(fr.topLeft()), sceneToWidget(fr.bottomRight())).normalized();
		p.setPen(QPen(QColor(0, 188, 212), 2.0, Qt::DashLine));
		p.setBrush(QColor(0, 188, 212, 28));
		p.drawRect(w);
		p.restore();
	}
	if (m_pickHover.blockRefIndex >= 0 && m_pickHover.blockRefIndex < m_blockRefs.size())
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), 2.0));
		p.setBrush(QColor(0, 188, 212, 80));
		p.drawEllipse(sceneToWidget(m_blockRefs[m_pickHover.blockRefIndex].insert), 7, 7);
		p.restore();
	}

	if (m_modifyPicking && m_modifyStep >= 1)
	{
		drawPickPointMarker(p, m_modifyP1, QStringLiteral("基"));
		if (m_modifyStep >= 2 && QLineF(m_modifyP1, m_modifyP2).length() > 1e-6)
		{
			p.save();
			p.setPen(QPen(QColor(192, 57, 43), 1.0, Qt::DashLine));
			p.drawLine(sceneToWidget(m_modifyP1), sceneToWidget(m_modifyP2));
			p.restore();
			drawPickPointMarker(p, m_modifyP2, QStringLiteral("到"));
		}
	}
	if (m_notePicking)
	{
		p.save();
		p.setPen(QPen(QColor(52, 73, 94), 1.0, Qt::DashLine));
		p.drawLine(sceneToWidget(m_noteAnchor), sceneToWidget(m_dimP2));
		p.restore();
		drawPickPointMarker(p, m_noteAnchor, QStringLiteral("锚"));
	}
	if (m_extendPicking)
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), 2.0, Qt::DashLine));
		p.drawLine(sceneToWidget(m_extendBoundary.p1()), sceneToWidget(m_extendBoundary.p2()));
		p.restore();
	}
	if (!m_hatchPickPts.isEmpty())
	{
		p.save();
		p.setPen(QPen(QColor(0, 188, 212), 1.2, Qt::DashLine));
		QPolygonF w;
		for (const QPointF& pt : m_hatchPickPts)
			w << sceneToWidget(pt);
		if (w.size() >= 2)
			p.drawPolyline(w);
		for (int i = 0; i < m_hatchPickPts.size(); ++i)
			drawPickPointMarker(p, m_hatchPickPts[i], QString::number(i + 1));
		p.restore();
	}

	drawDimToolPreview(p);

	if (m_pickHover.snap.snapped)
		drawSnapMarker(p, sceneToWidget(m_pickHover.snap.pos), m_pickHover.snap.kind);
}

void DrawingSheetCanvasWidget::emitSelectionChanged()
{
	emit selectionChanged();
}

void DrawingSheetCanvasWidget::clearSelection()
{
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_selectedNoteIndex = -1;
	m_selectedViewIndex = -1;
	m_selectedViewIndices.clear();
	m_selectedHatchIndex = -1;
	m_selectedBlockRefIndex = -1;
	emitSelectionChanged();
}

bool DrawingSheetCanvasWidget::isViewSelected(int index) const
{
	return m_selectedViewIndices.contains(index);
}

void DrawingSheetCanvasWidget::setViewSelection(const QVector<int>& indices, int primary)
{
	m_selectedViewIndices.clear();
	for (int i : indices)
	{
		if (i >= 0 && i < m_views.size() && !m_selectedViewIndices.contains(i))
			m_selectedViewIndices.push_back(i);
	}
	if (primary >= 0 && m_selectedViewIndices.contains(primary))
		m_selectedViewIndex = primary;
	else if (!m_selectedViewIndices.isEmpty())
		m_selectedViewIndex = m_selectedViewIndices.first();
	else
		m_selectedViewIndex = -1;
	emitSelectionChanged();
}

void DrawingSheetCanvasWidget::toggleViewSelection(int index)
{
	if (index < 0 || index >= m_views.size())
		return;
	const int pos = m_selectedViewIndices.indexOf(index);
	if (pos >= 0)
		m_selectedViewIndices.removeAt(pos);
	else
		m_selectedViewIndices.push_back(index);
	m_selectedViewIndex = m_selectedViewIndices.isEmpty() ? -1 : m_selectedViewIndices.last();
	emitSelectionChanged();
}

int DrawingSheetCanvasWidget::findViewIndexByKind(const QString& kind) const
{
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].kind == kind)
			return i;
	}
	return -1;
}

bool DrawingSheetCanvasWidget::alignSelectedViews(ViewAlignMode mode)
{
	QVector<int> idxs = m_selectedViewIndices;
	if (idxs.isEmpty() && m_selectedViewIndex >= 0)
		idxs.push_back(m_selectedViewIndex);
	if (idxs.isEmpty())
	{
		emit statusMessage(QStringLiteral("请先选中视图（Ctrl+点可多选）"));
		return false;
	}

	QRectF ref;
	if (idxs.size() == 1)
	{
		ref = paperDrawableRect();
		if (!ref.isValid())
			ref = paperRect();
		if (!ref.isValid())
		{
			emit statusMessage(QStringLiteral("无有效图幅参考"));
			return false;
		}
	}
	else
	{
		const int anchor = idxs.first();
		if (anchor < 0 || anchor >= m_views.size())
			return false;
		ref = m_views[anchor].frame;
	}

	bool moved = false;
	const int start = (idxs.size() == 1) ? 0 : 1;
	for (int k = start; k < idxs.size(); ++k)
	{
		const int i = idxs[k];
		if (i < 0 || i >= m_views.size())
			continue;
		const QRectF f = m_views[i].frame;
		QPointF delta(0, 0);
		switch (mode)
		{
		case ViewAlignMode::Left:
			delta.setX(ref.left() - f.left());
			break;
		case ViewAlignMode::HCenter:
			delta.setX(ref.center().x() - f.center().x());
			break;
		case ViewAlignMode::Right:
			delta.setX(ref.right() - f.right());
			break;
		case ViewAlignMode::Top:
			delta.setY(ref.top() - f.top());
			break;
		case ViewAlignMode::VCenter:
			delta.setY(ref.center().y() - f.center().y());
			break;
		case ViewAlignMode::Bottom:
			delta.setY(ref.bottom() - f.bottom());
			break;
		}
		if (QLineF(QPointF(0, 0), delta).length() < 1e-9)
			continue;
		moveViewBy(i, delta);
		moved = true;
	}
	emit statusMessage(moved ? QStringLiteral("视图已对齐") : QStringLiteral("已对齐（无需移动）"));
	return true;
}

bool DrawingSheetCanvasWidget::alignProjectionViews()
{
	const int front = findViewIndexByKind(QStringLiteral("front"));
	const int top = findViewIndexByKind(QStringLiteral("top"));
	const int right = findViewIndexByKind(QStringLiteral("right"));
	if (front < 0)
	{
		emit statusMessage(QStringLiteral("未找到正视图"));
		return false;
	}
	bool moved = false;
	const QRectF fr = m_views[front].frame;
	if (top >= 0)
	{
		const double dx = fr.center().x() - m_views[top].frame.center().x();
		if (std::abs(dx) > 1e-9)
		{
			moveViewBy(top, QPointF(dx, 0));
			moved = true;
		}
	}
	if (right >= 0)
	{
		const double dy = fr.center().y() - m_views[right].frame.center().y();
		if (std::abs(dy) > 1e-9)
		{
			moveViewBy(right, QPointF(0, dy));
			moved = true;
		}
	}
	emit statusMessage(moved ? QStringLiteral("已按投影关系中心对齐") : QStringLiteral("投影视图已对齐"));
	rebuildProjectionGuides();
	return true;
}

void DrawingSheetCanvasWidget::translateViewGeometry(DrawingView& v, const QPointF& delta)
{
	v.frame.translate(delta);
	for (Polyline2d& poly : v.visible)
		for (QPointF& pt : poly.points)
			pt += delta;
	for (Polyline2d& poly : v.hidden)
		for (QPointF& pt : poly.points)
			pt += delta;
	v.visCache.clear();
	v.hidCache.clear();
	if (m_sheetModel)
	{
		if (m_sheetModel->hasPath(v.id))
			m_sheetModel->translatePath(v.id, delta);
		else
			m_sheetModel->updateViewGeometry(v);
	}
}

void DrawingSheetCanvasWidget::setProjectionDragLock(bool on)
{
	m_projectionDragLock = on;
	emit statusMessage(on ? QStringLiteral("投影约束拖动：开") : QStringLiteral("投影约束拖动：关"));
}

void DrawingSheetCanvasWidget::setProjectionPinned(bool on)
{
	m_projectionPinned = on;
	emit statusMessage(on ? QStringLiteral("钉住投影：开") : QStringLiteral("钉住投影：关"));
}

QLineF DrawingSheetCanvasWidget::projectionGuideLine(const SheetProjectionGuide& g) const
{
	const DrawingView* a = nullptr;
	const DrawingView* b = nullptr;
	for (const DrawingView& v : m_views)
	{
		if (v.id == g.fromViewId)
			a = &v;
		if (v.id == g.toViewId)
			b = &v;
	}
	if (!a || !b)
		return {};
	if (g.tipsCustom)
		return QLineF(g.tipA, g.tipB);
	if (g.axis == SheetProjectionGuide::Axis::Horizontal)
	{
		const double x = a->frame.center().x();
		return QLineF(QPointF(x, qMin(a->frame.top(), b->frame.top()) - 4),
					  QPointF(x, qMax(a->frame.bottom(), b->frame.bottom()) + 4));
	}
	const double y = a->frame.center().y();
	return QLineF(QPointF(qMin(a->frame.left(), b->frame.left()) - 4, y),
				  QPointF(qMax(a->frame.right(), b->frame.right()) + 4, y));
}

QPointF DrawingSheetCanvasWidget::snapGuideTipToAxis(const SheetProjectionGuide& g, const QPointF& raw) const
{
	const DrawingView* a = nullptr;
	const DrawingView* b = nullptr;
	for (const DrawingView& v : m_views)
	{
		if (v.id == g.fromViewId)
			a = &v;
		if (v.id == g.toViewId)
			b = &v;
	}
	if (!a || !b)
		return raw;
	if (g.axis == SheetProjectionGuide::Axis::Horizontal)
	{
		const double x = a->frame.center().x();
		QVector<double> ys{a->frame.top(), a->frame.bottom(), b->frame.top(), b->frame.bottom()};
		double bestY = raw.y();
		double bestD = 1e100;
		for (double y : ys)
		{
			const double d = std::abs(y - raw.y());
			if (d < bestD)
			{
				bestD = d;
				bestY = y;
			}
		}
		if (bestD <= snapTolMm() * 4.0)
			return QPointF(x, bestY);
		return QPointF(x, raw.y());
	}
	const double y = a->frame.center().y();
	QVector<double> xs{a->frame.left(), a->frame.right(), b->frame.left(), b->frame.right()};
	double bestX = raw.x();
	double bestD = 1e100;
	for (double x : xs)
	{
		const double d = std::abs(x - raw.x());
		if (d < bestD)
		{
			bestD = d;
			bestX = x;
		}
	}
	if (bestD <= snapTolMm() * 4.0)
		return QPointF(bestX, y);
	return QPointF(raw.x(), y);
}

bool DrawingSheetCanvasWidget::beginGuideTipDrag(const QPointF& scenePos)
{
	const double tol = snapTolMm() * 3.0;
	for (int i = 0; i < m_projectionGuides.size(); ++i)
	{
		SheetProjectionGuide& g = m_projectionGuides[i];
		if (!g.visible)
			continue;
		const QLineF line = projectionGuideLine(g);
		if (QLineF(line.p1(), scenePos).length() <= tol)
		{
			m_guideTipDragging = true;
			m_guideTipIndex = i;
			m_guideTipWhich = 0;
			if (!g.tipsCustom)
			{
				g.tipsCustom = true;
				g.tipA = line.p1();
				g.tipB = line.p2();
			}
			return true;
		}
		if (QLineF(line.p2(), scenePos).length() <= tol)
		{
			m_guideTipDragging = true;
			m_guideTipIndex = i;
			m_guideTipWhich = 1;
			if (!g.tipsCustom)
			{
				g.tipsCustom = true;
				g.tipA = line.p1();
				g.tipB = line.p2();
			}
			return true;
		}
	}
	return false;
}

void DrawingSheetCanvasWidget::updateGuideTipDrag(const QPointF& scenePos)
{
	if (!m_guideTipDragging || m_guideTipIndex < 0 || m_guideTipIndex >= m_projectionGuides.size())
		return;
	SheetProjectionGuide& g = m_projectionGuides[m_guideTipIndex];
	const QPointF p = snapGuideTipToAxis(g, scenePos);
	if (m_guideTipWhich == 0)
		g.tipA = p;
	else
		g.tipB = p;
	update();
}

void DrawingSheetCanvasWidget::endGuideTipDrag()
{
	if (m_guideTipDragging)
	{
		m_guideTipDragging = false;
		emit sheetChanged();
	}
	m_guideTipIndex = -1;
}

bool DrawingSheetCanvasWidget::beginSectionMarkDrag(const QPointF& scenePos)
{
	const double tol = snapTolMm() * 3.0;
	for (int i = 0; i < m_views.size(); ++i)
	{
		DrawingView& v = m_views[i];
		if (!v.hasMark || (v.kind != QLatin1String("section") && v.kind != QLatin1String("detail")))
			continue;
		if (QLineF(v.markP1, scenePos).length() <= tol)
		{
			m_sectionMarkDragging = true;
			m_sectionMarkViewIndex = i;
			m_sectionMarkWhich = 0;
			return true;
		}
		if (QLineF(v.markP2, scenePos).length() <= tol)
		{
			m_sectionMarkDragging = true;
			m_sectionMarkViewIndex = i;
			m_sectionMarkWhich = 1;
			return true;
		}
	}
	return false;
}

void DrawingSheetCanvasWidget::updateSectionMarkDrag(const QPointF& scenePos)
{
	if (!m_sectionMarkDragging || m_sectionMarkViewIndex < 0 || m_sectionMarkViewIndex >= m_views.size())
		return;
	DrawingView& v = m_views[m_sectionMarkViewIndex];
	if (m_sectionMarkWhich == 0)
		v.markP1 = scenePos;
	else
		v.markP2 = scenePos;
	update();
}

void DrawingSheetCanvasWidget::endSectionMarkDrag()
{
	if (m_sectionMarkDragging)
	{
		const int idx = m_sectionMarkViewIndex;
		m_sectionMarkDragging = false;
		if (idx >= 0 && idx < m_views.size() && m_views[idx].kind == QLatin1String("detail"))
			refreshDetailViewFromMark(idx);
		emit sheetChanged();
	}
	m_sectionMarkViewIndex = -1;
}

QPointF DrawingSheetCanvasWidget::constrainViewDragDelta(int viewIndex, const QPointF& delta) const
{
	if (viewIndex < 0 || viewIndex >= m_views.size())
		return delta;
	const QString& kind = m_views[viewIndex].kind;
	if (kind == QLatin1String("top"))
		return QPointF(delta.x(), 0.0);
	if (kind == QLatin1String("right"))
		return QPointF(0.0, delta.y());
	return delta;
}

void DrawingSheetCanvasWidget::rebuildProjectionGuides()
{
	const int front = findViewIndexByKind(QStringLiteral("front"));
	const int top = findViewIndexByKind(QStringLiteral("top"));
	const int right = findViewIndexByKind(QStringLiteral("right"));
	QVector<SheetProjectionGuide> next;
	auto keep = [&](const QString& from, const QString& to, SheetProjectionGuide& out) {
		for (const auto& g : m_projectionGuides)
		{
			if (g.fromViewId == from && g.toViewId == to)
			{
				out.visible = g.visible;
				out.tipsCustom = g.tipsCustom;
				out.tipA = g.tipA;
				out.tipB = g.tipB;
				return;
			}
		}
		out.visible = true;
	};
	if (front >= 0 && top >= 0)
	{
		SheetProjectionGuide g;
		g.id = QStringLiteral("pg_front_top");
		g.fromViewId = m_views[front].id;
		g.toViewId = m_views[top].id;
		g.axis = SheetProjectionGuide::Axis::Horizontal;
		g.layerId = m_currentLayerId;
		keep(g.fromViewId, g.toViewId, g);
		next.push_back(g);
	}
	if (front >= 0 && right >= 0)
	{
		SheetProjectionGuide g;
		g.id = QStringLiteral("pg_front_right");
		g.fromViewId = m_views[front].id;
		g.toViewId = m_views[right].id;
		g.axis = SheetProjectionGuide::Axis::Vertical;
		g.layerId = m_currentLayerId;
		keep(g.fromViewId, g.toViewId, g);
		next.push_back(g);
	}
	m_projectionGuides = next;
	update();
}

int DrawingSheetCanvasWidget::hitProjectionGuideIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm() * 2.0;
	for (int i = 0; i < m_projectionGuides.size(); ++i)
	{
		const SheetProjectionGuide& g = m_projectionGuides[i];
		if (!g.visible)
			continue;
		const QLineF line = projectionGuideLine(g);
		if (line.length() < 1e-9)
			continue;
		const QPointF d = scenePos - line.p1();
		const double along = QPointF::dotProduct(d, QPointF(line.dx(), line.dy()) / line.length());
		if (along < -tol || along > line.length() + tol)
			continue;
		const QPointF closest = line.p1() + QPointF(line.dx(), line.dy()) * (along / line.length());
		if (QLineF(closest, scenePos).length() <= tol)
			return i;
	}
	return -1;
}

bool DrawingSheetCanvasWidget::removeProjectionGuideAt(const QPointF& scenePos)
{
	const int idx = hitProjectionGuideIndex(scenePos);
	if (idx < 0)
		return false;
	m_projectionGuides[idx].visible = false;
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已隐藏投影线"));
	update();
	return true;
}

void DrawingSheetCanvasWidget::setProjectionGuidesVisible(bool visible)
{
	for (auto& g : m_projectionGuides)
		g.visible = visible;
	emit sheetChanged();
	update();
}

int DrawingSheetCanvasWidget::recalculateDimensions(bool selectedOnly)
{
	int n = 0;
	auto clearOne = [&](SheetDimension& d) {
		if (std::isfinite(d.overrideValue))
		{
			d.overrideValue = std::numeric_limits<double>::quiet_NaN();
			++n;
		}
	};
	if (selectedOnly)
	{
		if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
			clearOne(m_dims[m_selectedDimIndex]);
	}
	else
	{
		for (SheetDimension& d : m_dims)
			clearOne(d);
	}
	if (n > 0)
	{
		emit sheetChanged();
		update();
	}
	emit statusMessage(n > 0 ? QStringLiteral("已重算 %1 个尺寸").arg(n) : QStringLiteral("无需重算"));
	return n;
}

bool DrawingSheetCanvasWidget::setSelectedDimTolerance(bool show, double tolPlus, double tolMinus, bool useOverride)
{
	if (m_selectedDimIndex < 0 || m_selectedDimIndex >= m_dims.size())
		return false;
	SheetDimension& d = m_dims[m_selectedDimIndex];
	d.tolOverride = useOverride;
	d.showTolerance = show;
	d.tolPlus = tolPlus;
	d.tolMinus = tolMinus;
	emit sheetChanged();
	update();
	return true;
}

QString DrawingSheetCanvasWidget::nextMarkLetter()
{
	const int idx = m_nextMarkLetterIdx % 26;
	++m_nextMarkLetterIdx;
	return QString(QChar(QLatin1Char('A' + idx)));
}

void DrawingSheetCanvasWidget::ensureSectionHatchForView(DrawingView& sectionView)
{
	for (int i = m_hatches.size() - 1; i >= 0; --i)
	{
		if (m_hatches[i].anchorViewId == sectionView.id)
			m_hatches.removeAt(i);
	}
	QVector<QPointF> bestLoop;
	double bestLen = 0.0;
	for (const Polyline2d& poly : sectionView.visible)
	{
		if (poly.points.size() < 3)
			continue;
		const double close = QLineF(poly.points.first(), poly.points.last()).length();
		double len = 0.0;
		for (int i = 1; i < poly.points.size(); ++i)
			len += QLineF(poly.points[i - 1], poly.points[i]).length();
		if (close < snapTolMm() * 4.0 && len > bestLen)
		{
			bestLen = len;
			bestLoop = poly.points;
		}
	}
	QVector<QPointF> boundary;
	if (bestLoop.size() >= 3)
		boundary = bestLoop;
	else
	{
		QVector<QPointF> pts;
		for (const Polyline2d& poly : sectionView.visible)
			for (const QPointF& p : poly.points)
				pts.push_back(p);
		QRectF box;
		if (pts.size() >= 3)
		{
			box = QRectF(pts.first(), QSizeF(0, 0));
			for (const QPointF& p : pts)
				box = box.united(QRectF(p, QSizeF(0, 0)));
		}
		else
			box = sectionView.frame.adjusted(8, 20, -8, -8);
		boundary = {box.topLeft(), box.topRight(), box.bottomRight(), box.bottomLeft()};
	}
	SheetHatch h;
	h.id = QStringLiteral("h_%1").arg(m_nextHatchId++);
	h.boundary = boundary;
	h.pattern = QStringLiteral("ANSI31");
	h.angleDeg = 45.0;
	h.layerId = sectionView.layerId;
	h.anchorViewId = sectionView.id;
	m_hatches.push_back(h);
}

QString DrawingSheetCanvasWidget::makeDimEdgeKey(const QString& viewId, int polyIndex, int ptIndex) const
{
	return QStringLiteral("%1#%2#%3").arg(viewId).arg(polyIndex).arg(ptIndex);
}

bool DrawingSheetCanvasWidget::resolveDimEdgeKey(const QString& key, QPointF& out) const
{
	const QStringList parts = key.split(QLatin1Char('#'));
	if (parts.size() != 3)
		return false;
	const QString viewId = parts[0];
	bool ok1 = false, ok2 = false;
	const int polyIndex = parts[1].toInt(&ok1);
	const int ptIndex = parts[2].toInt(&ok2);
	if (!ok1 || !ok2)
		return false;
	for (const DrawingView& v : m_views)
	{
		if (v.id != viewId)
			continue;
		if (polyIndex < 0 || polyIndex >= v.visible.size())
			return false;
		const auto& pts = v.visible[polyIndex].points;
		if (ptIndex < 0 || ptIndex >= pts.size())
			return false;
		out = pts[ptIndex];
		return true;
	}
	return false;
}

void DrawingSheetCanvasWidget::assignDimEdgeKeys(SheetDimension& dim)
{
	if (dim.kind != SheetDimension::Kind::Linear)
		return;
	auto nearest = [&](const QPointF& p, QString& keyOut) -> bool {
		double best = snapTolMm() * 8.0;
		bool found = false;
		for (const DrawingView& v : m_views)
		{
			for (int pi = 0; pi < v.visible.size(); ++pi)
			{
				const auto& pts = v.visible[pi].points;
				for (int ti = 0; ti < pts.size(); ++ti)
				{
					const double d = QLineF(p, pts[ti]).length();
					if (d < best)
					{
						best = d;
						keyOut = makeDimEdgeKey(v.id, pi, ti);
						found = true;
					}
				}
			}
		}
		return found;
	};
	QString k1, k2;
	if (nearest(dim.p1, k1) && nearest(dim.p2, k2))
		dim.anchorEdgeKey = k1 + QLatin1Char('|') + k2;
}

void DrawingSheetCanvasWidget::rebindAssociatedDimensions()
{
	bool any = false;
	for (SheetDimension& d : m_dims)
	{
		if (d.anchorEdgeKey.isEmpty() || d.kind != SheetDimension::Kind::Linear)
			continue;
		const int sep = d.anchorEdgeKey.indexOf(QLatin1Char('|'));
		if (sep <= 0)
			continue;
		QPointF p1, p2;
		if (!resolveDimEdgeKey(d.anchorEdgeKey.left(sep), p1))
			continue;
		if (!resolveDimEdgeKey(d.anchorEdgeKey.mid(sep + 1), p2))
			continue;
		d.p1 = p1;
		d.p2 = p2;
		any = true;
	}
	if (any)
	{
		emit sheetChanged();
		update();
	}
}

double DrawingSheetCanvasWidget::ctbWidthForColor(const QColor& color, double fallbackMm) const
{
	if (!m_ctbEnabled)
		return fallbackMm;
	if (!m_ctbTable.isEmpty())
	{
		const int gray = qGray(color.rgb());
		int bestAci = 7;
		double bestDiff = 1e9;
		for (const CtbEntry& e : m_ctbTable)
		{
			const double diff = std::abs(e.aci * 36.0 - gray);
			if (diff < bestDiff)
			{
				bestDiff = diff;
				bestAci = e.aci;
			}
		}
		for (const CtbEntry& e : m_ctbTable)
		{
			if (e.aci == bestAci)
				return e.widthMm > 1e-6 ? e.widthMm : fallbackMm;
		}
	}
	const int g = qGray(color.rgb());
	return fallbackMm * (0.55 + (255 - g) / 255.0 * 1.2);
}

bool DrawingSheetCanvasWidget::editCtbTable()
{
	bool ok = false;
	QString text;
	for (const CtbEntry& e : m_ctbTable)
		text += QStringLiteral("%1,%2\n").arg(e.aci).arg(e.widthMm, 0, 'f', 2);
	if (text.isEmpty())
		text = QStringLiteral("1,0.25\n7,0.35\n8,0.18\n");
	text = QInputDialog::getMultiLineText(this, QStringLiteral("CTB 线宽表"),
										  QStringLiteral("每行：ACI,线宽mm"), text, &ok);
	if (!ok)
		return false;
	QVector<CtbEntry> next;
	for (const QString& line : text.split(QLatin1Char('\n'), QString::SkipEmptyParts))
	{
		const QStringList parts = line.split(QLatin1Char(','));
		if (parts.size() < 2)
			continue;
		CtbEntry e;
		e.aci = parts[0].trimmed().toInt();
		e.widthMm = parts[1].trimmed().toDouble();
		if (e.aci >= 0 && e.widthMm > 0)
			next.push_back(e);
	}
	m_ctbTable = next;
	m_ctbEnabled = true;
	emit sheetChanged();
	update();
	emit statusMessage(QStringLiteral("已更新 CTB 表（%1 项）").arg(m_ctbTable.size()));
	return true;
}

void DrawingSheetCanvasWidget::setCtbTable(const QVector<CtbEntry>& table)
{
	m_ctbTable = table;
}

bool DrawingSheetCanvasWidget::syncTitleBlockAttrsFromPaper()
{
	for (SheetBlockDef& def : m_blockDefs)
	{
		if (def.attrDefs.isEmpty())
		{
			def.attrDefs.push_back({QStringLiteral("TITLE"), QStringLiteral("图名"), m_paper.title, QPointF(8, -8)});
			def.attrDefs.push_back(
				{QStringLiteral("DWGNO"), QStringLiteral("图号"), m_paper.drawingNo, QPointF(8, -14)});
			def.attrDefs.push_back({QStringLiteral("DATE"), QStringLiteral("日期"), m_paper.date, QPointF(8, -20)});
			def.attrDefs.push_back(
				{QStringLiteral("MATERIAL"), QStringLiteral("材料"), m_paper.material, QPointF(8, -26)});
		}
		else
		{
			for (auto& a : def.attrDefs)
			{
				if (a.tag == QLatin1String("TITLE"))
					a.defaultValue = m_paper.title;
				else if (a.tag == QLatin1String("DWGNO"))
					a.defaultValue = m_paper.drawingNo;
				else if (a.tag == QLatin1String("DATE"))
					a.defaultValue = m_paper.date;
				else if (a.tag == QLatin1String("MATERIAL"))
					a.defaultValue = m_paper.material;
			}
		}
	}
	for (SheetBlockRef& r : m_blockRefs)
	{
		r.attrValues.insert(QStringLiteral("TITLE"), m_paper.title);
		r.attrValues.insert(QStringLiteral("DWGNO"), m_paper.drawingNo);
		r.attrValues.insert(QStringLiteral("DATE"), m_paper.date);
		r.attrValues.insert(QStringLiteral("MATERIAL"), m_paper.material);
	}
	emit sheetChanged();
	return true;
}

bool DrawingSheetCanvasWidget::selectionStyle(SheetEntityStyle& outStyle, QString& outLayerId) const
{
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		outStyle = m_views[m_selectedViewIndex].style;
		outLayerId = m_views[m_selectedViewIndex].layerId;
		return true;
	}
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		outStyle = m_dims[m_selectedDimIndex].style;
		outLayerId = m_dims[m_selectedDimIndex].layerId;
		return true;
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		outStyle = m_notes[m_selectedNoteIndex].style;
		outLayerId = m_notes[m_selectedNoteIndex].layerId;
		return true;
	}
	if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
	{
		outStyle = m_hatches[m_selectedHatchIndex].style;
		outLayerId = m_hatches[m_selectedHatchIndex].layerId;
		return true;
	}
	if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
	{
		outStyle = m_blockRefs[m_selectedBlockRefIndex].style;
		outLayerId = m_blockRefs[m_selectedBlockRefIndex].layerId;
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::applyStyleToSelection(const SheetEntityStyle& style, const QString& layerId)
{
	bool ok = false;
	auto apply = [&](SheetEntityStyle& s, QString& lid) {
		s = style;
		if (!layerId.isEmpty())
			lid = layerId;
		ok = true;
	};
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
		apply(m_views[m_selectedViewIndex].style, m_views[m_selectedViewIndex].layerId);
	else if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
		apply(m_dims[m_selectedDimIndex].style, m_dims[m_selectedDimIndex].layerId);
	else if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
		apply(m_notes[m_selectedNoteIndex].style, m_notes[m_selectedNoteIndex].layerId);
	else if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
		apply(m_hatches[m_selectedHatchIndex].style, m_hatches[m_selectedHatchIndex].layerId);
	else if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
		apply(m_blockRefs[m_selectedBlockRefIndex].style, m_blockRefs[m_selectedBlockRefIndex].layerId);
	if (!ok)
		return false;
	emit sheetChanged();
	emitSelectionChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::matchPropFromSelection()
{
	SheetEntityStyle s;
	QString lid;
	if (!selectionStyle(s, lid))
	{
		emit statusMessage(QStringLiteral("请先选中源对象"));
		return false;
	}
	m_matchStyle = s;
	m_matchLayerId = lid;
	m_hasMatchStyle = true;
	m_tool = DrawingCanvasTool::MatchProp;
	emit statusMessage(QStringLiteral("匹配特性：再点目标对象"));
	return true;
}

bool DrawingSheetCanvasWidget::isModifyTool(DrawingCanvasTool t) const
{
	return t == DrawingCanvasTool::ModifyMove || t == DrawingCanvasTool::ModifyCopy ||
		   t == DrawingCanvasTool::ModifyRotate || t == DrawingCanvasTool::ModifyMirror ||
		   t == DrawingCanvasTool::ModifyErase || t == DrawingCanvasTool::MatchProp ||
		   t == DrawingCanvasTool::ModifyScale || t == DrawingCanvasTool::ModifyTrim ||
		   t == DrawingCanvasTool::ModifyOffset || t == DrawingCanvasTool::ModifyFillet ||
		   t == DrawingCanvasTool::ModifyChamfer || t == DrawingCanvasTool::ModifyExtend ||
		   t == DrawingCanvasTool::ModifyArray || t == DrawingCanvasTool::ModifyPolarArray ||
		   t == DrawingCanvasTool::ModifyBreak || t == DrawingCanvasTool::ModifyJoin ||
		   t == DrawingCanvasTool::ModifyStretch || t == DrawingCanvasTool::InsertBlock ||
		   t == DrawingCanvasTool::ExplodeBlock;
}

void DrawingSheetCanvasWidget::scaleSelection(const QPointF& pivot, double factor)
{
	if (!(std::abs(factor) > 1e-9))
		return;
	auto scalePt = [&](QPointF& p) { p = pivot + (p - pivot) * factor; };
	auto scalePoly = [&](QVector<Polyline2d>& polys) {
		for (auto& poly : polys)
			for (QPointF& p : poly.points)
				scalePt(p);
	};
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		auto& v = m_views[m_selectedViewIndex];
		scalePoly(v.visible);
		scalePoly(v.hidden);
		QPointF c = v.frame.center();
		scalePt(c);
		v.frame = QRectF(c - QPointF(v.frame.width() * factor * 0.5, v.frame.height() * factor * 0.5),
						 QSizeF(v.frame.width() * factor, v.frame.height() * factor));
	}
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		auto& d = m_dims[m_selectedDimIndex];
		scalePt(d.p1);
		scalePt(d.p2);
		scalePt(d.p3);
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		scalePt(m_notes[m_selectedNoteIndex].anchor);
		scalePt(m_notes[m_selectedNoteIndex].textPos);
	}
	if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
	{
		for (QPointF& p : m_hatches[m_selectedHatchIndex].boundary)
			scalePt(p);
	}
	if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
	{
		scalePt(m_blockRefs[m_selectedBlockRefIndex].insert);
		m_blockRefs[m_selectedBlockRefIndex].scale *= factor;
	}
	emit sheetChanged();
	update();
}

bool DrawingSheetCanvasWidget::trimSketchAt(const QPointF& scenePos)
{
	if (!m_sketch.trimAt(scenePos, snapTolMm()))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::offsetSketchAt(const QPointF& scenePos, double distMm)
{
	if (!m_sketch.offsetClosedAt(scenePos, distMm, snapTolMm(), m_currentLayerId))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::filletSketchAt(const QPointF& scenePos, double radiusMm)
{
	if (!m_sketch.filletLinesAt(scenePos, radiusMm, snapTolMm(), m_currentLayerId))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::chamferSketchAt(const QPointF& scenePos, double distMm)
{
	if (!m_sketch.chamferLinesAt(scenePos, distMm, snapTolMm(), m_currentLayerId))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::breakSketchAt(const QPointF& scenePos)
{
	if (!m_sketch.breakLineAt(scenePos, snapTolMm()))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::joinSketchAt(const QPointF& scenePos)
{
	if (!m_sketch.joinLinesAt(scenePos, snapTolMm()))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::stretchSketchWindow(const QRectF& win, const QPointF& delta)
{
	if (!m_sketch.stretchInWindow(win, delta))
	{
		emit statusMessage(QStringLiteral("拉伸失败：窗内无顶点"));
		return false;
	}
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::sectionMarkOriginHint(double outOriginMm[3], double outNormal[3]) const
{
	if (!outOriginMm || !outNormal)
		return false;
	const DrawingView* front = nullptr;
	const DrawingView* section = nullptr;
	for (const DrawingView& v : m_views)
	{
		if (v.kind == QLatin1String("front"))
			front = &v;
		if (v.kind == QLatin1String("section") && v.hasMark)
			section = &v;
	}
	if (!front || !section)
		return false;
	const QPointF mid = (section->markP1 + section->markP2) * 0.5;
	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	outOriginMm[0] = 0.0;
	outOriginMm[1] = (mid.y() - front->frame.center().y()) / s;
	outOriginMm[2] = 0.0;
	outNormal[0] = 0.0;
	outNormal[1] = 1.0;
	outNormal[2] = 0.0;
	return true;
}

void DrawingSheetCanvasWidget::setHalfSection(bool on)
{
	m_halfSection = on;
	emit statusMessage(on ? QStringLiteral("半剖：开") : QStringLiteral("半剖：关"));
}

void DrawingSheetCanvasWidget::applyHalfSectionClip()
{
	if (!m_halfSection)
		return;
	int frontIdx = findViewIndexByKind(QStringLiteral("front"));
	if (frontIdx < 0)
		return;
	DrawingView& front = m_views[frontIdx];
	const double cx = front.frame.center().x();
	auto clipSide = [&](QVector<Polyline2d>& polys) {
		QVector<Polyline2d> out;
		for (const Polyline2d& poly : polys)
		{
			Polyline2d cur;
			for (const QPointF& p : poly.points)
			{
				if (p.x() <= cx + 1e-6)
					cur.points.push_back(p);
				else if (!cur.points.isEmpty())
				{
					if (cur.points.size() >= 2)
						out.push_back(cur);
					cur.points.clear();
				}
			}
			if (cur.points.size() >= 2)
				out.push_back(cur);
		}
		polys = out;
	};
	clipSide(front.visible);
	clipSide(front.hidden);
	front.visCache.clear();
	front.hidCache.clear();
	if (m_sheetModel)
		m_sheetModel->updateViewGeometry(front);
}

bool DrawingSheetCanvasWidget::extendSketchAt(const QPointF& boundaryPos, const QPointF& linePos)
{
	QLineF boundary;
	bool haveBound = false;
	double bestD = snapTolMm() * 6.0;
	for (const auto& ln : m_sketch.document().lines())
	{
		const SkPoint* p1 = m_sketch.document().findPoint(ln.p1);
		const SkPoint* p2 = m_sketch.document().findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		QLineF seg(SheetSketchAdapter::toScene(p1->p), SheetSketchAdapter::toScene(p2->p));
		const QPointF ab = seg.p2() - seg.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(boundaryPos - seg.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		const double d = QLineF(seg.p1() + ab * t, boundaryPos).length();
		if (d < bestD)
		{
			bestD = d;
			boundary = seg;
			haveBound = true;
		}
	}
	if (!haveBound)
	{
		const int vi = hitViewIndex(boundaryPos);
		if (vi >= 0)
		{
			const QRectF f = m_views[vi].frame;
			const QPointF c = f.center();
			if (std::abs(boundaryPos.x() - f.left()) < std::abs(boundaryPos.y() - c.y()))
				boundary = QLineF(f.topLeft(), f.bottomLeft());
			else if (std::abs(boundaryPos.x() - f.right()) < std::abs(boundaryPos.y() - c.y()))
				boundary = QLineF(f.topRight(), f.bottomRight());
			else if (boundaryPos.y() < c.y())
				boundary = QLineF(f.topLeft(), f.topRight());
			else
				boundary = QLineF(f.bottomLeft(), f.bottomRight());
			haveBound = true;
		}
	}
	if (!haveBound)
		return false;
	if (!m_sketch.extendToBoundary(boundary, linePos, snapTolMm()))
		return false;
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::arraySelectionPolar(int count, double angleDeg, const QPointF& pivot)
{
	count = qBound(2, count, 36);
	const bool hadView = m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size();
	const bool hadDim = m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size();
	const bool hadNote = m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size();
	const bool hadBlock = m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size();
	const bool hadSketch = m_selectedSketchId >= 0;
	if (!hadView && !hadDim && !hadNote && !hadBlock && !hadSketch)
		return false;
	DrawingView baseView;
	SheetDimension baseDim;
	SheetNote baseNote;
	SheetBlockRef baseBlock;
	if (hadView)
		baseView = m_views[m_selectedViewIndex];
	if (hadDim)
		baseDim = m_dims[m_selectedDimIndex];
	if (hadNote)
		baseNote = m_notes[m_selectedNoteIndex];
	if (hadBlock)
		baseBlock = m_blockRefs[m_selectedBlockRefIndex];
	const double step = angleDeg * 3.141592653589793 / 180.0;
	auto rot = [&](QPointF p, double ang) {
		const double c = std::cos(ang), s = std::sin(ang);
		const QPointF d = p - pivot;
		return pivot + QPointF(d.x() * c - d.y() * s, d.x() * s + d.y() * c);
	};
	for (int i = 1; i < count; ++i)
	{
		const double ang = step * i;
		if (hadView)
		{
			DrawingView v = baseView;
			v.id = QStringLiteral("view_%1").arg(m_nextCatalogViewId++);
			const QPointF c0 = v.frame.center();
			const QPointF c1 = rot(c0, ang);
			const QPointF delta = c1 - c0;
			v.frame.translate(delta);
			for (auto& poly : v.visible)
				for (QPointF& p : poly.points)
					p = rot(p, ang);
			for (auto& poly : v.hidden)
				for (QPointF& p : poly.points)
					p = rot(p, ang);
			m_views.push_back(v);
		}
		if (hadDim)
		{
			SheetDimension d = baseDim;
			d.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			d.p1 = rot(d.p1, ang);
			d.p2 = rot(d.p2, ang);
			d.p3 = rot(d.p3, ang);
			m_dims.push_back(d);
		}
		if (hadNote)
		{
			SheetNote n = baseNote;
			n.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
			n.anchor = rot(n.anchor, ang);
			n.textPos = rot(n.textPos, ang);
			m_notes.push_back(n);
		}
		if (hadBlock)
		{
			SheetBlockRef br = baseBlock;
			br.id = QStringLiteral("BR%1").arg(m_nextBlockRefId++);
			br.insert = rot(br.insert, ang);
			br.rotationDeg += angleDeg * i;
			m_blockRefs.push_back(br);
		}
		if (hadSketch)
			m_sketch.duplicateEntityRotated(m_selectedSketchId, pivot, ang, m_currentLayerId);
	}
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::arraySelectionRect(int cols, int rows, double dx, double dy)
{
	cols = qBound(1, cols, 50);
	rows = qBound(1, rows, 50);
	if (cols * rows <= 1)
		return false;
	const bool hadView = m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size();
	const bool hadDim = m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size();
	const bool hadNote = m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size();
	const bool hadBlock = m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size();
	const bool hadSketch = m_selectedSketchId >= 0;
	if (!hadView && !hadDim && !hadNote && !hadBlock && !hadSketch)
		return false;
	DrawingView baseView;
	SheetDimension baseDim;
	SheetNote baseNote;
	SheetBlockRef baseBlock;
	if (hadView)
		baseView = m_views[m_selectedViewIndex];
	if (hadDim)
		baseDim = m_dims[m_selectedDimIndex];
	if (hadNote)
		baseNote = m_notes[m_selectedNoteIndex];
	if (hadBlock)
		baseBlock = m_blockRefs[m_selectedBlockRefIndex];

	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < cols; ++c)
		{
			if (r == 0 && c == 0)
				continue;
			const QPointF delta(c * dx, r * dy);
			if (hadView)
			{
				DrawingView v = baseView;
				v.id = QStringLiteral("view_%1").arg(m_nextCatalogViewId++);
				v.frame.translate(delta);
				for (auto& poly : v.visible)
					for (QPointF& p : poly.points)
						p += delta;
				for (auto& poly : v.hidden)
					for (QPointF& p : poly.points)
						p += delta;
				m_views.push_back(v);
			}
			if (hadDim)
			{
				SheetDimension d = baseDim;
				d.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
				d.p1 += delta;
				d.p2 += delta;
				d.p3 += delta;
				m_dims.push_back(d);
			}
			if (hadNote)
			{
				SheetNote n = baseNote;
				n.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
				n.anchor += delta;
				n.textPos += delta;
				m_notes.push_back(n);
			}
			if (hadBlock)
			{
				SheetBlockRef br = baseBlock;
				br.id = QStringLiteral("BR%1").arg(m_nextBlockRefId++);
				br.insert += delta;
				m_blockRefs.push_back(br);
			}
			if (hadSketch)
				m_sketch.duplicateEntityTranslated(m_selectedSketchId, delta, m_currentLayerId);
		}
	}
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::explodeSelectedBlock()
{
	if (m_selectedBlockRefIndex < 0 || m_selectedBlockRefIndex >= m_blockRefs.size())
	{
		emit statusMessage(QStringLiteral("请先选中块参照"));
		return false;
	}
	const SheetBlockRef r = m_blockRefs[m_selectedBlockRefIndex];
	const SheetBlockDef* def = nullptr;
	for (const auto& d : m_blockDefs)
	{
		if (d.id == r.defId)
		{
			def = &d;
			break;
		}
	}
	if (!def)
		return false;
	const double ang = r.rotationDeg * 3.141592653589793 / 180.0;
	const double c = std::cos(ang), s = std::sin(ang);
	DrawingView v;
	v.id = QStringLiteral("exploded_%1").arg(m_nextCatalogViewId++);
	v.title = QStringLiteral("炸开 %1").arg(def->name);
	v.kind = QStringLiteral("front");
	v.layerId = r.layerId;
	v.style = r.style;
	QRectF box;
	for (const auto& poly : def->geometry)
	{
		Polyline2d out;
		for (const QPointF& lp : poly.points)
		{
			QPointF sp = def->base + (lp - def->base) * r.scale;
			const QPointF d = sp - def->base;
			sp = def->base + QPointF(d.x() * c - d.y() * s, d.x() * s + d.y() * c);
			sp = sp - def->base + r.insert;
			out.points.push_back(sp);
			box = box.isNull() ? QRectF(sp, QSizeF(0, 0)) : box.united(QRectF(sp, QSizeF(0, 0)));
		}
		if (out.points.size() >= 2)
			v.visible.push_back(out);
	}
	v.frame = box.adjusted(-8, -18, 8, 8);
	m_blockRefs.removeAt(m_selectedBlockRefIndex);
	m_selectedBlockRefIndex = -1;
	m_views.push_back(v);
	emit sheetChanged();
	emitSelectionChanged();
	emit statusMessage(QStringLiteral("已炸开图块"));
	update();
	return true;
}

void DrawingSheetCanvasWidget::setCurrentDimStyleId(const QString& id)
{
	m_currentDimStyleId = id.isEmpty() ? QStringLiteral("Standard") : id;
	emit sheetChanged();
}

bool DrawingSheetCanvasWidget::editTitleBlockAttrs()
{
	bool ok = false;
	const QString title = QInputDialog::getText(this, QStringLiteral("图框属性"), QStringLiteral("图名"),
												QLineEdit::Normal, m_paper.title, &ok);
	if (!ok)
		return false;
	m_paper.title = title.trimmed();
	const QString no = QInputDialog::getText(this, QStringLiteral("图框属性"), QStringLiteral("图号"),
											 QLineEdit::Normal, m_paper.drawingNo, &ok);
	if (ok)
		m_paper.drawingNo = no.trimmed();
	const QString mat = QInputDialog::getText(this, QStringLiteral("图框属性"), QStringLiteral("材料"),
											  QLineEdit::Normal, m_paper.material, &ok);
	if (ok)
		m_paper.material = mat.trimmed();
	syncTitleBlockAttrsFromPaper();
	emit sheetChanged();
	update();
	return true;
}

void DrawingSheetCanvasWidget::transformSelection(const QPointF& delta)
{
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
		moveViewBy(m_selectedViewIndex, delta);
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		auto& d = m_dims[m_selectedDimIndex];
		d.p1 += delta;
		d.p2 += delta;
		d.p3 += delta;
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes[m_selectedNoteIndex].anchor += delta;
		m_notes[m_selectedNoteIndex].textPos += delta;
	}
	if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
	{
		for (QPointF& p : m_hatches[m_selectedHatchIndex].boundary)
			p += delta;
	}
	if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
		m_blockRefs[m_selectedBlockRefIndex].insert += delta;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::rotateSelection(const QPointF& pivot, double angleRad)
{
	auto rotPoly = [&](QVector<Polyline2d>& polys) {
		for (auto& poly : polys)
			for (QPointF& p : poly.points)
				p = rotPt(p, pivot, angleRad);
	};
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		auto& v = m_views[m_selectedViewIndex];
		rotPoly(v.visible);
		rotPoly(v.hidden);
		const QPointF c = rotPt(v.frame.center(), pivot, angleRad);
		v.frame.moveCenter(c);
	}
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		auto& d = m_dims[m_selectedDimIndex];
		d.p1 = rotPt(d.p1, pivot, angleRad);
		d.p2 = rotPt(d.p2, pivot, angleRad);
		d.p3 = rotPt(d.p3, pivot, angleRad);
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes[m_selectedNoteIndex].anchor = rotPt(m_notes[m_selectedNoteIndex].anchor, pivot, angleRad);
		m_notes[m_selectedNoteIndex].textPos = rotPt(m_notes[m_selectedNoteIndex].textPos, pivot, angleRad);
	}
	if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
	{
		m_blockRefs[m_selectedBlockRefIndex].insert = rotPt(m_blockRefs[m_selectedBlockRefIndex].insert, pivot, angleRad);
		m_blockRefs[m_selectedBlockRefIndex].rotationDeg += angleRad * 180.0 / 3.141592653589793;
	}
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::mirrorSelection(const QLineF& axis)
{
	auto mirPoly = [&](QVector<Polyline2d>& polys) {
		for (auto& poly : polys)
			for (QPointF& p : poly.points)
				p = mirrorPt(p, axis);
	};
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		auto& v = m_views[m_selectedViewIndex];
		mirPoly(v.visible);
		mirPoly(v.hidden);
		v.frame.moveCenter(mirrorPt(v.frame.center(), axis));
	}
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		auto& d = m_dims[m_selectedDimIndex];
		d.p1 = mirrorPt(d.p1, axis);
		d.p2 = mirrorPt(d.p2, axis);
		d.p3 = mirrorPt(d.p3, axis);
	}
	if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes[m_selectedNoteIndex].anchor = mirrorPt(m_notes[m_selectedNoteIndex].anchor, axis);
		m_notes[m_selectedNoteIndex].textPos = mirrorPt(m_notes[m_selectedNoteIndex].textPos, axis);
	}
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::eraseSelection()
{
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		m_views.removeAt(m_selectedViewIndex);
		m_selectedViewIndex = -1;
	}
	else if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		m_dims.removeAt(m_selectedDimIndex);
		m_selectedDimIndex = -1;
	}
	else if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes.removeAt(m_selectedNoteIndex);
		m_selectedNoteIndex = -1;
	}
	else if (m_selectedHatchIndex >= 0 && m_selectedHatchIndex < m_hatches.size())
	{
		m_hatches.removeAt(m_selectedHatchIndex);
		m_selectedHatchIndex = -1;
	}
	else if (m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size())
	{
		m_blockRefs.removeAt(m_selectedBlockRefIndex);
		m_selectedBlockRefIndex = -1;
	}
	else if (m_selectedSketchId >= 0)
	{
		m_sketch.removeEntity(m_selectedSketchId);
		m_selectedSketchId = -1;
	}
	else
		return;
	emit sheetChanged();
	emitSelectionChanged();
	update();
}

int DrawingSheetCanvasWidget::hitHatchIndex(const QPointF& scenePos) const
{
	for (int i = m_hatches.size() - 1; i >= 0; --i)
	{
		if (m_hatches[i].boundary.size() < 3)
			continue;
		QPolygonF poly(m_hatches[i].boundary);
		if (poly.containsPoint(scenePos, Qt::OddEvenFill))
			return i;
	}
	return -1;
}

int DrawingSheetCanvasWidget::hitBlockRefIndex(const QPointF& scenePos) const
{
	for (int i = m_blockRefs.size() - 1; i >= 0; --i)
	{
		if (QLineF(m_blockRefs[i].insert, scenePos).length() < snapTolMm() * 4.0)
			return i;
	}
	return -1;
}

void DrawingSheetCanvasWidget::drawHatch(QPainter& p, const SheetHatch& h) const
{
	if (h.boundary.size() < 3)
		return;
	p.save();
	QPolygonF w;
	for (const QPointF& pt : h.boundary)
		w << sceneToWidget(pt);
	QColor fill = resolvePen(h.layerId, h.style).color();
	if (h.pattern.compare(QLatin1String("SOLID"), Qt::CaseInsensitive) == 0 || h.pattern.isEmpty())
	{
		fill.setAlpha(90);
		p.setBrush(fill);
		p.setPen(resolvePen(h.layerId, h.style));
		p.drawPolygon(w);
	}
	else
	{
		// ANSI31 等：裁剪斜线填充
		p.setBrush(Qt::NoBrush);
		p.setPen(resolvePen(h.layerId, h.style));
		p.drawPolygon(w);
		p.setClipRegion(QRegion(w.toPolygon()));
		const QRectF box = w.boundingRect();
		const double step = qMax(4.0, 6.0 * h.scale * m_zoom);
		const double ang = h.angleDeg * 3.141592653589793 / 180.0;
		const double ca = std::cos(ang), sa = std::sin(ang);
		const QPointF origin = box.center();
		for (double t = -box.width() - box.height(); t < box.width() + box.height(); t += step)
		{
			const QPointF a = origin + QPointF(-ca * 2000 + sa * t, -sa * 2000 - ca * t);
			const QPointF b = origin + QPointF(ca * 2000 + sa * t, sa * 2000 - ca * t);
			p.drawLine(a, b);
		}
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawBlockRef(QPainter& p, const SheetBlockRef& r) const
{
	const SheetBlockDef* def = nullptr;
	for (const auto& d : m_blockDefs)
	{
		if (d.id == r.defId)
		{
			def = &d;
			break;
		}
	}
	if (!def)
		return;
	p.save();
	const bool sel = m_selectedBlockRefIndex >= 0 && m_selectedBlockRefIndex < m_blockRefs.size() &&
					 m_blockRefs[m_selectedBlockRefIndex].id == r.id;
	const double ang = r.rotationDeg * 3.141592653589793 / 180.0;
	auto xform = [&](QPointF lp) {
		QPointF sp = def->base + (lp - def->base) * r.scale;
		sp = rotPt(sp, def->base, ang);
		return sp - def->base + r.insert;
	};
	for (int gi = 0; gi < def->geometry.size(); ++gi)
	{
		SheetEntityStyle gs;
		if (gi < def->geometryStyles.size())
			gs = def->geometryStyles[gi];
		else
		{
			gs.colorByBlock = true;
			gs.lineTypeByBlock = true;
			gs.lineWidthByBlock = true;
			gs.colorByLayer = false;
			gs.lineTypeByLayer = false;
			gs.lineWidthByLayer = false;
		}
		p.setPen(resolvePen(r.layerId, gs, false, sel, &r.style, &r.layerId));
		QPolygonF w;
		for (const QPointF& lp : def->geometry[gi].points)
			w << sceneToWidget(xform(lp));
		if (w.size() >= 2)
			p.drawPolyline(w);
	}
	p.setPen(QPen(resolvePen(r.layerId, r.style, false, sel).color(), qMax(0.8, 0.9 * m_zoom)));
	QFont f = p.font();
	f.setPointSizeF(qMax(7.0, 8.0 * m_zoom));
	p.setFont(f);
	int row = 0;
	for (const SheetBlockDef::AttrDef& a : def->attrDefs)
	{
		const QString val = r.attrValues.value(a.tag, a.defaultValue);
		if (val.isEmpty())
			continue;
		QPointF lp = def->base + a.position;
		if (a.position.isNull())
			lp = def->base + QPointF(6.0, -6.0 - row * 5.0);
		p.drawText(sceneToWidget(xform(lp)), val);
		++row;
	}
	p.restore();
}

bool DrawingSheetCanvasWidget::createBlockFromSelection(const QString& name)
{
	if (m_selectedViewIndex < 0 || m_selectedViewIndex >= m_views.size())
	{
		emit statusMessage(QStringLiteral("请先选中视图以建块"));
		return false;
	}
	const DrawingView& v = m_views[m_selectedViewIndex];
	SheetBlockDef def;
	def.id = QStringLiteral("B%1").arg(m_nextBlockDefId++);
	def.name = name.isEmpty() ? def.id : name;
	def.base = v.frame.center();
	def.geometry = v.visible;
	SheetEntityStyle byBlock;
	byBlock.colorByBlock = true;
	byBlock.lineTypeByBlock = true;
	byBlock.lineWidthByBlock = true;
	byBlock.colorByLayer = false;
	byBlock.lineTypeByLayer = false;
	byBlock.lineWidthByLayer = false;
	def.geometryStyles = QVector<SheetEntityStyle>(def.geometry.size(), byBlock);
	def.attrDefs.push_back({QStringLiteral("TITLE"), QStringLiteral("图名"), m_paper.title, QPointF(8, -8)});
	def.attrDefs.push_back({QStringLiteral("DWGNO"), QStringLiteral("图号"), m_paper.drawingNo, QPointF(8, -14)});
	def.attrDefs.push_back({QStringLiteral("DATE"), QStringLiteral("日期"), m_paper.date, QPointF(8, -20)});
	def.attrDefs.push_back({QStringLiteral("MATERIAL"), QStringLiteral("材料"), m_paper.material, QPointF(8, -26)});
	m_blockDefs.push_back(def);
	emit statusMessage(QStringLiteral("已建块 %1").arg(def.name));
	emit sheetChanged();
	return true;
}

bool DrawingSheetCanvasWidget::insertBlock(const QString& defId, const QPointF& scenePos)
{
	bool found = false;
	for (const auto& d : m_blockDefs)
	{
		if (d.id == defId)
		{
			found = true;
			break;
		}
	}
	if (!found)
		return false;
	SheetBlockRef r;
	r.id = QStringLiteral("BR%1").arg(m_nextBlockRefId++);
	r.defId = defId;
	r.insert = scenePos;
	r.layerId = m_currentLayerId;
	for (const auto& d : m_blockDefs)
	{
		if (d.id != defId)
			continue;
		for (const auto& a : d.attrDefs)
			r.attrValues.insert(a.tag, a.defaultValue);
		break;
	}
	m_blockRefs.push_back(r);
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::addDimStyle(const DimStyle& style)
{
	for (auto& s : m_dimStyles)
	{
		if (s.id == style.id)
		{
			s = style;
			emit sheetChanged();
			return true;
		}
	}
	m_dimStyles.push_back(style);
	emit sheetChanged();
	return true;
}

bool DrawingSheetCanvasWidget::updateDimStyle(const DimStyle& style)
{
	return addDimStyle(style);
}

bool DrawingSheetCanvasWidget::addTextStyle(const TextStyle& style)
{
	for (auto& s : m_textStyles)
	{
		if (s.id == style.id)
		{
			s = style;
			emit sheetChanged();
			return true;
		}
	}
	m_textStyles.push_back(style);
	emit sheetChanged();
	return true;
}

const DrawingSheetCanvasWidget::DimStyle* DrawingSheetCanvasWidget::dimStyleById(const QString& id) const
{
	for (const auto& s : m_dimStyles)
	{
		if (s.id == id)
			return &s;
	}
	return m_dimStyles.isEmpty() ? nullptr : &m_dimStyles.first();
}

const DrawingSheetCanvasWidget::TextStyle* DrawingSheetCanvasWidget::textStyleById(const QString& id) const
{
	for (const auto& s : m_textStyles)
	{
		if (s.id == id)
			return &s;
	}
	return m_textStyles.isEmpty() ? nullptr : &m_textStyles.first();
}

QPixmap DrawingSheetCanvasWidget::renderPrintPreview(const QSize& pixelSize) const
{
	QPixmap pm(pixelSize.isValid() ? pixelSize : QSize(800, 600));
	pm.fill(Qt::white);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	const QRectF sceneBox = m_paper.visible ? paperRect() : contentBounds();
	if (!sceneBox.isValid())
		return pm;
	const double zx = pm.width() / sceneBox.width();
	const double zy = pm.height() / sceneBox.height();
	const double z = qMin(zx, zy) * 0.92;
	DrawingSheetCanvasWidget* self = const_cast<DrawingSheetCanvasWidget*>(this);
	const double oldZ = m_zoom;
	const QPointF oldPan = m_panOffset;
	self->m_zoom = z;
	self->m_panOffset = QPointF(-sceneBox.left() * z + (pm.width() - sceneBox.width() * z) * 0.5,
								-sceneBox.top() * z + (pm.height() - sceneBox.height() * z) * 0.5);
	paintSheet(p, true);
	self->m_zoom = oldZ;
	self->m_panOffset = oldPan;
	return pm;
}

bool DrawingSheetCanvasWidget::importDxf(const QString& filePath)
{
	QFile f(filePath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	QString code, value;
	auto readPair = [&]() -> bool {
		if (ts.atEnd())
			return false;
		code = ts.readLine().trimmed();
		if (ts.atEnd())
			return false;
		value = ts.readLine();
		return true;
	};

	QVector<Polyline2d> lines;
	QVector<SheetNote> importedNotes;
	QVector<SheetDimension> importedDims;
	QHash<QString, SheetBlockDef> importedBlocks;
	QVector<SheetBlockRef> importedInserts;
	const int circlesBefore = static_cast<int>(m_sketch.document().circles().size());
	const int arcsBefore = static_cast<int>(m_sketch.document().arcs().size());
	QHash<QString, QString> layerNameToId;
	auto mapLayer = [&](const QString& dxfName) -> QString {
		if (dxfName.isEmpty())
			return m_currentLayerId;
		const auto it = layerNameToId.constFind(dxfName);
		if (it != layerNameToId.cend())
			return it.value();
		return m_currentLayerId;
	};

	auto aciColor = [](int aci) -> QColor {
		switch (aci)
		{
		case 1:
			return QColor(255, 0, 0);
		case 2:
			return QColor(255, 255, 0);
		case 3:
			return QColor(0, 255, 0);
		case 4:
			return QColor(0, 255, 255);
		case 5:
			return QColor(0, 0, 255);
		case 6:
			return QColor(255, 0, 255);
		case 7:
		default:
			return QColor(30, 34, 42);
		}
	};

	while (readPair())
	{
		const QString ent = value.trimmed();
		if (code != QLatin1String("0"))
			continue;

		if (ent == QLatin1String("TABLE"))
		{
			QString tableName;
			bool havePending = false;
			for (;;)
			{
				if (!havePending)
				{
					if (!readPair())
						break;
				}
				havePending = false;
				if (code == QLatin1String("0") && value.trimmed() == QLatin1String("ENDTAB"))
					break;
				if (code == QLatin1String("2") && tableName.isEmpty())
				{
					tableName = value.trimmed();
					continue;
				}
				if (tableName != QLatin1String("LAYER") || code != QLatin1String("0") ||
					value.trimmed() != QLatin1String("LAYER"))
					continue;

				QString name;
				int flags = 0;
				int aci = 7;
				while (readPair())
				{
					if (code == QLatin1String("0"))
					{
						havePending = true;
						break;
					}
					if (code == QLatin1String("2"))
						name = value.trimmed();
					else if (code == QLatin1String("70"))
						flags = value.toInt();
					else if (code == QLatin1String("62"))
						aci = value.toInt();
				}
				if (name.isEmpty())
					continue;
				if (name == QLatin1String("0"))
				{
					layerNameToId.insert(name, defaultLayerId());
					continue;
				}
				const QString id = addLayer(name);
				setLayerColor(id, aciColor(qAbs(aci)));
				setLayerFrozen(id, (flags & 1) != 0);
				setLayerPlottable(id, aci >= 0);
				layerNameToId.insert(name, id);
			}
			continue;
		}

		if (ent == QLatin1String("BLOCK"))
		{
			SheetBlockDef def;
			def.id = QStringLiteral("B%1").arg(m_nextBlockDefId++);
			QString bname;
			double bx = 0, by = 0;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("2"))
					bname = value.trimmed();
				else if (code == QLatin1String("10"))
					bx = value.toDouble();
				else if (code == QLatin1String("20"))
					by = value.toDouble();
			}
			def.name = bname.isEmpty() ? def.id : bname;
			def.base = QPointF(bx, by);
			// 读到 0 时可能已是 LINE/LWPOLYLINE/ENDBLK
			bool pending = (code == QLatin1String("0"));
			for (;;)
			{
				if (!pending)
				{
					if (!readPair())
						break;
				}
				pending = false;
				const QString se = value.trimmed();
				if (code == QLatin1String("0") && se == QLatin1String("ENDBLK"))
					break;
				if (code == QLatin1String("0") && se == QLatin1String("LINE"))
				{
					double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
					while (readPair())
					{
						if (code == QLatin1String("0"))
						{
							pending = true;
							break;
						}
						if (code == QLatin1String("10"))
							x1 = value.toDouble();
						else if (code == QLatin1String("20"))
							y1 = value.toDouble();
						else if (code == QLatin1String("11"))
							x2 = value.toDouble();
						else if (code == QLatin1String("21"))
							y2 = value.toDouble();
					}
					Polyline2d poly;
					poly.points << QPointF(x1, y1) << QPointF(x2, y2);
					def.geometry.push_back(poly);
				}
				else if (code == QLatin1String("0") && se == QLatin1String("LWPOLYLINE"))
				{
					Polyline2d poly;
					double lastX = 0;
					bool haveX = false;
					while (readPair())
					{
						if (code == QLatin1String("0"))
						{
							pending = true;
							break;
						}
						if (code == QLatin1String("10"))
						{
							lastX = value.toDouble();
							haveX = true;
						}
						else if (code == QLatin1String("20") && haveX)
						{
							poly.points.push_back(QPointF(lastX, value.toDouble()));
							haveX = false;
						}
					}
					if (poly.points.size() >= 2)
						def.geometry.push_back(poly);
				}
				else if (code == QLatin1String("0") && se == QLatin1String("ATTDEF"))
				{
					SheetBlockDef::AttrDef a;
					double ax = bx, ay = by;
					while (readPair())
					{
						if (code == QLatin1String("0"))
						{
							pending = true;
							break;
						}
						if (code == QLatin1String("10"))
							ax = value.toDouble();
						else if (code == QLatin1String("20"))
							ay = value.toDouble();
						else if (code == QLatin1String("1"))
							a.defaultValue = value.trimmed();
						else if (code == QLatin1String("2"))
							a.tag = value.trimmed();
						else if (code == QLatin1String("3"))
							a.prompt = value.trimmed();
					}
					a.position = QPointF(ax, ay) - def.base;
					if (!a.tag.isEmpty())
						def.attrDefs.push_back(a);
				}
				else if (code == QLatin1String("0"))
				{
					// 跳过块内其它实体直到下一实体头
					while (readPair())
					{
						if (code == QLatin1String("0"))
						{
							pending = true;
							break;
						}
					}
				}
			}
			if (!bname.isEmpty() && !bname.startsWith(QLatin1Char('*')))
				importedBlocks.insert(bname, def);
			continue;
		}

		if (ent == QLatin1String("INSERT"))
		{
			QString bname;
			QString layerName;
			double x = 0, y = 0, sx = 1.0, rot = 0.0;
			int attrsFollow = 0;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("2"))
					bname = value.trimmed();
				else if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("10"))
					x = value.toDouble();
				else if (code == QLatin1String("20"))
					y = value.toDouble();
				else if (code == QLatin1String("41"))
					sx = value.toDouble();
				else if (code == QLatin1String("50"))
					rot = value.toDouble();
				else if (code == QLatin1String("66"))
					attrsFollow = value.toInt();
			}
			SheetBlockRef r;
			r.id = QStringLiteral("BR%1").arg(m_nextBlockRefId++);
			r.defId = bname;
			r.insert = QPointF(x, y);
			r.scale = sx;
			r.rotationDeg = rot;
			r.layerId = mapLayer(layerName);
			bool pending = (code == QLatin1String("0"));
			if (attrsFollow && !bname.isEmpty())
			{
				for (;;)
				{
					if (!pending)
					{
						if (!readPair())
							break;
					}
					pending = false;
					const QString se = value.trimmed();
					if (code == QLatin1String("0") && se == QLatin1String("SEQEND"))
						break;
					if (code == QLatin1String("0") && se == QLatin1String("ATTRIB"))
					{
						QString tag, val;
						while (readPair())
						{
							if (code == QLatin1String("0"))
							{
								pending = true;
								break;
							}
							if (code == QLatin1String("2"))
								tag = value.trimmed();
							else if (code == QLatin1String("1"))
								val = value.trimmed();
						}
						if (!tag.isEmpty())
							r.attrValues.insert(tag, val);
					}
					else if (code == QLatin1String("0"))
					{
						while (readPair())
						{
							if (code == QLatin1String("0"))
							{
								pending = true;
								break;
							}
						}
					}
				}
			}
			if (!bname.isEmpty())
				importedInserts.push_back(r);
			continue;
		}

		if (ent == QLatin1String("LINE"))
		{
			double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
			QString layerName;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("10"))
					x1 = value.toDouble();
				else if (code == QLatin1String("20"))
					y1 = value.toDouble();
				else if (code == QLatin1String("11"))
					x2 = value.toDouble();
				else if (code == QLatin1String("21"))
					y2 = value.toDouble();
			}
			Polyline2d poly;
			poly.points << QPointF(x1, y1) << QPointF(x2, y2);
			lines.push_back(poly);
			Q_UNUSED(layerName);
		}
		else if (ent == QLatin1String("CIRCLE"))
		{
			double cx = 0, cy = 0, r = 0;
			QString layerName;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("10"))
					cx = value.toDouble();
				else if (code == QLatin1String("20"))
					cy = value.toDouble();
				else if (code == QLatin1String("40"))
					r = value.toDouble();
			}
			m_sketch.addCircleAt(QPointF(cx, cy), r, mapLayer(layerName));
		}
		else if (ent == QLatin1String("ARC"))
		{
			double cx = 0, cy = 0, r = 0, a0 = 0, a1 = 0;
			QString layerName;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("10"))
					cx = value.toDouble();
				else if (code == QLatin1String("20"))
					cy = value.toDouble();
				else if (code == QLatin1String("40"))
					r = value.toDouble();
				else if (code == QLatin1String("50"))
					a0 = value.toDouble();
				else if (code == QLatin1String("51"))
					a1 = value.toDouble();
			}
			m_sketch.addArcAt(QPointF(cx, cy), r, a0, a1, mapLayer(layerName));
		}
		else if (ent == QLatin1String("LWPOLYLINE"))
		{
			Polyline2d poly;
			int nVerts = 0;
			double lastX = 0;
			bool haveX = false;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("90"))
					nVerts = value.toInt();
				else if (code == QLatin1String("10"))
				{
					lastX = value.toDouble();
					haveX = true;
				}
				else if (code == QLatin1String("20") && haveX)
				{
					poly.points.push_back(QPointF(lastX, value.toDouble()));
					haveX = false;
					Q_UNUSED(nVerts);
				}
			}
			if (poly.points.size() >= 2)
				lines.push_back(poly);
		}
		else if (ent == QLatin1String("TEXT") || ent == QLatin1String("MTEXT"))
		{
			double x = 0, y = 0;
			QString txt;
			QString layerName;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("10"))
					x = value.toDouble();
				else if (code == QLatin1String("20"))
					y = value.toDouble();
				else if (code == QLatin1String("1"))
					txt = value.trimmed();
			}
			if (!txt.isEmpty())
			{
				SheetNote n;
				n.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
				n.anchor = QPointF(x, y);
				n.textPos = QPointF(x + 4, y);
				n.text = txt;
				n.layerId = mapLayer(layerName);
				importedNotes.push_back(n);
			}
		}
		else if (ent == QLatin1String("DIMENSION"))
		{
			double x13 = 0, y13 = 0, x14 = 0, y14 = 0;
			double x10 = 0, y10 = 0;
			bool have13 = false, have14 = false;
			QString layerName;
			QString txt;
			while (readPair())
			{
				if (code == QLatin1String("0"))
					break;
				if (code == QLatin1String("8"))
					layerName = value.trimmed();
				else if (code == QLatin1String("1"))
					txt = value.trimmed();
				else if (code == QLatin1String("10"))
					x10 = value.toDouble();
				else if (code == QLatin1String("20"))
					y10 = value.toDouble();
				else if (code == QLatin1String("13"))
				{
					x13 = value.toDouble();
					have13 = true;
				}
				else if (code == QLatin1String("23"))
					y13 = value.toDouble();
				else if (code == QLatin1String("14"))
				{
					x14 = value.toDouble();
					have14 = true;
				}
				else if (code == QLatin1String("24"))
					y14 = value.toDouble();
			}
			if (have13 && have14)
			{
				SheetDimension d;
				d.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
				d.kind = SheetDimension::Kind::Linear;
				d.p1 = QPointF(x13, y13);
				d.p2 = QPointF(x14, y14);
				d.textOffset = QPointF(x10, y10) - QPointF(0.5 * (x13 + x14), 0.5 * (y13 + y14));
				d.layerId = mapLayer(layerName);
				d.styleId = m_currentDimStyleId;
				if (!txt.isEmpty() && txt != QLatin1String("<>"))
				{
					bool ok = false;
					const double v = txt.toDouble(&ok);
					if (ok)
						d.overrideValue = v;
				}
				importedDims.push_back(d);
			}
		}
	}

	if (lines.isEmpty() && importedNotes.isEmpty() && importedDims.isEmpty() && layerNameToId.isEmpty() &&
		importedBlocks.isEmpty() && importedInserts.isEmpty() &&
		static_cast<int>(m_sketch.document().circles().size()) == circlesBefore &&
		static_cast<int>(m_sketch.document().arcs().size()) == arcsBefore)
		return false;

	if (!lines.isEmpty())
	{
		DrawingView v;
		v.id = QStringLiteral("dxf_%1").arg(m_nextCatalogViewId++);
		v.title = QStringLiteral("DXF 导入");
		v.kind = QStringLiteral("front");
		v.visible = lines;
		v.layerId = m_currentLayerId;
		QRectF box;
		for (const auto& poly : lines)
			for (const QPointF& p : poly.points)
				box = box.isNull() ? QRectF(p, QSizeF(0, 0)) : box.united(QRectF(p, QSizeF(0, 0)));
		v.frame = box.adjusted(-10, -20, 10, 10);
		m_views.push_back(v);
	}
	QHash<QString, QString> blockNameToId;
	for (auto it = importedBlocks.begin(); it != importedBlocks.end(); ++it)
	{
		SheetBlockDef def = it.value();
		blockNameToId.insert(it.key(), def.id);
		m_blockDefs.push_back(def);
	}
	for (SheetBlockRef r : importedInserts)
	{
		r.defId = blockNameToId.value(r.defId, r.defId);
		m_blockRefs.push_back(r);
	}
	for (const SheetNote& n : importedNotes)
		m_notes.push_back(n);
	for (const SheetDimension& d : importedDims)
		m_dims.push_back(d);
	m_needInitialFit = true;
	fitToView();
	emit sheetChanged();
	emit layersChanged();
	emit statusMessage(QStringLiteral("已导入 DXF：%1 折线，%2 文字，%3 尺寸，%4 图层，%5 块")
						   .arg(lines.size())
						   .arg(importedNotes.size())
						   .arg(importedDims.size())
						   .arg(layerNameToId.size())
						   .arg(importedBlocks.size()));
	update();
	return true;
}
