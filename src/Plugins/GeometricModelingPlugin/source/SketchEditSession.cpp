/// @file SketchEditSession.cpp

#include "SketchEditSession.h"
#include "GeomodelingI18n.h"

#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "SketchConstraintSolver.h"

#include <QInputDialog>
#include <QString>
#include <algorithm>
#include <cmath>
#include <unordered_map>


void SketchEditSession::setUseChinese(bool useChinese)
{
	m_useChinese = useChinese;
}

QString SketchEditSession::tr(const QString& en, const QString& zh) const
{
	return gmTr(m_useChinese, en, zh);
}

QString SketchEditSession::statusText() const
{
	QString snap = tr(QStringLiteral("None"), QStringLiteral("无捕捉"));
	switch (m_lastSnap.kind)
	{
	case SkSnapKind::Endpoint:
		snap = tr(QStringLiteral("Endpoint"), QStringLiteral("端点"));
		break;
	case SkSnapKind::Midpoint:
		snap = tr(QStringLiteral("Midpoint"), QStringLiteral("中点"));
		break;
	case SkSnapKind::Center:
		snap = tr(QStringLiteral("Center"), QStringLiteral("圆心"));
		break;
	case SkSnapKind::OnCurve:
		snap = tr(QStringLiteral("On curve"), QStringLiteral("曲线上"));
		break;
	case SkSnapKind::Grid:
		snap = tr(QStringLiteral("Grid"), QStringLiteral("网格"));
		break;
	case SkSnapKind::Horizontal:
		snap = tr(QStringLiteral("Horizontal"), QStringLiteral("水平"));
		break;
	case SkSnapKind::Vertical:
		snap = tr(QStringLiteral("Vertical"), QStringLiteral("竖直"));
		break;
	default:
		break;
	}
	QString tool = tr(QStringLiteral("Line"), QStringLiteral("直线"));
	switch (m_toolKind)
	{
	case SketchToolKind::Arc:
		tool = tr(QStringLiteral("Arc"), QStringLiteral("圆弧"));
		break;
	case SketchToolKind::Circle:
		tool = tr(QStringLiteral("Circle"), QStringLiteral("圆"));
		break;
	case SketchToolKind::Rectangle:
		tool = tr(QStringLiteral("Rectangle"), QStringLiteral("矩形"));
		break;
	case SketchToolKind::Ellipse:
		tool = tr(QStringLiteral("Ellipse"), QStringLiteral("椭圆"));
		break;
	case SketchToolKind::Polygon:
		tool = tr(QStringLiteral("Polygon"), QStringLiteral("多边形"));
		break;
	case SketchToolKind::Slot:
		tool = tr(QStringLiteral("Slot"), QStringLiteral("槽口"));
		break;
	case SketchToolKind::Spline:
		tool = tr(QStringLiteral("Spline"), QStringLiteral("样条"));
		break;
	case SketchToolKind::DimLength:
		tool = tr(QStringLiteral("Length"), QStringLiteral("长度"));
		break;
	case SketchToolKind::DimDistance:
		tool = tr(QStringLiteral("Distance"), QStringLiteral("距离"));
		break;
	case SketchToolKind::DimRadius:
		tool = tr(QStringLiteral("Radius"), QStringLiteral("半径"));
		break;
	case SketchToolKind::DimAngle:
		tool = tr(QStringLiteral("Angle"), QStringLiteral("角度"));
		break;
	case SketchToolKind::DimArcRadius:
		tool = tr(QStringLiteral("Arc Radius"), QStringLiteral("圆弧半径"));
		break;
	case SketchToolKind::ToggleConstruction:
		tool = tr(QStringLiteral("Construction"), QStringLiteral("构造线"));
		break;
	case SketchToolKind::GeomHorizontal:
		tool = tr(QStringLiteral("Horizontal"), QStringLiteral("水平"));
		break;
	case SketchToolKind::GeomVertical:
		tool = tr(QStringLiteral("Vertical"), QStringLiteral("竖直"));
		break;
	case SketchToolKind::GeomCoincident:
		tool = tr(QStringLiteral("Coincident"), QStringLiteral("重合"));
		break;
	case SketchToolKind::GeomParallel:
		tool = tr(QStringLiteral("Parallel"), QStringLiteral("平行"));
		break;
	case SketchToolKind::GeomPerpendicular:
		tool = tr(QStringLiteral("Perpendicular"), QStringLiteral("垂直"));
		break;
	case SketchToolKind::GeomEqualLength:
		tool = tr(QStringLiteral("Equal"), QStringLiteral("等长"));
		break;
	case SketchToolKind::GeomTangent:
		tool = tr(QStringLiteral("Tangent"), QStringLiteral("相切"));
		break;
	case SketchToolKind::GeomSymmetric:
		tool = tr(QStringLiteral("Symmetric"), QStringLiteral("对称"));
		break;
	case SketchToolKind::GeomMidpoint:
		tool = tr(QStringLiteral("Midpoint"), QStringLiteral("中点"));
		break;
	case SketchToolKind::GeomFix:
		tool = tr(QStringLiteral("Fix"), QStringLiteral("固定"));
		break;
	case SketchToolKind::Trim:
		tool = tr(QStringLiteral("Trim"), QStringLiteral("修剪"));
		break;
	case SketchToolKind::Mirror:
		tool = tr(QStringLiteral("Mirror"), QStringLiteral("镜像"));
		break;
	case SketchToolKind::Delete:
		tool = tr(QStringLiteral("Delete"), QStringLiteral("删除"));
		break;
	default:
		break;
	}
	QString s = tr(QStringLiteral("Tool %1 | Snap %2 | %3"), QStringLiteral("工具 %1 | 捕捉 %2 | %3")).arg(tool, snap, dofStatusText());
	if (sketchToolIsPickSession(m_toolKind) && !m_dimHint.isEmpty())
		s += QStringLiteral(" | %1").arg(m_dimHint);
	return s;
}

QString SketchEditSession::dofStatusText() const
{
	QString s = QStringLiteral("DOF %1").arg(m_lastDof);
	if (m_lastConflict)
		s += tr(QStringLiteral(" | Conflict"), QStringLiteral(" | 冲突"));
	else if (m_lastRedundant)
		s += tr(QStringLiteral(" | Redundant"), QStringLiteral(" | 冗余"));
	else if (m_lastDof == 0 && !m_doc.points().empty())
		s += tr(QStringLiteral(" | Fully constrained"), QStringLiteral(" | 完全约束"));
	return s;
}

bool SketchEditSession::begin(IPluginGeometryHost* geo, IPluginDocument* doc, const PluginSketchPlane& plane,
							  QString* err)
{
	return beginWithDocument(geo, doc, plane, {}, err);
}

bool SketchEditSession::beginWithDocument(IPluginGeometryHost* geo, IPluginDocument* doc,
										  const PluginSketchPlane& plane, const QByteArray& sketchJson, QString* err)
{
	end();
	if (!geo || !doc || !plane.isPlanar)
	{
		if (err)
			*err = QStringLiteral("Invalid sketch plane");
		return false;
	}
	m_geo = geo;
	m_docPtr = doc;
	m_plane = plane;
	m_doc.clear();
	if (!sketchJson.isEmpty())
		(void)m_doc.fromJsonUtf8(sketchJson);
	setTool(SketchToolKind::Line);
	QString beginErr;
	if (!geo->beginSketchInput(doc, plane, [this](const PluginSketchInputEvent& ev) { return handleInput(ev); },
							   &beginErr))
	{
		if (err)
			*err = beginErr;
		m_geo = nullptr;
		m_docPtr = nullptr;
		return false;
	}
	m_active = true;
	(void)solveNow(nullptr);
	refreshOverlay();
	return true;
}

void SketchEditSession::end()
{
	if (m_geo && m_docPtr)
	{
		m_geo->endSketchInput(m_docPtr);
		m_geo->clearSketchOverlay(m_docPtr);
	}
	m_active = false;
	m_geo = nullptr;
	m_docPtr = nullptr;
	m_tool.reset();
	m_doc.clear();
	resetPickState();
	m_conflictEntities.clear();
	m_redundantEntities.clear();
	m_onChanged = nullptr;
}

void SketchEditSession::resetPickState()
{
	m_dimPickA = -1;
	m_dimPickB = -1;
	m_dimHoverId = -1;
	m_dimHint.clear();
	m_mirrorTargets.clear();
	m_mirrorPickAxis = true;
	m_dragPointId = -1;
}

QString SketchEditSession::entityDisplayName(int id) const
{
	if (m_doc.findLine(id))
		return tr(QStringLiteral("Line #%1"), QStringLiteral("直线 #%1")).arg(id);
	if (m_doc.findCircle(id))
		return tr(QStringLiteral("Circle #%1"), QStringLiteral("圆 #%1")).arg(id);
	if (m_doc.findArc(id))
		return tr(QStringLiteral("Arc #%1"), QStringLiteral("圆弧 #%1")).arg(id);
	if (m_doc.findSpline(id))
		return tr(QStringLiteral("Spline #%1"), QStringLiteral("样条 #%1")).arg(id);
	if (m_doc.findPoint(id))
		return QStringLiteral("点 #%1").arg(id);
	return QStringLiteral("#%1").arg(id);
}

void SketchEditSession::setMirrorPickingAxis(bool axis)
{
	m_mirrorPickAxis = axis;
	if (axis)
		m_dimHint = QStringLiteral("点选镜像轴（直线）");
	else if (m_dimPickA < 0)
		m_dimHint = QStringLiteral("请先选择镜像轴");
	else
		m_dimHint = QStringLiteral("点选要镜像的图元");
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
}

void SketchEditSession::clearMirrorTargets()
{
	m_mirrorTargets.clear();
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
}

bool SketchEditSession::removeMirrorTarget(int entityId)
{
	const auto it = std::find(m_mirrorTargets.begin(), m_mirrorTargets.end(), entityId);
	if (it == m_mirrorTargets.end())
		return false;
	m_mirrorTargets.erase(it);
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	return true;
}

void SketchEditSession::resetMirrorSelection()
{
	m_dimPickA = -1;
	m_mirrorTargets.clear();
	m_mirrorPickAxis = true;
	m_dimHint = QStringLiteral("先点选镜像轴（直线）");
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
}

bool SketchEditSession::confirmMirror(QString* err)
{
	if (m_toolKind != SketchToolKind::Mirror)
	{
		if (err)
			*err = QStringLiteral("当前不是镜像工具");
		return false;
	}
	if (m_dimPickA < 0)
	{
		if (err)
			*err = QStringLiteral("请先选择镜像轴");
		return false;
	}
	if (m_mirrorTargets.empty())
	{
		if (err)
			*err = QStringLiteral("请先点选要镜像的图元");
		return false;
	}
	if (!m_doc.mirrorEntities(m_dimPickA, m_mirrorTargets))
	{
		if (err)
			*err = QStringLiteral("镜像失败");
		return false;
	}
	m_mirrorTargets.clear();
	m_mirrorPickAxis = false;
	(void)solveNow(nullptr);
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	if (err)
		*err = QStringLiteral("镜像完成");
	return true;
}

int SketchEditSession::hitDimTarget(const SkVec2& uv, double* outMeasure) const
{
	const double tol = m_snapTolMm * 1.5;
	switch (m_toolKind)
	{
	case SketchToolKind::DimLength:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		const SkLine* ln = m_doc.findLine(lid);
		if (!ln)
			return -1;
		if (outMeasure)
		{
			const SkPoint* p1 = m_doc.findPoint(ln->p1);
			const SkPoint* p2 = m_doc.findPoint(ln->p2);
			*outMeasure = (p1 && p2) ? skDist(p1->p, p2->p) : 0.0;
		}
		return lid;
	}
	case SketchToolKind::DimDistance:
	{
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
			return -1;
		if (outMeasure && m_dimPickA >= 0)
		{
			const SkPoint* p1 = m_doc.findPoint(m_dimPickA);
			const SkPoint* p2 = m_doc.findPoint(pid);
			*outMeasure = (p1 && p2) ? skDist(p1->p, p2->p) : 0.0;
		}
		return pid;
	}
	case SketchToolKind::DimRadius:
	{
		const int cid = m_doc.hitTestCircle(uv, tol);
		if (const SkCircle* c = m_doc.findCircle(cid))
		{
			if (outMeasure)
				*outMeasure = c->radius;
			return cid;
		}
		const int eid = m_doc.hitTestEllipse(uv, tol);
		const SkEllipse* e = m_doc.findEllipse(eid);
		if (!e)
			return -1;
		if (outMeasure)
			*outMeasure = e->majorR;
		return eid;
	}
	case SketchToolKind::DimAngle:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		if (lid < 0)
			return -1;
		if (outMeasure && m_dimPickA >= 0)
		{
			const SkLine* l1 = m_doc.findLine(m_dimPickA);
			const SkLine* l2 = m_doc.findLine(lid);
			const SkPoint* a1 = l1 ? m_doc.findPoint(l1->p1) : nullptr;
			const SkPoint* a2 = l1 ? m_doc.findPoint(l1->p2) : nullptr;
			const SkPoint* b1 = l2 ? m_doc.findPoint(l2->p1) : nullptr;
			const SkPoint* b2 = l2 ? m_doc.findPoint(l2->p2) : nullptr;
			if (a1 && a2 && b1 && b2)
			{
				const double ang1 = std::atan2(a2->p.v - a1->p.v, a2->p.u - a1->p.u);
				const double ang2 = std::atan2(b2->p.v - b1->p.v, b2->p.u - b1->p.u);
				double deg = std::abs(ang2 - ang1) * 180.0 / 3.141592653589793;
				if (deg > 180.0)
					deg = 360.0 - deg;
				*outMeasure = deg;
			}
		}
		return lid;
	}
	case SketchToolKind::DimArcRadius:
	{
		const int aid = m_doc.hitTestArc(uv, tol);
		const SkArc* arc = m_doc.findArc(aid);
		if (!arc)
			return -1;
		if (outMeasure)
		{
			const SkPoint* s = m_doc.findPoint(arc->pStart);
			const SkPoint* m = m_doc.findPoint(arc->pMid);
			const SkPoint* e = m_doc.findPoint(arc->pEnd);
			SkVec2 cen;
			double r = 0.0;
			*outMeasure = (s && m && e && sketchCircumcenter(s->p, m->p, e->p, cen, r)) ? r : 0.0;
		}
		return aid;
	}
	default:
		return -1;
	}
}

void SketchEditSession::updateDimHover(const SkVec2& uv)
{
	double measure = 0.0;
	const int hit = hitDimTarget(uv, &measure);
	m_dimHoverId = hit;
	if (hit < 0)
	{
		if (m_dimPickA >= 0)
			m_dimHint = QStringLiteral("已选第一目标，请点选下一个");
		else
		{
			switch (m_toolKind)
			{
			case SketchToolKind::DimLength:
				m_dimHint = QStringLiteral("点选线段…");
				break;
			case SketchToolKind::DimDistance:
				m_dimHint = QStringLiteral("点选端点…");
				break;
			case SketchToolKind::DimRadius:
				m_dimHint = QStringLiteral("点选圆…");
				break;
			case SketchToolKind::DimAngle:
				m_dimHint = QStringLiteral("点选直线…");
				break;
			case SketchToolKind::DimArcRadius:
				m_dimHint = QStringLiteral("点选圆弧…");
				break;
			default:
				m_dimHint.clear();
				break;
			}
		}
		return;
	}
	switch (m_toolKind)
	{
	case SketchToolKind::DimLength:
		m_dimHint = QStringLiteral("命中线段 · 当前长度 %1 mm（单击确认）").arg(measure, 0, 'f', 2);
		break;
	case SketchToolKind::DimDistance:
		if (m_dimPickA < 0)
			m_dimHint = QStringLiteral("命中点（第 1/2）");
		else
			m_dimHint = QStringLiteral("命中点 · 距离 %1 mm（单击确认）").arg(measure, 0, 'f', 2);
		break;
	case SketchToolKind::DimRadius:
		m_dimHint = QStringLiteral("命中圆 · 半径 %1 mm（单击确认）").arg(measure, 0, 'f', 2);
		break;
	case SketchToolKind::DimAngle:
		if (m_dimPickA < 0)
			m_dimHint = QStringLiteral("命中直线（第 1/2）");
		else
			m_dimHint = QStringLiteral("命中直线 · 夹角 %1°（单击确认）").arg(measure, 0, 'f', 1);
		break;
	case SketchToolKind::DimArcRadius:
		m_dimHint = QStringLiteral("命中圆弧 · 半径 %1 mm（单击确认）").arg(measure, 0, 'f', 2);
		break;
	default:
		break;
	}
}

void SketchEditSession::setPolygonSides(int sides)
{
	if (sides < PolygonSketchTool::kMinSides)
		sides = PolygonSketchTool::kMinSides;
	if (sides > PolygonSketchTool::kMaxSides)
		sides = PolygonSketchTool::kMaxSides;
	m_polygonSides = sides;
}

void SketchEditSession::setTool(SketchToolKind kind)
{
	m_toolKind = kind;
	resetPickState();
	if (sketchToolIsPickSession(kind))
	{
		m_tool.reset();
		switch (kind)
		{
		case SketchToolKind::DimLength:
			m_dimHint = tr(QStringLiteral("Click a line for length"), QStringLiteral("点选线段标注长度"));
			break;
		case SketchToolKind::DimDistance:
			m_dimHint = tr(QStringLiteral("Click two points for distance"), QStringLiteral("依次点选两点标注距离"));
			break;
		case SketchToolKind::DimRadius:
			m_dimHint = tr(QStringLiteral("Click a circle for radius"), QStringLiteral("点选圆标注半径"));
			break;
		case SketchToolKind::DimAngle:
			m_dimHint = tr(QStringLiteral("Click two lines for angle"), QStringLiteral("依次点选两直线标注角度"));
			break;
		case SketchToolKind::DimArcRadius:
			m_dimHint = tr(QStringLiteral("Click an arc for radius"), QStringLiteral("点选圆弧标注半径"));
			break;
		case SketchToolKind::ToggleConstruction:
			m_dimHint = tr(QStringLiteral("Click entity to toggle construction"), QStringLiteral("点选图元切换构造线"));
			break;
		case SketchToolKind::GeomHorizontal:
			m_dimHint = tr(QStringLiteral("Click a line, or two points for horizontal"),
						   QStringLiteral("点选直线，或依次点选两点施加水平"));
			break;
		case SketchToolKind::GeomVertical:
			m_dimHint = tr(QStringLiteral("Click a line, or two points for vertical"),
						   QStringLiteral("点选直线，或依次点选两点施加竖直"));
			break;
		case SketchToolKind::GeomCoincident:
			m_dimHint = tr(QStringLiteral("Click two points for coincident"), QStringLiteral("依次点选两点施加重合"));
			break;
		case SketchToolKind::GeomParallel:
			m_dimHint = tr(QStringLiteral("Click two lines for parallel"), QStringLiteral("依次点选两直线施加平行"));
			break;
		case SketchToolKind::GeomPerpendicular:
			m_dimHint = tr(QStringLiteral("Click two lines for perpendicular"), QStringLiteral("依次点选两直线施加垂直"));
			break;
		case SketchToolKind::GeomEqualLength:
			m_dimHint = tr(QStringLiteral("Click two lines for equal length"), QStringLiteral("依次点选两直线施加等长"));
			break;
		case SketchToolKind::GeomFix:
			m_dimHint = tr(QStringLiteral("Click a point to fix"), QStringLiteral("点选点固定位置"));
			break;
		case SketchToolKind::GeomFixOrigin:
			m_dimHint = tr(QStringLiteral("Click a point to fix at sketch origin (0,0)"),
						   QStringLiteral("点选点固定到草图原点 (0,0)"));
			break;
		case SketchToolKind::Trim:
			m_dimHint = tr(QStringLiteral("Click a segment to trim"), QStringLiteral("点选要裁掉的线段段"));
			break;
		case SketchToolKind::Mirror:
			m_dimHint = tr(QStringLiteral("Pick mirror axis, then entities, confirm in panel"), QStringLiteral("先点选镜像轴，再选图元，侧栏确认"));
			m_mirrorPickAxis = true;
			break;
		case SketchToolKind::Delete:
			m_dimHint = tr(QStringLiteral("Click entity to delete, or press Delete"), QStringLiteral("点选图元删除，或按 Delete"));
			break;
		default:
			break;
		}
		refreshOverlay();
		if (m_onChanged)
			m_onChanged();
		return;
	}
	switch (kind)
	{
	case SketchToolKind::Arc:
		m_tool = std::make_unique<ArcSketchTool>();
		break;
	case SketchToolKind::Circle:
		m_tool = std::make_unique<CircleSketchTool>();
		break;
	case SketchToolKind::Rectangle:
		m_tool = std::make_unique<RectSketchTool>();
		break;
	case SketchToolKind::Ellipse:
		m_tool = std::make_unique<EllipseSketchTool>();
		break;
	case SketchToolKind::Polygon:
	{
		auto poly = std::make_unique<PolygonSketchTool>();
		poly->setSides(m_polygonSides);
		m_tool = std::move(poly);
		break;
	}
	case SketchToolKind::Slot:
		m_tool = std::make_unique<SlotSketchTool>();
		break;
	case SketchToolKind::Spline:
		m_tool = std::make_unique<SplineSketchTool>();
		break;
	default:
		m_tool = std::make_unique<LineSketchTool>();
		break;
	}
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
}

void SketchEditSession::applySnap(SkVec2& uv)
{
	const SkVec2* ref = nullptr;
	SkVec2 refStorage;
	if (m_tool)
	{
		if (auto r = m_tool->referencePoint())
		{
			refStorage = *r;
			ref = &refStorage;
		}
	}
	m_lastSnap = sketchSnap(m_doc, uv, m_snapTolMm, m_gridOn ? m_gridMm : 0.0, ref);
	if (m_lastSnap.snapped)
		uv = m_lastSnap.pos;
}

bool SketchEditSession::promptAndAddConstraint(SkConstraintKind kind, int a, int b, double defaultValue, QString* err)
{
	bool ok = false;
	const QString title = m_doc.constraintLabel(SkConstraint{kind, a, b, defaultValue});
	const double v = QInputDialog::getDouble(nullptr, QStringLiteral("尺寸约束"), title, defaultValue, 0.001, 1e9, 3,
											 &ok);
	if (!ok)
	{
		if (err)
			*err = QStringLiteral("已取消尺寸输入");
		return false;
	}
	SkConstraint c;
	c.kind = kind;
	c.a = a;
	c.b = b;
	c.value = v;
	m_doc.addConstraint(c);
	std::string solveErr;
	if (!solveNow(&solveErr))
	{
		if (err)
			*err = QString::fromStdString(solveErr.empty() ? "求解失败" : solveErr);
		if (m_onChanged)
			m_onChanged();
		return false;
	}
	m_dimHoverId = -1;
	m_dimPickA = -1;
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	return true;
}

bool SketchEditSession::tryAddDimensionAt(const SkVec2& uv, QString* err)
{
	const double tol = m_snapTolMm * 1.5;
	switch (m_toolKind)
	{
	case SketchToolKind::DimLength:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		const SkLine* ln = m_doc.findLine(lid);
		if (!ln)
		{
			if (err)
				*err = QStringLiteral("请点选线段");
			return false;
		}
		const SkPoint* p1 = m_doc.findPoint(ln->p1);
		const SkPoint* p2 = m_doc.findPoint(ln->p2);
		if (!p1 || !p2)
			return false;
		return promptAndAddConstraint(SkConstraintKind::Distance, ln->p1, ln->p2, skDist(p1->p, p2->p), err);
	}
	case SketchToolKind::DimDistance:
	{
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选端点");
			return false;
		}
		if (m_dimPickA < 0)
		{
			m_dimPickA = pid;
			if (err)
				*err = QStringLiteral("已选第一点，请再选第二点");
			return false;
		}
		const SkPoint* p1 = m_doc.findPoint(m_dimPickA);
		const SkPoint* p2 = m_doc.findPoint(pid);
		const int a = m_dimPickA;
		m_dimPickA = -1;
		if (!p1 || !p2)
			return false;
		return promptAndAddConstraint(SkConstraintKind::Distance, a, pid, skDist(p1->p, p2->p), err);
	}
	case SketchToolKind::DimRadius:
	{
		const int cid = m_doc.hitTestCircle(uv, tol);
		if (const SkCircle* c = m_doc.findCircle(cid))
			return promptAndAddConstraint(SkConstraintKind::Radius, cid, -1, c->radius, err);
		const int eid = m_doc.hitTestEllipse(uv, tol);
		const SkEllipse* e = m_doc.findEllipse(eid);
		if (!e)
		{
			if (err)
				*err = QStringLiteral("请点选圆或椭圆");
			return false;
		}
		return promptAndAddConstraint(SkConstraintKind::MajorRadius, eid, -1, e->majorR, err);
	}
	case SketchToolKind::DimAngle:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		if (lid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选直线");
			return false;
		}
		if (m_dimPickA < 0)
		{
			m_dimPickA = lid;
			if (err)
				*err = QStringLiteral("已选第一条线，请再选第二条");
			return false;
		}
		const SkLine* l1 = m_doc.findLine(m_dimPickA);
		const SkLine* l2 = m_doc.findLine(lid);
		const int a = m_dimPickA;
		m_dimPickA = -1;
		if (!l1 || !l2)
			return false;
		const SkPoint* a1 = m_doc.findPoint(l1->p1);
		const SkPoint* a2 = m_doc.findPoint(l1->p2);
		const SkPoint* b1 = m_doc.findPoint(l2->p1);
		const SkPoint* b2 = m_doc.findPoint(l2->p2);
		if (!a1 || !a2 || !b1 || !b2)
			return false;
		const double ang1 = std::atan2(a2->p.v - a1->p.v, a2->p.u - a1->p.u);
		const double ang2 = std::atan2(b2->p.v - b1->p.v, b2->p.u - b1->p.u);
		double deg = std::abs(ang2 - ang1) * 180.0 / 3.141592653589793;
		if (deg > 180.0)
			deg = 360.0 - deg;
		return promptAndAddConstraint(SkConstraintKind::Angle, a, lid, deg, err);
	}
	case SketchToolKind::DimArcRadius:
	{
		const int aid = m_doc.hitTestArc(uv, tol);
		const SkArc* arc = m_doc.findArc(aid);
		if (!arc)
		{
			if (err)
				*err = QStringLiteral("请点选圆弧");
			return false;
		}
		const SkPoint* s = m_doc.findPoint(arc->pStart);
		const SkPoint* m = m_doc.findPoint(arc->pMid);
		const SkPoint* e = m_doc.findPoint(arc->pEnd);
		SkVec2 cen;
		double r = 0.0;
		if (!s || !m || !e || !sketchCircumcenter(s->p, m->p, e->p, cen, r))
		{
			if (err)
				*err = QStringLiteral("无法计算圆弧半径");
			return false;
		}
		return promptAndAddConstraint(SkConstraintKind::ArcRadius, aid, -1, r, err);
	}
	default:
		return false;
	}
}

bool SketchEditSession::addGeomConstraintNoPrompt(SkConstraintKind kind, int a, int b, QString* err)
{
	SkConstraint c;
	c.kind = kind;
	c.a = a;
	c.b = b;
	c.value = 0.0;
	m_doc.addConstraint(c);
	std::string solveErr;
	if (!solveNow(&solveErr))
	{
		if (err)
			*err = QString::fromStdString(solveErr.empty() ? "求解失败" : solveErr);
		if (m_onChanged)
			m_onChanged();
		return false;
	}
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	return true;
}

int SketchEditSession::hitAnyCurve(const SkVec2& uv) const
{
	const double tol = m_snapTolMm * 1.5;
	if (const int lid = m_doc.hitTestLine(uv, tol); lid >= 0)
		return lid;
	if (const int cid = m_doc.hitTestCircle(uv, tol); cid >= 0)
		return cid;
	if (const int eid = m_doc.hitTestEllipse(uv, tol); eid >= 0)
		return eid;
	if (const int aid = m_doc.hitTestArc(uv, tol); aid >= 0)
		return aid;
	if (const int sid = m_doc.hitTestSpline(uv, tol); sid >= 0)
		return sid;
	return -1;
}

bool SketchEditSession::deleteEntityAt(const SkVec2& uv, QString* err)
{
	const int id = hitAnyCurve(uv);
	if (id < 0)
	{
		if (err)
			*err = QStringLiteral("请点选要删除的线/弧/圆");
		return false;
	}
	if (!m_doc.removeEntity(id))
	{
		if (err)
			*err = QStringLiteral("删除失败");
		return false;
	}
	m_mirrorTargets.erase(std::remove(m_mirrorTargets.begin(), m_mirrorTargets.end(), id), m_mirrorTargets.end());
	if (m_dimPickA == id)
		m_dimPickA = -1;
	m_dimHoverId = -1;
	(void)solveNow(nullptr);
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	if (err)
		*err = QStringLiteral("已删除");
	return true;
}

void SketchEditSession::updatePickHover(const SkVec2& uv)
{
	if (sketchToolIsDimension(m_toolKind))
	{
		updateDimHover(uv);
		return;
	}
	const double tol = m_snapTolMm * 1.5;
	m_dimHoverId = -1;
	switch (m_toolKind)
	{
	case SketchToolKind::ToggleConstruction:
	case SketchToolKind::Trim:
	case SketchToolKind::Delete:
		m_dimHoverId = hitAnyCurve(uv);
		m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中图元（单击）") : QStringLiteral("点选图元…");
		break;
	case SketchToolKind::GeomHorizontal:
	case SketchToolKind::GeomVertical:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		const int pid = m_doc.hitTestPoint(uv, tol);
		m_dimHoverId = lid >= 0 ? lid : pid;
		if (m_dimPickA >= 0)
			m_dimHint = pid >= 0 ? QStringLiteral("命中点（第 2/2）") : QStringLiteral("已选第一点，请再选一点（或点直线）");
		else if (lid >= 0)
			m_dimHint = QStringLiteral("命中直线（单击）");
		else if (pid >= 0)
			m_dimHint = QStringLiteral("命中点（两点约束第 1/2）");
		else
			m_dimHint = QStringLiteral("点选直线，或依次点选两点…");
		break;
	}
	case SketchToolKind::GeomParallel:
	case SketchToolKind::GeomPerpendicular:
	case SketchToolKind::GeomEqualLength:
		m_dimHoverId = m_doc.hitTestLine(uv, tol);
		if (m_dimPickA >= 0)
			m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中直线（第 2/2）") : QStringLiteral("已选第一条，请再选一条");
		else
			m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中直线（单击）") : QStringLiteral("点选直线…");
		break;
	case SketchToolKind::GeomCoincident:
	case SketchToolKind::GeomFix:
	case SketchToolKind::GeomFixOrigin:
		m_dimHoverId = m_doc.hitTestPoint(uv, tol);
		if (m_toolKind == SketchToolKind::GeomCoincident && m_dimPickA >= 0)
			m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中点（第 2/2）") : QStringLiteral("已选第一点，请再选一点");
		else
			m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中点（单击）") : QStringLiteral("点选点…");
		break;
	case SketchToolKind::Mirror:
		if (m_mirrorPickAxis || m_dimPickA < 0)
		{
			m_dimHoverId = m_doc.hitTestLine(uv, tol);
			m_dimHint = m_dimHoverId >= 0 ? QStringLiteral("命中镜像轴（单击）") : QStringLiteral("点选镜像轴（直线）");
		}
		else
		{
			m_dimHoverId = hitAnyCurve(uv);
			m_dimHint = QStringLiteral("已选 %1 个图元；点选继续，侧栏确认")
							.arg(static_cast<int>(m_mirrorTargets.size()));
		}
		break;
	default:
		m_dimHint.clear();
		break;
	}
}

bool SketchEditSession::tryPickSessionAt(const SkVec2& uv, bool rightButton, QString* err)
{
	if (sketchToolIsDimension(m_toolKind))
		return tryAddDimensionAt(uv, err);

	const double tol = m_snapTolMm * 1.5;
	if (m_toolKind == SketchToolKind::Mirror && rightButton)
		return confirmMirror(err);

	switch (m_toolKind)
	{
	case SketchToolKind::ToggleConstruction:
	{
		const int id = hitAnyCurve(uv);
		if (id < 0)
		{
			if (err)
				*err = QStringLiteral("请点选线/弧/圆/样条");
			return false;
		}
		if (!m_doc.toggleConstruction(id))
			return false;
		refreshOverlay();
		if (m_onChanged)
			m_onChanged();
		if (err)
			*err = QStringLiteral("已切换构造线");
		return true;
	}
	case SketchToolKind::Delete:
		return deleteEntityAt(uv, err);
	case SketchToolKind::Trim:
	{
		if (!m_doc.trimLineAt(uv, tol) && !m_doc.trimSplineAt(uv, tol))
		{
			if (err)
				*err = QStringLiteral("无法裁剪（直线需相交；样条点选曲线劈开）");
			return false;
		}
		(void)solveNow(nullptr);
		refreshOverlay();
		if (m_onChanged)
			m_onChanged();
		if (err)
			*err = QStringLiteral("已裁剪");
		return true;
	}
	case SketchToolKind::GeomHorizontal:
	case SketchToolKind::GeomVertical:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		if (lid >= 0)
		{
			m_dimPickA = -1;
			const auto kind =
				m_toolKind == SketchToolKind::GeomHorizontal ? SkConstraintKind::Horizontal : SkConstraintKind::Vertical;
			if (!addGeomConstraintNoPrompt(kind, lid, -1, err))
				return false;
			if (err)
				*err = QStringLiteral("已施加约束");
			return true;
		}
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选直线，或样条/草图点");
			return false;
		}
		if (m_dimPickA < 0)
		{
			m_dimPickA = pid;
			if (err)
				*err = QStringLiteral("已选第一点，请再选第二点");
			refreshOverlay();
			return false;
		}
		const int a = m_dimPickA;
		m_dimPickA = -1;
		if (a == pid)
		{
			if (err)
				*err = QStringLiteral("请点选不同的两点");
			return false;
		}
		const auto kind =
			m_toolKind == SketchToolKind::GeomHorizontal ? SkConstraintKind::Horizontal : SkConstraintKind::Vertical;
		if (!addGeomConstraintNoPrompt(kind, a, pid, err))
			return false;
		if (err)
			*err = QStringLiteral("已施加两点约束");
		return true;
	}
	case SketchToolKind::GeomFix:
	{
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选点");
			return false;
		}
		SkPoint* pt = m_doc.findPoint(pid);
		if (!pt)
			return false;
		pt->fixed = true;
		(void)solveNow(nullptr);
		refreshOverlay();
		if (m_onChanged)
			m_onChanged();
		if (err)
			*err = QStringLiteral("点已固定");
		return true;
	}
	case SketchToolKind::GeomFixOrigin:
	{
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选点");
			return false;
		}
		return fixPointToOrigin(pid, err);
	}
	case SketchToolKind::GeomCoincident:
	{
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (pid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选点");
			return false;
		}
		if (m_dimPickA < 0)
		{
			m_dimPickA = pid;
			if (err)
				*err = QStringLiteral("已选第一点，请再选第二点");
			return false;
		}
		const int a = m_dimPickA;
		m_dimPickA = -1;
		if (!addGeomConstraintNoPrompt(SkConstraintKind::Coincident, a, pid, err))
			return false;
		if (err)
			*err = QStringLiteral("已施加重合");
		return true;
	}
	case SketchToolKind::GeomParallel:
	case SketchToolKind::GeomPerpendicular:
	case SketchToolKind::GeomEqualLength:
	case SketchToolKind::GeomTangent:
	case SketchToolKind::GeomSymmetric:
	case SketchToolKind::GeomMidpoint:
	{
		const int lid = m_doc.hitTestLine(uv, tol);
		const int cid = m_doc.hitTestCircle(uv, tol);
		const int aid = m_doc.hitTestArc(uv, tol);
		const int pid = m_doc.hitTestPoint(uv, tol);
		if (m_toolKind == SketchToolKind::GeomTangent)
		{
			if (lid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选直线");
				return false;
			}
			if (m_dimPickA < 0)
			{
				m_dimPickA = lid;
				if (err)
					*err = QStringLiteral("已选直线，请再选圆或弧");
				return false;
			}
			const int curveId = cid >= 0 ? cid : aid;
			if (curveId < 0)
			{
				if (err)
					*err = QStringLiteral("请点选圆或弧");
				return false;
			}
			SkConstraint c;
			c.kind = SkConstraintKind::Tangent;
			c.a = m_dimPickA;
			c.b = curveId;
			m_doc.addConstraint(c);
			m_dimPickA = -1;
			(void)solveNow();
			refreshOverlay();
			if (m_onChanged)
				m_onChanged();
			return true;
		}
		if (m_toolKind == SketchToolKind::GeomMidpoint)
		{
			if (pid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选点");
				return false;
			}
			if (m_dimPickA < 0)
			{
				m_dimPickA = pid;
				if (err)
					*err = QStringLiteral("已选点，请再选直线");
				return false;
			}
			if (lid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选直线");
				return false;
			}
			SkConstraint c;
			c.kind = SkConstraintKind::Midpoint;
			c.a = m_dimPickA;
			c.b = lid;
			m_doc.addConstraint(c);
			m_dimPickA = -1;
			(void)solveNow();
			refreshOverlay();
			if (m_onChanged)
				m_onChanged();
			return true;
		}
		if (m_toolKind == SketchToolKind::GeomSymmetric)
		{
			if (pid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选点");
				return false;
			}
			if (m_dimPickA < 0)
			{
				m_dimPickA = pid;
				if (err)
					*err = QStringLiteral("已选第一点，请再选第二点");
				return false;
			}
			if (m_dimPickB < 0)
			{
				if (pid == m_dimPickA)
				{
					if (err)
						*err = QStringLiteral("请选择不同的点");
					return false;
				}
				m_dimPickB = pid;
				if (err)
					*err = QStringLiteral("已选两点，请点选对称轴直线");
				return false;
			}
			if (lid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选对称轴直线");
				return false;
			}
			SkConstraint c;
			c.kind = SkConstraintKind::Symmetric;
			c.a = m_dimPickA;
			c.b = m_dimPickB;
			c.c = lid;
			m_doc.addConstraint(c);
			m_dimPickA = -1;
			m_dimPickB = -1;
			(void)solveNow();
			refreshOverlay();
			if (m_onChanged)
				m_onChanged();
			return true;
		}
		if (lid < 0)
		{
			if (err)
				*err = QStringLiteral("请点选直线");
			return false;
		}
		if (m_dimPickA < 0)
		{
			m_dimPickA = lid;
			if (err)
				*err = QStringLiteral("已选第一条线，请再选第二条");
			return false;
		}
		const int a = m_dimPickA;
		m_dimPickA = -1;
		SkConstraintKind kind = SkConstraintKind::Parallel;
		if (m_toolKind == SketchToolKind::GeomPerpendicular)
			kind = SkConstraintKind::Perpendicular;
		else if (m_toolKind == SketchToolKind::GeomEqualLength)
			kind = SkConstraintKind::EqualLength;
		if (!addGeomConstraintNoPrompt(kind, a, lid, err))
			return false;
		if (err)
			*err = QStringLiteral("已施加约束");
		return true;
	}
	case SketchToolKind::Mirror:
	{
		if (m_mirrorPickAxis || m_dimPickA < 0)
		{
			const int lid = m_doc.hitTestLine(uv, tol);
			if (lid < 0)
			{
				if (err)
					*err = QStringLiteral("请点选镜像轴直线");
				return false;
			}
			m_dimPickA = lid;
			m_mirrorPickAxis = false;
			if (err)
				*err = QStringLiteral("已选轴，请点选要镜像的图元");
			return false;
		}
		const int id = hitAnyCurve(uv);
		if (id < 0 || id == m_dimPickA)
		{
			if (err)
				*err = QStringLiteral("请点选要镜像的线/弧/圆");
			return false;
		}
		if (std::find(m_mirrorTargets.begin(), m_mirrorTargets.end(), id) == m_mirrorTargets.end())
			m_mirrorTargets.push_back(id);
		if (err)
			*err = QStringLiteral("已选 %1 个图元").arg(static_cast<int>(m_mirrorTargets.size()));
		return false;
	}
	default:
		return false;
	}
}

bool SketchEditSession::fixPointToOrigin(int pointId, QString* err)
{
	if (!m_active)
	{
		if (err)
			*err = tr(QStringLiteral("No active sketch."), QStringLiteral("当前没有活动草图"));
		return false;
	}
	SkPoint* pt = m_doc.findPoint(pointId);
	if (!pt)
	{
		if (err)
			*err = tr(QStringLiteral("Point not found."), QStringLiteral("点不存在"));
		return false;
	}
	pt->p = {0.0, 0.0};
	pt->fixed = true;
	(void)solveNow(nullptr);
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	if (err)
		*err = tr(QStringLiteral("Point fixed at sketch origin."), QStringLiteral("点已固定到草图原点"));
	return true;
}

bool SketchEditSession::beginPointDrag(const SkVec2& uv)
{
	const double tol = m_snapTolMm * 1.5;
	const int pid = m_doc.hitTestPoint(uv, tol);
	if (pid < 0 || !m_doc.isSplineThroughPoint(pid))
		return false;
	const SkPoint* pt = m_doc.findPoint(pid);
	if (!pt || pt->fixed)
		return false;
	m_dragPointId = pid;
	return true;
}

void SketchEditSession::updatePointDrag(const SkVec2& uv)
{
	SkPoint* pt = m_doc.findPoint(m_dragPointId);
	if (!pt || pt->fixed)
		return;
	pt->p = uv;
	refreshOverlay();
}

void SketchEditSession::endPointDrag()
{
	if (m_dragPointId < 0)
		return;
	m_dragPointId = -1;
	(void)solveNow(nullptr);
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
}

bool SketchEditSession::handleInput(const PluginSketchInputEvent& ev)
{
	if (!m_active)
		return false;

	if (ev.kind == PluginSketchInputKind::KeyPress)
	{
		if (ev.buttonOrKey == 0x01000000)
		{
			if (m_dragPointId >= 0)
			{
				m_dragPointId = -1;
				refreshOverlay();
				return true;
			}
			if (m_tool)
				m_tool->cancel();
			resetPickState();
			refreshOverlay();
			return true;
		}
		// Delete / Backspace：删光标下图元
		if (ev.buttonOrKey == 0x01000007 || ev.buttonOrKey == 0x01000003)
		{
			if (!m_hasCursorUv)
				return true;
			QString delErr;
			(void)deleteEntityAt(m_lastCursorUv, &delErr);
			return true;
		}
		return false;
	}

	if (!ev.hasWorldHit)
		return ev.kind == PluginSketchInputKind::MousePress;

	SkVec2 uv = m_doc.worldToUv(m_plane, ev.worldMm);
	applySnap(uv);
	m_lastCursorUv = uv;
	m_hasCursorUv = true;

	if (ev.kind == PluginSketchInputKind::MousePress && ev.buttonOrKey == 1)
	{
		const int hit = hitAnyCurve(uv);
		if (hit >= 0)
			m_selectedEntityId = hit;
	}

	if (m_dragPointId >= 0)
	{
		if (ev.kind == PluginSketchInputKind::MouseMove)
		{
			updatePointDrag(uv);
			return true;
		}
		if (ev.kind == PluginSketchInputKind::MouseRelease ||
			(ev.kind == PluginSketchInputKind::MousePress && ev.buttonOrKey == 2))
		{
			endPointDrag();
			return true;
		}
		if (ev.kind == PluginSketchInputKind::MousePress && ev.buttonOrKey == 1)
		{
			updatePointDrag(uv);
			endPointDrag();
			return true;
		}
	}

	if (sketchToolIsPickSession(m_toolKind))
	{
		if (ev.kind == PluginSketchInputKind::MouseMove)
		{
			updatePickHover(uv);
			refreshOverlay();
			return false;
		}
		if (ev.kind == PluginSketchInputKind::MousePress && (ev.buttonOrKey == 1 || ev.buttonOrKey == 2))
		{
			QString pickErr;
			(void)tryPickSessionAt(uv, ev.buttonOrKey == 2, &pickErr);
			if (!pickErr.isEmpty())
				m_dimHint = pickErr;
			updatePickHover(uv);
			refreshOverlay();
			if (m_onChanged)
				m_onChanged();
			return true;
		}
		return ev.kind == PluginSketchInputKind::MousePress;
	}

	if (!m_tool)
		return false;

	if (ev.kind == PluginSketchInputKind::MouseMove)
	{
		m_tool->onMove(uv);
		refreshOverlay();
		return false;
	}
	if (ev.kind == PluginSketchInputKind::MousePress)
	{
		const bool right = (ev.buttonOrKey == 2);
		if (!right && beginPointDrag(uv))
		{
			refreshOverlay();
			return true;
		}
		m_tool->onPress(uv, right, m_doc);
		(void)solveNow(nullptr);
		refreshOverlay();
		return true;
	}
	if (ev.kind == PluginSketchInputKind::MouseRelease && m_dragPointId >= 0)
	{
		endPointDrag();
		return true;
	}
	return false;
}

void SketchEditSession::syncConstraintsToSolver(SketchConstraintSolver& solver,
												std::unordered_map<int, int>& pointIdToIdx,
												std::unordered_map<int, int>& lineIdToIdx,
												std::unordered_map<int, int>& arcIdToIdx,
												std::unordered_map<int, int>& constraintTagToDocIndex)
{
	std::unordered_map<int, int> circleIdToIdx;
	pointIdToIdx.clear();
	lineIdToIdx.clear();
	arcIdToIdx.clear();
	constraintTagToDocIndex.clear();
	for (const auto& p : m_doc.points())
		pointIdToIdx[p.id] = solver.addPoint(p.p.u, p.p.v, p.fixed);
	for (const auto& ln : m_doc.lines())
	{
		const auto i1 = pointIdToIdx.find(ln.p1);
		const auto i2 = pointIdToIdx.find(ln.p2);
		if (i1 == pointIdToIdx.end() || i2 == pointIdToIdx.end())
			continue;
		lineIdToIdx[ln.id] = solver.addLine(i1->second, i2->second);
	}
	for (const auto& arc : m_doc.arcs())
	{
		const SkPoint* s = m_doc.findPoint(arc.pStart);
		const SkPoint* m = m_doc.findPoint(arc.pMid);
		const SkPoint* e = m_doc.findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		SkVec2 cen;
		double r = 0.0;
		if (!sketchCircumcenter(s->p, m->p, e->p, cen, r))
			continue;
		const int ci = solver.addPoint(cen.u, cen.v, false);
		arcIdToIdx[arc.id] = solver.addArc(ci, pointIdToIdx[arc.pStart], pointIdToIdx[arc.pEnd], r);
	}
	for (const auto& c : m_doc.circles())
	{
		const auto ic = pointIdToIdx.find(c.center);
		if (ic == pointIdToIdx.end())
			continue;
		circleIdToIdx[c.id] = solver.addCircle(ic->second, c.radius);
	}
	std::unordered_map<int, int> ellipseIdToIdx;
	for (const auto& e : m_doc.ellipses())
	{
		const auto ic = pointIdToIdx.find(e.center);
		if (ic == pointIdToIdx.end())
			continue;
		ellipseIdToIdx[e.id] = solver.addEllipse(ic->second, e.majorR, e.minorR, e.angleRad);
	}
	(void)ellipseIdToIdx;
	for (std::size_t ci = 0; ci < m_doc.constraints().size(); ++ci)
	{
		const auto& c = m_doc.constraints()[ci];
		const int tag = static_cast<int>(ci) + 1;
		constraintTagToDocIndex[tag] = static_cast<int>(ci);
		SketchConstraint2d sc;
		sc.value = c.value;
		sc.tagId = tag;
		switch (c.kind)
		{
		case SkConstraintKind::Coincident:
			sc.kind = SketchConstraintKind::Coincident;
			sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
			sc.b = pointIdToIdx.count(c.b) ? pointIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::Horizontal:
			sc.kind = SketchConstraintKind::Horizontal;
			if (c.b >= 0)
			{
				sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
				sc.b = pointIdToIdx.count(c.b) ? pointIdToIdx[c.b] : -1;
			}
			else
			{
				sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
				sc.b = -1;
			}
			break;
		case SkConstraintKind::Vertical:
			sc.kind = SketchConstraintKind::Vertical;
			if (c.b >= 0)
			{
				sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
				sc.b = pointIdToIdx.count(c.b) ? pointIdToIdx[c.b] : -1;
			}
			else
			{
				sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
				sc.b = -1;
			}
			break;
		case SkConstraintKind::EqualLength:
			sc.kind = SketchConstraintKind::EqualLength;
			sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
			sc.b = lineIdToIdx.count(c.b) ? lineIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::Distance:
			sc.kind = SketchConstraintKind::Distance;
			sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
			sc.b = pointIdToIdx.count(c.b) ? pointIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::Parallel:
			sc.kind = SketchConstraintKind::Parallel;
			sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
			sc.b = lineIdToIdx.count(c.b) ? lineIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::Perpendicular:
			sc.kind = SketchConstraintKind::Perpendicular;
			sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
			sc.b = lineIdToIdx.count(c.b) ? lineIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::Angle:
			sc.kind = SketchConstraintKind::Angle;
			sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
			sc.b = lineIdToIdx.count(c.b) ? lineIdToIdx[c.b] : -1;
			break;
		case SkConstraintKind::ArcRadius:
			sc.kind = SketchConstraintKind::ArcRadius;
			sc.a = arcIdToIdx.count(c.a) ? arcIdToIdx[c.a] : -1;
			break;
		case SkConstraintKind::Radius:
		{
			const SkCircle* cir = m_doc.findCircle(c.a);
			if (!cir)
				continue;
			const auto ic = pointIdToIdx.find(cir->center);
			if (ic == pointIdToIdx.end())
				continue;
			const SkPoint* cen = m_doc.findPoint(cir->center);
			if (!cen)
				continue;
			const int rim = solver.addPoint(cen->p.u + c.value, cen->p.v, false);
			sc.kind = SketchConstraintKind::Distance;
			sc.a = ic->second;
			sc.b = rim;
			sc.value = c.value;
			break;
		}
		case SkConstraintKind::MajorRadius:
		case SkConstraintKind::MinorRadius:
		{
			const SkEllipse* el = m_doc.findEllipse(c.a);
			if (!el)
				continue;
			const auto ic = pointIdToIdx.find(el->center);
			if (ic == pointIdToIdx.end())
				continue;
			const SkPoint* cen = m_doc.findPoint(el->center);
			if (!cen)
				continue;
			const double ang =
				(c.kind == SkConstraintKind::MajorRadius) ? el->angleRad : (el->angleRad + 1.5707963267948966);
			const int rim =
				solver.addPoint(cen->p.u + c.value * std::cos(ang), cen->p.v + c.value * std::sin(ang), false);
			sc.kind = SketchConstraintKind::Distance;
			sc.a = ic->second;
			sc.b = rim;
			sc.value = c.value;
			break;
		}
		case SkConstraintKind::Tangent:
		{
			sc.kind = SketchConstraintKind::Tangent;
			sc.a = lineIdToIdx.count(c.a) ? lineIdToIdx[c.a] : -1;
			if (arcIdToIdx.count(c.b))
				sc.b = arcIdToIdx[c.b];
			else if (circleIdToIdx.count(c.b))
				sc.b = circleIdToIdx[c.b];
			else
				sc.b = -1;
			break;
		}
		case SkConstraintKind::Symmetric:
		{
			sc.kind = SketchConstraintKind::Symmetric;
			sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
			sc.b = pointIdToIdx.count(c.b) ? pointIdToIdx[c.b] : -1;
			sc.c = lineIdToIdx.count(c.c) ? lineIdToIdx[c.c] : -1;
			break;
		}
		case SkConstraintKind::Midpoint:
		{
			sc.kind = SketchConstraintKind::Midpoint;
			sc.a = pointIdToIdx.count(c.a) ? pointIdToIdx[c.a] : -1;
			sc.b = lineIdToIdx.count(c.b) ? lineIdToIdx[c.b] : -1;
			break;
		}
		}
		if (sc.a < 0)
			continue;
		if ((sc.kind == SketchConstraintKind::Distance || sc.kind == SketchConstraintKind::Coincident ||
			 sc.kind == SketchConstraintKind::Angle || sc.kind == SketchConstraintKind::Parallel ||
			 sc.kind == SketchConstraintKind::Perpendicular || sc.kind == SketchConstraintKind::EqualLength) &&
			sc.b < 0)
			continue;
		solver.addConstraint(sc);
	}
}

void SketchEditSession::rebuildDiagEntitySets()
{
	m_conflictEntities.clear();
	m_redundantEntities.clear();
}

bool SketchEditSession::solveNow(std::string* err)
{
	m_lastConflict = false;
	m_lastRedundant = false;
	m_conflictEntities.clear();
	m_redundantEntities.clear();
	if (m_doc.points().empty())
	{
		m_lastDof = 0;
		return true;
	}
	SketchConstraintSolver solver;
	std::unordered_map<int, int> pmap, lmap, amap, tagMap;
	syncConstraintsToSolver(solver, pmap, lmap, amap, tagMap);
	std::string localErr;
	const int rc = solver.solve(&localErr);
	m_lastDof = solver.dof();
	m_lastConflict = solver.hasConflicting();
	m_lastRedundant = solver.hasRedundant();

	auto markConstraintEntities = [&](int tag, bool conflict)
	{
		const auto it = tagMap.find(tag);
		if (it == tagMap.end())
			return;
		const int idx = it->second;
		if (idx < 0 || idx >= static_cast<int>(m_doc.constraints().size()))
			return;
		const auto& c = m_doc.constraints()[static_cast<std::size_t>(idx)];
		auto& set = conflict ? m_conflictEntities : m_redundantEntities;
		auto addEnt = [&](int id)
		{
			if (id >= 0)
				set.insert(id);
		};
		switch (c.kind)
		{
		case SkConstraintKind::Distance:
			addEnt(c.a);
			addEnt(c.b);
			for (const auto& ln : m_doc.lines())
				if ((ln.p1 == c.a && ln.p2 == c.b) || (ln.p1 == c.b && ln.p2 == c.a))
					addEnt(ln.id);
			break;
		case SkConstraintKind::Angle:
		case SkConstraintKind::Horizontal:
		case SkConstraintKind::Vertical:
		case SkConstraintKind::EqualLength:
		case SkConstraintKind::Parallel:
		case SkConstraintKind::Perpendicular:
			addEnt(c.a);
			addEnt(c.b);
			break;
		case SkConstraintKind::Radius:
		case SkConstraintKind::ArcRadius:
		case SkConstraintKind::MajorRadius:
		case SkConstraintKind::MinorRadius:
			addEnt(c.a);
			break;
		default:
			addEnt(c.a);
			addEnt(c.b);
			break;
		}
	};
	for (int t : solver.conflictingTags())
		markConstraintEntities(t, true);
	for (int t : solver.redundantTags())
	{
		// 尺寸冗余只影响标注诊断，不改图元颜色（避免标注后线段一直橘黄）
		const auto it = tagMap.find(t);
		if (it == tagMap.end())
			continue;
		const int idx = it->second;
		if (idx < 0 || idx >= static_cast<int>(m_doc.constraints().size()))
			continue;
		const auto kind = m_doc.constraints()[static_cast<std::size_t>(idx)].kind;
		if (kind == SkConstraintKind::Distance || kind == SkConstraintKind::Radius ||
			kind == SkConstraintKind::Angle || kind == SkConstraintKind::ArcRadius ||
			kind == SkConstraintKind::MajorRadius || kind == SkConstraintKind::MinorRadius)
			continue;
		markConstraintEntities(t, false);
	}

	if (rc != 0)
	{
		if (err)
			*err = localErr;
		for (const auto& ln : m_doc.lines())
			m_conflictEntities.insert(ln.id);
		return false;
	}
	const auto& pts = solver.points();
	for (auto& pt : m_doc.pointsMut())
	{
		const auto it = pmap.find(pt.id);
		if (it == pmap.end())
			continue;
		const auto& sp = pts[static_cast<std::size_t>(it->second)];
		pt.p.u = sp.x;
		pt.p.v = sp.y;
	}
	for (auto& c : m_doc.circlesMut())
	{
		for (const auto& cons : m_doc.constraints())
		{
			if (cons.kind == SkConstraintKind::Radius && cons.a == c.id)
				c.radius = cons.value;
		}
	}
	for (auto& e : m_doc.ellipsesMut())
	{
		bool hasMaj = false, hasMin = false;
		for (const auto& cons : m_doc.constraints())
		{
			if (cons.a != e.id)
				continue;
			if (cons.kind == SkConstraintKind::MajorRadius)
			{
				e.majorR = cons.value;
				hasMaj = true;
			}
			if (cons.kind == SkConstraintKind::MinorRadius)
			{
				e.minorR = cons.value;
				hasMin = true;
			}
		}
		(void)hasMaj;
		(void)hasMin;
	}
	{
		const auto& epts = solver.ellipses();
		auto& ells = m_doc.ellipsesMut();
		for (std::size_t i = 0; i < ells.size() && i < epts.size(); ++i)
			ells[i].angleRad = epts[i].angleRad;
	}
	return true;
}

bool SketchEditSession::readNamedParams(int entityId, std::vector<std::pair<QString, double>>& out) const
{
	out.clear();
	if (const SkCircle* c = m_doc.findCircle(entityId))
	{
		const SkPoint* p = m_doc.findPoint(c->center);
		if (p)
		{
			out.emplace_back(QStringLiteral("centerX"), p->p.u);
			out.emplace_back(QStringLiteral("centerY"), p->p.v);
		}
		out.emplace_back(QStringLiteral("radius"), c->radius);
		return true;
	}
	if (const SkLine* ln = m_doc.findLine(entityId))
	{
		const SkPoint* p1 = m_doc.findPoint(ln->p1);
		const SkPoint* p2 = m_doc.findPoint(ln->p2);
		if (p1 && p2)
		{
			out.emplace_back(QStringLiteral("startX"), p1->p.u);
			out.emplace_back(QStringLiteral("startY"), p1->p.v);
			out.emplace_back(QStringLiteral("endX"), p2->p.u);
			out.emplace_back(QStringLiteral("endY"), p2->p.v);
			out.emplace_back(QStringLiteral("length"), skDist(p1->p, p2->p));
		}
		return p1 && p2;
	}
	if (const SkEllipse* e = m_doc.findEllipse(entityId))
	{
		const SkPoint* p = m_doc.findPoint(e->center);
		if (p)
		{
			out.emplace_back(QStringLiteral("centerX"), p->p.u);
			out.emplace_back(QStringLiteral("centerY"), p->p.v);
		}
		out.emplace_back(QStringLiteral("majorRadius"), e->majorR);
		out.emplace_back(QStringLiteral("minorRadius"), e->minorR);
		out.emplace_back(QStringLiteral("majorAxisAngle"), e->angleRad * 180.0 / 3.141592653589793);
		return true;
	}
	if (const SkSpline* sp = m_doc.findSpline(entityId))
	{
		out.emplace_back(QStringLiteral("mode"), static_cast<double>(sp->mode));
		out.emplace_back(QStringLiteral("degreeHint"), static_cast<double>(sp->throughPts.size()));
		return true;
	}
	return false;
}

bool SketchEditSession::applyNamedParam(int entityId, const QString& key, double value, QString* err)
{
	auto upsertRadiusLike = [&](SkConstraintKind kind, int id, double v) {
		for (auto& c : m_doc.constraintsMut())
		{
			if (c.kind == kind && c.a == id)
			{
				c.value = v;
				return;
			}
		}
		m_doc.addConstraint(SkConstraint{kind, id, -1, v, -1});
	};

	if (SkCircle* c = m_doc.findCircle(entityId))
	{
		if (key == QStringLiteral("radius") && value > 1e-9)
		{
			c->radius = value;
			upsertRadiusLike(SkConstraintKind::Radius, entityId, value);
		}
		else if ((key == QStringLiteral("centerX") || key == QStringLiteral("centerY")))
		{
			SkPoint* p = m_doc.findPoint(c->center);
			if (!p)
				return false;
			if (key == QStringLiteral("centerX"))
				p->p.u = value;
			else
				p->p.v = value;
		}
		else
			return false;
	}
	else if (SkLine* ln = m_doc.findLine(entityId))
	{
		SkPoint* p1 = m_doc.findPoint(ln->p1);
		SkPoint* p2 = m_doc.findPoint(ln->p2);
		if (!p1 || !p2)
			return false;
		if (key == QStringLiteral("startX"))
			p1->p.u = value;
		else if (key == QStringLiteral("startY"))
			p1->p.v = value;
		else if (key == QStringLiteral("endX"))
			p2->p.u = value;
		else if (key == QStringLiteral("endY"))
			p2->p.v = value;
		else if (key == QStringLiteral("length") && value > 1e-9)
		{
			const double dx = p2->p.u - p1->p.u;
			const double dy = p2->p.v - p1->p.v;
			const double len = std::sqrt(dx * dx + dy * dy);
			if (len < 1e-9)
				return false;
			const double s = value / len;
			p2->p.u = p1->p.u + dx * s;
			p2->p.v = p1->p.v + dy * s;
		}
		else
			return false;
	}
	else if (SkEllipse* e = m_doc.findEllipse(entityId))
	{
		if (key == QStringLiteral("majorRadius") && value > 1e-9)
		{
			e->majorR = value;
			upsertRadiusLike(SkConstraintKind::MajorRadius, entityId, value);
		}
		else if (key == QStringLiteral("minorRadius") && value > 1e-9)
		{
			e->minorR = value;
			upsertRadiusLike(SkConstraintKind::MinorRadius, entityId, value);
		}
		else if (key == QStringLiteral("majorAxisAngle"))
			e->angleRad = value * 3.141592653589793 / 180.0;
		else if (key == QStringLiteral("centerX") || key == QStringLiteral("centerY"))
		{
			SkPoint* p = m_doc.findPoint(e->center);
			if (!p)
				return false;
			if (key == QStringLiteral("centerX"))
				p->p.u = value;
			else
				p->p.v = value;
		}
		else
			return false;
	}
	else if (SkSpline* sp = m_doc.findSpline(entityId))
	{
		if (key == QStringLiteral("mode"))
		{
			const auto mode = (value >= 0.5) ? SkSplineMode::ControlPoints : SkSplineMode::ThroughPoints;
			if (!m_doc.setSplineMode(entityId, mode))
			{
				if (err)
					*err = QStringLiteral("无法切换样条模式");
				return false;
			}
			(void)sp;
		}
		else
			return false;
	}
	else
		return false;

	std::string solveErr;
	if (!solveNow(&solveErr))
	{
		if (err)
			*err = QString::fromStdString(solveErr);
		return false;
	}
	refreshOverlay();
	if (m_onChanged)
		m_onChanged();
	return true;
}

bool SketchEditSession::exportClosedProfile(std::vector<float>& outXyzMm, std::string* err) const
{
	return m_doc.exportClosedProfileXyz(m_plane, outXyzMm, err);
}

void SketchEditSession::refreshOverlay()
{
	if (!m_geo || !m_docPtr)
		return;
	SkVec2 a, b;
	const SkVec2* pa = nullptr;
	const SkVec2* pb = nullptr;
	std::vector<SkVec2> previewPoly;
	const std::vector<SkVec2>* pPoly = nullptr;
	if (m_tool && m_tool->previewPolyline(previewPoly))
		pPoly = &previewPoly;
	else if (m_tool && m_tool->hasPreview(a, b))
	{
		pa = &a;
		pb = &b;
	}
	SkOverlayStyle style;
	style.conflictEntityIds = m_conflictEntities;
	style.redundantEntityIds = m_redundantEntities;
	style.normalBiasMm = 0.05;
	if (m_dimHoverId >= 0)
		style.highlightEntityIds.insert(m_dimHoverId);
	if (m_dimPickA >= 0)
		style.highlightEntityIds.insert(m_dimPickA);
	if (m_selectedEntityId >= 0)
		style.highlightEntityIds.insert(m_selectedEntityId);
	for (int id : m_mirrorTargets)
		style.highlightEntityIds.insert(id);
	std::vector<PluginSketchOverlaySegment> segs;
	m_doc.tessellateOverlay(m_plane, segs, pa, pb, m_lastSnap.snapped ? &m_lastSnap : nullptr, &style, pPoly);
	if (m_backgroundOverlay)
		m_backgroundOverlay(segs);
	m_geo->setSketchOverlay(m_docPtr, segs);
}
