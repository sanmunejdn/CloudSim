/// @file SketchTools.cpp

#include "SketchTools.h"

#include <cmath>

void LineSketchTool::cancel()
{
	m_active = false;
	m_lastPointId = -1;
}

void LineSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (!m_active)
	{
		m_lastPointId = doc.addPoint(pos.u, pos.v);
		m_start = pos;
		m_curr = pos;
		m_active = true;
		return;
	}
	const int p2 = doc.addPoint(pos.u, pos.v);
	const int lineId = doc.addLine(m_lastPointId, p2);
	if (std::abs(pos.u - m_start.u) < 1e-3)
		doc.addConstraint({SkConstraintKind::Vertical, lineId, -1, 0});
	if (std::abs(pos.v - m_start.v) < 1e-3)
		doc.addConstraint({SkConstraintKind::Horizontal, lineId, -1, 0});
	// 折线续画
	m_lastPointId = p2;
	m_start = pos;
	m_curr = pos;
}

void LineSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool LineSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (!m_active)
		return false;
	outA = m_start;
	outB = m_curr;
	return true;
}

std::optional<SkVec2> LineSketchTool::referencePoint() const
{
	if (!m_active)
		return std::nullopt;
	return m_start;
}

void ArcSketchTool::cancel()
{
	m_step = 0;
	m_idStart = m_idMid = -1;
}

void ArcSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (m_step == 0)
	{
		m_idStart = doc.addPoint(pos.u, pos.v);
		m_start = pos;
		m_curr = pos;
		m_step = 1;
		return;
	}
	if (m_step == 1)
	{
		m_idMid = doc.addPoint(pos.u, pos.v);
		m_mid = pos;
		m_curr = pos;
		m_step = 2;
		return;
	}
	const int idEnd = doc.addPoint(pos.u, pos.v);
	doc.addArc(m_idStart, m_idMid, idEnd);
	cancel();
}

void ArcSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool ArcSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (m_step == 0)
		return false;
	outA = (m_step == 1) ? m_start : m_mid;
	outB = m_curr;
	return true;
}

std::optional<SkVec2> ArcSketchTool::referencePoint() const
{
	if (m_step == 0)
		return std::nullopt;
	return (m_step == 1) ? m_start : m_mid;
}

void CircleSketchTool::cancel()
{
	m_haveCenter = false;
	m_centerId = -1;
}

void CircleSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (!m_haveCenter)
	{
		m_centerId = doc.addPoint(pos.u, pos.v, true);
		m_center = pos;
		m_curr = pos;
		m_haveCenter = true;
		return;
	}
	const double r = skDist(m_center, pos);
	doc.addCircle(m_centerId, r);
	doc.addConstraint({SkConstraintKind::Radius, m_centerId, -1, r});
	cancel();
}

void CircleSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool CircleSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (!m_haveCenter)
		return false;
	outA = m_center;
	outB = m_curr;
	return true;
}

std::optional<SkVec2> CircleSketchTool::referencePoint() const
{
	if (!m_haveCenter)
		return std::nullopt;
	return m_center;
}

void RectSketchTool::cancel()
{
	m_haveFirst = false;
}

void RectSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (!m_haveFirst)
	{
		m_a = pos;
		m_curr = pos;
		m_haveFirst = true;
		return;
	}
	const SkVec2 b = pos;
	const int p0 = doc.addPoint(m_a.u, m_a.v);
	const int p1 = doc.addPoint(b.u, m_a.v);
	const int p2 = doc.addPoint(b.u, b.v);
	const int p3 = doc.addPoint(m_a.u, b.v);
	const int l0 = doc.addLine(p0, p1);
	const int l1 = doc.addLine(p1, p2);
	const int l2 = doc.addLine(p2, p3);
	const int l3 = doc.addLine(p3, p0);
	doc.addConstraint({SkConstraintKind::Horizontal, l0, -1, 0});
	doc.addConstraint({SkConstraintKind::Vertical, l1, -1, 0});
	doc.addConstraint({SkConstraintKind::Horizontal, l2, -1, 0});
	doc.addConstraint({SkConstraintKind::Vertical, l3, -1, 0});
	doc.addConstraint({SkConstraintKind::EqualLength, l0, l2, 0});
	doc.addConstraint({SkConstraintKind::EqualLength, l1, l3, 0});
	doc.addConstraint({SkConstraintKind::Perpendicular, l0, l1, 0});
	cancel();
}

void RectSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool RectSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (!m_haveFirst)
		return false;
	outA = m_a;
	outB = m_curr;
	return true;
}

std::optional<SkVec2> RectSketchTool::referencePoint() const
{
	if (!m_haveFirst)
		return std::nullopt;
	return m_a;
}

void SplineSketchTool::cancel()
{
	m_pts.clear();
	m_haveCursor = false;
}

void SplineSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		if (m_pts.size() >= 2)
		{
			std::vector<int> ids;
			ids.reserve(m_pts.size());
			for (const SkVec2& p : m_pts)
				ids.push_back(doc.addPoint(p.u, p.v));
			doc.addSpline(ids);
		}
		cancel();
		return;
	}
	m_pts.push_back(pos);
	m_curr = pos;
	m_haveCursor = true;
}

void SplineSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
	m_haveCursor = true;
}

bool SplineSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	(void)outA;
	(void)outB;
	return false;
}

bool SplineSketchTool::previewPolyline(std::vector<SkVec2>& out) const
{
	if (m_pts.empty())
		return false;
	std::vector<SkVec2> through = m_pts;
	if (m_haveCursor)
		through.push_back(m_curr);
	if (through.size() < 2)
		return false;
	sketchSampleCatmullRom(through, out, 12);
	return out.size() >= 2;
}

std::optional<SkVec2> SplineSketchTool::referencePoint() const
{
	if (m_pts.empty())
		return std::nullopt;
	return m_pts.back();
}

namespace
{
SkVec2 slotPerpNormal(const SkVec2& a, const SkVec2& b)
{
	const double dx = b.u - a.u;
	const double dy = b.v - a.v;
	const double len = std::sqrt(dx * dx + dy * dy);
	if (len < 1e-9)
		return {0.0, 1.0};
	return {-dy / len, dx / len};
}

void appendArcSamplesLocal(std::vector<SkVec2>& out, const SkVec2& s, const SkVec2& m, const SkVec2& e, int segs)
{
	SkVec2 cen;
	double r = 0.0;
	if (!sketchCircumcenter(s, m, e, cen, r))
	{
		out.push_back(s);
		out.push_back(e);
		return;
	}
	auto ang = [&](const SkVec2& p) { return std::atan2(p.v - cen.v, p.u - cen.u); };
	double a0 = ang(s);
	double a1 = ang(m);
	double a2 = ang(e);
	auto norm = [](double a)
	{
		while (a < 0)
			a += 2.0 * 3.141592653589793;
		while (a >= 2.0 * 3.141592653589793)
			a -= 2.0 * 3.141592653589793;
		return a;
	};
	a0 = norm(a0);
	a1 = norm(a1);
	a2 = norm(a2);
	double sweep = a2 - a0;
	if (sweep < 0)
		sweep += 2.0 * 3.141592653589793;
	const double midRel = norm(a1 - a0);
	if (midRel > sweep)
		sweep -= 2.0 * 3.141592653589793;
	for (int i = 0; i <= segs; ++i)
	{
		const double t = static_cast<double>(i) / segs;
		const double a = a0 + sweep * t;
		out.push_back({cen.u + r * std::cos(a), cen.v + r * std::sin(a)});
	}
}

double slotHalfWidth(const SkVec2& a, const SkVec2& b, const SkVec2& pick)
{
	const SkVec2 n = slotPerpNormal(a, b);
	const double dx = pick.u - a.u;
	const double dy = pick.v - a.v;
	return std::abs(dx * n.u + dy * n.v);
}

void slotPreviewPoly(const SkVec2& a, const SkVec2& b, const SkVec2& widthPick, std::vector<SkVec2>& out)
{
	out.clear();
	const double hw = slotHalfWidth(a, b, widthPick);
	if (hw < 1e-6 || skDist(a, b) < 1e-6)
		return;
	const SkVec2 n = slotPerpNormal(a, b);
	const SkVec2 dir{(b.u - a.u) / skDist(a, b), (b.v - a.v) / skDist(a, b)};
	const SkVec2 p1{a.u + n.u * hw, a.v + n.v * hw};
	const SkVec2 p2{b.u + n.u * hw, b.v + n.v * hw};
	const SkVec2 p3{b.u - n.u * hw, b.v - n.v * hw};
	const SkVec2 p4{a.u - n.u * hw, a.v - n.v * hw};
	out.push_back(p1);
	out.push_back(p2);
	const SkVec2 midB{b.u + dir.u * hw, b.v + dir.v * hw};
	appendArcSamplesLocal(out, p2, midB, p3, 12);
	out.push_back(p3);
	out.push_back(p4);
	const SkVec2 midA{a.u - dir.u * hw, a.v - dir.v * hw};
	appendArcSamplesLocal(out, p4, midA, p1, 12);
}

void buildSlot(SketchDocument2d& doc, const SkVec2& a, const SkVec2& b, const SkVec2& widthPick)
{
	const double hw = slotHalfWidth(a, b, widthPick);
	if (hw < 1e-6 || skDist(a, b) < 1e-6)
		return;
	const SkVec2 n = slotPerpNormal(a, b);
	const double len = skDist(a, b);
	const SkVec2 dir{(b.u - a.u) / len, (b.v - a.v) / len};

	const int p1 = doc.addPoint(a.u + n.u * hw, a.v + n.v * hw);
	const int p2 = doc.addPoint(b.u + n.u * hw, b.v + n.v * hw);
	const int p3 = doc.addPoint(b.u - n.u * hw, b.v - n.v * hw);
	const int p4 = doc.addPoint(a.u - n.u * hw, a.v - n.v * hw);
	doc.addLine(p1, p2, false);
	doc.addLine(p3, p4, false);

	const int midB = doc.addPoint(b.u + dir.u * hw, b.v + dir.v * hw);
	doc.addArc(p2, midB, p3, false);
	const int midA = doc.addPoint(a.u - dir.u * hw, a.v - dir.v * hw);
	doc.addArc(p4, midA, p1, false);
}
} // namespace

void EllipseSketchTool::cancel()
{
	m_haveCenter = false;
	m_centerId = -1;
}

void EllipseSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (!m_haveCenter)
	{
		m_centerId = doc.addPoint(pos.u, pos.v, true);
		m_center = pos;
		m_curr = pos;
		m_haveCenter = true;
		return;
	}
	const double majorR = skDist(m_center, pos);
	if (majorR < 1e-6)
	{
		cancel();
		return;
	}
	const double minorR = majorR * 0.6;
	const double angleRad = std::atan2(pos.v - m_center.v, pos.u - m_center.u);
	doc.addEllipse(m_centerId, majorR, minorR, angleRad);
	cancel();
}

void EllipseSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool EllipseSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (!m_haveCenter)
		return false;
	outA = m_center;
	outB = m_curr;
	return true;
}

bool EllipseSketchTool::previewPolyline(std::vector<SkVec2>& out) const
{
	if (!m_haveCenter)
		return false;
	const double majorR = skDist(m_center, m_curr);
	if (majorR < 1e-6)
		return false;
	const double minorR = majorR * 0.6;
	const double angleRad = std::atan2(m_curr.v - m_center.v, m_curr.u - m_center.u);
	sketchSampleEllipse(m_center, majorR, minorR, angleRad, out, 48);
	return out.size() >= 3;
}

std::optional<SkVec2> EllipseSketchTool::referencePoint() const
{
	if (!m_haveCenter)
		return std::nullopt;
	return m_center;
}

void PolygonSketchTool::cancel()
{
	m_haveCenter = false;
}

void PolygonSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (!m_haveCenter)
	{
		m_center = pos;
		m_curr = pos;
		m_haveCenter = true;
		return;
	}
	const double radius = skDist(m_center, pos);
	if (radius < 1e-6)
	{
		cancel();
		return;
	}
	const double startAng = std::atan2(pos.v - m_center.v, pos.u - m_center.u);
	const int sides = m_sides;
	constexpr double kPi = 3.141592653589793;
	std::vector<int> ptIds;
	ptIds.reserve(static_cast<std::size_t>(sides));
	for (int i = 0; i < sides; ++i)
	{
		const double ang = startAng + 2.0 * kPi * i / sides;
		ptIds.push_back(doc.addPoint(m_center.u + radius * std::cos(ang), m_center.v + radius * std::sin(ang)));
	}
	std::vector<int> lineIds;
	lineIds.reserve(static_cast<std::size_t>(sides));
	for (int i = 0; i < sides; ++i)
	{
		const int j = (i + 1) % sides;
		lineIds.push_back(doc.addLine(ptIds[static_cast<std::size_t>(i)], ptIds[static_cast<std::size_t>(j)]));
	}
	for (int i = 0; i < sides; ++i)
		doc.addConstraint({SkConstraintKind::EqualLength, lineIds[0], lineIds[static_cast<std::size_t>(i)], 0.0});
	cancel();
}

void PolygonSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool PolygonSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (!m_haveCenter)
		return false;
	outA = m_center;
	outB = m_curr;
	return true;
}

bool PolygonSketchTool::previewPolyline(std::vector<SkVec2>& out) const
{
	if (!m_haveCenter)
		return false;
	const double radius = skDist(m_center, m_curr);
	if (radius < 1e-6)
		return false;
	const double startAng = std::atan2(m_curr.v - m_center.v, m_curr.u - m_center.u);
	const int sides = m_sides;
	constexpr double kPi = 3.141592653589793;
	out.clear();
	for (int i = 0; i <= sides; ++i)
	{
		const double ang = startAng + 2.0 * kPi * (i % sides) / sides;
		out.push_back({m_center.u + radius * std::cos(ang), m_center.v + radius * std::sin(ang)});
	}
	return out.size() >= 3;
}

std::optional<SkVec2> PolygonSketchTool::referencePoint() const
{
	if (!m_haveCenter)
		return std::nullopt;
	return m_center;
}

void SlotSketchTool::cancel()
{
	m_step = 0;
}

void SlotSketchTool::onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc)
{
	if (rightButton)
	{
		cancel();
		return;
	}
	if (m_step == 0)
	{
		m_start = pos;
		m_curr = pos;
		m_step = 1;
		return;
	}
	if (m_step == 1)
	{
		if (skDist(m_start, pos) < 1e-6)
			return;
		m_end = pos;
		m_curr = pos;
		m_step = 2;
		return;
	}
	buildSlot(doc, m_start, m_end, pos);
	cancel();
}

void SlotSketchTool::onMove(const SkVec2& pos)
{
	m_curr = pos;
}

bool SlotSketchTool::hasPreview(SkVec2& outA, SkVec2& outB) const
{
	if (m_step == 0)
		return false;
	outA = (m_step == 1) ? m_start : m_end;
	outB = m_curr;
	return true;
}

bool SlotSketchTool::previewPolyline(std::vector<SkVec2>& out) const
{
	if (m_step == 2)
	{
		slotPreviewPoly(m_start, m_end, m_curr, out);
		return out.size() >= 3;
	}
	if (m_step == 1 && skDist(m_start, m_curr) >= 1e-6)
	{
		out = {m_start, m_curr};
		return true;
	}
	return false;
}

std::optional<SkVec2> SlotSketchTool::referencePoint() const
{
	if (m_step == 0)
		return std::nullopt;
	return (m_step == 1) ? m_start : m_end;
}
