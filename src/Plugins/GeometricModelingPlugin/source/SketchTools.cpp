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
