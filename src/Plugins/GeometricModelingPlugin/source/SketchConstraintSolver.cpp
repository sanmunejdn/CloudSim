/// @file SketchConstraintSolver.cpp
/// @brief 调用 vendored PlaneGCS（LGPL）

#include "SketchConstraintSolver.h"

#include "GCS.h"
#include "Geo.h"

#include <cmath>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void SketchConstraintSolver::clear()
{
	m_points.clear();
	m_lines.clear();
	m_arcs.clear();
	m_circles.clear();
	m_constraints.clear();
	m_dof = 0;
	m_hasConflicting = false;
	m_hasRedundant = false;
	m_conflictingTags.clear();
	m_redundantTags.clear();
}

int SketchConstraintSolver::addPoint(double x, double y, bool fixed)
{
	m_points.push_back(SketchPoint2d{x, y, fixed});
	return static_cast<int>(m_points.size()) - 1;
}

int SketchConstraintSolver::addLine(int p1, int p2)
{
	m_lines.push_back(SketchLine2d{p1, p2});
	return static_cast<int>(m_lines.size()) - 1;
}

int SketchConstraintSolver::addArc(int center, int start, int end, double radius)
{
	m_arcs.push_back(SketchArc2d{center, start, end, radius});
	return static_cast<int>(m_arcs.size()) - 1;
}

int SketchConstraintSolver::addCircle(int center, double radius)
{
	m_circles.push_back(SketchCircle2d{center, radius});
	return static_cast<int>(m_circles.size()) - 1;
}

void SketchConstraintSolver::addConstraint(const SketchConstraint2d& c)
{
	m_constraints.push_back(c);
}

int SketchConstraintSolver::solve(std::string* errMsg)
{
	m_hasConflicting = false;
	m_hasRedundant = false;
	m_conflictingTags.clear();
	m_redundantTags.clear();

	GCS::System sys;
	std::vector<double> px(m_points.size()), py(m_points.size());
	std::vector<GCS::Point> gpts(m_points.size());
	GCS::VEC_pD unknowns;
	for (std::size_t i = 0; i < m_points.size(); ++i)
	{
		px[i] = m_points[i].x;
		py[i] = m_points[i].y;
		gpts[i] = GCS::Point(&px[i], &py[i]);
		if (!m_points[i].fixed)
		{
			unknowns.push_back(&px[i]);
			unknowns.push_back(&py[i]);
		}
	}
	std::vector<GCS::Line> glines;
	glines.reserve(m_lines.size());
	for (const auto& ln : m_lines)
	{
		if (ln.p1 < 0 || ln.p2 < 0)
			continue;
		GCS::Line L;
		L.p1 = gpts[static_cast<std::size_t>(ln.p1)];
		L.p2 = gpts[static_cast<std::size_t>(ln.p2)];
		glines.push_back(L);
	}

	std::vector<double> radStorage(m_arcs.size());
	std::vector<double> startAng(m_arcs.size()), endAng(m_arcs.size());
	std::vector<GCS::Arc> garcs(m_arcs.size());
	for (std::size_t i = 0; i < m_arcs.size(); ++i)
	{
		const auto& a = m_arcs[i];
		radStorage[i] = a.radius > 1e-9 ? a.radius : 1.0;
		const double sx = px[static_cast<std::size_t>(a.start)] - px[static_cast<std::size_t>(a.center)];
		const double sy = py[static_cast<std::size_t>(a.start)] - py[static_cast<std::size_t>(a.center)];
		const double ex = px[static_cast<std::size_t>(a.end)] - px[static_cast<std::size_t>(a.center)];
		const double ey = py[static_cast<std::size_t>(a.end)] - py[static_cast<std::size_t>(a.center)];
		startAng[i] = std::atan2(sy, sx);
		endAng[i] = std::atan2(ey, ex);
		garcs[i].center = gpts[static_cast<std::size_t>(a.center)];
		garcs[i].start = gpts[static_cast<std::size_t>(a.start)];
		garcs[i].end = gpts[static_cast<std::size_t>(a.end)];
		garcs[i].rad = &radStorage[i];
		garcs[i].startAngle = &startAng[i];
		garcs[i].endAngle = &endAng[i];
		unknowns.push_back(&radStorage[i]);
		unknowns.push_back(&startAng[i]);
		unknowns.push_back(&endAng[i]);
		sys.addConstraintArcRules(garcs[i], 9000 + static_cast<int>(i));
	}

	std::vector<double> circleRadStorage(m_circles.size());
	std::vector<GCS::Circle> gcircles(m_circles.size());
	for (std::size_t i = 0; i < m_circles.size(); ++i)
	{
		const auto& c = m_circles[i];
		if (c.center < 0 || c.center >= static_cast<int>(gpts.size()))
			continue;
		circleRadStorage[i] = c.radius > 1e-9 ? c.radius : 1.0;
		gcircles[i].center = gpts[static_cast<std::size_t>(c.center)];
		gcircles[i].rad = &circleRadStorage[i];
		unknowns.push_back(&circleRadStorage[i]);
	}

	std::vector<double> distStorage;
	distStorage.reserve(m_constraints.size() + 8);
	std::vector<double> angleStorage;
	angleStorage.reserve(m_constraints.size());
	int autoTag = 1;
	for (const auto& c : m_constraints)
	{
		const int tag = c.tagId > 0 ? c.tagId : autoTag++;
		switch (c.kind)
		{
		case SketchConstraintKind::Distance:
			distStorage.push_back(c.value);
			sys.addConstraintP2PDistance(gpts[static_cast<std::size_t>(c.a)], gpts[static_cast<std::size_t>(c.b)],
										 &distStorage.back(), tag);
			break;
		case SketchConstraintKind::Horizontal:
			if (c.b >= 0 && c.a >= 0 && c.a < static_cast<int>(gpts.size()) && c.b < static_cast<int>(gpts.size()))
				sys.addConstraintHorizontal(gpts[static_cast<std::size_t>(c.a)], gpts[static_cast<std::size_t>(c.b)],
											tag);
			else if (c.a >= 0 && c.a < static_cast<int>(glines.size()))
				sys.addConstraintHorizontal(glines[static_cast<std::size_t>(c.a)], tag);
			break;
		case SketchConstraintKind::Vertical:
			if (c.b >= 0 && c.a >= 0 && c.a < static_cast<int>(gpts.size()) && c.b < static_cast<int>(gpts.size()))
				sys.addConstraintVertical(gpts[static_cast<std::size_t>(c.a)], gpts[static_cast<std::size_t>(c.b)],
										  tag);
			else if (c.a >= 0 && c.a < static_cast<int>(glines.size()))
				sys.addConstraintVertical(glines[static_cast<std::size_t>(c.a)], tag);
			break;
		case SketchConstraintKind::EqualLength:
			if (c.a >= 0 && c.b >= 0 && c.a < static_cast<int>(glines.size()) &&
				c.b < static_cast<int>(glines.size()))
				sys.addConstraintEqualLength(glines[static_cast<std::size_t>(c.a)],
											 glines[static_cast<std::size_t>(c.b)], tag);
			break;
		case SketchConstraintKind::Coincident:
			sys.addConstraintEqual(gpts[static_cast<std::size_t>(c.a)].x, gpts[static_cast<std::size_t>(c.b)].x, tag);
			sys.addConstraintEqual(gpts[static_cast<std::size_t>(c.a)].y, gpts[static_cast<std::size_t>(c.b)].y, tag);
			break;
		case SketchConstraintKind::Parallel:
			if (c.a >= 0 && c.b >= 0 && c.a < static_cast<int>(glines.size()) &&
				c.b < static_cast<int>(glines.size()))
				sys.addConstraintParallel(glines[static_cast<std::size_t>(c.a)], glines[static_cast<std::size_t>(c.b)],
										  tag);
			break;
		case SketchConstraintKind::Perpendicular:
			if (c.a >= 0 && c.b >= 0 && c.a < static_cast<int>(glines.size()) &&
				c.b < static_cast<int>(glines.size()))
				sys.addConstraintPerpendicular(glines[static_cast<std::size_t>(c.a)],
											   glines[static_cast<std::size_t>(c.b)], tag);
			break;
		case SketchConstraintKind::Angle:
			if (c.a >= 0 && c.b >= 0 && c.a < static_cast<int>(glines.size()) &&
				c.b < static_cast<int>(glines.size()))
			{
				angleStorage.push_back(c.value * M_PI / 180.0);
				sys.addConstraintL2LAngle(glines[static_cast<std::size_t>(c.a)], glines[static_cast<std::size_t>(c.b)],
										  &angleStorage.back(), tag);
			}
			break;
		case SketchConstraintKind::ArcRadius:
			if (c.a >= 0 && c.a < static_cast<int>(garcs.size()))
			{
				radStorage[static_cast<std::size_t>(c.a)] = c.value;
				sys.addConstraintArcRadius(garcs[static_cast<std::size_t>(c.a)], &radStorage[static_cast<std::size_t>(c.a)],
										   tag);
			}
			break;
		case SketchConstraintKind::Radius:
			break;
		case SketchConstraintKind::Tangent:
			if (c.a >= 0 && c.a < static_cast<int>(glines.size()))
			{
				if (c.b >= 0 && c.b < static_cast<int>(garcs.size()))
					sys.addConstraintTangent(glines[static_cast<std::size_t>(c.a)],
											   garcs[static_cast<std::size_t>(c.b)], tag);
				else if (c.b >= 0 && c.b < static_cast<int>(gcircles.size()))
					sys.addConstraintTangent(glines[static_cast<std::size_t>(c.a)],
											   gcircles[static_cast<std::size_t>(c.b)], tag);
			}
			break;
		case SketchConstraintKind::Symmetric:
			if (c.c >= 0 && c.c < static_cast<int>(glines.size()) && c.a >= 0 && c.b >= 0 &&
				c.a < static_cast<int>(gpts.size()) && c.b < static_cast<int>(gpts.size()))
				sys.addConstraintP2PSymmetric(gpts[static_cast<std::size_t>(c.a)], gpts[static_cast<std::size_t>(c.b)],
											  glines[static_cast<std::size_t>(c.c)], tag);
			break;
		case SketchConstraintKind::Midpoint:
			if (c.a >= 0 && c.a < static_cast<int>(gpts.size()) && c.b >= 0 &&
				c.b < static_cast<int>(glines.size()))
			{
				// 线段中点与目标点重合：退化第二段为点-点
				GCS::Line& ln = glines[static_cast<std::size_t>(c.b)];
				GCS::Point& mid = gpts[static_cast<std::size_t>(c.a)];
				sys.addConstraintMidpointOnLine(ln.p1, ln.p2, mid, mid, tag);
			}
			break;
		}
	}
	sys.declareUnknowns(unknowns);
	sys.initSolution();
	int rc = sys.solve(true, GCS::DogLeg, false);
	// DogLeg 失败时回退 LM（与 OneCAD ConstraintSolver 一致）
	if (rc == static_cast<int>(GCS::Failed))
		rc = sys.solve(true, GCS::LevenbergMarquardt, false);
	m_dof = sys.dofsNumber();
	sys.getConflicting(m_conflictingTags);
	sys.getRedundant(m_redundantTags);
	m_hasConflicting = sys.hasConflicting();
	m_hasRedundant = !m_redundantTags.empty();
	// Success/Converged 均视为可用；PlaneGCS 解在内部缓冲，须 apply 才写回未知量
	const bool ok = (rc == static_cast<int>(GCS::Success) || rc == static_cast<int>(GCS::Converged));
	if (!ok)
	{
		if (errMsg)
			*errMsg = "PlaneGCS solve failed";
		return rc == 0 ? -1 : rc;
	}
	sys.applySolution();
	for (std::size_t i = 0; i < m_points.size(); ++i)
	{
		m_points[i].x = px[i];
		m_points[i].y = py[i];
	}
	for (std::size_t i = 0; i < m_arcs.size(); ++i)
		m_arcs[i].radius = radStorage[i];
	return 0;
}

bool SketchConstraintSolver::runEquilateralTriangleSelfTest(std::string* errMsg)
{
	SketchConstraintSolver s;
	const int p0 = s.addPoint(0.0, 0.0, true);
	const int p1 = s.addPoint(100.0, 0.0, true);
	const int p2 = s.addPoint(40.0, 50.0, false);
	const int l0 = s.addLine(p0, p1);
	const int l1 = s.addLine(p1, p2);
	const int l2 = s.addLine(p2, p0);
	(void)l0;
	s.addConstraint(SketchConstraint2d{SketchConstraintKind::EqualLength, l1, l0, 0.0, 1});
	s.addConstraint(SketchConstraint2d{SketchConstraintKind::EqualLength, l2, l0, 0.0, 2});
	std::string err;
	if (s.solve(&err) != 0)
	{
		if (errMsg)
			*errMsg = err;
		return false;
	}
	const auto& pts = s.points();
	const double dx = pts[2].x - 50.0;
	const double dy = pts[2].y - (50.0 * std::sqrt(3.0));
	if (std::abs(dx) > 1.0 || std::abs(dy) > 1.0)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "equilateral tip off: (" << pts[2].x << "," << pts[2].y << ")";
			*errMsg = oss.str();
		}
		return false;
	}
	return true;
}
