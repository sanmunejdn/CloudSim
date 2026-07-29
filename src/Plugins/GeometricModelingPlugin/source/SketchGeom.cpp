/// @file SketchGeom.cpp

#include "SketchGeom.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

int SketchDocument2d::nextId()
{
	return m_seq++;
}

void SketchDocument2d::clear()
{
	m_points.clear();
	m_lines.clear();
	m_arcs.clear();
	m_circles.clear();
	m_ellipses.clear();
	m_splines.clear();
	m_constraints.clear();
	m_seq = 1;
}

int SketchDocument2d::addPoint(double u, double v, bool fixed)
{
	SkPoint pt;
	pt.id = nextId();
	pt.p = {u, v};
	pt.fixed = fixed;
	m_points.push_back(pt);
	return pt.id;
}

int SketchDocument2d::addLine(int p1, int p2, bool construction)
{
	SkLine ln;
	ln.id = nextId();
	ln.p1 = p1;
	ln.p2 = p2;
	ln.construction = construction;
	m_lines.push_back(ln);
	return ln.id;
}

int SketchDocument2d::addArc(int pStart, int pMid, int pEnd, bool construction)
{
	SkArc a;
	a.id = nextId();
	a.pStart = pStart;
	a.pMid = pMid;
	a.pEnd = pEnd;
	a.construction = construction;
	m_arcs.push_back(a);
	return a.id;
}

int SketchDocument2d::addCircle(int center, double radius, bool construction)
{
	SkCircle c;
	c.id = nextId();
	c.center = center;
	c.radius = radius;
	c.construction = construction;
	m_circles.push_back(c);
	return c.id;
}

int SketchDocument2d::addEllipse(int center, double majorR, double minorR, double angleRad, bool construction)
{
	SkEllipse e;
	e.id = nextId();
	e.center = center;
	e.majorR = majorR;
	e.minorR = minorR;
	e.angleRad = angleRad;
	e.construction = construction;
	m_ellipses.push_back(e);
	return e.id;
}

int SketchDocument2d::addSpline(const std::vector<int>& throughPts, bool construction)
{
	if (throughPts.size() < 2)
		return -1;
	SkSpline sp;
	sp.id = nextId();
	sp.throughPts = throughPts;
	sp.construction = construction;
	m_splines.push_back(sp);
	return sp.id;
}

void SketchDocument2d::addConstraint(const SkConstraint& c)
{
	m_constraints.push_back(c);
}

bool SketchDocument2d::toggleConstruction(int entityId)
{
	if (auto* ln = findLine(entityId))
	{
		ln->construction = !ln->construction;
		return true;
	}
	if (auto* arc = findArc(entityId))
	{
		arc->construction = !arc->construction;
		return true;
	}
	if (auto* cir = findCircle(entityId))
	{
		cir->construction = !cir->construction;
		return true;
	}
	if (auto* el = findEllipse(entityId))
	{
		el->construction = !el->construction;
		return true;
	}
	if (auto* sp = findSpline(entityId))
	{
		sp->construction = !sp->construction;
		return true;
	}
	return false;
}

bool SketchDocument2d::removeLine(int id)
{
	const auto n = m_lines.size();
	m_lines.erase(std::remove_if(m_lines.begin(), m_lines.end(), [&](const SkLine& l) { return l.id == id; }),
				  m_lines.end());
	return m_lines.size() != n;
}

bool SketchDocument2d::removeArc(int id)
{
	const auto n = m_arcs.size();
	m_arcs.erase(std::remove_if(m_arcs.begin(), m_arcs.end(), [&](const SkArc& a) { return a.id == id; }), m_arcs.end());
	return m_arcs.size() != n;
}

bool SketchDocument2d::removeCircle(int id)
{
	const auto n = m_circles.size();
	m_circles.erase(std::remove_if(m_circles.begin(), m_circles.end(), [&](const SkCircle& c) { return c.id == id; }),
					m_circles.end());
	return m_circles.size() != n;
}

bool SketchDocument2d::removeEllipse(int id)
{
	const auto n = m_ellipses.size();
	m_ellipses.erase(std::remove_if(m_ellipses.begin(), m_ellipses.end(), [&](const SkEllipse& e) { return e.id == id; }),
					 m_ellipses.end());
	return m_ellipses.size() != n;
}

bool SketchDocument2d::removeSpline(int id)
{
	const auto n = m_splines.size();
	m_splines.erase(std::remove_if(m_splines.begin(), m_splines.end(), [&](const SkSpline& s) { return s.id == id; }),
					m_splines.end());
	return m_splines.size() != n;
}

bool SketchDocument2d::removeEntity(int id)
{
	std::vector<int> ownedPts;
	if (const SkLine* ln = findLine(id))
	{
		ownedPts = {ln->p1, ln->p2};
		if (!removeLine(id))
			return false;
	}
	else if (const SkArc* arc = findArc(id))
	{
		ownedPts = {arc->pStart, arc->pMid, arc->pEnd};
		if (!removeArc(id))
			return false;
	}
	else if (const SkCircle* cir = findCircle(id))
	{
		ownedPts = {cir->center};
		if (!removeCircle(id))
			return false;
	}
	else if (const SkEllipse* el = findEllipse(id))
	{
		ownedPts = {el->center};
		if (!removeEllipse(id))
			return false;
	}
	else if (const SkSpline* sp = findSpline(id))
	{
		ownedPts = sp->throughPts;
		if (!removeSpline(id))
			return false;
	}
	else
		return false;

	m_constraints.erase(std::remove_if(m_constraints.begin(), m_constraints.end(),
									   [&](const SkConstraint& c) {
										   if (c.a == id || c.b == id)
											   return true;
										   for (int pid : ownedPts)
										   {
											   if (pid >= 0 && (c.a == pid || c.b == pid))
												   return true;
										   }
										   return false;
									   }),
						m_constraints.end());

	auto pointStillUsed = [&](int pid) {
		for (const SkLine& ln : m_lines)
		{
			if (ln.p1 == pid || ln.p2 == pid)
				return true;
		}
		for (const SkArc& arc : m_arcs)
		{
			if (arc.pStart == pid || arc.pMid == pid || arc.pEnd == pid)
				return true;
		}
		for (const SkCircle& cir : m_circles)
		{
			if (cir.center == pid)
				return true;
		}
		for (const SkEllipse& el : m_ellipses)
		{
			if (el.center == pid)
				return true;
		}
		for (const SkSpline& sp : m_splines)
		{
			for (int tid : sp.throughPts)
			{
				if (tid == pid)
					return true;
			}
		}
		for (const SkConstraint& c : m_constraints)
		{
			if (c.a == pid || c.b == pid)
				return true;
		}
		return false;
	};

	for (int pid : ownedPts)
	{
		if (pid < 0 || pointStillUsed(pid))
			continue;
		m_points.erase(std::remove_if(m_points.begin(), m_points.end(), [&](const SkPoint& p) { return p.id == pid; }),
					   m_points.end());
	}
	return true;
}

SkPoint* SketchDocument2d::findPoint(int id)
{
	for (auto& p : m_points)
		if (p.id == id)
			return &p;
	return nullptr;
}

const SkPoint* SketchDocument2d::findPoint(int id) const
{
	for (const auto& p : m_points)
		if (p.id == id)
			return &p;
	return nullptr;
}

SkLine* SketchDocument2d::findLine(int id)
{
	for (auto& l : m_lines)
		if (l.id == id)
			return &l;
	return nullptr;
}

const SkLine* SketchDocument2d::findLine(int id) const
{
	for (const auto& l : m_lines)
		if (l.id == id)
			return &l;
	return nullptr;
}

SkArc* SketchDocument2d::findArc(int id)
{
	for (auto& a : m_arcs)
		if (a.id == id)
			return &a;
	return nullptr;
}

const SkArc* SketchDocument2d::findArc(int id) const
{
	for (const auto& a : m_arcs)
		if (a.id == id)
			return &a;
	return nullptr;
}

SkCircle* SketchDocument2d::findCircle(int id)
{
	for (auto& c : m_circles)
		if (c.id == id)
			return &c;
	return nullptr;
}

const SkCircle* SketchDocument2d::findCircle(int id) const
{
	for (const auto& c : m_circles)
		if (c.id == id)
			return &c;
	return nullptr;
}

SkEllipse* SketchDocument2d::findEllipse(int id)
{
	for (auto& e : m_ellipses)
		if (e.id == id)
			return &e;
	return nullptr;
}

const SkEllipse* SketchDocument2d::findEllipse(int id) const
{
	for (const auto& e : m_ellipses)
		if (e.id == id)
			return &e;
	return nullptr;
}

SkSpline* SketchDocument2d::findSpline(int id)
{
	for (auto& s : m_splines)
		if (s.id == id)
			return &s;
	return nullptr;
}

const SkSpline* SketchDocument2d::findSpline(int id) const
{
	for (const auto& s : m_splines)
		if (s.id == id)
			return &s;
	return nullptr;
}

SkVec2 SketchDocument2d::worldToUv(const PluginSketchPlane& plane, const PluginPoint3d& w) const
{
	const double dx = w.x - plane.origin.x;
	const double dy = w.y - plane.origin.y;
	const double dz = w.z - plane.origin.z;
	return {dx * plane.axisX.x + dy * plane.axisX.y + dz * plane.axisX.z,
			dx * plane.axisY.x + dy * plane.axisY.y + dz * plane.axisY.z};
}

PluginPoint3d SketchDocument2d::uvToWorld(const PluginSketchPlane& plane, const SkVec2& uv, double normalBiasMm) const
{
	return {plane.origin.x + plane.axisX.x * uv.u + plane.axisY.x * uv.v + plane.normal.x * normalBiasMm,
			plane.origin.y + plane.axisX.y * uv.u + plane.axisY.y * uv.v + plane.normal.y * normalBiasMm,
			plane.origin.z + plane.axisX.z * uv.u + plane.axisY.z * uv.v + plane.normal.z * normalBiasMm};
}

namespace
{
void pushWorld(std::vector<float>& xyz, const PluginPoint3d& p)
{
	xyz.push_back(static_cast<float>(p.x));
	xyz.push_back(static_cast<float>(p.y));
	xyz.push_back(static_cast<float>(p.z));
}

bool circumcenter(const SkVec2& a, const SkVec2& b, const SkVec2& c, SkVec2& out, double& radius)
{
	const double d = 2.0 * (a.u * (b.v - c.v) + b.u * (c.v - a.v) + c.u * (a.v - b.v));
	if (std::abs(d) < 1e-12)
		return false;
	const double a2 = a.u * a.u + a.v * a.v;
	const double b2 = b.u * b.u + b.v * b.v;
	const double c2 = c.u * c.u + c.v * c.v;
	out.u = (a2 * (b.v - c.v) + b2 * (c.v - a.v) + c2 * (a.v - b.v)) / d;
	out.v = (a2 * (c.u - b.u) + b2 * (a.u - c.u) + c2 * (b.u - a.u)) / d;
	radius = skDist(out, a);
	return radius > 1e-9;
}

void appendArcSamples(std::vector<SkVec2>& out, const SkVec2& s, const SkVec2& m, const SkVec2& e, int segs)
{
	SkVec2 cen;
	double r = 0.0;
	if (!circumcenter(s, m, e, cen, r))
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
} // namespace

void sketchSampleCatmullRom(const std::vector<SkVec2>& through, std::vector<SkVec2>& out, int segsPerSpan)
{
	out.clear();
	if (through.size() < 2 || segsPerSpan < 1)
		return;
	if (through.size() == 2)
	{
		out = through;
		return;
	}
	auto at = [&](int i) -> SkVec2
	{
		if (i < 0)
			return through.front();
		if (i >= static_cast<int>(through.size()))
			return through.back();
		return through[static_cast<std::size_t>(i)];
	};
	out.push_back(through.front());
	for (int i = 0; i < static_cast<int>(through.size()) - 1; ++i)
	{
		const SkVec2 p0 = at(i - 1);
		const SkVec2 p1 = at(i);
		const SkVec2 p2 = at(i + 1);
		const SkVec2 p3 = at(i + 2);
		for (int s = 1; s <= segsPerSpan; ++s)
		{
			const double t = static_cast<double>(s) / segsPerSpan;
			const double t2 = t * t;
			const double t3 = t2 * t;
			SkVec2 q;
			q.u = 0.5 * ((2.0 * p1.u) + (-p0.u + p2.u) * t + (2.0 * p0.u - 5.0 * p1.u + 4.0 * p2.u - p3.u) * t2
						 + (-p0.u + 3.0 * p1.u - 3.0 * p2.u + p3.u) * t3);
			q.v = 0.5 * ((2.0 * p1.v) + (-p0.v + p2.v) * t + (2.0 * p0.v - 5.0 * p1.v + 4.0 * p2.v - p3.v) * t2
						 + (-p0.v + 3.0 * p1.v - 3.0 * p2.v + p3.v) * t3);
			out.push_back(q);
		}
	}
}

void sketchSampleEllipse(const SkVec2& center, double majorR, double minorR, double angleRad, std::vector<SkVec2>& out,
						 int segs)
{
	out.clear();
	if (segs < 3 || majorR < 1e-12)
		return;
	const double minor = minorR > 1e-12 ? minorR : majorR;
	const double ca = std::cos(angleRad);
	const double sa = std::sin(angleRad);
	for (int i = 0; i <= segs; ++i)
	{
		const double t = 2.0 * 3.141592653589793 * i / segs;
		const double lx = majorR * std::cos(t);
		const double ly = minor * std::sin(t);
		out.push_back({center.u + lx * ca - ly * sa, center.v + lx * sa + ly * ca});
	}
}

bool offsetClosedUv(const std::vector<SkVec2>& poly, double dist, std::vector<SkVec2>& out, std::string* err)
{
	const std::size_t n = poly.size();
	if (n < 3)
	{
		if (err)
			*err = "need at least 3 points";
		return false;
	}

	double area2 = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t j = (i + 1) % n;
		area2 += poly[i].u * poly[j].v - poly[j].u * poly[i].v;
	}
	const bool ccw = area2 > 0.0;
	const double signedOff = (ccw ? 1.0 : -1.0) * dist;

	auto edgeNormal = [](const SkVec2& a, const SkVec2& b, bool left) -> SkVec2
	{
		const double dx = b.u - a.u;
		const double dy = b.v - a.v;
		const double len = std::sqrt(dx * dx + dy * dy);
		if (len < 1e-12)
			return {0.0, 0.0};
		const double nx = -dy / len;
		const double ny = dx / len;
		return left ? SkVec2{nx, ny} : SkVec2{-nx, -ny};
	};

	auto intersectLines = [](const SkVec2& p1, const SkVec2& d1, const SkVec2& p2, const SkVec2& d2, SkVec2& outPt) -> bool
	{
		const double cross = d1.u * d2.v - d1.v * d2.u;
		if (std::abs(cross) < 1e-12)
			return false;
		const double dx = p2.u - p1.u;
		const double dy = p2.v - p1.v;
		const double t = (dx * d2.v - dy * d2.u) / cross;
		outPt = {p1.u + d1.u * t, p1.v + d1.v * t};
		return true;
	};

	out.clear();
	out.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t im = (i + n - 1) % n;
		const std::size_t ip = (i + 1) % n;
		const SkVec2& p0 = poly[im];
		const SkVec2& p1 = poly[i];
		const SkVec2& p2 = poly[ip];

		const SkVec2 e0{p1.u - p0.u, p1.v - p0.v};
		const SkVec2 e1{p2.u - p1.u, p2.v - p1.v};
		const SkVec2 n0 = edgeNormal(p0, p1, true);
		const SkVec2 n1 = edgeNormal(p1, p2, true);
		const SkVec2 q0{p0.u + n0.u * signedOff, p0.v + n0.v * signedOff};
		const SkVec2 q1{p1.u + n1.u * signedOff, p1.v + n1.v * signedOff};

		SkVec2 vtx;
		if (!intersectLines(q0, e0, q1, e1, vtx))
			vtx = {p1.u + n1.u * signedOff, p1.v + n1.v * signedOff};
		out.push_back(vtx);
	}
	return out.size() >= 3;
}

bool closedPolylineSelfIntersectsUv(const std::vector<SkVec2>& poly, double eps)
{
	const std::size_t n = poly.size();
	if (n < 4)
		return false;

	auto orient = [eps](const SkVec2& a, const SkVec2& b, const SkVec2& c) -> int
	{
		const double v = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
		if (v > eps)
			return 1;
		if (v < -eps)
			return -1;
		return 0;
	};
	auto onSeg = [eps](const SkVec2& a, const SkVec2& b, const SkVec2& p) -> bool
	{
		return p.u >= std::min(a.u, b.u) - eps && p.u <= std::max(a.u, b.u) + eps &&
			   p.v >= std::min(a.v, b.v) - eps && p.v <= std::max(a.v, b.v) + eps;
	};
	auto segmentsIntersect = [&](const SkVec2& a1, const SkVec2& a2, const SkVec2& b1, const SkVec2& b2) -> bool
	{
		const int o1 = orient(a1, a2, b1);
		const int o2 = orient(a1, a2, b2);
		const int o3 = orient(b1, b2, a1);
		const int o4 = orient(b1, b2, a2);
		if (o1 != o2 && o3 != o4)
			return true;
		if (o1 == 0 && onSeg(a1, a2, b1))
			return true;
		if (o2 == 0 && onSeg(a1, a2, b2))
			return true;
		if (o3 == 0 && onSeg(b1, b2, a1))
			return true;
		if (o4 == 0 && onSeg(b1, b2, a2))
			return true;
		return false;
	};

	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t i2 = (i + 1) % n;
		for (std::size_t j = i + 1; j < n; ++j)
		{
			const std::size_t j2 = (j + 1) % n;
			if (i == j || i2 == j || i == j2 || i2 == j2)
				continue;
			if (segmentsIntersect(poly[i], poly[i2], poly[j], poly[j2]))
				return true;
		}
	}
	return false;
}

bool SketchDocument2d::sampleSplineUv(const SkSpline& sp, std::vector<SkVec2>& out, int segsPerSpan) const
{
	std::vector<SkVec2> through;
	through.reserve(sp.throughPts.size());
	for (int pid : sp.throughPts)
	{
		const SkPoint* p = findPoint(pid);
		if (!p)
			return false;
		through.push_back(p->p);
	}
	sketchSampleCatmullRom(through, out, segsPerSpan);
	return out.size() >= 2;
}

bool SketchDocument2d::isSplineThroughPoint(int pointId) const
{
	for (const SkSpline& sp : m_splines)
	{
		for (int tid : sp.throughPts)
		{
			if (tid == pointId)
				return true;
		}
	}
	return false;
}

namespace
{
double loopAreaAbs(const std::vector<SkVec2>& poly)
{
	if (poly.size() < 3)
		return 0.0;
	double a = 0.0;
	for (std::size_t i = 0; i < poly.size(); ++i)
	{
		const std::size_t j = (i + 1) % poly.size();
		a += poly[i].u * poly[j].v - poly[j].u * poly[i].v;
	}
	return std::abs(a) * 0.5;
}
} // namespace

bool SketchDocument2d::exportClosedProfilesUv(std::vector<std::vector<SkVec2>>& outLoops, std::string* err) const
{
	outLoops.clear();
	std::vector<std::vector<SkVec2>> uvLoops;

	auto countSolid = [](const auto& container)
	{
		int n = 0;
		for (const auto& item : container)
		{
			if (!item.construction)
				++n;
		}
		return n;
	};

	// 单圆/单椭圆轮廓快速路径
	{
		const int solidCircles = countSolid(m_circles);
		const int solidEllipses = countSolid(m_ellipses);
		const int solidLines = countSolid(m_lines);
		const int solidArcs = countSolid(m_arcs);
		const int solidSplines = countSolid(m_splines);
		if (solidLines == 0 && solidArcs == 0 && solidSplines == 0)
		{
			if (solidCircles == 1 && solidEllipses == 0)
			{
				const SkCircle* only = nullptr;
				for (const auto& c : m_circles)
				{
					if (!c.construction)
					{
						only = &c;
						break;
					}
				}
				const SkPoint* cen = only ? findPoint(only->center) : nullptr;
				if (!cen || only->radius < 1e-6)
				{
					if (err)
						*err = "invalid circle";
					return false;
				}
				std::vector<SkVec2> poly;
				sketchSampleEllipse(cen->p, only->radius, only->radius, 0.0, poly, 48);
				uvLoops.push_back(std::move(poly));
			}
			else if (solidEllipses == 1 && solidCircles == 0)
			{
				const SkEllipse* only = nullptr;
				for (const auto& e : m_ellipses)
				{
					if (!e.construction)
					{
						only = &e;
						break;
					}
				}
				const SkPoint* cen = only ? findPoint(only->center) : nullptr;
				if (!cen || only->majorR < 1e-6 || only->minorR < 1e-6)
				{
					if (err)
						*err = "invalid ellipse";
					return false;
				}
				std::vector<SkVec2> poly;
				sketchSampleEllipse(cen->p, only->majorR, only->minorR, only->angleRad, poly, 48);
				uvLoops.push_back(std::move(poly));
			}
		}
	}

	std::vector<const SkLine*> solidLines;
	for (const auto& ln : m_lines)
		if (!ln.construction)
			solidLines.push_back(&ln);

	std::vector<char> used(solidLines.size(), 0);
	for (std::size_t si = 0; si < solidLines.size(); ++si)
	{
		if (used[si])
			continue;
		const SkPoint* pa = findPoint(solidLines[si]->p1);
		const SkPoint* pb = findPoint(solidLines[si]->p2);
		if (!pa || !pb)
			continue;
		std::vector<SkVec2> poly;
		poly.push_back(pa->p);
		poly.push_back(pb->p);
		used[si] = 1;
		SkVec2 tip = pb->p;
		const SkVec2 start = pa->p;
		bool progressed = true;
		while (progressed)
		{
			progressed = false;
			for (std::size_t i = 0; i < solidLines.size(); ++i)
			{
				if (used[i])
					continue;
				const SkPoint* p1 = findPoint(solidLines[i]->p1);
				const SkPoint* p2 = findPoint(solidLines[i]->p2);
				if (!p1 || !p2)
					continue;
				if (skDist(tip, p1->p) < 1e-4)
				{
					poly.push_back(p2->p);
					tip = p2->p;
					used[i] = 1;
					progressed = true;
					break;
				}
				if (skDist(tip, p2->p) < 1e-4)
				{
					poly.push_back(p1->p);
					tip = p1->p;
					used[i] = 1;
					progressed = true;
					break;
				}
			}
		}
		if (poly.size() >= 3 && skDist(poly.front(), poly.back()) < 1e-3)
			poly.pop_back();
		if (poly.size() >= 3 && skDist(start, tip) < 1e-3)
			uvLoops.push_back(std::move(poly));
	}

	for (const auto& arc : m_arcs)
	{
		if (arc.construction)
			continue;
		const SkPoint* s = findPoint(arc.pStart);
		const SkPoint* m = findPoint(arc.pMid);
		const SkPoint* e = findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		std::vector<SkVec2> poly;
		appendArcSamples(poly, s->p, m->p, e->p, 24);
		if (poly.size() >= 3 && skDist(poly.front(), poly.back()) < 1e-3)
			uvLoops.push_back(std::move(poly));
	}

	for (const auto& el : m_ellipses)
	{
		if (el.construction)
			continue;
		const SkPoint* cen = findPoint(el.center);
		if (!cen || el.majorR < 1e-6 || el.minorR < 1e-6)
			continue;
		std::vector<SkVec2> poly;
		sketchSampleEllipse(cen->p, el.majorR, el.minorR, el.angleRad, poly, 48);
		if (poly.size() >= 3)
			uvLoops.push_back(std::move(poly));
	}

	if (uvLoops.empty())
	{
		if (err)
			*err = "no closed profile loop";
		return false;
	}

	std::sort(uvLoops.begin(), uvLoops.end(),
			  [](const std::vector<SkVec2>& a, const std::vector<SkVec2>& b)
			  { return loopAreaAbs(a) > loopAreaAbs(b); });
	outLoops = std::move(uvLoops);
	return !outLoops.empty();
}

bool SketchDocument2d::exportClosedProfilesXyz(const PluginSketchPlane& plane,
											 std::vector<std::vector<float>>& outLoops, std::string* err) const
{
	outLoops.clear();
	std::vector<std::vector<SkVec2>> uvLoops;
	if (!exportClosedProfilesUv(uvLoops, err))
		return false;

	for (const auto& poly : uvLoops)
	{
		std::vector<float> xyz;
		for (const auto& uv : poly)
			pushWorld(xyz, uvToWorld(plane, uv));
		if (xyz.size() >= 12)
			outLoops.push_back(std::move(xyz));
	}
	return !outLoops.empty();
}

bool SketchDocument2d::exportClosedProfileXyz(const PluginSketchPlane& plane, std::vector<float>& outXyzMm,
											  std::string* err) const
{
	outXyzMm.clear();
	std::vector<std::vector<float>> loops;
	if (!exportClosedProfilesXyz(plane, loops, err) || loops.empty())
		return false;
	outXyzMm = loops.front();
	return outXyzMm.size() >= 12;
}

bool SketchDocument2d::exportOpenPathXyz(const PluginSketchPlane& plane, std::vector<float>& outXyzMm,
										 std::string* err) const
{
	std::vector<PluginSketchSweepPathSegment> segs;
	if (!exportOpenPathSegments(plane, segs, err))
		return false;
	outXyzMm.clear();
	for (std::size_t i = 0; i < segs.size(); ++i)
	{
		const auto& s = segs[i];
		PluginPoint3d pa{s.ax, s.ay, s.az};
		PluginPoint3d pb{s.bx, s.by, s.bz};
		if (i == 0)
			pushWorld(outXyzMm, pa);
		if (s.kind == PluginSketchSweepPathSegKind::Arc)
		{
			PluginPoint3d pm{s.mx, s.my, s.mz};
			std::vector<SkVec2> samples;
			appendArcSamples(samples, worldToUv(plane, pa), worldToUv(plane, pm), worldToUv(plane, pb), 24);
			for (std::size_t k = 1; k < samples.size(); ++k)
				pushWorld(outXyzMm, uvToWorld(plane, samples[k]));
		}
		else
		{
			pushWorld(outXyzMm, pb);
		}
	}
	return outXyzMm.size() >= 6;
}

bool SketchDocument2d::exportOpenPathSegments(const PluginSketchPlane& plane,
											  std::vector<PluginSketchSweepPathSegment>& outSegs,
											  std::string* err) const
{
	outSegs.clear();
	struct SegUv
	{
		enum class Kind
		{
			Line = 0,
			Arc,
			Spline
		};
		Kind kind = Kind::Line;
		SkVec2 a{};
		SkVec2 b{};
		SkVec2 m{};
	};
	std::vector<SegUv> segs;
	for (const auto& ln : m_lines)
	{
		if (ln.construction)
			continue;
		const SkPoint* p1 = findPoint(ln.p1);
		const SkPoint* p2 = findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		segs.push_back({SegUv::Kind::Line, p1->p, p2->p, {}});
	}
	for (const auto& arc : m_arcs)
	{
		if (arc.construction)
			continue;
		const SkPoint* s = findPoint(arc.pStart);
		const SkPoint* m = findPoint(arc.pMid);
		const SkPoint* e = findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		segs.push_back({SegUv::Kind::Arc, s->p, e->p, m->p});
	}
	for (const auto& sp : m_splines)
	{
		if (sp.construction)
			continue;
		// 过点直接导出，避免密采样折线进 MakePipe 切成平面条带
		std::vector<SkVec2> through;
		through.reserve(sp.throughPts.size());
		bool ok = true;
		for (int pid : sp.throughPts)
		{
			const SkPoint* p = findPoint(pid);
			if (!p)
			{
				ok = false;
				break;
			}
			through.push_back(p->p);
		}
		if (!ok || through.size() < 2)
			continue;
		for (std::size_t i = 0; i + 1 < through.size(); ++i)
			segs.push_back({SegUv::Kind::Spline, through[i], through[i + 1], {}});
	}
	if (segs.empty())
	{
		if (err)
			*err = "path sketch has no lines/arcs/splines";
		return false;
	}

	std::vector<char> used(segs.size(), 0);
	std::vector<SegUv> ordered;
	ordered.push_back(segs[0]);
	used[0] = 1;
	SkVec2 head = segs[0].a;
	SkVec2 tip = segs[0].b;

	auto tryExtend = [&](SkVec2& endPt, bool prepend) -> bool
	{
		for (std::size_t i = 0; i < segs.size(); ++i)
		{
			if (used[i])
				continue;
			const bool atA = skDist(endPt, segs[i].a) < 1e-4;
			const bool atB = skDist(endPt, segs[i].b) < 1e-4;
			if (!atA && !atB)
				continue;
			used[i] = 1;
			SegUv chunk = segs[i];
			if (atB)
			{
				std::swap(chunk.a, chunk.b);
				endPt = segs[i].a;
			}
			else
			{
				endPt = segs[i].b;
			}
			if (prepend)
				ordered.insert(ordered.begin(), chunk);
			else
				ordered.push_back(chunk);
			return true;
		}
		return false;
	};

	while (tryExtend(tip, false))
	{
	}
	while (tryExtend(head, true))
	{
	}

	for (std::size_t i = 0; i < used.size(); ++i)
	{
		if (!used[i])
		{
			if (err)
				*err = "path sketch is branched or disconnected";
			return false;
		}
	}
	if (ordered.empty())
	{
		if (err)
			*err = "path needs >= 1 segment";
		return false;
	}
	// 闭环路径不能作 MakePipe spine
	if (skDist(head, tip) < 1e-3 && ordered.size() > 1)
	{
		if (err)
			*err = "path sketch must be open (not a closed loop)";
		return false;
	}

	for (const auto& uv : ordered)
	{
		PluginSketchSweepPathSegment s;
		if (uv.kind == SegUv::Kind::Arc)
			s.kind = PluginSketchSweepPathSegKind::Arc;
		else if (uv.kind == SegUv::Kind::Spline)
			s.kind = PluginSketchSweepPathSegKind::SplineThrough;
		else
			s.kind = PluginSketchSweepPathSegKind::Line;
		const PluginPoint3d wa = uvToWorld(plane, uv.a);
		const PluginPoint3d wb = uvToWorld(plane, uv.b);
		s.ax = static_cast<float>(wa.x);
		s.ay = static_cast<float>(wa.y);
		s.az = static_cast<float>(wa.z);
		s.bx = static_cast<float>(wb.x);
		s.by = static_cast<float>(wb.y);
		s.bz = static_cast<float>(wb.z);
		if (uv.kind == SegUv::Kind::Arc)
		{
			const PluginPoint3d wm = uvToWorld(plane, uv.m);
			s.mx = static_cast<float>(wm.x);
			s.my = static_cast<float>(wm.y);
			s.mz = static_cast<float>(wm.z);
		}
		outSegs.push_back(s);
	}
	return !outSegs.empty();
}

namespace
{
// 尺寸=琥珀金；选中/捕捉=亮黄；冲突红另走 colorForEntity，三者互不抢色
constexpr float kDimR = 1.00f, kDimG = 0.72f, kDimB = 0.12f;
constexpr float kHiR = 1.00f, kHiG = 0.92f, kHiB = 0.20f;

void appendStrokeGlyph(std::vector<std::vector<SkVec2>>& strokes, char ch, const SkVec2& origin, const SkVec2& xu,
					   const SkVec2& yu)
{
	auto P = [&](double x, double y) -> SkVec2
	{ return {origin.u + xu.u * x + yu.u * y, origin.v + xu.v * x + yu.v * y}; };
	auto add = [&](std::initializer_list<std::pair<double, double>> pts)
	{
		std::vector<SkVec2> s;
		s.reserve(pts.size());
		for (const auto& q : pts)
			s.push_back(P(q.first, q.second));
		if (s.size() >= 2)
			strokes.push_back(std::move(s));
	};
	switch (ch)
	{
	case '0':
		add({{0.15, 0.05}, {0.85, 0.05}, {0.85, 0.95}, {0.15, 0.95}, {0.15, 0.05}});
		break;
	case '1':
		add({{0.45, 0.05}, {0.45, 0.95}});
		add({{0.25, 0.75}, {0.45, 0.95}});
		break;
	case '2':
		add({{0.15, 0.95}, {0.85, 0.95}, {0.85, 0.55}, {0.15, 0.55}, {0.15, 0.05}, {0.85, 0.05}});
		break;
	case '3':
		add({{0.15, 0.95}, {0.85, 0.95}, {0.85, 0.55}, {0.35, 0.55}});
		add({{0.85, 0.55}, {0.85, 0.05}, {0.15, 0.05}});
		break;
	case '4':
		add({{0.15, 0.95}, {0.15, 0.55}, {0.85, 0.55}});
		add({{0.7, 0.95}, {0.7, 0.05}});
		break;
	case '5':
		add({{0.85, 0.95}, {0.15, 0.95}, {0.15, 0.55}, {0.85, 0.55}, {0.85, 0.05}, {0.15, 0.05}});
		break;
	case '6':
		add({{0.85, 0.95}, {0.15, 0.95}, {0.15, 0.05}, {0.85, 0.05}, {0.85, 0.55}, {0.15, 0.55}});
		break;
	case '7':
		add({{0.15, 0.95}, {0.85, 0.95}, {0.35, 0.05}});
		break;
	case '8':
		add({{0.15, 0.05}, {0.85, 0.05}, {0.85, 0.95}, {0.15, 0.95}, {0.15, 0.05}});
		add({{0.15, 0.55}, {0.85, 0.55}});
		break;
	case '9':
		add({{0.15, 0.05}, {0.85, 0.05}, {0.85, 0.95}, {0.15, 0.95}, {0.15, 0.55}, {0.85, 0.55}});
		break;
	case '.':
		add({{0.4, 0.05}, {0.6, 0.05}, {0.6, 0.2}, {0.4, 0.2}, {0.4, 0.05}});
		break;
	case '-':
		add({{0.2, 0.5}, {0.8, 0.5}});
		break;
	case 'R':
		add({{0.15, 0.05}, {0.15, 0.95}, {0.75, 0.95}, {0.75, 0.55}, {0.15, 0.55}});
		break;
	case 'd':
		add({{0.2, 0.05}, {0.2, 0.55}, {0.7, 0.55}, {0.7, 0.05}, {0.2, 0.05}});
		add({{0.45, 0.7}, {0.45, 0.95}});
		break;
	default:
		break;
	}
}

void appendStrokeText(std::vector<std::vector<SkVec2>>& strokes, const std::string& text, const SkVec2& center,
					  const SkVec2& xDirUnit, double height)
{
	if (text.empty() || height < 1e-6)
		return;
	const double width = height * 0.72;
	const double gap = height * 0.18;
	const double totalW = text.size() * width + (text.size() > 1 ? (text.size() - 1) * gap : 0.0);
	SkVec2 yu{-xDirUnit.v * height, xDirUnit.u * height};
	SkVec2 xu{xDirUnit.u * width, xDirUnit.v * width};
	SkVec2 origin{center.u - xDirUnit.u * totalW * 0.5 - yu.u * 0.5, center.v - xDirUnit.v * totalW * 0.5 - yu.v * 0.5};
	for (char ch : text)
	{
		appendStrokeGlyph(strokes, ch, origin, xu, yu);
		origin.u += xDirUnit.u * (width + gap);
		origin.v += xDirUnit.v * (width + gap);
	}
}
} // namespace

void SketchDocument2d::tessellateOverlay(const PluginSketchPlane& plane, std::vector<PluginSketchOverlaySegment>& out,
										 const SkVec2* previewA, const SkVec2* previewB, const SkSnapResult* snap,
										 const SkOverlayStyle* style, const std::vector<SkVec2>* previewPoly) const
{
	out.clear();
	const double bias = style ? style->normalBiasMm : 0.05;
	auto colorForEntity = [&](int entityId, bool construction) -> std::array<float, 4>
	{
		if (style && style->highlightEntityIds.count(entityId))
			return {kHiR, kHiG, kHiB, 1.0f};
		if (construction)
			return {0.70f, 0.70f, 0.72f, 1.0f};
		if (style && style->conflictEntityIds.count(entityId))
			return {1.0f, 0.22f, 0.18f, 1.0f};
		if (style && style->redundantEntityIds.count(entityId))
			return {1.0f, 0.55f, 0.15f, 1.0f};
		return {0.20f, 0.85f, 1.00f, 1.0f};
	};
	auto addSegColored = [&](const std::vector<SkVec2>& pts, const std::array<float, 4>& rgba, float widthPx,
							 bool asConstruction = false)
	{
		if (pts.size() < 2)
			return;
		PluginSketchOverlaySegment seg;
		seg.construction = asConstruction;
		seg.lineWidthPx = widthPx;
		seg.rgba[0] = rgba[0];
		seg.rgba[1] = rgba[1];
		seg.rgba[2] = rgba[2];
		seg.rgba[3] = rgba[3];
		for (const auto& uv : pts)
			pushWorld(seg.xyzMm, uvToWorld(plane, uv, bias));
		out.push_back(std::move(seg));
	};
	auto addDashed = [&](const SkVec2& a, const SkVec2& b, const std::array<float, 4>& rgba, float widthPx)
	{
		const double len = skDist(a, b);
		if (len < 1e-9)
			return;
		const double dash = std::clamp(len * 0.08, 1.2, 3.5);
		const double gap = dash * 0.7;
		const SkVec2 dir{(b.u - a.u) / len, (b.v - a.v) / len};
		double t = 0.0;
		while (t < len - 1e-9)
		{
			const double t1 = std::min(len, t + dash);
			addSegColored({{a.u + dir.u * t, a.v + dir.v * t}, {a.u + dir.u * t1, a.v + dir.v * t1}}, rgba, widthPx,
						  true);
			t = t1 + gap;
		}
	};
	auto addSeg = [&](const std::vector<SkVec2>& pts, bool construction, int entityId, float widthPx = 0.0f)
	{
		const bool hi = style && style->highlightEntityIds.count(entityId);
		const auto rgba = colorForEntity(entityId, construction);
		const float w = widthPx > 0.1f ? widthPx : (hi ? 4.5f : (construction ? 1.8f : 2.8f));
		if (construction && pts.size() == 2)
		{
			addDashed(pts[0], pts[1], rgba, w);
			return;
		}
		if (construction && pts.size() > 2)
		{
			for (std::size_t i = 0; i + 1 < pts.size(); ++i)
				addDashed(pts[i], pts[i + 1], rgba, w);
			return;
		}
		addSegColored(pts, rgba, w, false);
	};

	for (const auto& ln : m_lines)
	{
		const SkPoint* p1 = findPoint(ln.p1);
		const SkPoint* p2 = findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		addSeg({p1->p, p2->p}, ln.construction, ln.id);
	}
	for (const auto& arc : m_arcs)
	{
		const SkPoint* s = findPoint(arc.pStart);
		const SkPoint* m = findPoint(arc.pMid);
		const SkPoint* e = findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		std::vector<SkVec2> samples;
		appendArcSamples(samples, s->p, m->p, e->p, 24);
		addSeg(samples, arc.construction, arc.id);
	}
	for (const auto& c : m_circles)
	{
		const SkPoint* cen = findPoint(c.center);
		if (!cen)
			continue;
		std::vector<SkVec2> samples;
		sketchSampleEllipse(cen->p, c.radius, c.radius, 0.0, samples, 48);
		addSeg(samples, c.construction, c.id);
	}
	for (const auto& el : m_ellipses)
	{
		const SkPoint* cen = findPoint(el.center);
		if (!cen)
			continue;
		std::vector<SkVec2> samples;
		sketchSampleEllipse(cen->p, el.majorR, el.minorR, el.angleRad, samples, 48);
		addSeg(samples, el.construction, el.id);
	}
	for (const auto& sp : m_splines)
	{
		std::vector<SkVec2> samples;
		if (!sampleSplineUv(sp, samples, 12))
			continue;
		addSeg(samples, sp.construction, sp.id);
		// 过点手柄，便于拖拽
		for (int pid : sp.throughPts)
		{
			const SkPoint* pt = findPoint(pid);
			if (!pt)
				continue;
			const double s = pt->fixed ? 2.8 : 2.2;
			const std::array<float, 4> rgba =
				pt->fixed ? std::array<float, 4>{0.95f, 0.45f, 0.15f, 1.0f}
						  : std::array<float, 4>{0.95f, 0.95f, 0.98f, 1.0f};
			addSegColored({{pt->p.u - s, pt->p.v - s},
						   {pt->p.u + s, pt->p.v - s},
						   {pt->p.u + s, pt->p.v + s},
						   {pt->p.u - s, pt->p.v + s},
						   {pt->p.u - s, pt->p.v - s}},
						  rgba, 2.0f);
		}
	}

	const std::array<float, 4> dimRgba{kDimR, kDimG, kDimB, 1.0f};
	auto emitDimStrokes = [&](const std::vector<std::vector<SkVec2>>& strokes, float w)
	{
		for (const auto& s : strokes)
			addSegColored(s, dimRgba, w);
	};

	for (const auto& c : m_constraints)
	{
		if (c.kind == SkConstraintKind::Distance)
		{
			const SkPoint* p1 = findPoint(c.a);
			const SkPoint* p2 = findPoint(c.b);
			if (!p1 || !p2)
				continue;
			const double len = skDist(p1->p, p2->p);
			if (len < 1e-9)
				continue;
			const SkVec2 dir{(p2->p.u - p1->p.u) / len, (p2->p.v - p1->p.v) / len};
			const SkVec2 n{-dir.v, dir.u};
			const double off = std::clamp(len * 0.12, 4.0, 14.0);
			const double tick = std::clamp(len * 0.04, 1.5, 4.0);
			const double arrow = std::clamp(len * 0.05, 1.8, 5.0);
			const SkVec2 a0{p1->p.u + n.u * off, p1->p.v + n.v * off};
			const SkVec2 a1{p2->p.u + n.u * off, p2->p.v + n.v * off};
			addSegColored({p1->p, {p1->p.u + n.u * (off + tick * 0.4), p1->p.v + n.v * (off + tick * 0.4)}}, dimRgba,
						  2.2f);
			addSegColored({p2->p, {p2->p.u + n.u * (off + tick * 0.4), p2->p.v + n.v * (off + tick * 0.4)}}, dimRgba,
						  2.2f);
			addSegColored({a0, a1}, dimRgba, 3.2f);
			addSegColored({a0, {a0.u + dir.u * arrow + n.u * tick * 0.45, a0.v + dir.v * arrow + n.v * tick * 0.45}},
						  dimRgba, 2.8f);
			addSegColored({a0, {a0.u + dir.u * arrow - n.u * tick * 0.45, a0.v + dir.v * arrow - n.v * tick * 0.45}},
						  dimRgba, 2.8f);
			addSegColored({a1, {a1.u - dir.u * arrow + n.u * tick * 0.45, a1.v - dir.v * arrow + n.v * tick * 0.45}},
						  dimRgba, 2.8f);
			addSegColored({a1, {a1.u - dir.u * arrow - n.u * tick * 0.45, a1.v - dir.v * arrow - n.v * tick * 0.45}},
						  dimRgba, 2.8f);
			const SkVec2 mid = skLerp(a0, a1, 0.5);
			const double th = std::clamp(len * 0.07, 3.0, 8.0);
			std::vector<std::vector<SkVec2>> glyphs;
			char buf[32];
			std::snprintf(buf, sizeof(buf), "%.2f", c.value);
			appendStrokeText(glyphs, buf, mid, dir, th);
			emitDimStrokes(glyphs, 2.6f);
		}
		else if (c.kind == SkConstraintKind::Radius || c.kind == SkConstraintKind::ArcRadius)
		{
			SkVec2 cen{}, rim{};
			bool ok = false;
			if (c.kind == SkConstraintKind::Radius)
			{
				const SkCircle* cir = findCircle(c.a);
				const SkPoint* cp = cir ? findPoint(cir->center) : nullptr;
				if (cp)
				{
					cen = cp->p;
					rim = {cen.u + c.value, cen.v};
					ok = true;
				}
			}
			else
			{
				const SkArc* arc = findArc(c.a);
				const SkPoint* s = arc ? findPoint(arc->pStart) : nullptr;
				const SkPoint* m = arc ? findPoint(arc->pMid) : nullptr;
				const SkPoint* e = arc ? findPoint(arc->pEnd) : nullptr;
				double r = 0.0;
				if (s && m && e && circumcenter(s->p, m->p, e->p, cen, r))
				{
					const double ang = std::atan2(m->p.v - cen.v, m->p.u - cen.u);
					rim = {cen.u + c.value * std::cos(ang), cen.v + c.value * std::sin(ang)};
					ok = true;
				}
			}
			if (!ok)
				continue;
			const double len = skDist(cen, rim);
			if (len < 1e-9)
				continue;
			const SkVec2 dir{(rim.u - cen.u) / len, (rim.v - cen.v) / len};
			addSegColored({cen, rim}, dimRgba, 3.0f);
			const double arrow = std::clamp(len * 0.08, 1.5, 4.0);
			const SkVec2 n{-dir.v, dir.u};
			addSegColored({rim, {rim.u - dir.u * arrow + n.u * arrow * 0.4, rim.v - dir.v * arrow + n.v * arrow * 0.4}},
						  dimRgba, 2.6f);
			addSegColored({rim, {rim.u - dir.u * arrow - n.u * arrow * 0.4, rim.v - dir.v * arrow - n.v * arrow * 0.4}},
						  dimRgba, 2.6f);
			const SkVec2 mid = skLerp(cen, rim, 0.55);
			const double th = std::clamp(len * 0.1, 2.8, 7.0);
			std::vector<std::vector<SkVec2>> glyphs;
			char buf[32];
			std::snprintf(buf, sizeof(buf), "R%.2f", c.value);
			appendStrokeText(glyphs, buf, mid, dir, th);
			emitDimStrokes(glyphs, 2.6f);
		}
		else if (c.kind == SkConstraintKind::Angle)
		{
			const SkLine* l1 = findLine(c.a);
			const SkLine* l2 = findLine(c.b);
			const SkPoint* a1 = l1 ? findPoint(l1->p1) : nullptr;
			const SkPoint* a2 = l1 ? findPoint(l1->p2) : nullptr;
			const SkPoint* b1 = l2 ? findPoint(l2->p1) : nullptr;
			const SkPoint* b2 = l2 ? findPoint(l2->p2) : nullptr;
			if (!a1 || !a2 || !b1 || !b2)
				continue;
			SkVec2 apex = a1->p;
			double best = 1e99;
			for (const SkVec2& p : {a1->p, a2->p})
				for (const SkVec2& q : {b1->p, b2->p})
				{
					const double d = skDist(p, q);
					if (d < best)
					{
						best = d;
						apex = skLerp(p, q, 0.5);
					}
				}
			const double rr = 10.0;
			const double ang1 = std::atan2(a2->p.v - a1->p.v, a2->p.u - a1->p.u);
			const double ang2 = std::atan2(b2->p.v - b1->p.v, b2->p.u - b1->p.u);
			std::vector<SkVec2> arcPts;
			constexpr int n = 20;
			for (int i = 0; i <= n; ++i)
			{
				const double t = static_cast<double>(i) / n;
				const double a = ang1 + (ang2 - ang1) * t;
				arcPts.push_back({apex.u + rr * std::cos(a), apex.v + rr * std::sin(a)});
			}
			addSegColored(arcPts, dimRgba, 3.0f);
			const double amid = (ang1 + ang2) * 0.5;
			const SkVec2 labelPos{apex.u + (rr + 4.0) * std::cos(amid), apex.v + (rr + 4.0) * std::sin(amid)};
			const SkVec2 xdir{std::cos(amid + 1.57079632679), std::sin(amid + 1.57079632679)};
			std::vector<std::vector<SkVec2>> glyphs;
			char buf[32];
			std::snprintf(buf, sizeof(buf), "%.1fd", c.value);
			appendStrokeText(glyphs, buf, labelPos, xdir, 4.0);
			emitDimStrokes(glyphs, 2.6f);
		}
	}
	if (previewPoly && previewPoly->size() >= 2)
		addSeg(*previewPoly, true, -1, 2.2f);
	else if (previewA && previewB)
		addSeg({*previewA, *previewB}, true, -1, 2.2f);
	if (style)
	{
		for (int id : style->highlightEntityIds)
		{
			const SkPoint* pt = findPoint(id);
			if (!pt)
				continue;
			const double s = 3.5;
			addSegColored({{pt->p.u - s, pt->p.v}, {pt->p.u + s, pt->p.v}}, {kHiR, kHiG, kHiB, 1.0f}, 3.5f);
			addSegColored({{pt->p.u, pt->p.v - s}, {pt->p.u, pt->p.v + s}}, {kHiR, kHiG, kHiB, 1.0f}, 3.5f);
			addSegColored({{pt->p.u - s * 0.7, pt->p.v - s * 0.7}, {pt->p.u + s * 0.7, pt->p.v + s * 0.7}},
						  {kHiR, kHiG, kHiB, 1.0f}, 2.5f);
		}
	}
	if (snap && snap->snapped)
	{
		double rOuter = 2.8;
		double rInner = 1.1;
		switch (snap->kind)
		{
		case SkSnapKind::Endpoint:
			rOuter = 3.4;
			rInner = 1.4;
			break;
		case SkSnapKind::Midpoint:
			rOuter = 2.8;
			rInner = 1.1;
			break;
		case SkSnapKind::Center:
			rOuter = 3.0;
			rInner = 1.2;
			break;
		default:
			rOuter = 2.4;
			rInner = 0.9;
			break;
		}
		std::vector<SkVec2> ring;
		constexpr int n = 28;
		for (int i = 0; i <= n; ++i)
		{
			const double a = 2.0 * 3.141592653589793 * i / n;
			ring.push_back({snap->pos.u + rOuter * std::cos(a), snap->pos.v + rOuter * std::sin(a)});
		}
		addSegColored(ring, {kHiR, kHiG, kHiB, 1.0f}, 3.2f);
		std::vector<SkVec2> disc;
		for (int i = 0; i <= n; ++i)
		{
			const double a = 2.0 * 3.141592653589793 * i / n;
			disc.push_back({snap->pos.u + rInner * std::cos(a), snap->pos.v + rInner * std::sin(a)});
		}
		addSegColored(disc, {kHiR, kHiG, kHiB, 1.0f}, 2.4f);
		if (snap->kind == SkSnapKind::Center || snap->kind == SkSnapKind::Midpoint)
		{
			const double s = rOuter * 0.85;
			addSegColored({{snap->pos.u - s, snap->pos.v}, {snap->pos.u + s, snap->pos.v}}, {kHiR, kHiG, kHiB, 1.0f},
						  2.2f);
			addSegColored({{snap->pos.u, snap->pos.v - s}, {snap->pos.u, snap->pos.v + s}}, {kHiR, kHiG, kHiB, 1.0f},
						  2.2f);
		}
	}
}

SkSnapResult sketchSnap(const SketchDocument2d& doc, const SkVec2& raw, double tolMm, double gridMm,
						const SkVec2* refForOrtho)
{
	SkSnapResult best;
	double bestD = tolMm;

	auto consider = [&](const SkVec2& p, SkSnapKind kind)
	{
		const double d = skDist(raw, p);
		if (d <= bestD)
		{
			bestD = d;
			best.snapped = true;
			best.kind = kind;
			best.pos = p;
		}
	};

	for (const auto& pt : doc.points())
		consider(pt.p, SkSnapKind::Endpoint);

	for (const auto& ln : doc.lines())
	{
		const SkPoint* p1 = doc.findPoint(ln.p1);
		const SkPoint* p2 = doc.findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		consider(skLerp(p1->p, p2->p, 0.5), SkSnapKind::Midpoint);
		// OnCurve：投影到线段
		const double dx = p2->p.u - p1->p.u;
		const double dy = p2->p.v - p1->p.v;
		const double len2 = dx * dx + dy * dy;
		if (len2 > 1e-12)
		{
			double t = ((raw.u - p1->p.u) * dx + (raw.v - p1->p.v) * dy) / len2;
			t = std::max(0.0, std::min(1.0, t));
			consider(skLerp(p1->p, p2->p, t), SkSnapKind::OnCurve);
		}
	}
	for (const auto& c : doc.circles())
	{
		const SkPoint* cen = doc.findPoint(c.center);
		if (cen)
			consider(cen->p, SkSnapKind::Center);
	}

	if (refForOrtho)
	{
		const SkVec2 h{raw.u, refForOrtho->v};
		const SkVec2 v{refForOrtho->u, raw.v};
		if (skDist(raw, h) < bestD)
		{
			bestD = skDist(raw, h);
			best = {true, SkSnapKind::Horizontal, h};
		}
		if (skDist(raw, v) < bestD)
		{
			bestD = skDist(raw, v);
			best = {true, SkSnapKind::Vertical, v};
		}
	}

	if (gridMm > 1e-6)
	{
		const SkVec2 g{std::round(raw.u / gridMm) * gridMm, std::round(raw.v / gridMm) * gridMm};
		if (skDist(raw, g) < bestD)
			best = {true, SkSnapKind::Grid, g};
	}

	if (!best.snapped)
		best.pos = raw;
	return best;
}

bool sketchCircumcenter(const SkVec2& a, const SkVec2& b, const SkVec2& c, SkVec2& out, double& radius)
{
	return circumcenter(a, b, c, out, radius);
}

QString SketchDocument2d::constraintLabel(const SkConstraint& c) const
{
	switch (c.kind)
	{
	case SkConstraintKind::Distance:
		return QStringLiteral("距离 %1 mm").arg(c.value, 0, 'f', 2);
	case SkConstraintKind::Radius:
		return QStringLiteral("半径 %1 mm").arg(c.value, 0, 'f', 2);
	case SkConstraintKind::ArcRadius:
		return QStringLiteral("圆弧半径 %1 mm").arg(c.value, 0, 'f', 2);
	case SkConstraintKind::Angle:
		return QStringLiteral("角度 %1°").arg(c.value, 0, 'f', 2);
	case SkConstraintKind::Horizontal:
		return QStringLiteral("水平");
	case SkConstraintKind::Vertical:
		return QStringLiteral("竖直");
	case SkConstraintKind::EqualLength:
		return QStringLiteral("等长");
	case SkConstraintKind::Parallel:
		return QStringLiteral("平行");
	case SkConstraintKind::Perpendicular:
		return QStringLiteral("垂直");
	case SkConstraintKind::Coincident:
		return QStringLiteral("重合");
	case SkConstraintKind::Tangent:
		return QStringLiteral("相切");
	case SkConstraintKind::Symmetric:
		return QStringLiteral("对称");
	case SkConstraintKind::Midpoint:
		return QStringLiteral("中点");
	}
	return QStringLiteral("约束");
}

int SketchDocument2d::hitTestPoint(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& p : m_points)
	{
		const double d = skDist(uv, p.p);
		if (d <= bestD)
		{
			bestD = d;
			best = p.id;
		}
	}
	return best;
}

int SketchDocument2d::hitTestLine(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& ln : m_lines)
	{
		const SkPoint* p1 = findPoint(ln.p1);
		const SkPoint* p2 = findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		const SkVec2 a = p1->p;
		const SkVec2 b = p2->p;
		const double dx = b.u - a.u;
		const double dy = b.v - a.v;
		const double len2 = dx * dx + dy * dy;
		if (len2 < 1e-18)
			continue;
		double t = ((uv.u - a.u) * dx + (uv.v - a.v) * dy) / len2;
		t = std::max(0.0, std::min(1.0, t));
		const SkVec2 q{a.u + t * dx, a.v + t * dy};
		const double d = skDist(uv, q);
		if (d <= bestD)
		{
			bestD = d;
			best = ln.id;
		}
	}
	return best;
}

int SketchDocument2d::hitTestCircle(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& c : m_circles)
	{
		const SkPoint* cen = findPoint(c.center);
		if (!cen)
			continue;
		const double d = std::abs(skDist(uv, cen->p) - c.radius);
		if (d <= bestD)
		{
			bestD = d;
			best = c.id;
		}
	}
	return best;
}

int SketchDocument2d::hitTestEllipse(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& el : m_ellipses)
	{
		const SkPoint* cen = findPoint(el.center);
		if (!cen || el.majorR < 1e-9)
			continue;
		const double ca = std::cos(-el.angleRad);
		const double sa = std::sin(-el.angleRad);
		const double du = uv.u - cen->p.u;
		const double dv = uv.v - cen->p.v;
		const double lu = du * ca - dv * sa;
		const double lv = du * sa + dv * ca;
		const double minor = el.minorR > 1e-9 ? el.minorR : el.majorR;
		const double norm = (lu * lu) / (el.majorR * el.majorR) + (lv * lv) / (minor * minor);
		const double d = std::abs(std::sqrt(std::max(0.0, norm)) - 1.0) * el.majorR;
		if (d <= bestD)
		{
			bestD = d;
			best = el.id;
		}
	}
	return best;
}

int SketchDocument2d::hitTestArc(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& arc : m_arcs)
	{
		const SkPoint* s = findPoint(arc.pStart);
		const SkPoint* m = findPoint(arc.pMid);
		const SkPoint* e = findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		SkVec2 cen;
		double r = 0.0;
		if (!circumcenter(s->p, m->p, e->p, cen, r))
			continue;
		const double d = std::abs(skDist(uv, cen) - r);
		if (d <= bestD)
		{
			bestD = d;
			best = arc.id;
		}
	}
	return best;
}

int SketchDocument2d::hitTestSpline(const SkVec2& uv, double tolMm) const
{
	int best = -1;
	double bestD = tolMm;
	for (const auto& sp : m_splines)
	{
		std::vector<SkVec2> samples;
		if (!sampleSplineUv(sp, samples, 16) || samples.size() < 2)
			continue;
		for (std::size_t i = 0; i + 1 < samples.size(); ++i)
		{
			const SkVec2& a = samples[i];
			const SkVec2& b = samples[i + 1];
			const double dx = b.u - a.u;
			const double dy = b.v - a.v;
			const double len2 = dx * dx + dy * dy;
			if (len2 < 1e-18)
				continue;
			double t = ((uv.u - a.u) * dx + (uv.v - a.v) * dy) / len2;
			t = std::max(0.0, std::min(1.0, t));
			const SkVec2 q{a.u + t * dx, a.v + t * dy};
			const double d = skDist(uv, q);
			if (d <= bestD)
			{
				bestD = d;
				best = sp.id;
			}
		}
	}
	return best;
}

QByteArray SketchDocument2d::toJsonUtf8() const
{
	QJsonObject root;
	root.insert(QStringLiteral("seq"), m_seq);
	QJsonArray pts;
	for (const auto& p : m_points)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), p.id);
		o.insert(QStringLiteral("u"), p.p.u);
		o.insert(QStringLiteral("v"), p.p.v);
		o.insert(QStringLiteral("fixed"), p.fixed);
		pts.append(o);
	}
	root.insert(QStringLiteral("points"), pts);
	QJsonArray lines;
	for (const auto& ln : m_lines)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), ln.id);
		o.insert(QStringLiteral("p1"), ln.p1);
		o.insert(QStringLiteral("p2"), ln.p2);
		o.insert(QStringLiteral("construction"), ln.construction);
		lines.append(o);
	}
	root.insert(QStringLiteral("lines"), lines);
	QJsonArray arcs;
	for (const auto& a : m_arcs)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), a.id);
		o.insert(QStringLiteral("pStart"), a.pStart);
		o.insert(QStringLiteral("pMid"), a.pMid);
		o.insert(QStringLiteral("pEnd"), a.pEnd);
		o.insert(QStringLiteral("construction"), a.construction);
		arcs.append(o);
	}
	root.insert(QStringLiteral("arcs"), arcs);
	QJsonArray circles;
	for (const auto& c : m_circles)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), c.id);
		o.insert(QStringLiteral("center"), c.center);
		o.insert(QStringLiteral("radius"), c.radius);
		o.insert(QStringLiteral("construction"), c.construction);
		circles.append(o);
	}
	root.insert(QStringLiteral("circles"), circles);
	QJsonArray ellipses;
	for (const auto& e : m_ellipses)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), e.id);
		o.insert(QStringLiteral("center"), e.center);
		o.insert(QStringLiteral("majorR"), e.majorR);
		o.insert(QStringLiteral("minorR"), e.minorR);
		o.insert(QStringLiteral("angleRad"), e.angleRad);
		o.insert(QStringLiteral("construction"), e.construction);
		ellipses.append(o);
	}
	root.insert(QStringLiteral("ellipses"), ellipses);
	QJsonArray splines;
	for (const auto& sp : m_splines)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), sp.id);
		QJsonArray ids;
		for (int pid : sp.throughPts)
			ids.append(pid);
		o.insert(QStringLiteral("points"), ids);
		o.insert(QStringLiteral("construction"), sp.construction);
		splines.append(o);
	}
	root.insert(QStringLiteral("splines"), splines);
	QJsonArray cons;
	for (const auto& c : m_constraints)
	{
		QJsonObject o;
		o.insert(QStringLiteral("kind"), static_cast<int>(c.kind));
		o.insert(QStringLiteral("a"), c.a);
		o.insert(QStringLiteral("b"), c.b);
		o.insert(QStringLiteral("value"), c.value);
		if (c.c >= 0)
			o.insert(QStringLiteral("c"), c.c);
		cons.append(o);
	}
	root.insert(QStringLiteral("constraints"), cons);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool SketchDocument2d::fromJsonUtf8(const QByteArray& utf8)
{
	QJsonParseError pe{};
	const QJsonDocument doc = QJsonDocument::fromJson(utf8, &pe);
	if (pe.error != QJsonParseError::NoError || !doc.isObject())
		return false;
	clear();
	const QJsonObject root = doc.object();
	m_seq = root.value(QStringLiteral("seq")).toInt(1);
	for (const QJsonValue& v : root.value(QStringLiteral("points")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkPoint p;
		p.id = o.value(QStringLiteral("id")).toInt();
		p.p.u = o.value(QStringLiteral("u")).toDouble();
		p.p.v = o.value(QStringLiteral("v")).toDouble();
		p.fixed = o.value(QStringLiteral("fixed")).toBool();
		m_points.push_back(p);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("lines")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkLine ln;
		ln.id = o.value(QStringLiteral("id")).toInt();
		ln.p1 = o.value(QStringLiteral("p1")).toInt();
		ln.p2 = o.value(QStringLiteral("p2")).toInt();
		ln.construction = o.value(QStringLiteral("construction")).toBool(false);
		m_lines.push_back(ln);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("arcs")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkArc a;
		a.id = o.value(QStringLiteral("id")).toInt();
		a.pStart = o.value(QStringLiteral("pStart")).toInt();
		a.pMid = o.value(QStringLiteral("pMid")).toInt();
		a.pEnd = o.value(QStringLiteral("pEnd")).toInt();
		a.construction = o.value(QStringLiteral("construction")).toBool(false);
		m_arcs.push_back(a);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("circles")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkCircle c;
		c.id = o.value(QStringLiteral("id")).toInt();
		c.center = o.value(QStringLiteral("center")).toInt();
		c.radius = o.value(QStringLiteral("radius")).toDouble();
		c.construction = o.value(QStringLiteral("construction")).toBool(false);
		m_circles.push_back(c);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("ellipses")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkEllipse e;
		e.id = o.value(QStringLiteral("id")).toInt();
		e.center = o.value(QStringLiteral("center")).toInt();
		e.majorR = o.value(QStringLiteral("majorR")).toDouble();
		e.minorR = o.value(QStringLiteral("minorR")).toDouble();
		e.angleRad = o.value(QStringLiteral("angleRad")).toDouble();
		e.construction = o.value(QStringLiteral("construction")).toBool(false);
		m_ellipses.push_back(e);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("splines")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkSpline sp;
		sp.id = o.value(QStringLiteral("id")).toInt();
		for (const QJsonValue& pv : o.value(QStringLiteral("points")).toArray())
			sp.throughPts.push_back(pv.toInt());
		sp.construction = o.value(QStringLiteral("construction")).toBool(false);
		if (sp.throughPts.size() >= 2)
			m_splines.push_back(sp);
	}
	for (const QJsonValue& v : root.value(QStringLiteral("constraints")).toArray())
	{
		const QJsonObject o = v.toObject();
		SkConstraint c;
		c.kind = static_cast<SkConstraintKind>(o.value(QStringLiteral("kind")).toInt());
		c.a = o.value(QStringLiteral("a")).toInt();
		c.b = o.value(QStringLiteral("b")).toInt();
		c.value = o.value(QStringLiteral("value")).toDouble();
		c.c = o.value(QStringLiteral("c")).toInt(-1);
		m_constraints.push_back(c);
	}
	return true;
}

namespace
{
bool lineLineIntersect(const SkVec2& a1, const SkVec2& a2, const SkVec2& b1, const SkVec2& b2, double& ta, SkVec2& out)
{
	const double dxa = a2.u - a1.u;
	const double dya = a2.v - a1.v;
	const double dxb = b2.u - b1.u;
	const double dyb = b2.v - b1.v;
	const double den = dxa * dyb - dya * dxb;
	if (std::abs(den) < 1e-12)
		return false;
	const double dx = b1.u - a1.u;
	const double dy = b1.v - a1.v;
	ta = (dx * dyb - dy * dxb) / den;
	const double tb = (dx * dya - dy * dxa) / den;
	if (ta < 1e-6 || ta > 1.0 - 1e-6 || tb < 1e-6 || tb > 1.0 - 1e-6)
		return false;
	out = {a1.u + ta * dxa, a1.v + ta * dya};
	return true;
}

SkVec2 mirrorPointAcrossLine(const SkVec2& p, const SkVec2& a, const SkVec2& b)
{
	const double dx = b.u - a.u;
	const double dy = b.v - a.v;
	const double len2 = dx * dx + dy * dy;
	if (len2 < 1e-12)
		return p;
	const double t = ((p.u - a.u) * dx + (p.v - a.v) * dy) / len2;
	const SkVec2 proj{a.u + t * dx, a.v + t * dy};
	return {2.0 * proj.u - p.u, 2.0 * proj.v - p.v};
}
} // namespace

bool SketchDocument2d::trimLineAt(const SkVec2& uv, double tolMm)
{
	const int lid = hitTestLine(uv, tolMm);
	SkLine* target = findLine(lid);
	if (!target)
		return false;
	const SkPoint* p1 = findPoint(target->p1);
	const SkPoint* p2 = findPoint(target->p2);
	if (!p1 || !p2)
		return false;
	const double len = skDist(p1->p, p2->p);
	if (len < 1e-9)
		return false;
	const double clickT =
		std::clamp(((uv.u - p1->p.u) * (p2->p.u - p1->p.u) + (uv.v - p1->p.v) * (p2->p.v - p1->p.v)) / (len * len), 0.0,
				   1.0);

	std::vector<double> ts;
	ts.push_back(0.0);
	ts.push_back(1.0);
	for (const auto& other : m_lines)
	{
		if (other.id == target->id)
			continue;
		const SkPoint* o1 = findPoint(other.p1);
		const SkPoint* o2 = findPoint(other.p2);
		if (!o1 || !o2)
			continue;
		double ta = 0.0;
		SkVec2 hit;
		if (lineLineIntersect(p1->p, p2->p, o1->p, o2->p, ta, hit))
			ts.push_back(ta);
	}
	std::sort(ts.begin(), ts.end());
	ts.erase(std::unique(ts.begin(), ts.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }), ts.end());
	if (ts.size() < 3)
		return false;

	int seg = -1;
	for (std::size_t i = 0; i + 1 < ts.size(); ++i)
	{
		if (clickT >= ts[i] - 1e-9 && clickT <= ts[i + 1] + 1e-9)
		{
			seg = static_cast<int>(i);
			break;
		}
	}
	if (seg < 0)
		return false;

	const bool keepFront = seg > 0;
	const bool keepBack = seg + 1 < static_cast<int>(ts.size()) - 1;
	if (!keepFront && !keepBack)
	{
		removeLine(target->id);
		return true;
	}

	const bool constr = target->construction;
	const int oldP1 = target->p1;
	const int oldP2 = target->p2;
	removeLine(lid);

	auto pointAt = [&](double t) -> SkVec2
	{ return {p1->p.u + (p2->p.u - p1->p.u) * t, p1->p.v + (p2->p.v - p1->p.v) * t}; };

	if (keepFront)
	{
		const SkVec2 mid = pointAt(ts[static_cast<std::size_t>(seg)]);
		const int midId = addPoint(mid.u, mid.v);
		addLine(oldP1, midId, constr);
	}
	if (keepBack)
	{
		const SkVec2 mid = pointAt(ts[static_cast<std::size_t>(seg + 1)]);
		const int midId = addPoint(mid.u, mid.v);
		addLine(midId, oldP2, constr);
	}
	return true;
}

bool SketchDocument2d::trimSplineAt(const SkVec2& uv, double tolMm)
{
	const int sid = hitTestSpline(uv, tolMm);
	const SkSpline* target = findSpline(sid);
	if (!target || target->throughPts.size() < 2)
		return false;

	constexpr int segsPerSpan = 12;
	std::vector<SkVec2> samples;
	if (!sampleSplineUv(*target, samples, segsPerSpan) || samples.size() < 2)
		return false;

	double bestD = tolMm;
	std::size_t bestSeg = 0;
	SkVec2 bestQ = samples.front();
	for (std::size_t i = 0; i + 1 < samples.size(); ++i)
	{
		const SkVec2& a = samples[i];
		const SkVec2& b = samples[i + 1];
		const double dx = b.u - a.u;
		const double dy = b.v - a.v;
		const double len2 = dx * dx + dy * dy;
		if (len2 < 1e-18)
			continue;
		double t = ((uv.u - a.u) * dx + (uv.v - a.v) * dy) / len2;
		t = std::max(0.0, std::min(1.0, t));
		const SkVec2 q{a.u + t * dx, a.v + t * dy};
		const double d = skDist(uv, q);
		if (d <= bestD)
		{
			bestD = d;
			bestSeg = i;
			bestQ = q;
		}
	}
	if (bestD > tolMm)
		return false;

	const int span = static_cast<int>(bestSeg / static_cast<std::size_t>(segsPerSpan));
	const int nThru = static_cast<int>(target->throughPts.size());
	if (span < 0 || span >= nThru - 1)
		return false;

	int splitId = -1;
	const SkPoint* leftPt = findPoint(target->throughPts[static_cast<std::size_t>(span)]);
	const SkPoint* rightPt = findPoint(target->throughPts[static_cast<std::size_t>(span + 1)]);
	if (leftPt && skDist(bestQ, leftPt->p) < 1e-3)
		splitId = leftPt->id;
	else if (rightPt && skDist(bestQ, rightPt->p) < 1e-3)
		splitId = rightPt->id;
	else
		splitId = addPoint(bestQ.u, bestQ.v);

	std::vector<int> left;
	left.reserve(static_cast<std::size_t>(span) + 2);
	for (int i = 0; i <= span; ++i)
		left.push_back(target->throughPts[static_cast<std::size_t>(i)]);
	if (left.empty() || left.back() != splitId)
		left.push_back(splitId);

	std::vector<int> right;
	right.reserve(static_cast<std::size_t>(nThru - span));
	right.push_back(splitId);
	for (int i = span + 1; i < nThru; ++i)
		right.push_back(target->throughPts[static_cast<std::size_t>(i)]);

	const bool constr = target->construction;
	removeSpline(sid);
	bool any = false;
	if (left.size() >= 2)
	{
		addSpline(left, constr);
		any = true;
	}
	if (right.size() >= 2)
	{
		addSpline(right, constr);
		any = true;
	}
	return any;
}

bool SketchDocument2d::mirrorEntities(int mirrorLineId, const std::vector<int>& entityIds)
{
	const SkLine* axis = findLine(mirrorLineId);
	if (!axis)
		return false;
	const SkPoint* a = findPoint(axis->p1);
	const SkPoint* b = findPoint(axis->p2);
	if (!a || !b)
		return false;

	auto mapPt = [&](int pid) -> int
	{
		const SkPoint* p = findPoint(pid);
		if (!p)
			return -1;
		const SkVec2 m = mirrorPointAcrossLine(p->p, a->p, b->p);
		return addPoint(m.u, m.v, p->fixed);
	};

	bool any = false;
	for (int id : entityIds)
	{
		if (id == mirrorLineId)
			continue;
		if (const SkLine* ln = findLine(id))
		{
			const int np1 = mapPt(ln->p1);
			const int np2 = mapPt(ln->p2);
			if (np1 >= 0 && np2 >= 0)
			{
				addLine(np1, np2, ln->construction);
				any = true;
			}
		}
		else if (const SkCircle* cir = findCircle(id))
		{
			const int nc = mapPt(cir->center);
			if (nc >= 0)
			{
				addCircle(nc, cir->radius, cir->construction);
				any = true;
			}
		}
		else if (const SkEllipse* el = findEllipse(id))
		{
			const int nc = mapPt(el->center);
			const SkPoint* cen = nc >= 0 ? findPoint(nc) : nullptr;
			if (cen)
			{
				const double ca = std::cos(el->angleRad);
				const double sa = std::sin(el->angleRad);
				const SkVec2 majorTip{cen->p.u + el->majorR * ca, cen->p.v + el->majorR * sa};
				const SkVec2 mMajor = mirrorPointAcrossLine(majorTip, a->p, b->p);
				const double newAngle = std::atan2(mMajor.v - cen->p.v, mMajor.u - cen->p.u);
				addEllipse(nc, el->majorR, el->minorR, newAngle, el->construction);
				any = true;
			}
		}
		else if (const SkArc* arc = findArc(id))
		{
			const int ns = mapPt(arc->pStart);
			const int nm = mapPt(arc->pMid);
			const int ne = mapPt(arc->pEnd);
			if (ns >= 0 && nm >= 0 && ne >= 0)
			{
				addArc(ns, nm, ne, arc->construction);
				any = true;
			}
		}
		else if (const SkSpline* sp = findSpline(id))
		{
			std::vector<int> mirrored;
			mirrored.reserve(sp->throughPts.size());
			bool ok = true;
			for (int pid : sp->throughPts)
			{
				const int nid = mapPt(pid);
				if (nid < 0)
				{
					ok = false;
					break;
				}
				mirrored.push_back(nid);
			}
			if (ok && mirrored.size() >= 2)
			{
				addSpline(mirrored, sp->construction);
				any = true;
			}
		}
	}
	return any;
}
