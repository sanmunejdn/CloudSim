/// @file SheetSketchAdapter.cpp

#include "SheetSketchAdapter.h"

#include <QLineF>
#include <QRectF>
#include <QtGlobal>

#include <algorithm>
#include <cmath>

namespace
{
void appendArcUv(std::vector<SkVec2>& out, const SkVec2& s, const SkVec2& m, const SkVec2& e, int segs)
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
	auto norm = [](double a) {
		constexpr double kPi2 = 2.0 * 3.141592653589793;
		while (a < 0)
			a += kPi2;
		while (a >= kPi2)
			a -= kPi2;
		return a;
	};
	double a0 = norm(ang(s));
	double a1 = norm(ang(m));
	double a2 = norm(ang(e));
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

std::unique_ptr<ISketchTool> makeTool(SketchToolKind kind)
{
	switch (kind)
	{
	case SketchToolKind::Line:
		return std::make_unique<LineSketchTool>();
	case SketchToolKind::Arc:
		return std::make_unique<ArcSketchTool>();
	case SketchToolKind::Circle:
		return std::make_unique<CircleSketchTool>();
	case SketchToolKind::Rectangle:
		return std::make_unique<RectSketchTool>();
	case SketchToolKind::Spline:
		return std::make_unique<SplineSketchTool>();
	default:
		return nullptr;
	}
}
} // namespace

SkVec2 SheetSketchAdapter::toUv(const QPointF& scene)
{
	return {scene.x(), scene.y()};
}

QPointF SheetSketchAdapter::toScene(const SkVec2& uv)
{
	return QPointF(uv.u, uv.v);
}

void SheetSketchAdapter::clear()
{
	clearTool();
	m_doc.clear();
	m_entityLayer.clear();
	m_lastSnap = {};
}

void SheetSketchAdapter::setTool(SketchToolKind kind)
{
	m_tool = makeTool(kind);
}

void SheetSketchAdapter::clearTool()
{
	if (m_tool)
		m_tool->cancel();
	m_tool.reset();
}

void SheetSketchAdapter::cancelTool()
{
	if (m_tool)
		m_tool->cancel();
}

SkVec2 SheetSketchAdapter::snapScene(const QPointF& scene, double tolMm, const QVector<QPointF>& extra,
									 const SkVec2* refForOrtho) const
{
	const SkVec2 raw = toUv(scene);
	SkSnapResult best = sketchSnap(m_doc, raw, tolMm, 0.0, refForOrtho);
	double bestD = best.snapped ? skDist(raw, best.pos) : tolMm;
	for (const QPointF& ep : extra)
	{
		const SkVec2 p = toUv(ep);
		const double d = skDist(raw, p);
		if (d <= bestD)
		{
			bestD = d;
			best.snapped = true;
			best.kind = SkSnapKind::Endpoint;
			best.pos = p;
		}
	}
	m_lastSnap = best;
	return best.snapped ? best.pos : raw;
}

SkVec2 SheetSketchAdapter::applySnap(const QPointF& scene, double tolMm, const QVector<QPointF>& extra) const
{
	const SkVec2* ref = nullptr;
	SkVec2 refBuf{};
	if (m_tool)
	{
		if (const auto rp = m_tool->referencePoint())
		{
			refBuf = *rp;
			ref = &refBuf;
		}
	}
	return snapScene(scene, tolMm, extra, ref);
}

bool SheetSketchAdapter::press(const QPointF& scene, bool rightButton, double snapTolMm,
							   const QVector<QPointF>& extraSnap, const QString& layerId)
{
	if (!m_tool)
		return false;
	const int beforeMax = maxEntityId();
	const SkVec2 uv = rightButton ? toUv(scene) : applySnap(scene, snapTolMm, extraSnap);
	m_tool->onPress(uv, rightButton, m_doc);
	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > beforeMax)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

void SheetSketchAdapter::move(const QPointF& scene, double snapTolMm, const QVector<QPointF>& extraSnap)
{
	if (!m_tool)
		return;
	m_tool->onMove(applySnap(scene, snapTolMm, extraSnap));
}

QVector<SheetSketchPolyline> SheetSketchAdapter::tessellate() const
{
	QVector<SheetSketchPolyline> out;
	auto push = [&](const std::vector<SkVec2>& uv, bool construction, int entityId) {
		if (uv.size() < 2)
			return;
		SheetSketchPolyline poly;
		poly.construction = construction;
		poly.entityId = entityId;
		poly.points.reserve(static_cast<int>(uv.size()));
		for (const SkVec2& p : uv)
			poly.points.push_back(toScene(p));
		out.push_back(poly);
	};

	for (const auto& ln : m_doc.lines())
	{
		const SkPoint* p1 = m_doc.findPoint(ln.p1);
		const SkPoint* p2 = m_doc.findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		push({p1->p, p2->p}, ln.construction, ln.id);
	}
	for (const auto& arc : m_doc.arcs())
	{
		const SkPoint* s = m_doc.findPoint(arc.pStart);
		const SkPoint* m = m_doc.findPoint(arc.pMid);
		const SkPoint* e = m_doc.findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		std::vector<SkVec2> samples;
		appendArcUv(samples, s->p, m->p, e->p, 24);
		push(samples, arc.construction, arc.id);
	}
	for (const auto& c : m_doc.circles())
	{
		const SkPoint* cen = m_doc.findPoint(c.center);
		if (!cen)
			continue;
		std::vector<SkVec2> samples;
		constexpr int segs = 48;
		for (int i = 0; i <= segs; ++i)
		{
			const double a = 2.0 * 3.141592653589793 * i / segs;
			samples.push_back({cen->p.u + c.radius * std::cos(a), cen->p.v + c.radius * std::sin(a)});
		}
		push(samples, c.construction, c.id);
	}
	for (const auto& sp : m_doc.splines())
	{
		std::vector<SkVec2> samples;
		if (!m_doc.sampleSplineUv(sp, samples, 12))
			continue;
		push(samples, sp.construction, sp.id);
	}
	return out;
}

QVector<QPointF> SheetSketchAdapter::previewPolyline() const
{
	QVector<QPointF> out;
	if (!m_tool)
		return out;
	std::vector<SkVec2> poly;
	if (m_tool->previewPolyline(poly) && poly.size() >= 2)
	{
		for (const SkVec2& p : poly)
			out.push_back(toScene(p));
		return out;
	}
	SkVec2 a, b;
	if (m_tool->hasPreview(a, b))
	{
		if (m_tool->kind() == SketchToolKind::Rectangle)
		{
			out << toScene(a) << QPointF(b.u, a.v) << toScene(b) << QPointF(a.u, b.v) << toScene(a);
			return out;
		}
		if (m_tool->kind() == SketchToolKind::Circle)
		{
			const double r = skDist(a, b);
			constexpr int segs = 48;
			for (int i = 0; i <= segs; ++i)
			{
				const double ang = 2.0 * 3.141592653589793 * i / segs;
				out.push_back(QPointF(a.u + r * std::cos(ang), a.v + r * std::sin(ang)));
			}
			return out;
		}
		if (m_tool->kind() == SketchToolKind::Arc)
		{
			// 三步弧：预览用弦
			out << toScene(a) << toScene(b);
			return out;
		}
		out << toScene(a) << toScene(b);
	}
	return out;
}

int SheetSketchAdapter::hitTestEntity(const QPointF& scene, double tolMm) const
{
	return hitTestEntity(scene, tolMm, {});
}

int SheetSketchAdapter::hitTestEntity(const QPointF& scene, double tolMm,
									  const std::function<bool(int)>& accept) const
{
	const SkVec2 uv = toUv(scene);
	auto tryId = [&](int id) -> int {
		if (id < 0)
			return -1;
		if (accept && !accept(id))
			return -1;
		return id;
	};
	if (const int id = tryId(m_doc.hitTestCircle(uv, tolMm)); id >= 0)
		return id;
	if (const int id = tryId(m_doc.hitTestArc(uv, tolMm)); id >= 0)
		return id;
	if (const int id = tryId(m_doc.hitTestSpline(uv, tolMm)); id >= 0)
		return id;
	if (const int id = tryId(m_doc.hitTestLine(uv, tolMm)); id >= 0)
		return id;
	return -1;
}

bool SheetSketchAdapter::removeEntity(int id)
{
	m_entityLayer.remove(id);
	return m_doc.removeEntity(id);
}

QString SheetSketchAdapter::layerOf(int entityId) const
{
	return m_entityLayer.value(entityId, QStringLiteral("L0"));
}

void SheetSketchAdapter::setLayerOf(int entityId, const QString& layerId)
{
	if (entityId < 0)
		return;
	m_entityLayer.insert(entityId, layerId.isEmpty() ? QStringLiteral("L0") : layerId);
}

void SheetSketchAdapter::setEntityLayers(const QHash<int, QString>& map)
{
	m_entityLayer = map;
}

void SheetSketchAdapter::remapLayer(const QString& fromId, const QString& toId)
{
	for (auto it = m_entityLayer.begin(); it != m_entityLayer.end(); ++it)
	{
		if (it.value() == fromId)
			it.value() = toId;
	}
}

int SheetSketchAdapter::maxEntityId() const
{
	int m = 0;
	auto bump = [&](int id) {
		if (id > m)
			m = id;
	};
	for (const auto& ln : m_doc.lines())
		bump(ln.id);
	for (const auto& a : m_doc.arcs())
		bump(a.id);
	for (const auto& c : m_doc.circles())
		bump(c.id);
	for (const auto& e : m_doc.ellipses())
		bump(e.id);
	for (const auto& s : m_doc.splines())
		bump(s.id);
	return m;
}

void SheetSketchAdapter::collectEntityIds(QVector<int>& out) const
{
	out.clear();
	for (const auto& ln : m_doc.lines())
		out.push_back(ln.id);
	for (const auto& a : m_doc.arcs())
		out.push_back(a.id);
	for (const auto& c : m_doc.circles())
		out.push_back(c.id);
	for (const auto& e : m_doc.ellipses())
		out.push_back(e.id);
	for (const auto& s : m_doc.splines())
		out.push_back(s.id);
}

bool SheetSketchAdapter::fromJsonUtf8(const QByteArray& utf8)
{
	clearTool();
	m_entityLayer.clear();
	return m_doc.fromJsonUtf8(utf8);
}

bool SheetSketchAdapter::trimAt(const QPointF& scene, double tolMm)
{
	const SkVec2 uv = toUv(scene);
	return m_doc.trimLineAt(uv, tolMm) || m_doc.trimSplineAt(uv, tolMm);
}

bool SheetSketchAdapter::extendToBoundary(const QLineF& boundary, const QPointF& lineHit, double tolMm)
{
	if (boundary.length() < 1e-9)
		return false;
	struct Cand
	{
		int lineId = -1;
		int nearPtId = -1;
		int farPtId = -1;
		double dist = 1e100;
	};
	Cand best;
	for (const auto& ln : m_doc.lines())
	{
		SkPoint* p1 = m_doc.findPoint(ln.p1);
		SkPoint* p2 = m_doc.findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		QLineF seg(toScene(p1->p), toScene(p2->p));
		const QPointF ab = seg.p2() - seg.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(lineHit - seg.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		const double d = QLineF(seg.p1() + ab * t, lineHit).length();
		if (d > tolMm * 4.0 || d >= best.dist)
			continue;
		best.dist = d;
		best.lineId = ln.id;
		const double d1 = QLineF(toScene(p1->p), lineHit).length();
		const double d2 = QLineF(toScene(p2->p), lineHit).length();
		best.nearPtId = d1 <= d2 ? ln.p1 : ln.p2;
		best.farPtId = d1 <= d2 ? ln.p2 : ln.p1;
	}
	if (best.lineId < 0)
		return false;
	SkPoint* nearPt = m_doc.findPoint(best.nearPtId);
	SkPoint* farPt = m_doc.findPoint(best.farPtId);
	if (!nearPt || !farPt)
		return false;
	QLineF ray(toScene(farPt->p), toScene(nearPt->p));
	if (ray.length() < 1e-9)
		return false;
	ray.setLength(ray.length() * 1000.0);
	QPointF hit;
	QLineF bound(boundary.p1(), boundary.p2());
	bound.setLength(bound.length() * 1000.0);
	bound.setP1(boundary.p1() - (boundary.p2() - boundary.p1()) * 500.0);
	if (QLineF(toScene(farPt->p), ray.p2()).intersects(bound, &hit) == QLineF::NoIntersection)
		return false;
	const QPointF dir = toScene(nearPt->p) - toScene(farPt->p);
	const QPointF toHit = hit - toScene(farPt->p);
	if (QPointF::dotProduct(dir, toHit) < QPointF::dotProduct(dir, dir) - 1e-6)
		return false;
	nearPt->p = toUv(hit);
	return true;
}

bool SheetSketchAdapter::offsetClosedAt(const QPointF& scene, double distMm, double tolMm, const QString& layerId)
{
	const int eid = hitTestEntity(scene, tolMm);
	if (eid < 0)
		return false;

	QVector<QPointF> pts;
	for (const SheetSketchPolyline& poly : tessellate())
	{
		if (poly.entityId != eid)
			continue;
		pts = poly.points;
		break;
	}
	if (pts.size() < 2)
		return false;

	const bool closed = pts.size() >= 3 && QLineF(pts.first(), pts.last()).length() < tolMm * 2.0;
	const int beforeMax = maxEntityId();
	if (closed)
	{
		std::vector<SkVec2> loop;
		loop.reserve(static_cast<size_t>(pts.size()));
		for (const QPointF& p : pts)
			loop.push_back(toUv(p));
		if (!loop.empty() && skDist(loop.front(), loop.back()) > 1e-6)
			loop.push_back(loop.front());
		std::vector<SkVec2> out;
		std::string err;
		if (!offsetClosedUv(loop, distMm, out, &err) || out.size() < 2)
			return false;
		int prev = -1;
		for (std::size_t i = 0; i < out.size(); ++i)
		{
			const int pid = m_doc.addPoint(out[i].u, out[i].v);
			if (prev >= 0)
				m_doc.addLine(prev, pid, false);
			prev = pid;
		}
	}
	else
	{
		const QPointF a = pts.first();
		const QPointF b = pts.last();
		QLineF seg(a, b);
		if (seg.length() < 1e-9)
			return false;
		QLineF n = seg.normalVector();
		n.setLength(distMm);
		const QPointF o1 = a + n.p2() - n.p1();
		const QPointF o2 = b + n.p2() - n.p1();
		const int p1 = m_doc.addPoint(o1.x(), o1.y());
		const int p2 = m_doc.addPoint(o2.x(), o2.y());
		m_doc.addLine(p1, p2, false);
	}

	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > beforeMax)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

bool SheetSketchAdapter::filletLinesAt(const QPointF& scene, double radiusMm, double tolMm, const QString& layerId)
{
	if (!(radiusMm > 1e-6))
		return false;

	struct Edge
	{
		enum class Kind
		{
			Line,
			Arc
		} kind = Kind::Line;
		int id = -1;
		double dist = 1e100;
		QPointF a{}, b{}, mid{};
		QPointF nearPt{};
	};

	QVector<Edge> edges;
	const double hitTol = tolMm * 4.0;

	for (const auto& ln : m_doc.lines())
	{
		const SkPoint* p1 = m_doc.findPoint(ln.p1);
		const SkPoint* p2 = m_doc.findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		QLineF seg(toScene(p1->p), toScene(p2->p));
		const QPointF ab = seg.p2() - seg.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(scene - seg.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		const QPointF foot = seg.p1() + ab * t;
		const double d = QLineF(foot, scene).length();
		if (d > hitTol)
			continue;
		Edge e;
		e.kind = Edge::Kind::Line;
		e.id = ln.id;
		e.dist = d;
		e.a = seg.p1();
		e.b = seg.p2();
		e.nearPt = foot;
		edges.push_back(e);
	}
	for (const auto& arc : m_doc.arcs())
	{
		const SkPoint* s = m_doc.findPoint(arc.pStart);
		const SkPoint* m = m_doc.findPoint(arc.pMid);
		const SkPoint* ept = m_doc.findPoint(arc.pEnd);
		if (!s || !m || !ept)
			continue;
		std::vector<SkVec2> samples;
		appendArcUv(samples, s->p, m->p, ept->p, 32);
		double best = hitTol;
		QPointF near = toScene(s->p);
		for (size_t i = 1; i < samples.size(); ++i)
		{
			QLineF seg(toScene(samples[i - 1]), toScene(samples[i]));
			const QPointF ab = seg.p2() - seg.p1();
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scene - seg.p1(), ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			const QPointF foot = seg.p1() + ab * t;
			const double d = QLineF(foot, scene).length();
			if (d < best)
			{
				best = d;
				near = foot;
			}
		}
		if (best > hitTol)
			continue;
		Edge e;
		e.kind = Edge::Kind::Arc;
		e.id = arc.id;
		e.dist = best;
		e.a = toScene(s->p);
		e.mid = toScene(m->p);
		e.b = toScene(ept->p);
		e.nearPt = near;
		edges.push_back(e);
	}
	if (edges.size() < 2)
		return false;
	std::sort(edges.begin(), edges.end(), [](const Edge& x, const Edge& y) { return x.dist < y.dist; });
	const Edge E1 = edges[0];
	const Edge E2 = edges[1];

	auto dirToward = [](const QPointF& from, const QPointF& towardHint) {
		QLineF d(from, towardHint);
		if (d.length() < 1e-9)
			return QPointF(1, 0);
		d.setLength(1.0);
		return d.p2() - d.p1();
	};
	auto arcCenter = [](const Edge& e, QPointF& c, double& r) -> bool {
		SkVec2 out;
		if (!sketchCircumcenter(toUv(e.a), toUv(e.mid), toUv(e.b), out, r))
			return false;
		c = toScene(out);
		return r > 1e-9;
	};
	auto edgeDirAt = [&](const Edge& e, const QPointF& at) -> QPointF {
		if (e.kind == Edge::Kind::Line)
			return dirToward(at, (e.a + e.b) * 0.5);
		QPointF c;
		double r = 0;
		if (!arcCenter(e, c, r))
			return dirToward(at, e.mid);
		QPointF rad = at - c;
		const double rl = std::hypot(rad.x(), rad.y());
		if (rl < 1e-9)
			return QPointF(1, 0);
		rad /= rl;
		QPointF tang(-rad.y(), rad.x());
		const QPointF toMid = e.mid - at;
		if (QPointF::dotProduct(tang, toMid) < 0)
			tang = -tang;
		return tang;
	};

	QPointF i = scene;
	const bool lineArc = (E1.kind == Edge::Kind::Line && E2.kind == Edge::Kind::Arc) ||
						 (E1.kind == Edge::Kind::Arc && E2.kind == Edge::Kind::Line);
	if (E1.kind == Edge::Kind::Line && E2.kind == Edge::Kind::Line)
	{
		QLineF s1(E1.a, E1.b);
		QLineF s2(E2.a, E2.b);
		if (s1.intersects(s2, &i) == QLineF::NoIntersection)
			return false;
	}
	else if (lineArc)
	{
		const Edge& L = E1.kind == Edge::Kind::Line ? E1 : E2;
		const Edge& A = E1.kind == Edge::Kind::Arc ? E1 : E2;
		QPointF c;
		double r = 0;
		if (!arcCenter(A, c, r))
			return false;
		QLineF s1(L.a, L.b);
		const QPointF ab = s1.p2() - s1.p1();
		const double AA = QPointF::dotProduct(ab, ab);
		const QPointF f = s1.p1() - c;
		const double B = 2.0 * QPointF::dotProduct(f, ab);
		const double C = QPointF::dotProduct(f, f) - r * r;
		const double disc = B * B - 4 * AA * C;
		if (disc >= 0 && AA > 1e-18)
		{
			const double sq = std::sqrt(disc);
			const double t0 = (-B - sq) / (2 * AA);
			const double t1v = (-B + sq) / (2 * AA);
			const QPointF p0 = s1.p1() + ab * t0;
			const QPointF p1 = s1.p1() + ab * t1v;
			i = QLineF(p0, scene).length() <= QLineF(p1, scene).length() ? p0 : p1;
		}
		else
			i = L.nearPt;
	}
	else
	{
		i = (E1.nearPt + E2.nearPt) * 0.5;
	}

	const QPointF d1 = edgeDirAt(E1, i);
	const QPointF d2 = edgeDirAt(E2, i);
	const QPointF t1 = i + d1 * radiusMm;
	const QPointF t2 = i + d2 * radiusMm;
	const QPointF sum = d1 + d2;
	const double sl = std::hypot(sum.x(), sum.y());
	const QPointF mid = i + (sl > 1e-9 ? sum * (radiusMm * 0.707 / sl) : QPointF(radiusMm, 0));

	m_doc.removeEntity(E1.id);
	m_doc.removeEntity(E2.id);
	m_entityLayer.remove(E1.id);
	m_entityLayer.remove(E2.id);

	const int beforeMax = maxEntityId();
	auto keepLineFar = [&](const Edge& L, const QPointF& touch) {
		const QPointF far = QLineF(L.a, i).length() >= QLineF(L.b, i).length() ? L.a : L.b;
		const int pA = m_doc.addPoint(far.x(), far.y());
		const int pB = m_doc.addPoint(touch.x(), touch.y());
		return m_doc.addLine(pA, pB, false);
	};
	auto keepArcFar = [&](const Edge& A, const QPointF& touch) {
		const QPointF far = QLineF(A.a, i).length() >= QLineF(A.b, i).length() ? A.a : A.b;
		const QPointF nm = (touch + far) * 0.5 + (A.mid - (A.a + A.b) * 0.5) * 0.35;
		const int ps = m_doc.addPoint(touch.x(), touch.y());
		const int pm = m_doc.addPoint(nm.x(), nm.y());
		const int pe = m_doc.addPoint(far.x(), far.y());
		return m_doc.addArc(ps, pm, pe, false);
	};

	if (E1.kind == Edge::Kind::Line)
		keepLineFar(E1, t1);
	else
		keepArcFar(E1, t1);
	if (E2.kind == Edge::Kind::Line)
		keepLineFar(E2, t2);
	else
		keepArcFar(E2, t2);

	const int ps = m_doc.addPoint(t1.x(), t1.y());
	const int pm = m_doc.addPoint(mid.x(), mid.y());
	const int pe = m_doc.addPoint(t2.x(), t2.y());
	m_doc.addArc(ps, pm, pe, false);

	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > beforeMax)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

bool SheetSketchAdapter::chamferLinesAt(const QPointF& scene, double distMm, double tolMm, const QString& layerId)
{
	if (!(distMm > 1e-6))
		return false;
	struct Cand
	{
		enum class Kind
		{
			Line,
			Arc
		} kind = Kind::Line;
		int id = -1;
		double dist = 1e100;
		QPointF a{}, b{}, mid{};
	};
	QVector<Cand> cands;
	const double hitTol = tolMm * 4.0;
	for (const auto& ln : m_doc.lines())
	{
		const SkPoint* p1 = m_doc.findPoint(ln.p1);
		const SkPoint* p2 = m_doc.findPoint(ln.p2);
		if (!p1 || !p2)
			continue;
		QLineF seg(toScene(p1->p), toScene(p2->p));
		const QPointF ab = seg.p2() - seg.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(scene - seg.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		const double d = QLineF(seg.p1() + ab * t, scene).length();
		if (d > hitTol)
			continue;
		Cand c;
		c.kind = Cand::Kind::Line;
		c.id = ln.id;
		c.dist = d;
		c.a = seg.p1();
		c.b = seg.p2();
		cands.push_back(c);
	}
	for (const auto& arc : m_doc.arcs())
	{
		const SkPoint* s = m_doc.findPoint(arc.pStart);
		const SkPoint* m = m_doc.findPoint(arc.pMid);
		const SkPoint* e = m_doc.findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		std::vector<SkVec2> samples;
		appendArcUv(samples, s->p, m->p, e->p, 24);
		double best = hitTol;
		for (size_t i = 1; i < samples.size(); ++i)
		{
			QLineF seg(toScene(samples[i - 1]), toScene(samples[i]));
			const QPointF ab = seg.p2() - seg.p1();
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scene - seg.p1(), ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			best = qMin(best, QLineF(seg.p1() + ab * t, scene).length());
		}
		if (best > hitTol)
			continue;
		Cand c;
		c.kind = Cand::Kind::Arc;
		c.id = arc.id;
		c.dist = best;
		c.a = toScene(s->p);
		c.mid = toScene(m->p);
		c.b = toScene(e->p);
		cands.push_back(c);
	}
	if (cands.size() < 2)
		return false;
	std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.dist < y.dist; });
	const Cand L1 = cands[0];
	const Cand L2 = cands[1];

	QPointF i = scene;
	if (L1.kind == Cand::Kind::Line && L2.kind == Cand::Kind::Line)
	{
		QLineF s1(L1.a, L1.b);
		QLineF s2(L2.a, L2.b);
		if (s1.intersects(s2, &i) == QLineF::NoIntersection)
			return false;
	}
	else
		i = ((L1.a + L1.b) * 0.5 + (L2.a + L2.b) * 0.5) * 0.5;

	auto dirToward = [](const QPointF& from, const QPointF& towardHint) {
		QLineF d(from, towardHint);
		if (d.length() < 1e-9)
			return QPointF(1, 0);
		d.setLength(1.0);
		return d.p2() - d.p1();
	};
	const QPointF d1 = dirToward(i, (L1.a + L1.b) * 0.5);
	const QPointF d2 = dirToward(i, (L2.a + L2.b) * 0.5);
	const QPointF t1 = i + d1 * distMm;
	const QPointF t2 = i + d2 * distMm;

	m_doc.removeEntity(L1.id);
	m_doc.removeEntity(L2.id);
	m_entityLayer.remove(L1.id);
	m_entityLayer.remove(L2.id);

	const int beforeMax = maxEntityId();
	auto keepLineFar = [&](const Cand& L, const QPointF& touch) {
		const QPointF far = QLineF(L.a, i).length() >= QLineF(L.b, i).length() ? L.a : L.b;
		const int pA = m_doc.addPoint(far.x(), far.y());
		const int pB = m_doc.addPoint(touch.x(), touch.y());
		return m_doc.addLine(pA, pB, false);
	};
	auto keepArcFar = [&](const Cand& A, const QPointF& touch) {
		const QPointF far = QLineF(A.a, i).length() >= QLineF(A.b, i).length() ? A.a : A.b;
		const QPointF nm = (touch + far) * 0.5;
		const int ps = m_doc.addPoint(touch.x(), touch.y());
		const int pm = m_doc.addPoint(nm.x(), nm.y());
		const int pe = m_doc.addPoint(far.x(), far.y());
		return m_doc.addArc(ps, pm, pe, false);
	};
	if (L1.kind == Cand::Kind::Line)
		keepLineFar(L1, t1);
	else
		keepArcFar(L1, t1);
	if (L2.kind == Cand::Kind::Line)
		keepLineFar(L2, t2);
	else
		keepArcFar(L2, t2);
	const int pa = m_doc.addPoint(t1.x(), t1.y());
	const int pb = m_doc.addPoint(t2.x(), t2.y());
	m_doc.addLine(pa, pb, false);

	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > beforeMax)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

bool SheetSketchAdapter::breakLineAt(const QPointF& scene, double tolMm)
{
	struct Cand
	{
		int lineId = -1;
		int p1 = -1;
		int p2 = -1;
		QPointF hit;
		double dist = 1e100;
	};
	Cand c;
	c.dist = tolMm * 4.0;
	for (const auto& ln : m_doc.lines())
	{
		const SkPoint* a = m_doc.findPoint(ln.p1);
		const SkPoint* b = m_doc.findPoint(ln.p2);
		if (!a || !b)
			continue;
		QLineF seg(toScene(a->p), toScene(b->p));
		const QPointF ab = seg.p2() - seg.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(scene - seg.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		if (t < 0.05 || t > 0.95)
			continue;
		const QPointF hit = seg.p1() + ab * t;
		const double d = QLineF(hit, scene).length();
		if (d < c.dist)
		{
			c.dist = d;
			c.lineId = ln.id;
			c.p1 = ln.p1;
			c.p2 = ln.p2;
			c.hit = hit;
		}
	}
	if (c.lineId < 0)
		return false;
	const QString lid = layerOf(c.lineId);
	m_doc.removeLine(c.lineId);
	const int mid = m_doc.addPoint(c.hit.x(), c.hit.y());
	const int id1 = m_doc.addLine(c.p1, mid, false);
	const int id2 = m_doc.addLine(mid, c.p2, false);
	if (!lid.isEmpty())
	{
		m_entityLayer.insert(id1, lid);
		m_entityLayer.insert(id2, lid);
	}
	return true;
}

bool SheetSketchAdapter::joinLinesAt(const QPointF& scene, double tolMm)
{
	struct End
	{
		int lineId = -1;
		int nearId = -1;
		int farId = -1;
		QPointF nearPt;
	};
	QVector<End> nearEnds;
	for (const auto& ln : m_doc.lines())
	{
		const SkPoint* a = m_doc.findPoint(ln.p1);
		const SkPoint* b = m_doc.findPoint(ln.p2);
		if (!a || !b)
			continue;
		const QPointF A = toScene(a->p);
		const QPointF B = toScene(b->p);
		if (QLineF(A, scene).length() <= tolMm * 4.0)
			nearEnds.push_back({ln.id, ln.p1, ln.p2, A});
		if (QLineF(B, scene).length() <= tolMm * 4.0)
			nearEnds.push_back({ln.id, ln.p2, ln.p1, B});
	}
	if (nearEnds.size() < 2)
		return false;
	for (int i = 0; i < nearEnds.size(); ++i)
	{
		for (int j = i + 1; j < nearEnds.size(); ++j)
		{
			if (nearEnds[i].lineId == nearEnds[j].lineId)
				continue;
			if (QLineF(nearEnds[i].nearPt, nearEnds[j].nearPt).length() > tolMm * 4.0)
				continue;
			const SkPoint* f1 = m_doc.findPoint(nearEnds[i].farId);
			const SkPoint* f2 = m_doc.findPoint(nearEnds[j].farId);
			if (!f1 || !f2)
				continue;
			const QPointF F1 = toScene(f1->p);
			const QPointF F2 = toScene(f2->p);
			QLineF l1(F1, nearEnds[i].nearPt);
			QLineF l2(nearEnds[j].nearPt, F2);
			if (l1.length() < 1e-9 || l2.length() < 1e-9)
				continue;
			l1.setLength(1.0);
			l2.setLength(1.0);
			const QPointF d1 = l1.p2() - l1.p1();
			const QPointF d2 = l2.p2() - l2.p1();
			if (std::abs(QPointF::dotProduct(d1, d2)) < 0.98)
				continue;
			const QString lid = layerOf(nearEnds[i].lineId);
			m_doc.removeLine(nearEnds[i].lineId);
			m_doc.removeLine(nearEnds[j].lineId);
			const int id = m_doc.addLine(nearEnds[i].farId, nearEnds[j].farId, false);
			if (!lid.isEmpty())
				m_entityLayer.insert(id, lid);
			return true;
		}
	}
	return false;
}

bool SheetSketchAdapter::addCircleAt(const QPointF& center, double radiusMm, const QString& layerId)
{
	if (!(radiusMm > 1e-9))
		return false;
	const int before = maxEntityId();
	const int cid = m_doc.addPoint(center.x(), center.y());
	m_doc.addCircle(cid, radiusMm, false);
	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > before)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

bool SheetSketchAdapter::addArcAt(const QPointF& center, double radiusMm, double startDeg, double endDeg,
								  const QString& layerId)
{
	if (!(radiusMm > 1e-9))
		return false;
	const double a0 = startDeg * 3.141592653589793 / 180.0;
	const double a1 = endDeg * 3.141592653589793 / 180.0;
	double sweep = a1 - a0;
	while (sweep <= 0)
		sweep += 2.0 * 3.141592653589793;
	const double am = a0 + sweep * 0.5;
	const QPointF ps(center.x() + radiusMm * std::cos(a0), center.y() + radiusMm * std::sin(a0));
	const QPointF pm(center.x() + radiusMm * std::cos(am), center.y() + radiusMm * std::sin(am));
	const QPointF pe(center.x() + radiusMm * std::cos(a1), center.y() + radiusMm * std::sin(a1));
	const int before = maxEntityId();
	const int i0 = m_doc.addPoint(ps.x(), ps.y());
	const int i1 = m_doc.addPoint(pm.x(), pm.y());
	const int i2 = m_doc.addPoint(pe.x(), pe.y());
	m_doc.addArc(i0, i1, i2, false);
	const QString lid = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	QVector<int> ids;
	collectEntityIds(ids);
	for (int id : ids)
	{
		if (id > before)
			m_entityLayer.insert(id, lid);
	}
	return true;
}

bool SheetSketchAdapter::stretchInWindow(const QRectF& winScene, const QPointF& delta)
{
	if (!(std::abs(delta.x()) > 1e-12 || std::abs(delta.y()) > 1e-12))
		return false;
	const QRectF w = winScene.normalized();
	bool any = false;
	for (SkPoint& pt : m_doc.pointsMut())
	{
		const QPointF s = toScene(pt.p);
		if (!w.contains(s))
			continue;
		pt.p.u += delta.x();
		pt.p.v += delta.y();
		any = true;
	}
	return any;
}

int SheetSketchAdapter::duplicateEntityTranslated(int entityId, const QPointF& delta, const QString& layerFallback)
{
	const QString lid = layerOf(entityId).isEmpty() ? layerFallback : layerOf(entityId);
	auto mapPt = [&](int pid) -> int {
		const SkPoint* p = m_doc.findPoint(pid);
		if (!p)
			return -1;
		return m_doc.addPoint(p->p.u + delta.x(), p->p.v + delta.y());
	};
	if (const SkLine* ln = m_doc.findLine(entityId))
	{
		const int a = mapPt(ln->p1);
		const int b = mapPt(ln->p2);
		if (a < 0 || b < 0)
			return -1;
		const int nid = m_doc.addLine(a, b, ln->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkArc* arc = m_doc.findArc(entityId))
	{
		const int a = mapPt(arc->pStart);
		const int m = mapPt(arc->pMid);
		const int e = mapPt(arc->pEnd);
		if (a < 0 || m < 0 || e < 0)
			return -1;
		const int nid = m_doc.addArc(a, m, e, arc->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkCircle* cir = m_doc.findCircle(entityId))
	{
		const int cen = mapPt(cir->center);
		if (cen < 0)
			return -1;
		const int nid = m_doc.addCircle(cen, cir->radius, cir->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkSpline* sp = m_doc.findSpline(entityId))
	{
		std::vector<int> pts;
		for (int pid : sp->throughPts)
		{
			const int np = mapPt(pid);
			if (np < 0)
				return -1;
			pts.push_back(np);
		}
		const int nid = m_doc.addSpline(pts, sp->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	return -1;
}

int SheetSketchAdapter::duplicateEntityRotated(int entityId, const QPointF& pivot, double angleRad,
											   const QString& layerFallback)
{
	const QString lid = layerOf(entityId).isEmpty() ? layerFallback : layerOf(entityId);
	const double c = std::cos(angleRad), s = std::sin(angleRad);
	auto mapPt = [&](int pid) -> int {
		const SkPoint* p = m_doc.findPoint(pid);
		if (!p)
			return -1;
		const QPointF d = toScene(p->p) - pivot;
		const QPointF r = pivot + QPointF(d.x() * c - d.y() * s, d.x() * s + d.y() * c);
		return m_doc.addPoint(r.x(), r.y());
	};
	if (const SkLine* ln = m_doc.findLine(entityId))
	{
		const int a = mapPt(ln->p1);
		const int b = mapPt(ln->p2);
		if (a < 0 || b < 0)
			return -1;
		const int nid = m_doc.addLine(a, b, ln->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkArc* arc = m_doc.findArc(entityId))
	{
		const int a = mapPt(arc->pStart);
		const int m = mapPt(arc->pMid);
		const int e = mapPt(arc->pEnd);
		if (a < 0 || m < 0 || e < 0)
			return -1;
		const int nid = m_doc.addArc(a, m, e, arc->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkCircle* cir = m_doc.findCircle(entityId))
	{
		const int cen = mapPt(cir->center);
		if (cen < 0)
			return -1;
		const int nid = m_doc.addCircle(cen, cir->radius, cir->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	if (const SkSpline* sp = m_doc.findSpline(entityId))
	{
		std::vector<int> pts;
		for (int pid : sp->throughPts)
		{
			const int np = mapPt(pid);
			if (np < 0)
				return -1;
			pts.push_back(np);
		}
		const int nid = m_doc.addSpline(pts, sp->construction);
		m_entityLayer.insert(nid, lid);
		return nid;
	}
	return -1;
}
