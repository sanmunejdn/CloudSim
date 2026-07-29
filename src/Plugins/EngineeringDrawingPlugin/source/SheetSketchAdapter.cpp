/// @file SheetSketchAdapter.cpp

#include "SheetSketchAdapter.h"

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
