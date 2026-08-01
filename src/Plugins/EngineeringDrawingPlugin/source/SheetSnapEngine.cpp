/// @file SheetSnapEngine.cpp
/// @brief 图纸对象捕捉实现

#include "SheetSnapEngine.h"

#include <QtMath>

#include <cmath>

bool SheetSnapEngine::segmentIntersection(const QLineF& a, const QLineF& b, QPointF& out)
{
	QPointF i;
	const auto t = a.intersects(b, &i);
	if (t != QLineF::BoundedIntersection)
		return false;
	out = i;
	return true;
}

SheetSnapResult SheetSnapEngine::snap(const QPointF& raw, double tolMm, const QPointF* orthoRef) const
{
	SheetSnapResult best;
	best.pos = raw;
	double bestD = tolMm;

	auto consider = [&](const QPointF& p, const QString& kind) {
		const double d = QLineF(raw, p).length();
		if (d <= bestD)
		{
			bestD = d;
			best.pos = p;
			best.snapped = true;
			best.kind = kind;
		}
	};

	if (m_flags.endpoint || m_flags.midpoint || m_flags.intersection || m_flags.perpendicular || m_flags.nearest)
	{
		for (const QLineF& s : m_segs)
		{
			if (m_flags.endpoint)
			{
				consider(s.p1(), QStringLiteral("end"));
				consider(s.p2(), QStringLiteral("end"));
			}
			if (m_flags.midpoint)
				consider((s.p1() + s.p2()) * 0.5, QStringLiteral("mid"));
			if (m_flags.nearest || m_flags.perpendicular)
			{
				const QPointF ab = s.p2() - s.p1();
				const double len2 = QPointF::dotProduct(ab, ab);
				double t = len2 > 1e-12 ? QPointF::dotProduct(raw - s.p1(), ab) / len2 : 0.0;
				if (m_flags.perpendicular)
				{
					const QPointF foot = s.p1() + ab * t;
					if (t >= -0.05 && t <= 1.05)
						consider(foot, QStringLiteral("perp"));
				}
				if (m_flags.nearest)
				{
					t = qBound(0.0, t, 1.0);
					consider(s.p1() + ab * t, QStringLiteral("near"));
				}
			}
		}
	}

	if (m_flags.intersection)
	{
		for (int i = 0; i < m_segs.size(); ++i)
		{
			for (int j = i + 1; j < m_segs.size(); ++j)
			{
				QPointF ip;
				if (segmentIntersection(m_segs[i], m_segs[j], ip))
					consider(ip, QStringLiteral("int"));
			}
		}
	}

	if (m_flags.center)
	{
		for (const QPointF& c : m_centers)
			consider(c, QStringLiteral("cen"));
	}

	if (orthoRef && (m_flags.ortho || m_flags.polar))
	{
		if (m_flags.polar)
		{
			const double step = qMax(1.0, m_flags.polarStepDeg) * 3.141592653589793 / 180.0;
			const QPointF d = raw - *orthoRef;
			const double len = std::hypot(d.x(), d.y());
			if (len > 1e-9)
			{
				double ang = std::atan2(d.y(), d.x());
				ang = std::round(ang / step) * step;
				const QPointF p = *orthoRef + QPointF(std::cos(ang), std::sin(ang)) * len;
				if (!best.snapped || QLineF(raw, p).length() <= bestD + 1e-9)
				{
					best.pos = p;
					best.snapped = true;
					best.kind = QStringLiteral("polar");
					bestD = QLineF(raw, p).length();
				}
			}
		}
		if (m_flags.ortho)
		{
			QPointF o = raw;
			const double dx = std::abs(raw.x() - orthoRef->x());
			const double dy = std::abs(raw.y() - orthoRef->y());
			if (dx >= dy)
				o.setY(orthoRef->y());
			else
				o.setX(orthoRef->x());
			if (!best.snapped || QLineF(raw, o).length() <= bestD + 1e-9)
			{
				best.pos = o;
				best.snapped = true;
				best.kind = QStringLiteral("ortho");
			}
		}
	}

	return best;
}
