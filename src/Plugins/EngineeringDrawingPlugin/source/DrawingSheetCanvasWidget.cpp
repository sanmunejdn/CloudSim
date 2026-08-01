/// @file DrawingSheetCanvasWidget.cpp
/// @brief 工程图图幅：布局、拖拽、标注、局部放大、导出

#include "DrawingSheetCanvasWidget.h"

#include "DrawingExport.h"
#include "DrawingSheetModel.h"
#include "DrawingSidePanel.h"

#include <QDate>
#include <QDragEnterEvent>
#include <QFont>
#include <QHash>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLineEdit>
#include <QLineF>
#include <QMimeData>
#include <QMouseEvent>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPaintEvent>
#include <QPdfWriter>
#include <QResizeEvent>
#include <QSet>
#include <QTimer>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

namespace
{
QVector<QPointF> xyArrayToPoints(const QJsonArray& arr)
{
	QVector<QPointF> pts;
	pts.reserve(arr.size() / 2);
	for (int i = 0; i + 1 < arr.size(); i += 2)
		pts.push_back(QPointF(arr.at(i).toDouble(), arr.at(i + 1).toDouble()));
	return pts;
}

QJsonArray pointsToXyArray(const QVector<QPointF>& pts)
{
	QJsonArray arr;
	for (const QPointF& p : pts)
	{
		arr.append(p.x());
		arr.append(p.y());
	}
	return arr;
}

QRectF boundsOfPolylines(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visible,
						 const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidden)
{
	QVector<QPointF> pts;
	auto collect = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys) {
		for (const auto& poly : polys)
			for (const QPointF& p : poly.points)
				if (std::isfinite(p.x()) && std::isfinite(p.y()))
					pts.push_back(p);
	};
	collect(visible);
	collect(hidden);
	if (pts.isEmpty())
		return QRectF(0, 0, 10, 10);

	// 先用全体点估包围盒；若被少数野点撑大，则按分位裁掉后再算
	auto bboxOf = [](const QVector<QPointF>& in) {
		double minX = in[0].x(), maxX = in[0].x(), minY = in[0].y(), maxY = in[0].y();
		for (const QPointF& p : in)
		{
			minX = qMin(minX, p.x());
			maxX = qMax(maxX, p.x());
			minY = qMin(minY, p.y());
			maxY = qMax(maxY, p.y());
		}
		return QRectF(minX, minY, qMax(1e-3, maxX - minX), qMax(1e-3, maxY - minY));
	};

	QRectF box = bboxOf(pts);
	if (pts.size() >= 16)
	{
		QVector<double> xs, ys;
		xs.reserve(pts.size());
		ys.reserve(pts.size());
		for (const QPointF& p : pts)
		{
			xs.push_back(p.x());
			ys.push_back(p.y());
		}
		std::sort(xs.begin(), xs.end());
		std::sort(ys.begin(), ys.end());
		const int lo = qMax(0, pts.size() / 50);
		const int hi = qMin(pts.size() - 1, pts.size() - 1 - lo);
		const double rw = xs[hi] - xs[lo];
		const double rh = ys[hi] - ys[lo];
		// 全量包围盒远大于 98% 分位，说明存在野点
		if (box.width() > rw * 20.0 || box.height() > rh * 20.0)
		{
			const double padX = qMax(1.0, rw * 0.05);
			const double padY = qMax(1.0, rh * 0.05);
			box = QRectF(xs[lo] - padX, ys[lo] - padY, rw + 2 * padX, rh + 2 * padY);
		}
	}

	double w = box.width();
	double h = box.height();
	if (w < 1e-3)
		w = 1.0;
	if (h < 1e-3)
		h = 1.0;
	return QRectF(box.x(), box.y(), w, h).adjusted(-2, -2, 2, 2);
}

QVector<DrawingSheetCanvasWidget::Polyline2d> offsetPolylines(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QPointF& delta)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(src.size());
	for (const auto& poly : src)
	{
		DrawingSheetCanvasWidget::Polyline2d n;
		n.points.reserve(poly.points.size());
		for (const QPointF& p : poly.points)
			n.points.push_back(p + delta);
		out.push_back(n);
	}
	return out;
}

QVector<DrawingSheetCanvasWidget::Polyline2d> scalePolylinesAbout(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QPointF& origin, double scale)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(src.size());
	for (const auto& poly : src)
	{
		DrawingSheetCanvasWidget::Polyline2d n;
		n.points.reserve(poly.points.size());
		for (const QPointF& p : poly.points)
			n.points.push_back(origin + (p - origin) * scale);
		out.push_back(n);
	}
	return out;
}

void normalizeViewLocal(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visIn,
						const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidIn,
						QVector<DrawingSheetCanvasWidget::Polyline2d>& visOut,
						QVector<DrawingSheetCanvasWidget::Polyline2d>& hidOut, double& outW, double& outH)
{
	const QRectF box = boundsOfPolylines(visIn, hidIn);
	if (!box.isValid() || !(box.width() > 0.0 || box.height() > 0.0))
	{
		visOut.clear();
		hidOut.clear();
		outW = 0;
		outH = 0;
		return;
	}
	// QRectF::contains 不含右/下边，圆极值点会被裁掉；改用闭区间
	const QRectF keep = box.adjusted(-1.0, -1.0, 1.0, 1.0);
	auto clip = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& src) {
		QVector<DrawingSheetCanvasWidget::Polyline2d> out;
		out.reserve(src.size());
		for (const auto& poly : src)
		{
			DrawingSheetCanvasWidget::Polyline2d n;
			n.points.reserve(poly.points.size());
			for (const QPointF& p : poly.points)
			{
				if (!std::isfinite(p.x()) || !std::isfinite(p.y()))
					continue;
				if (p.x() < keep.left() || p.x() > keep.right() || p.y() < keep.top() || p.y() > keep.bottom())
					continue;
				n.points.push_back(p - box.topLeft());
			}
			if (n.points.size() >= 2)
				out.push_back(n);
		}
		return out;
	};
	visOut = clip(visIn);
	hidOut = clip(hidIn);
	outW = qMax(1.0, box.width());
	outH = qMax(1.0, box.height());
}

DrawingSheetCanvasWidget::DrawingView placeView(const QString& id, const QString& title, const QString& kind,
												const QPointF& origin, double w, double h,
												const QVector<DrawingSheetCanvasWidget::Polyline2d>& lVis,
												const QVector<DrawingSheetCanvasWidget::Polyline2d>& lHid)
{
	DrawingSheetCanvasWidget::DrawingView view;
	view.id = id;
	view.title = title;
	view.kind = kind;
	view.visible = offsetPolylines(lVis, origin);
	view.hidden = offsetPolylines(lHid, origin);
	view.frame = QRectF(origin.x(), origin.y(), w, h).adjusted(-8, -18, 8, 8);
	return view;
}

/// Liang-Barsky：矩形裁剪线段
bool clipSegmentRect(QPointF a, QPointF b, const QRectF& r, QPointF& oa, QPointF& ob)
{
	const double xmin = r.left(), xmax = r.right(), ymin = r.top(), ymax = r.bottom();
	double u1 = 0.0, u2 = 1.0;
	const double dx = b.x() - a.x();
	const double dy = b.y() - a.y();
	auto clip = [&](double p, double q) {
		if (std::abs(p) < 1e-15)
			return q >= 0.0;
		const double t = q / p;
		if (p < 0.0)
		{
			if (t > u2)
				return false;
			if (t > u1)
				u1 = t;
		}
		else
		{
			if (t < u1)
				return false;
			if (t < u2)
				u2 = t;
		}
		return true;
	};
	if (!clip(-dx, a.x() - xmin) || !clip(dx, xmax - a.x()) || !clip(-dy, a.y() - ymin) || !clip(dy, ymax - a.y()))
		return false;
	oa = a + (b - a) * u1;
	ob = a + (b - a) * u2;
	return true;
}

QVector<DrawingSheetCanvasWidget::Polyline2d> clipScalePolylines(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QRectF& region, double scale)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	const QPointF tl = region.topLeft();
	auto mapPt = [&](const QPointF& p) { return (p - tl) * scale; };
	for (const auto& poly : src)
	{
		if (poly.points.size() < 2)
			continue;
		DrawingSheetCanvasWidget::Polyline2d cur;
		auto flush = [&]() {
			if (cur.points.size() >= 2)
				out.push_back(cur);
			cur.points.clear();
		};
		auto pushMapped = [&](const QPointF& p) {
			const QPointF m = mapPt(p);
			if (cur.points.isEmpty() || QLineF(cur.points.constLast(), m).length() > 1e-9)
				cur.points.push_back(m);
		};
		for (int i = 1; i < poly.points.size(); ++i)
		{
			QPointF a, b;
			if (!clipSegmentRect(poly.points[i - 1], poly.points[i], region, a, b))
			{
				flush();
				continue;
			}
			pushMapped(a);
			pushMapped(b);
		}
		flush();
	}
	return out;
}

bool fitCircle2d(const QVector<QPointF>& pts, QPointF& center, double& radius, double& maxErr)
{
	if (pts.size() < 3)
		return false;
	double sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumX3 = 0, sumY3 = 0, sumX2Y = 0, sumXY2 = 0;
	double minX = pts[0].x(), maxX = pts[0].x(), minY = pts[0].y(), maxY = pts[0].y();
	const int n = pts.size();
	for (const QPointF& p : pts)
	{
		const double x = p.x(), y = p.y();
		const double x2 = x * x, y2 = y * y;
		sumX += x;
		sumY += y;
		sumX2 += x2;
		sumY2 += y2;
		sumXY += x * y;
		sumX3 += x2 * x;
		sumY3 += y2 * y;
		sumX2Y += x2 * y;
		sumXY2 += x * y2;
		minX = qMin(minX, x);
		maxX = qMax(maxX, x);
		minY = qMin(minY, y);
		maxY = qMax(maxY, y);
	}
	const double C = n * sumX2 - sumX * sumX;
	const double D = n * sumXY - sumX * sumY;
	const double E = n * sumY2 - sumY * sumY;
	const double G = 0.5 * (n * sumX3 + n * sumXY2 - sumX * (sumX2 + sumY2));
	const double H = 0.5 * (n * sumY3 + n * sumX2Y - sumY * (sumX2 + sumY2));
	const double denom = C * E - D * D;
	if (std::abs(denom) < 1e-12)
		return false;
	const double cx = (G * E - D * H) / denom;
	const double cy = (C * H - D * G) / denom;
	center = QPointF(cx, cy);
	double sumR = 0;
	maxErr = 0;
	for (const QPointF& p : pts)
	{
		const double ri = QLineF(center, p).length();
		sumR += ri;
	}
	radius = sumR / n;
	if (radius < 1e-6)
		return false;
	// 共线/微弯折线拟出的圆心极远，半径远大于点集跨度 → 框外巨圆
	const double boxDiag = std::hypot(maxX - minX, maxY - minY);
	if (radius > boxDiag * 1.15 + 1e-6)
		return false;
	for (const QPointF& p : pts)
		maxErr = qMax(maxErr, std::abs(QLineF(center, p).length() - radius));
	return maxErr <= qMax(0.05 * radius, 0.5);
}

/// 近圆/圆弧折线用椭圆或弧绘制，避免放大后仍见弦线
bool paintPolylineAsCircleOrArc(QPainter& p, const QVector<QPointF>& pts,
								const std::function<QPointF(const QPointF&)>& toWidget, double uniformScale)
{
	if (pts.size() < 5 || !(uniformScale > 1e-12))
		return false;
	QPointF c;
	double r = 0, err = 0;
	if (!fitCircle2d(pts, c, r, err))
		return false;
	if (err > qMax(0.03 * r, 0.25))
		return false;

	auto angOf = [&](const QPointF& pt) { return std::atan2(pt.y() - c.y(), pt.x() - c.x()); };
	double span = 0;
	double prev = angOf(pts.first());
	for (int i = 1; i < pts.size(); ++i)
	{
		double a = angOf(pts.at(i));
		double d = a - prev;
		while (d > 3.141592653589793)
			d -= 2.0 * 3.141592653589793;
		while (d < -3.141592653589793)
			d += 2.0 * 3.141592653589793;
		span += d;
		prev = a;
	}
	const double closeTol = qMax(0.04 * r, 0.3);
	const bool closedEnds = QLineF(pts.first(), pts.last()).length() <= closeTol;
	double peri = 0;
	for (int i = 1; i < pts.size(); ++i)
		peri += QLineF(pts[i - 1], pts[i]).length();
	// 勿仅凭 atan2 累加判整圆：共线点角跳变会得到 ~2π 假整圆
	const bool fullCircle =
		closedEnds && (std::abs(span) > 2.0 * 3.141592653589793 * 0.75 || peri >= 2.0 * 3.141592653589793 * r * 0.55);

	const QPointF wc = toWidget(c);
	const double rw = r * uniformScale;
	if (!(rw > 0.4))
		return false;

	if (fullCircle)
	{
		p.drawEllipse(wc, rw, rw);
		return true;
	}

	const double a0 = angOf(pts.first());
	// Qt：0°在东侧，正角逆时针；场景 Y 向下时与 atan2 差一个符号
	const double startQt = -a0 * 180.0 / 3.141592653589793;
	const double spanQt = -span * 180.0 / 3.141592653589793;
	if (std::abs(spanQt) < 1.0)
		return false;
	const QRectF box(wc.x() - rw, wc.y() - rw, 2.0 * rw, 2.0 * rw);
	p.drawArc(box, static_cast<int>(std::lround(startQt * 16.0)), static_cast<int>(std::lround(spanQt * 16.0)));
	return true;
}

double endpointDist(const QPointF& a, const QPointF& b)
{
	return QLineF(a, b).length();
}

void reversePoints(QVector<QPointF>& pts)
{
	std::reverse(pts.begin(), pts.end());
}

QVector<DrawingSheetCanvasWidget::Polyline2d> stitchPolylines2d(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& in, double tolMm)
{
	if (in.size() <= 1)
		return in;
	QVector<char> used(in.size(), 0);
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(in.size());
	for (int seed = 0; seed < in.size(); ++seed)
	{
		if (used[seed] || in[seed].points.size() < 2)
			continue;
		used[seed] = 1;
		QVector<QPointF> chain = in[seed].points;
		bool grew = true;
		while (grew)
		{
			grew = false;
			for (int j = 0; j < in.size(); ++j)
			{
				if (used[j] || in[j].points.size() < 2)
					continue;
				const QVector<QPointF>& o = in[j].points;
				if (endpointDist(chain.last(), o.first()) <= tolMm)
				{
					for (int k = 1; k < o.size(); ++k)
						chain.push_back(o[k]);
					used[j] = 1;
					grew = true;
					break;
				}
				if (endpointDist(chain.last(), o.last()) <= tolMm)
				{
					for (int k = o.size() - 2; k >= 0; --k)
						chain.push_back(o[k]);
					used[j] = 1;
					grew = true;
					break;
				}
				if (endpointDist(chain.first(), o.last()) <= tolMm)
				{
					QVector<QPointF> joined = o;
					for (int k = 1; k < chain.size(); ++k)
						joined.push_back(chain[k]);
					chain.swap(joined);
					used[j] = 1;
					grew = true;
					break;
				}
				if (endpointDist(chain.first(), o.first()) <= tolMm)
				{
					QVector<QPointF> joined = o;
					reversePoints(joined);
					for (int k = 1; k < chain.size(); ++k)
						joined.push_back(chain[k]);
					chain.swap(joined);
					used[j] = 1;
					grew = true;
					break;
				}
			}
		}
		DrawingSheetCanvasWidget::Polyline2d poly;
		poly.points = std::move(chain);
		out.push_back(std::move(poly));
	}
	return out;
}

DrawingSheetCanvasWidget::Polyline2d makeDenseCirclePoly(const QPointF& c, double r, int samples = 256)
{
	DrawingSheetCanvasWidget::Polyline2d poly;
	poly.points.reserve(samples + 1);
	for (int i = 0; i < samples; ++i)
	{
		const double a = 2.0 * 3.141592653589793 * static_cast<double>(i) / samples;
		poly.points.push_back(QPointF(c.x() + r * std::cos(a), c.y() + r * std::sin(a)));
	}
	poly.points.push_back(poly.points.first());
	return poly;
}

bool pointsLieOnCircle(const QVector<QPointF>& pts, const QPointF& c, double r, double tol)
{
	if (pts.isEmpty())
		return false;
	for (const QPointF& p : pts)
	{
		if (std::abs(QLineF(c, p).length() - r) > tol)
			return false;
	}
	return true;
}

bool angularCoverageOk(const QVector<QPointF>& pts, const QPointF& c, double r, double onTol, double maxGapRad)
{
	QVector<double> angs;
	angs.reserve(pts.size());
	for (const QPointF& p : pts)
	{
		if (std::abs(QLineF(c, p).length() - r) <= onTol)
			angs.push_back(std::atan2(p.y() - c.y(), p.x() - c.x()));
	}
	if (angs.size() < 5)
		return false;
	std::sort(angs.begin(), angs.end());
	QVector<double> uniq;
	for (double a : angs)
	{
		if (uniq.isEmpty() || std::abs(a - uniq.last()) > 1e-3)
			uniq.push_back(a);
	}
	if (uniq.size() < 5)
		return false;
	double maxGap = 0;
	for (int i = 1; i < uniq.size(); ++i)
		maxGap = qMax(maxGap, uniq[i] - uniq[i - 1]);
	maxGap = qMax(maxGap, uniq.first() + 2.0 * 3.141592653589793 - uniq.last());
	return maxGap < maxGapRad;
}

/// HLR 常把圆拆成大量 2 点弦；拼链 + 按共圆合并后再稠密采样
QVector<DrawingSheetCanvasWidget::Polyline2d> promoteCircularPolylines(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& in)
{
	if (in.isEmpty())
		return in;

	double extent = 1.0;
	for (const auto& poly : in)
	{
		for (const QPointF& p : poly.points)
			extent = qMax(extent, qMax(std::abs(p.x()), std::abs(p.y())));
	}
	const double stitchTol = qMax(0.25, extent * 2e-3);
	QVector<DrawingSheetCanvasWidget::Polyline2d> chains = stitchPolylines2d(in, stitchTol);
	QVector<char> used(chains.size(), 0);
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;

	auto tryPromoteChain = [&](int idx) -> bool {
		const QVector<QPointF>& pts = chains[idx].points;
		if (pts.size() < 8)
			return false;
		QPointF c;
		double r = 0, err = 0;
		if (!fitCircle2d(pts, c, r, err) || err > qMax(0.05 * r, 0.5))
			return false;
		QRectF box;
		for (const QPointF& p : pts)
			box = box.isNull() ? QRectF(p, QSizeF(0, 0)) : box.united(QRectF(p, QSizeF(0, 0)));
		const double boxDiag = std::hypot(box.width(), box.height());
		// 共线短边会拟出超大假圆
		if (r > boxDiag * 1.15 + 1e-6)
			return false;
		double peri = 0;
		for (int i = 1; i < pts.size(); ++i)
			peri += QLineF(pts[i - 1], pts[i]).length();
		const double onTol = qMax(0.08 * r, 0.6);
		const bool closed = endpointDist(pts.first(), pts.last()) <= qMax(0.08 * r, stitchTol * 3.0);
		if (closed)
		{
			if (peri < 2.0 * 3.141592653589793 * r * 0.55)
				return false;
		}
		else if (!angularCoverageOk(pts, c, r, onTol, 3.141592653589793 * 0.6))
		{
			return false;
		}
		out.push_back(makeDenseCirclePoly(c, r));
		used[idx] = 1;
		return true;
	};

	for (int i = 0; i < chains.size(); ++i)
	{
		if (!used[i])
			(void)tryPromoteChain(i);
	}

	QVector<int> leftovers;
	QVector<QPointF> pool;
	for (int i = 0; i < chains.size(); ++i)
	{
		if (used[i])
			continue;
		leftovers.push_back(i);
		for (const QPointF& p : chains[i].points)
			pool.push_back(p);
	}
	if (pool.size() >= 6)
	{
		QPointF c;
		double r = 0, err = 0;
		if (fitCircle2d(pool, c, r, err) && err <= qMax(0.08 * r, 0.6))
		{
			QRectF box;
			for (const QPointF& p : pool)
				box = box.isNull() ? QRectF(p, QSizeF(0, 0)) : box.united(QRectF(p, QSizeF(0, 0)));
			if (r > std::hypot(box.width(), box.height()) * 1.15 + 1e-6)
			{
				// 假圆，跳过
			}
			else
			{
			const double onTol = qMax(0.1 * r, 0.8);
			QVector<int> onCircle;
			for (int idx : leftovers)
			{
				if (pointsLieOnCircle(chains[idx].points, c, r, onTol))
					onCircle.push_back(idx);
			}
			QVector<QPointF> onPts;
			for (int idx : onCircle)
				for (const QPointF& p : chains[idx].points)
					onPts.push_back(p);
			if (onCircle.size() >= 1 && angularCoverageOk(onPts, c, r, onTol, 3.141592653589793 * 0.6))
			{
				out.push_back(makeDenseCirclePoly(c, r));
				for (int idx : onCircle)
					used[idx] = 1;
			}
			}
		}
	}

	for (int i = 0; i < chains.size(); ++i)
	{
		if (!used[i])
			out.push_back(chains[i]);
	}
	return out;
}

void drawPolylineCache(QPainter& p, const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys,
					   QVector<QPolygonF>& cache, const std::function<QPointF(const QPointF&)>& toWidget)
{
	// 生成侧已抽稀/拼接；绘制仅做场景→窗口线性映射
	if (cache.size() != polys.size())
	{
		cache.clear();
		cache.reserve(polys.size());
		for (const DrawingSheetCanvasWidget::Polyline2d& poly : polys)
		{
			QPolygonF w;
			if (poly.points.size() >= 2)
			{
				w.reserve(poly.points.size());
				for (const QPointF& pt : poly.points)
					w << toWidget(pt);
			}
			cache.push_back(std::move(w));
		}
	}
	else
	{
		for (int i = 0; i < polys.size(); ++i)
		{
			const auto& src = polys[i].points;
			QPolygonF& w = cache[i];
			if (w.size() != src.size())
			{
				w.clear();
				if (src.size() >= 2)
				{
					w.reserve(src.size());
					for (const QPointF& pt : src)
						w << toWidget(pt);
				}
				continue;
			}
			for (int k = 0; k < src.size(); ++k)
				w[k] = toWidget(src[k]);
		}
	}
	for (const QPolygonF& w : cache)
	{
		if (w.size() >= 2)
			p.drawPolyline(w);
	}
}

} // namespace

DrawingSheetCanvasWidget::DrawingSheetCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(200, 200);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAcceptDrops(true);
	m_paper.date = QDate::currentDate().toString(Qt::ISODate);
	m_sheetModel = std::make_unique<DrawingSheetModel>();
	ensureDefaultLayer();
	ensureDefaultStyles();
}

DrawingSheetCanvasWidget::~DrawingSheetCanvasWidget() = default;

void DrawingSheetCanvasWidget::clearSheet()
{
	m_views.clear();
	if (m_sheetModel)
		m_sheetModel->clear();
	m_dims.clear();
	m_notes.clear();
	m_hatches.clear();
	m_blockRefs.clear();
	m_projectionGuides.clear();
	m_sketch.clear();
	clearSelection();
	m_backendId.clear();
	m_needInitialFit = true;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setViews(const QVector<DrawingView>& views, bool preserveLayout)
{
	QHash<QString, QPointF> oldCenters;
	QVector<DrawingView> keptDetails;
	struct SavedMark
	{
		QString letter;
		QPointF p1;
		QPointF p2;
	};
	QVector<SavedMark> oldSectionMarks;
	if (preserveLayout)
	{
		for (const DrawingView& v : m_views)
		{
			if (!oldCenters.contains(v.kind) && v.kind != QLatin1String("detail"))
				oldCenters.insert(v.kind, v.frame.center());
			if (v.kind == QLatin1String("detail"))
				keptDetails.push_back(v);
			if (v.kind == QLatin1String("section") && v.hasMark)
				oldSectionMarks.push_back({v.markLetter, v.markP1, v.markP2});
		}
	}

	m_views = views;
	m_sceneCacheValid = false;
	m_interactiveCacheValid = false;
	for (DrawingView& v : m_views)
	{
		if (v.layerId.isEmpty())
			v.layerId = m_currentLayerId;
		// HLR 常把闭合圆拆成多段短边；只按端点接链，不再圆拟合
		v.visible = stitchPolylines2d(v.visible, 0.3);
		v.hidden = stitchPolylines2d(v.hidden, 0.3);
		v.visCache.clear();
		v.hidCache.clear();
	}

	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	syncScaleTextFromSheetScale();
	if (std::abs(s - 1.0) > 1e-12)
	{
		auto scalePt = [&](QPointF& p) { p = QPointF(p.x() * s, p.y() * s); };
		for (DrawingView& v : m_views)
		{
			v.frame = QRectF(v.frame.x() * s, v.frame.y() * s, v.frame.width() * s, v.frame.height() * s);
			for (Polyline2d& poly : v.visible)
				for (QPointF& p : poly.points)
					scalePt(p);
			for (Polyline2d& poly : v.hidden)
				for (QPointF& p : poly.points)
					scalePt(p);
		}
	}

	if (preserveLayout)
	{
		for (DrawingView& v : m_views)
		{
			const auto it = oldCenters.constFind(v.kind);
			if (it == oldCenters.cend())
				continue;
			const QPointF delta = it.value() - v.frame.center();
			if (QLineF(QPointF(0, 0), delta).length() > 1e-9)
				translateViewGeometry(v, delta);
		}
		m_views.append(keptDetails);
	}
	else
	{
		placeViewsInPaper();
		m_needInitialFit = true;
		fitToView();
	}

	{
		QSet<QString> liveIds;
		for (const DrawingView& v : m_views)
			liveIds.insert(v.id);
		QVector<SheetHatch> keptHatches;
		keptHatches.reserve(m_hatches.size());
		for (const SheetHatch& h : m_hatches)
		{
			if (!h.anchorViewId.isEmpty() && !liveIds.contains(h.anchorViewId))
				continue;
			keptHatches.push_back(h);
		}
		m_hatches = keptHatches;
	}

	DrawingView* front = nullptr;
	for (DrawingView& v : m_views)
	{
		if (v.kind == QLatin1String("front"))
			front = &v;
	}
	for (DrawingView& v : m_views)
	{
		if (v.kind != QLatin1String("section"))
			continue;
		if (!v.hasMark && front)
		{
			if (!oldSectionMarks.isEmpty())
			{
				const SavedMark om = oldSectionMarks.takeFirst();
				v.markLetter = om.letter.isEmpty() ? nextMarkLetter() : om.letter;
				v.markP1 = om.p1;
				v.markP2 = om.p2;
			}
			else
			{
				v.markLetter = nextMarkLetter();
				const QRectF f = front->frame;
				v.markP1 = QPointF(f.left() + 6.0, f.center().y());
				v.markP2 = QPointF(f.right() - 6.0, f.center().y());
			}
			v.hasMark = true;
			v.parentViewId = front->id;
			v.title = QStringLiteral("剖视 %1-%1").arg(v.markLetter);
		}
		ensureSectionHatchForView(v);
	}

	rebuildProjectionGuides();
	rebindAssociatedDimensions();
	if (m_halfSection)
		applyHalfSectionClip();
	if (m_sheetModel)
		m_sheetModel->setViews(m_views);
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setDimensions(const QVector<SheetDimension>& dims)
{
	m_dims = dims;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setNotes(const QVector<SheetNote>& notes)
{
	m_notes = notes;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setPaper(const SheetPaper& paper)
{
	m_paper = paper;
	emit sheetChanged();
	m_needInitialFit = true;
	update();
}

void DrawingSheetCanvasWidget::setBackendId(const QString& id)
{
	m_backendId = id;
	if (m_paper.title.isEmpty() && !id.isEmpty())
	{
		const int slash = qMax(id.lastIndexOf(QLatin1Char('/')), id.lastIndexOf(QLatin1Char('\\')));
		m_paper.title = (slash >= 0) ? id.mid(slash + 1) : id;
	}
}

QSizeF DrawingSheetCanvasWidget::paperSizeMm() const
{
	double w = 210.0, h = 297.0;
	switch (m_paper.size)
	{
	case DrawingPaperSize::A3:
		w = 297.0;
		h = 420.0;
		break;
	case DrawingPaperSize::A2:
		w = 420.0;
		h = 594.0;
		break;
	case DrawingPaperSize::A1:
		w = 594.0;
		h = 841.0;
		break;
	case DrawingPaperSize::A0:
		w = 841.0;
		h = 1189.0;
		break;
	case DrawingPaperSize::Custom:
		w = qMax(10.0, m_paper.customWidthMm);
		h = qMax(10.0, m_paper.customHeightMm);
		break;
	case DrawingPaperSize::A4:
	default:
		w = 210.0;
		h = 297.0;
		break;
	}
	if (m_paper.size != DrawingPaperSize::Custom && m_paper.landscape)
		qSwap(w, h);
	return QSizeF(w, h);
}

QRectF DrawingSheetCanvasWidget::paperRect() const
{
	const QSizeF s = paperSizeMm();
	return QRectF(0, 0, s.width(), s.height());
}

QRectF DrawingSheetCanvasWidget::paperDrawableRect() const
{
	const QRectF pr = paperRect();
	// 留边 + 右下标题栏占位
	const double m = 12.0;
	const double titleH = qMin(pr.height() * 0.18, 42.0);
	const double titleW = qMin(pr.width() * 0.42, 180.0);
	return pr.adjusted(m, m, -(m + 4.0), -(m + titleH * 0.35)).adjusted(0, 0, -qMax(0.0, titleW - pr.width() * 0.5), 0);
}

QRectF DrawingSheetCanvasWidget::viewsContentBounds() const
{
	QRectF box;
	for (const DrawingView& v : m_views)
		box = box.isValid() ? box.united(v.frame) : v.frame;
	for (const SheetDimension& d : m_dims)
	{
		box = box.united(QRectF(d.p1, QSizeF(0, 0)));
		box = box.united(QRectF(d.p2, QSizeF(0, 0)));
		if (d.kind == SheetDimension::Kind::Angle)
			box = box.united(QRectF(d.p3, QSizeF(0, 0)));
	}
	for (const SheetNote& n : m_notes)
	{
		box = box.united(QRectF(n.anchor, QSizeF(0, 0)));
		box = box.united(QRectF(n.textPos, QSizeF(0, 0)));
	}
	return box;
}

void DrawingSheetCanvasWidget::syncScaleTextFromSheetScale()
{
	const double s = m_paper.sheetScale;
	if (s <= 1e-12)
	{
		m_paper.scaleText = QStringLiteral("1:1");
		return;
	}
	if (std::abs(s - 1.0) < 1e-6)
		m_paper.scaleText = QStringLiteral("1:1");
	else if (s < 1.0)
	{
		const double n = 1.0 / s;
		m_paper.scaleText = (std::abs(n - std::round(n)) < 1e-3)
								? QStringLiteral("1:%1").arg(qRound(n))
								: QStringLiteral("1:%1").arg(n, 0, 'f', 2);
	}
	else
	{
		m_paper.scaleText = (std::abs(s - std::round(s)) < 1e-3)
								? QStringLiteral("%1:1").arg(qRound(s))
								: QStringLiteral("%1:1").arg(s, 0, 'f', 2);
	}
}

void DrawingSheetCanvasWidget::scaleSceneContent(double factor)
{
	if (!std::isfinite(factor) || std::abs(factor - 1.0) < 1e-12)
		return;
	auto scalePt = [&](QPointF& p) { p = QPointF(p.x() * factor, p.y() * factor); };
	for (DrawingView& v : m_views)
	{
		v.frame = QRectF(v.frame.x() * factor, v.frame.y() * factor, v.frame.width() * factor, v.frame.height() * factor);
		for (Polyline2d& poly : v.visible)
			for (QPointF& p : poly.points)
				scalePt(p);
		for (Polyline2d& poly : v.hidden)
			for (QPointF& p : poly.points)
				scalePt(p);
		v.visCache.clear();
		v.hidCache.clear();
	}
	for (SheetDimension& d : m_dims)
	{
		scalePt(d.p1);
		scalePt(d.p2);
		scalePt(d.p3);
		d.textOffset = QPointF(d.textOffset.x() * factor, d.textOffset.y() * factor);
	}
	for (SheetNote& n : m_notes)
	{
		scalePt(n.anchor);
		scalePt(n.textPos);
	}
	for (SkPoint& pt : m_sketch.document().pointsMut())
	{
		pt.p.u *= factor;
		pt.p.v *= factor;
	}
	for (SkCircle& c : m_sketch.document().circlesMut())
		c.radius *= factor;
	for (SkEllipse& e : m_sketch.document().ellipsesMut())
	{
		e.majorR *= factor;
		e.minorR *= factor;
	}
	if (m_sheetModel)
		m_sheetModel->setViews(m_views);
	m_sceneCacheValid = false;
	m_interactiveCacheValid = false;
}

void DrawingSheetCanvasWidget::placeViewsInPaper()
{
	if (!m_paper.visible || m_views.isEmpty())
		return;
	const QRectF drawable = paperDrawableRect();
	QRectF vb = viewsContentBounds();
	if (!vb.isValid() || vb.width() < 1e-6 || vb.height() < 1e-6)
		return;
	const QPointF delta = drawable.topLeft() - vb.topLeft() +
						  QPointF(qMax(0.0, (drawable.width() - vb.width()) * 0.5),
								  qMax(0.0, (drawable.height() - vb.height()) * 0.5));
	if (QLineF(QPointF(0, 0), delta).length() < 1e-9)
		return;
	for (DrawingView& v : m_views)
	{
		v.frame.translate(delta);
		for (Polyline2d& poly : v.visible)
			for (QPointF& p : poly.points)
				p += delta;
		for (Polyline2d& poly : v.hidden)
			for (QPointF& p : poly.points)
				p += delta;
		v.visCache.clear();
		v.hidCache.clear();
	}
	for (SheetDimension& d : m_dims)
	{
		d.p1 += delta;
		d.p2 += delta;
		d.p3 += delta;
	}
	for (SheetNote& n : m_notes)
	{
		n.anchor += delta;
		n.textPos += delta;
	}
	for (SkPoint& pt : m_sketch.document().pointsMut())
	{
		pt.p.u += delta.x();
		pt.p.v += delta.y();
	}
	// path 与折线同场景坐标，平移后必须同步，否则框在图内线在图外
	if (m_sheetModel)
		m_sheetModel->setViews(m_views);
	m_sceneCacheValid = false;
	m_interactiveCacheValid = false;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setSheetScale(double scale, bool rescaleContent)
{
	if (!std::isfinite(scale) || scale < 1e-6)
		return;
	const double old = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	if (rescaleContent && !m_views.isEmpty())
		scaleSceneContent(scale / old);
	m_paper.sheetScale = scale;
	syncScaleTextFromSheetScale();
	if (rescaleContent && !m_views.isEmpty())
		placeViewsInPaper();
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::applySheetScaleFromModel()
{
	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	syncScaleTextFromSheetScale();
	// 仅缩新生成的视图；标注/草图可能已是图面坐标，不能再乘 sheetScale
	if (std::abs(s - 1.0) > 1e-12)
	{
		auto scalePt = [&](QPointF& p) { p = QPointF(p.x() * s, p.y() * s); };
		for (DrawingView& v : m_views)
		{
			v.frame = QRectF(v.frame.x() * s, v.frame.y() * s, v.frame.width() * s, v.frame.height() * s);
			for (Polyline2d& poly : v.visible)
				for (QPointF& p : poly.points)
					scalePt(p);
			for (Polyline2d& poly : v.hidden)
				for (QPointF& p : poly.points)
					scalePt(p);
		}
	}
	placeViewsInPaper();
	m_needInitialFit = true;
	fitToView();
	emit sheetChanged();
	update();
}

bool DrawingSheetCanvasWidget::fitViewsToPaper()
{
	if (!m_paper.visible || m_views.isEmpty())
		return false;
	const QRectF drawable = paperDrawableRect();
	QRectF vb = viewsContentBounds();
	if (!vb.isValid() || vb.width() < 1e-6 || vb.height() < 1e-6)
		return false;
	const double sx = drawable.width() / vb.width();
	const double sy = drawable.height() / vb.height();
	const double k = qMin(sx, sy) * 0.92;
	if (!std::isfinite(k) || k < 1e-6)
		return false;
	scaleSceneContent(k);
	m_paper.sheetScale = (m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0) * k;
	syncScaleTextFromSheetScale();
	placeViewsInPaper();
	m_needInitialFit = true;
	fitToView();
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已适应图幅，比例 %1").arg(m_paper.scaleText));
	update();
	return true;
}

bool DrawingSheetCanvasWidget::isEmpty() const
{
	return m_views.isEmpty() && m_dims.isEmpty() && m_notes.isEmpty() && m_sketch.document().lines().empty() &&
		   m_sketch.document().arcs().empty() && m_sketch.document().circles().empty() &&
		   m_sketch.document().splines().empty();
}

void DrawingSheetCanvasWidget::ensureDefaultLayer()
{
	if (!m_layers.isEmpty())
		return;
	SheetLayer def;
	def.id = defaultLayerId();
	def.name = QStringLiteral("0");
	def.visible = true;
	def.locked = false;
	m_layers.push_back(def);
	m_currentLayerId = def.id;
	m_nextLayerSeq = 1;
}

int DrawingSheetCanvasWidget::layerIndex(const QString& layerId) const
{
	for (int i = 0; i < m_layers.size(); ++i)
	{
		if (m_layers[i].id == layerId)
			return i;
	}
	return -1;
}

const DrawingSheetCanvasWidget::SheetLayer* DrawingSheetCanvasWidget::layerById(const QString& layerId) const
{
	const int idx = layerIndex(layerId.isEmpty() ? defaultLayerId() : layerId);
	return idx >= 0 ? &m_layers[idx] : nullptr;
}

bool DrawingSheetCanvasWidget::isLayerDrawable(const QString& layerId) const
{
	const SheetLayer* L = layerById(layerId);
	return L && L->visible && !L->frozen;
}

bool DrawingSheetCanvasWidget::isLayerPlottable(const QString& layerId) const
{
	const SheetLayer* L = layerById(layerId);
	return !L || L->plottable;
}

bool DrawingSheetCanvasWidget::isLayerEditable(const QString& layerId) const
{
	const SheetLayer* L = layerById(layerId);
	return L && L->visible && !L->frozen && !L->locked;
}

QString DrawingSheetCanvasWidget::uniqueLayerName(const QString& base) const
{
	QString name = base.trimmed().isEmpty() ? QStringLiteral("图层") : base.trimmed();
	auto exists = [&](const QString& n) {
		for (const SheetLayer& L : m_layers)
		{
			if (L.name.compare(n, Qt::CaseInsensitive) == 0)
				return true;
		}
		return false;
	};
	if (!exists(name))
		return name;
	for (int i = 1; i < 1000; ++i)
	{
		const QString cand = QStringLiteral("%1_%2").arg(name).arg(i);
		if (!exists(cand))
			return cand;
	}
	return name + QStringLiteral("_x");
}

bool DrawingSheetCanvasWidget::setCurrentLayer(const QString& layerId)
{
	if (layerIndex(layerId) < 0)
		return false;
	if (m_currentLayerId == layerId)
		return true;
	m_currentLayerId = layerId;
	emit layersChanged();
	return true;
}

QString DrawingSheetCanvasWidget::addLayer(const QString& name)
{
	ensureDefaultLayer();
	SheetLayer L;
	L.id = QStringLiteral("L%1").arg(m_nextLayerSeq++);
	L.name = uniqueLayerName(name);
	m_layers.push_back(L);
	m_currentLayerId = L.id;
	emit layersChanged();
	emit sheetChanged();
	return L.id;
}

bool DrawingSheetCanvasWidget::renameLayer(const QString& layerId, const QString& name)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	const QString n = name.trimmed();
	if (n.isEmpty())
		return false;
	for (int i = 0; i < m_layers.size(); ++i)
	{
		if (i != idx && m_layers[i].name.compare(n, Qt::CaseInsensitive) == 0)
			return false;
	}
	m_layers[idx].name = n;
	emit layersChanged();
	emit sheetChanged();
	return true;
}

void DrawingSheetCanvasWidget::migrateLayerEntities(const QString& fromId, const QString& toId)
{
	for (DrawingView& v : m_views)
	{
		if (v.layerId == fromId)
			v.layerId = toId;
	}
	for (SheetDimension& d : m_dims)
	{
		if (d.layerId == fromId)
			d.layerId = toId;
	}
	for (SheetNote& n : m_notes)
	{
		if (n.layerId == fromId)
			n.layerId = toId;
	}
	m_sketch.remapLayer(fromId, toId);
}

bool DrawingSheetCanvasWidget::removeLayer(const QString& layerId)
{
	if (layerId == defaultLayerId())
		return false;
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	migrateLayerEntities(layerId, defaultLayerId());
	m_layers.removeAt(idx);
	if (m_currentLayerId == layerId)
		m_currentLayerId = defaultLayerId();
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerVisible(const QString& layerId, bool visible)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].visible == visible)
		return true;
	m_layers[idx].visible = visible;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerLocked(const QString& layerId, bool locked)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].locked == locked)
		return true;
	m_layers[idx].locked = locked;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerFrozen(const QString& layerId, bool frozen)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].frozen == frozen)
		return true;
	m_layers[idx].frozen = frozen;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerPlottable(const QString& layerId, bool plottable)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].plottable == plottable)
		return true;
	m_layers[idx].plottable = plottable;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

QString DrawingSheetCanvasWidget::lineTypeToString(SheetLineType t)
{
	switch (t)
	{
	case SheetLineType::Dashed:
		return QStringLiteral("dashed");
	case SheetLineType::Center:
		return QStringLiteral("center");
	case SheetLineType::DashDot:
		return QStringLiteral("dashDot");
	case SheetLineType::Continuous:
	default:
		return QStringLiteral("continuous");
	}
}

SheetLineType DrawingSheetCanvasWidget::lineTypeFromString(const QString& s)
{
	if (s == QLatin1String("dashed"))
		return SheetLineType::Dashed;
	if (s == QLatin1String("center"))
		return SheetLineType::Center;
	if (s == QLatin1String("dashDot") || s == QLatin1String("dashdot"))
		return SheetLineType::DashDot;
	return SheetLineType::Continuous;
}

bool DrawingSheetCanvasWidget::setLayerColor(const QString& layerId, const QColor& color)
{
	const int idx = layerIndex(layerId);
	if (idx < 0 || !color.isValid())
		return false;
	if (m_layers[idx].color == color)
		return true;
	m_layers[idx].color = color;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerLineType(const QString& layerId, SheetLineType lineType)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].lineType == lineType)
		return true;
	m_layers[idx].lineType = lineType;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerLineWidth(const QString& layerId, double widthMm)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	const double w = qBound(0.05, widthMm, 5.0);
	if (std::abs(m_layers[idx].lineWidthMm - w) < 1e-9)
		return true;
	m_layers[idx].lineWidthMm = w;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::reassignSelectionToCurrentLayer()
{
	if (!isLayerEditable(m_currentLayerId))
	{
		emit statusMessage(QStringLiteral("当前层不可编辑"));
		return false;
	}
	bool changed = false;
	if (m_selectedViewIndex >= 0 && m_selectedViewIndex < m_views.size())
	{
		m_views[m_selectedViewIndex].layerId = m_currentLayerId;
		changed = true;
	}
	else if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		m_dims[m_selectedDimIndex].layerId = m_currentLayerId;
		changed = true;
	}
	else if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes[m_selectedNoteIndex].layerId = m_currentLayerId;
		changed = true;
	}
	else if (m_selectedSketchId >= 0)
	{
		m_sketch.setLayerOf(m_selectedSketchId, m_currentLayerId);
		changed = true;
	}
	if (!changed)
	{
		emit statusMessage(QStringLiteral("请先选中对象"));
		return false;
	}
	emit sheetChanged();
	emitSelectionChanged();
	emit statusMessage(QStringLiteral("已移到当前层"));
	update();
	return true;
}

int DrawingSheetCanvasWidget::hitSketchEntity(const QPointF& scenePos, bool requireEditable) const
{
	return m_sketch.hitTestEntity(scenePos, snapTolMm(), [&](int id) {
		const QString lid = m_sketch.layerOf(id);
		if (requireEditable)
			return isLayerEditable(lid);
		return isLayerDrawable(lid);
	});
}

bool DrawingSheetCanvasWidget::isSketchTool(DrawingCanvasTool t) const
{
	return t == DrawingCanvasTool::SketchLine || t == DrawingCanvasTool::SketchRect ||
		   t == DrawingCanvasTool::SketchCircle || t == DrawingCanvasTool::SketchArc ||
		   t == DrawingCanvasTool::SketchSpline;
}

void DrawingSheetCanvasWidget::syncSketchTool()
{
	if (!isSketchTool(m_tool))
	{
		m_sketch.clearTool();
		return;
	}
	SketchToolKind kind = SketchToolKind::Line;
	switch (m_tool)
	{
	case DrawingCanvasTool::SketchLine:
		kind = SketchToolKind::Line;
		break;
	case DrawingCanvasTool::SketchRect:
		kind = SketchToolKind::Rectangle;
		break;
	case DrawingCanvasTool::SketchCircle:
		kind = SketchToolKind::Circle;
		break;
	case DrawingCanvasTool::SketchArc:
		kind = SketchToolKind::Arc;
		break;
	case DrawingCanvasTool::SketchSpline:
		kind = SketchToolKind::Spline;
		break;
	default:
		break;
	}
	m_sketch.setTool(kind);
}

double DrawingSheetCanvasWidget::snapTolMm() const
{
	return qMax(0.5, 8.0 / qMax(1e-6, m_zoom));
}

QVector<QPointF> DrawingSheetCanvasWidget::collectViewSnapPoints() const
{
	QVector<QPointF> pts;
	auto addPoly = [&](const QVector<Polyline2d>& polys) {
		for (const Polyline2d& poly : polys)
		{
			const int n = poly.points.size();
			if (n == 0)
				continue;
			for (int i = 0; i < n; ++i)
				pts.push_back(poly.points[i]);
			// 每段中点，便于点轮廓标尺寸
			for (int i = 1; i < n; ++i)
				pts.push_back((poly.points[i - 1] + poly.points[i]) * 0.5);
		}
	};
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		addPoly(v.visible);
		addPoly(v.hidden);
	}
	return pts;
}

void DrawingSheetCanvasWidget::setProjectionMethod(DrawingProjectionMethod m)
{
	m_projection = m;
}

void DrawingSheetCanvasWidget::setTool(DrawingCanvasTool tool)
{
	m_tool = tool;
	m_dimPicking = false;
	m_dimPickStep = 0;
	m_dimEntityId = -1;
	m_notePicking = false;
	m_detailDragging = false;
	m_modifyPicking = false;
	m_modifyStep = 0;
	m_extendPicking = false;
	m_chainParentId.clear();
	m_chainCount = 0;
	m_chainBaseOffset = -12.0;
	m_hatchPickPts.clear();
	m_stretchHasWindow = false;
	m_stretchWindow = {};
	m_pickHover = {};
	m_lastPickTip.clear();
	if (tool != DrawingCanvasTool::MatchProp)
		m_hasMatchStyle = false;
	if (!isModifyTool(tool) && tool != DrawingCanvasTool::SelectEntity && tool != DrawingCanvasTool::HatchPick &&
		tool != DrawingCanvasTool::TextNote)
	{
		// 切换绘图类工具时清选中，保留修改工具选中集
		m_selectedSketchId = -1;
		m_selectedDimIndex = -1;
		m_selectedNoteIndex = -1;
		m_selectedViewIndex = -1;
		m_selectedViewIndices.clear();
		m_selectedHatchIndex = -1;
		m_selectedBlockRefIndex = -1;
	}
	syncSketchTool();
	QString tip = QStringLiteral("选择/拖视图");
	switch (tool)
	{
	case DrawingCanvasTool::LinearDim:
		tip = QStringLiteral("线性尺寸：点视图轮廓或草图点，再点放置尺寸线");
		break;
	case DrawingCanvasTool::DimRadius:
		tip = QStringLiteral("半径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DimDiameter:
		tip = QStringLiteral("直径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DimAngle:
		tip = QStringLiteral("角度：顶点 → 第一边上点 → 第二边上点");
		break;
	case DrawingCanvasTool::NoteLeader:
		tip = QStringLiteral("文字：点锚点，再点文字位置");
		break;
	case DrawingCanvasTool::DetailRegion:
		tip = QStringLiteral("局部放大：拖框选区域（倍率见工具栏）");
		break;
	case DrawingCanvasTool::SketchLine:
		tip = QStringLiteral("直线：连续点，右键取消");
		break;
	case DrawingCanvasTool::SketchRect:
		tip = QStringLiteral("矩形：对角两点");
		break;
	case DrawingCanvasTool::SketchCircle:
		tip = QStringLiteral("圆：圆心再半径点");
		break;
	case DrawingCanvasTool::SketchArc:
		tip = QStringLiteral("圆弧：起点、中点、终点");
		break;
	case DrawingCanvasTool::SketchSpline:
		tip = QStringLiteral("样条：连续点，右键完成");
		break;
	case DrawingCanvasTool::SelectEntity:
		tip = QStringLiteral("选择图元/尺寸，Delete 删除");
		break;
	case DrawingCanvasTool::MatchProp:
		tip = QStringLiteral("匹配特性：先选源，再点目标");
		break;
	case DrawingCanvasTool::ModifyMove:
		tip = QStringLiteral("移动：选对象 → 基点 → 目标点");
		break;
	case DrawingCanvasTool::ModifyCopy:
		tip = QStringLiteral("复制：选对象 → 基点 → 目标点");
		break;
	case DrawingCanvasTool::ModifyRotate:
		tip = QStringLiteral("旋转：选对象 → 基点 → 角度点");
		break;
	case DrawingCanvasTool::ModifyMirror:
		tip = QStringLiteral("镜像：选对象 → 轴第一点 → 轴第二点");
		break;
	case DrawingCanvasTool::ModifyErase:
		tip = QStringLiteral("删除：点选对象立即删除");
		break;
	case DrawingCanvasTool::ModifyTrim:
		tip = QStringLiteral("修剪：点要裁掉的线段段落");
		break;
	case DrawingCanvasTool::ModifyOffset:
		tip = QStringLiteral("偏移：点线段后输入距离");
		break;
	case DrawingCanvasTool::ModifyScale:
		tip = QStringLiteral("缩放：选对象 → 基点 → 比例点");
		break;
	case DrawingCanvasTool::ModifyFillet:
		tip = QStringLiteral("圆角：线-线 / 线-弧 / 弧-弧交角附近点击");
		break;
	case DrawingCanvasTool::ModifyChamfer:
		tip = QStringLiteral("倒角：线或弧交角附近点击，输入对称距离");
		break;
	case DrawingCanvasTool::ModifyExtend:
		tip = QStringLiteral("延伸：先点边界，再点要延伸的线");
		break;
	case DrawingCanvasTool::ModifyArray:
		tip = QStringLiteral("阵列：选对象（含草图）后输入行列与间距");
		break;
	case DrawingCanvasTool::ModifyPolarArray:
		tip = QStringLiteral("环形阵列：选对象（含草图）后点中心，再输入数量与角度");
		break;
	case DrawingCanvasTool::ModifyBreak:
		tip = QStringLiteral("打断：点线段中间位置");
		break;
	case DrawingCanvasTool::ModifyJoin:
		tip = QStringLiteral("合并：在共线近端点附近点击");
		break;
	case DrawingCanvasTool::ModifyStretch:
		tip = QStringLiteral("拉伸：两点定窗 → 基点 → 目标点");
		break;
	case DrawingCanvasTool::SymRoughness:
		tip = QStringLiteral("粗糙度：点放置位置，输入 Ra");
		break;
	case DrawingCanvasTool::SymGdt:
		tip = QStringLiteral("形位公差：点引线锚点，再点框位置");
		break;
	case DrawingCanvasTool::DimContinuous:
		tip = QStringLiteral("连续尺寸：基点 → 连续点，右键结束");
		break;
	case DrawingCanvasTool::DimBaseline:
		tip = QStringLiteral("基线尺寸：两点定首段 → 再点延伸基线，右键结束");
		break;
	case DrawingCanvasTool::InsertBlock:
		tip = QStringLiteral("插入块：在图面点选插入点");
		break;
	case DrawingCanvasTool::ExplodeBlock:
		tip = QStringLiteral("炸开：点选块参照");
		break;
	case DrawingCanvasTool::HatchPick:
		tip = QStringLiteral("填充：连续点边界，右键闭合");
		break;
	case DrawingCanvasTool::TextNote:
		tip = QStringLiteral("单行文字：点插入点");
		break;
	case DrawingCanvasTool::MText:
		tip = QStringLiteral("多行文字：点插入点");
		break;
	case DrawingCanvasTool::ProjectionGuide:
		tip = QStringLiteral("投影线：点选隐藏引导线");
		break;
	default:
		break;
	}
	emit statusMessage(tip);
	update();
}

void DrawingSheetCanvasWidget::setGridVisible(bool visible)
{
	m_gridVisible = visible;
	update();
}

void DrawingSheetCanvasWidget::setDetailScale(double scale)
{
	m_detailScale = qBound(1.5, scale, 10.0);
}

void DrawingSheetCanvasWidget::clampZoom()
{
	// 大尺寸 STEP 图面可达 1e5+，下限过大会只看到视图间隙
	m_zoom = qBound(1e-4, m_zoom, 64.0);
}

QPointF DrawingSheetCanvasWidget::sceneToWidget(const QPointF& scene) const
{
	return scene * m_zoom + m_panOffset;
}

QPointF DrawingSheetCanvasWidget::widgetToScene(const QPointF& widget) const
{
	return (widget - m_panOffset) / qMax(1e-9, m_zoom);
}

QRectF DrawingSheetCanvasWidget::contentBounds() const
{
	QRectF box;
	if (m_paper.visible)
		box = paperRect();
	for (const DrawingView& v : m_views)
		box = box.isValid() ? box.united(v.frame) : v.frame;
	for (const SheetDimension& d : m_dims)
	{
		box = box.united(QRectF(d.p1, QSizeF(0, 0)));
		box = box.united(QRectF(d.p2, QSizeF(0, 0)));
		if (d.kind == SheetDimension::Kind::Angle)
			box = box.united(QRectF(d.p3, QSizeF(0, 0)));
	}
	for (const SheetNote& n : m_notes)
	{
		box = box.united(QRectF(n.anchor, QSizeF(0, 0)));
		box = box.united(QRectF(n.textPos, QSizeF(0, 0)));
	}
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		for (const QPointF& pt : poly.points)
			box = box.isValid() ? box.united(QRectF(pt, QSizeF(0, 0))) : QRectF(pt, QSizeF(1, 1));
	}
	if (!box.isValid())
		return paperRect().isValid() ? paperRect() : QRectF(0, 0, 200, 200);
	return box.adjusted(-20, -20, 20, 20);
}

void DrawingSheetCanvasWidget::fitToView()
{
	const QRectF box = contentBounds();
	if (width() < 10 || height() < 10)
	{
		m_needInitialFit = true;
		update();
		return;
	}
	const double sx = (width() - 40.0) / qMax(1.0, box.width());
	const double sy = (height() - 40.0) / qMax(1.0, box.height());
	m_zoom = qBound(1e-4, qMin(sx, sy), 64.0);
	m_panOffset = QPointF(width() * 0.5, height() * 0.5) - box.center() * m_zoom;
	m_needInitialFit = false;
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::zoomIn()
{
	m_zoom *= 1.15;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::zoomOut()
{
	m_zoom /= 1.15;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::resetView()
{
	m_zoom = 1.0;
	m_panOffset = QPointF(40, 40);
	emit viewChanged(m_zoom);
	update();
}

bool DrawingSheetCanvasWidget::exportSvg(const QString& filePath) const
{
	return drawing_export::writeSvg(filePath, m_views, m_dims, m_notes, m_sketch.tessellate(), m_paper, m_projection,
									m_layers, m_sketch.entityLayers());
}

bool DrawingSheetCanvasWidget::exportDxf(const QString& filePath) const
{
	QVector<drawing_export::DxfSketchCircle> circles;
	for (const auto& c : m_sketch.document().circles())
	{
		const SkPoint* cen = m_sketch.document().findPoint(c.center);
		if (!cen || !(c.radius > 1e-9))
			continue;
		drawing_export::DxfSketchCircle dc;
		dc.center = SheetSketchAdapter::toScene(cen->p);
		dc.radius = c.radius;
		dc.entityId = c.id;
		circles.push_back(dc);
	}
	QVector<drawing_export::DxfSketchArc> arcs;
	for (const auto& arc : m_sketch.document().arcs())
	{
		const SkPoint* s = m_sketch.document().findPoint(arc.pStart);
		const SkPoint* m = m_sketch.document().findPoint(arc.pMid);
		const SkPoint* e = m_sketch.document().findPoint(arc.pEnd);
		if (!s || !m || !e)
			continue;
		const QPointF A = SheetSketchAdapter::toScene(s->p);
		const QPointF B = SheetSketchAdapter::toScene(m->p);
		const QPointF C = SheetSketchAdapter::toScene(e->p);
		const double d = 2.0 * (A.x() * (B.y() - C.y()) + B.x() * (C.y() - A.y()) + C.x() * (A.y() - B.y()));
		if (std::abs(d) < 1e-12)
			continue;
		const double ux = ((A.x() * A.x() + A.y() * A.y()) * (B.y() - C.y()) +
						   (B.x() * B.x() + B.y() * B.y()) * (C.y() - A.y()) +
						   (C.x() * C.x() + C.y() * C.y()) * (A.y() - B.y())) /
						  d;
		const double uy = ((A.x() * A.x() + A.y() * A.y()) * (C.x() - B.x()) +
						   (B.x() * B.x() + B.y() * B.y()) * (A.x() - C.x()) +
						   (C.x() * C.x() + C.y() * C.y()) * (B.x() - A.x())) /
						  d;
		drawing_export::DxfSketchArc da;
		da.center = QPointF(ux, uy);
		da.radius = QLineF(da.center, A).length();
		da.startDeg = std::atan2(A.y() - uy, A.x() - ux) * 180.0 / 3.141592653589793;
		da.endDeg = std::atan2(C.y() - uy, C.x() - ux) * 180.0 / 3.141592653589793;
		if (da.startDeg < 0)
			da.startDeg += 360.0;
		if (da.endDeg < 0)
			da.endDeg += 360.0;
		da.entityId = arc.id;
		arcs.push_back(da);
	}
	return drawing_export::writeDxf(filePath, m_views, m_dims, m_notes, m_sketch.tessellate(), m_paper, m_projection,
									m_layers, m_sketch.entityLayers(), m_hatches, m_blockDefs, m_blockRefs, circles,
									arcs);
}

bool DrawingSheetCanvasWidget::exportPdf(const QString& filePath)
{
	if (filePath.isEmpty())
		return false;

	const QRectF sceneBox = m_paper.visible ? paperRect() : contentBounds();
	if (!sceneBox.isValid() || sceneBox.width() < 1e-3 || sceneBox.height() < 1e-3)
		return false;

	QPdfWriter writer(filePath);
	writer.setTitle(m_paper.title.isEmpty() ? QStringLiteral("Engineering Drawing") : m_paper.title);
	writer.setCreator(QStringLiteral("CloudSim EngineeringDrawing"));
	const QSizeF mm = paperSizeMm();
	writer.setPageSizeMM(mm);
	writer.setResolution(72);

	QPainter p(&writer);
	if (!p.isActive())
		return false;
	p.setRenderHint(QPainter::Antialiasing, true);

	const QRectF pageRect(0, 0, writer.width(), writer.height());
	p.fillRect(pageRect, Qt::white);

	// 场景 mm → PDF 页，居中铺满（与屏幕 zoom/pan 解耦）
	const double zx = pageRect.width() / sceneBox.width();
	const double zy = pageRect.height() / sceneBox.height();
	const double z = qMin(zx, zy);
	const double oldZoom = m_zoom;
	const QPointF oldPan = m_panOffset;
	m_zoom = z;
	m_panOffset = QPointF(pageRect.left() - sceneBox.left() * z + (pageRect.width() - sceneBox.width() * z) * 0.5,
						  pageRect.top() - sceneBox.top() * z + (pageRect.height() - sceneBox.height() * z) * 0.5);
	paintSheet(p, true);
	m_zoom = oldZoom;
	m_panOffset = oldPan;
	p.end();
	return true;
}

void DrawingSheetCanvasWidget::paintSheet(QPainter& p, bool forExport) const
{
	if (!forExport)
		drawGrid(p);
	drawPaper(p);
	auto layerOk = [this, forExport](const QString& lid) {
		if (!isLayerDrawable(lid))
			return false;
		return !forExport || isLayerPlottable(lid);
	};
	for (const SheetHatch& h : m_hatches)
	{
		if (!layerOk(h.layerId))
			continue;
		drawHatch(p, h);
	}
	for (const DrawingView& v : m_views)
	{
		if (!layerOk(v.layerId))
			continue;
		// 只绘制与当前视口相交的视图，避免全图逐帧重算
		if (!forExport && !v.frame.isValid())
			continue;
		if (!forExport)
		{
			const QRectF wf = QRectF(sceneToWidget(v.frame.topLeft()), sceneToWidget(v.frame.bottomRight()))
								  .normalized()
								  .adjusted(-2, -2, 2, 2);
			if (!wf.intersects(QRectF(0, 0, p.device()->width(), p.device()->height())))
				continue;
		}
		drawView(p, v);
	}
	for (const DrawingView& v : m_views)
	{
		if (v.hasMark)
			drawViewMarks(p, v);
	}
	for (const SheetProjectionGuide& g : m_projectionGuides)
	{
		if (!g.visible)
			continue;
		if (!layerOk(g.layerId))
			continue;
		drawProjectionGuide(p, g);
	}
	for (const SheetBlockRef& r : m_blockRefs)
	{
		if (!layerOk(r.layerId))
			continue;
		drawBlockRef(p, r);
	}
	drawSketch(p, !forExport);
	for (const SheetDimension& d : m_dims)
	{
		if (!layerOk(d.layerId))
			continue;
		drawDimension(p, d);
	}
	for (const SheetNote& n : m_notes)
	{
		if (!layerOk(n.layerId))
			continue;
		drawNote(p, n);
	}
}

void DrawingSheetCanvasWidget::drawGrid(QPainter& p) const
{
	if (!m_gridVisible)
		return;
	p.save();
	p.setPen(QPen(QColor(220, 225, 232), 1));
	const QPointF tl = widgetToScene(QPointF(0, 0));
	const QPointF br = widgetToScene(QPointF(width(), height()));
	const double step = 20.0;
	const int x0 = static_cast<int>(std::floor(tl.x() / step));
	const int x1 = static_cast<int>(std::ceil(br.x() / step));
	const int y0 = static_cast<int>(std::floor(tl.y() / step));
	const int y1 = static_cast<int>(std::ceil(br.y() / step));
	for (int x = x0; x <= x1; ++x)
	{
		const double sx = x * step;
		p.drawLine(sceneToWidget(QPointF(sx, tl.y())), sceneToWidget(QPointF(sx, br.y())));
	}
	for (int y = y0; y <= y1; ++y)
	{
		const double sy = y * step;
		p.drawLine(sceneToWidget(QPointF(tl.x(), sy)), sceneToWidget(QPointF(br.x(), sy)));
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawPaper(QPainter& p) const
{
	if (!m_paper.visible)
		return;
	const QRectF pr = paperRect();
	const QRectF outer = QRectF(sceneToWidget(pr.topLeft()), sceneToWidget(pr.bottomRight())).normalized();
	p.save();
	p.setPen(QPen(QColor(40, 45, 55), 2.0));
	p.setBrush(Qt::NoBrush);
	p.drawRect(outer);
	const QRectF inner = outer.adjusted(8 * m_zoom, 8 * m_zoom, -8 * m_zoom, -8 * m_zoom);
	p.setPen(QPen(QColor(60, 70, 85), 1.0));
	p.drawRect(inner);

	// 右下角标题栏
	const double tw = qMin(outer.width() * 0.42, 180.0 * m_zoom);
	const double th = qMin(outer.height() * 0.18, 42.0 * m_zoom);
	const QRectF block(inner.right() - tw, inner.bottom() - th, tw, th);
	p.setPen(QPen(QColor(40, 45, 55), 1.2));
	p.drawRect(block);
	p.drawLine(block.left(), block.center().y(), block.right(), block.center().y());
	p.drawLine(block.left() + tw * 0.55, block.top(), block.left() + tw * 0.55, block.bottom());

	const QString proj = m_projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("第三角")
																			: QStringLiteral("第一角");
	p.setPen(QColor(30, 35, 45));
	QFont f = p.font();
	f.setPointSizeF(qMax(7.0, 8.0 * qMin(1.5, m_zoom)));
	p.setFont(f);
	const QString title = m_paper.title.isEmpty() ? QStringLiteral("未命名") : m_paper.title;
	QString titleLine = title;
	if (!m_paper.drawingNo.isEmpty())
		titleLine = QStringLiteral("%1  %2").arg(m_paper.drawingNo, title);
	p.drawText(block.adjusted(4, 2, -4, -th * 0.5), Qt::AlignLeft | Qt::AlignVCenter, titleLine);
	QString scaleLine = QStringLiteral("比例 %1").arg(m_paper.scaleText);
	if (!m_paper.material.isEmpty())
		scaleLine = QStringLiteral("%1  %2").arg(scaleLine, m_paper.material);
	p.drawText(QRectF(block.left() + 4, block.center().y(), tw * 0.55 - 8, th * 0.5), Qt::AlignLeft | Qt::AlignVCenter,
			   scaleLine);
	p.drawText(QRectF(block.left() + tw * 0.55 + 4, block.top() + 2, tw * 0.45 - 8, th * 0.5 - 2),
			   Qt::AlignLeft | Qt::AlignVCenter, proj);
	p.drawText(QRectF(block.left() + tw * 0.55 + 4, block.center().y(), tw * 0.45 - 8, th * 0.5),
			   Qt::AlignLeft | Qt::AlignVCenter,
			   QStringLiteral("%1  mm").arg(m_paper.date.isEmpty() ? QStringLiteral("-") : m_paper.date));
	p.restore();
}

void DrawingSheetCanvasWidget::drawView(QPainter& p, const DrawingView& view) const
{
	const QRectF wFrame = QRectF(sceneToWidget(view.frame.topLeft()), sceneToWidget(view.frame.bottomRight())).normalized();
	p.save();
	int viewIdx = -1;
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id == view.id)
		{
			viewIdx = i;
			break;
		}
	}
	const bool sel = isViewSelected(viewIdx) || viewIdx == m_selectedViewIndex;
	const bool hover = viewIdx >= 0 && viewIdx == m_pickHover.viewIndex && !sel;
	p.setPen(QPen(sel ? QColor(142, 68, 173) : (hover ? QColor(0, 188, 212) : QColor(90, 100, 120)),
				  sel || hover ? 2.2 : 1.5));
	// 勿写 QColor : Qt::NoBrush 三元：NoBrush(=0) 会被收成黑色 QColor，视图框实心发黑
	if (sel)
		p.setBrush(QColor(142, 68, 173, 18));
	else if (hover)
		p.setBrush(QColor(0, 188, 212, 22));
	else
		p.setBrush(Qt::NoBrush);
	p.drawRect(wFrame);
	p.setBrush(Qt::NoBrush);
	p.setPen(QPen(QColor(50, 60, 80), 1));
	p.drawText(wFrame.adjusted(4, 2, -4, 0).topLeft() + QPointF(0, 12), view.title);

	// 假圆/越界折线不得画到图框外
	p.setClipRect(wFrame.adjusted(1, 14, -1, -1));
	if (m_sheetModel && m_sheetModel->hasPath(view.id))
	{
		// 场景 path + 视口变换：缩放/平移不再逐点重建
		QTransform xform;
		xform.translate(m_panOffset.x(), m_panOffset.y());
		xform.scale(m_zoom, m_zoom);
		p.setPen(resolvePen(view.layerId, view.style, true, false));
		p.drawPath(xform.map(m_sheetModel->hiddenPath(view.id)));
		p.setPen(resolvePen(view.layerId, view.style, false, false));
		p.drawPath(xform.map(m_sheetModel->visiblePath(view.id)));
	}
	else
	{
		const auto toWidget = [this](const QPointF& pt) { return sceneToWidget(pt); };
		p.setPen(resolvePen(view.layerId, view.style, true, false));
		drawPolylineCache(p, view.hidden, view.hidCache, toWidget);
		p.setPen(resolvePen(view.layerId, view.style, false, false));
		drawPolylineCache(p, view.visible, view.visCache, toWidget);
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawViewMarks(QPainter& p, const DrawingView& view) const
{
	if (!view.hasMark || view.markLetter.isEmpty())
		return;
	p.save();
	p.setPen(QPen(QColor(40, 50, 70), qMax(1.0, 1.2 * m_zoom)));
	p.setBrush(Qt::NoBrush);
	if (view.kind == QLatin1String("section"))
	{
		const QPointF a = sceneToWidget(view.markP1);
		const QPointF b = sceneToWidget(view.markP2);
		p.drawLine(a, b);
		QLineF seg(a, b);
		if (seg.length() > 1e-3)
		{
			seg.setLength(1.0);
			const QPointF d = seg.p2() - seg.p1();
			const QPointF n(-d.y(), d.x());
			drawDimArrow(p, a, a - b);
			drawDimArrow(p, b, b - a);
			p.drawText(a + n * 10 + QPointF(-8, -4), view.markLetter);
			p.drawText(b + n * 10 + QPointF(2, -4), view.markLetter);
		}
	}
	else if (view.kind == QLatin1String("detail"))
	{
		const QRectF r = QRectF(view.markP1, view.markP2).normalized();
		const QRectF wr = QRectF(sceneToWidget(r.topLeft()), sceneToWidget(r.bottomRight())).normalized();
		p.drawEllipse(wr);
		const QPointF tip = wr.topRight() + QPointF(12, -8);
		p.drawLine(wr.center(), tip);
		p.drawEllipse(QRectF(tip.x() - 8, tip.y() - 8, 16, 16));
		p.drawText(QRectF(tip.x() - 8, tip.y() - 8, 16, 16), Qt::AlignCenter, view.markLetter);
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawProjectionGuide(QPainter& p, const SheetProjectionGuide& g) const
{
	const QLineF line = projectionGuideLine(g);
	if (line.length() < 1e-9)
		return;
	p.save();
	QPen pen(QColor(120, 140, 160, 160), qMax(0.8, 0.9 * m_zoom), Qt::DashDotLine);
	p.setPen(pen);
	p.drawLine(sceneToWidget(line.p1()), sceneToWidget(line.p2()));
	p.setBrush(QColor(120, 140, 160));
	p.drawEllipse(sceneToWidget(line.p1()), 3.5, 3.5);
	p.drawEllipse(sceneToWidget(line.p2()), 3.5, 3.5);
	p.restore();
}

void DrawingSheetCanvasWidget::drawSketch(QPainter& p, bool interactive) const
{
	p.save();
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		if (poly.points.size() < 2)
			continue;
		const QString lid = m_sketch.layerOf(poly.entityId);
		if (!isLayerDrawable(lid))
			continue;
		if (!interactive && !isLayerPlottable(lid))
			continue;
		const bool sel = interactive && poly.entityId >= 0 && poly.entityId == m_selectedSketchId;
		const bool hover = interactive && poly.entityId >= 0 && poly.entityId == m_pickHover.sketchId && !sel;
		QPen pen = penForLayer(lid, poly.construction, sel);
		if (hover)
		{
			pen = QPen(QColor(0, 188, 212), qMax(2.2, pen.widthF() * 1.8), pen.style(), Qt::RoundCap,
					   Qt::RoundJoin);
		}
		else if (poly.construction && !sel)
		{
			QColor c = pen.color();
			c.setAlpha(170);
			pen.setColor(c);
		}
		p.setPen(pen);
		QPolygonF w;
		for (const QPointF& pt : poly.points)
			w << sceneToWidget(pt);
		p.drawPolyline(w);
	}
	if (interactive)
	{
		const QVector<QPointF> preview = m_sketch.previewPolyline();
		if (preview.size() >= 2)
		{
			p.setPen(QPen(QColor(41, 128, 185), 1.2, Qt::DashLine));
			QPolygonF w;
			for (const QPointF& pt : preview)
				w << sceneToWidget(pt);
			p.drawPolyline(w);
		}
		const SkSnapResult snap = m_sketch.lastSnap();
		if (snap.snapped)
		{
			const QPointF c = sceneToWidget(SheetSketchAdapter::toScene(snap.pos));
			p.setPen(QPen(QColor(230, 126, 34), 1.5));
			p.drawEllipse(c, 5, 5);
		}
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawDimArrow(QPainter& p, const QPointF& tipWidget, const QPointF& dirScene) const
{
	QLineF dir(QPointF(0, 0), dirScene);
	if (dir.length() < 1e-9)
		return;
	dir.setLength(1.0);
	const QPointF d = dir.p2();
	const QPointF n(-d.y(), d.x());
	const double len = 7.0;
	const QPointF base = tipWidget - d * len;
	QPolygonF tri;
	tri << tipWidget << (base + n * 3.0) << (base - n * 3.0);
	p.setBrush(p.pen().color());
	p.drawPolygon(tri);
	p.setBrush(Qt::NoBrush);
}

double DrawingSheetCanvasWidget::dimensionValue(const SheetDimension& dim) const
{
	if (std::isfinite(dim.overrideValue))
		return dim.overrideValue;
	if (dim.kind == SheetDimension::Kind::Linear)
		return QLineF(dim.p1, dim.p2).length();
	if (dim.kind == SheetDimension::Kind::Angle)
	{
		const QPointF a = dim.p2 - dim.p1;
		const QPointF b = dim.p3 - dim.p1;
		const double la = std::hypot(a.x(), a.y());
		const double lb = std::hypot(b.x(), b.y());
		if (la < 1e-9 || lb < 1e-9)
			return 0.0;
		double c = (a.x() * b.x() + a.y() * b.y()) / (la * lb);
		c = qBound(-1.0, c, 1.0);
		return std::acos(c) * 180.0 / 3.141592653589793;
	}
	const double r = QLineF(dim.p1, dim.p2).length();
	return dim.kind == SheetDimension::Kind::Diameter ? (2.0 * r) : r;
}

QString DrawingSheetCanvasWidget::dimensionText(const SheetDimension& dim) const
{
	const DimStyle* st = dimStyleById(dim.styleId);
	const int prec = st ? qBound(0, st->precision, 6) : 2;
	const double v = dimensionValue(dim);
	QString base;
	if (dim.kind == SheetDimension::Kind::Radius)
		base = QStringLiteral("R%1").arg(v, 0, 'f', prec);
	else if (dim.kind == SheetDimension::Kind::Diameter)
		base = QStringLiteral("Ø%1").arg(v, 0, 'f', prec);
	else if (dim.kind == SheetDimension::Kind::Angle)
		base = QStringLiteral("%1°").arg(v, 0, 'f', qMax(0, prec - 1));
	else
		base = QString::number(v, 'f', prec);
	bool showTol = false;
	double tp = 0.0, tm = 0.0;
	if (dim.tolOverride)
	{
		showTol = dim.showTolerance;
		tp = dim.tolPlus;
		tm = dim.tolMinus;
	}
	else if (st && st->showTolerance)
	{
		showTol = true;
		tp = st->tolPlus;
		tm = st->tolMinus;
	}
	if (showTol)
		base += QStringLiteral(" +%1/-%2").arg(tp, 0, 'f', prec).arg(tm, 0, 'f', prec);
	return base;
}

void DrawingSheetCanvasWidget::drawDimension(QPainter& p, const SheetDimension& dim) const
{
	p.save();
	const bool sel = m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size() &&
					 m_dims[m_selectedDimIndex].id == dim.id;
	const bool hover = !sel && m_pickHover.dimIndex >= 0 && m_pickHover.dimIndex < m_dims.size() &&
					   m_dims[m_pickHover.dimIndex].id == dim.id;
	p.setPen(resolvePen(dim.layerId, dim.style, false, sel));
	if (hover)
		p.setPen(QPen(QColor(0, 188, 212), qMax(1.8, p.pen().widthF() * 1.5)));

	if (dim.kind == SheetDimension::Kind::Linear)
	{
		QLineF base(dim.p1, dim.p2);
		if (base.length() < 1e-9)
		{
			p.restore();
			return;
		}
		base.setLength(1.0);
		const QPointF dir = base.p2() - base.p1();
		QPointF n(-dir.y(), dir.x());
		// textOffset 的长度决定尺寸线偏移
		double off = dim.textOffset.y();
		if (std::abs(off) < 1e-6)
			off = -12.0;
		const QPointF offset = n * off;
		const QPointF a = dim.p1 + offset;
		const QPointF b = dim.p2 + offset;
		const QPointF wa = sceneToWidget(a);
		const QPointF wb = sceneToWidget(b);
		const QPointF wp1 = sceneToWidget(dim.p1);
		const QPointF wp2 = sceneToWidget(dim.p2);
		p.drawLine(wp1, sceneToWidget(dim.p1 + offset * 1.15));
		p.drawLine(wp2, sceneToWidget(dim.p2 + offset * 1.15));
		p.drawLine(wa, wb);
		drawDimArrow(p, wa, -(b - a));
		drawDimArrow(p, wb, b - a);
		const QPointF mid = (wa + wb) * 0.5 + QPointF(dim.textOffset.x(), 0) * m_zoom;
		p.drawText(mid, dimensionText(dim));
	}
	else if (dim.kind == SheetDimension::Kind::Angle)
	{
		const QPointF v = dim.p1;
		p.drawLine(sceneToWidget(v), sceneToWidget(dim.p2));
		p.drawLine(sceneToWidget(v), sceneToWidget(dim.p3));
		const double a0 = std::atan2(dim.p2.y() - v.y(), dim.p2.x() - v.x());
		const double a1 = std::atan2(dim.p3.y() - v.y(), dim.p3.x() - v.x());
		double span = a1 - a0;
		while (span <= -3.141592653589793)
			span += 2.0 * 3.141592653589793;
		while (span > 3.141592653589793)
			span -= 2.0 * 3.141592653589793;
		const double rad = qMax(12.0, 0.35 * qMin(QLineF(v, dim.p2).length(), QLineF(v, dim.p3).length()));
		QRectF arcBox(sceneToWidget(QPointF(v.x() - rad, v.y() - rad)),
					  sceneToWidget(QPointF(v.x() + rad, v.y() + rad)));
		arcBox = arcBox.normalized();
		const int start16 = static_cast<int>(-a0 * 180.0 / 3.141592653589793 * 16.0);
		const int span16 = static_cast<int>(-span * 180.0 / 3.141592653589793 * 16.0);
		p.drawArc(arcBox, start16, span16);
		const double amid = a0 + span * 0.5;
		const QPointF textScene = v + QPointF(std::cos(amid), std::sin(amid)) * (rad + 8.0) + dim.textOffset;
		p.drawText(sceneToWidget(textScene), dimensionText(dim));
	}
	else
	{
		const QPointF c = sceneToWidget(dim.p1);
		const QPointF r = sceneToWidget(dim.p2);
		const QPointF textPos = sceneToWidget(dim.p2 + dim.textOffset);
		if (dim.kind == SheetDimension::Kind::Diameter)
		{
			const QPointF other = dim.p1 - (dim.p2 - dim.p1);
			p.drawLine(sceneToWidget(other), r);
			drawDimArrow(p, r, dim.p2 - dim.p1);
			drawDimArrow(p, sceneToWidget(other), other - dim.p1);
		}
		else
		{
			p.drawLine(c, r);
			drawDimArrow(p, r, dim.p2 - dim.p1);
		}
		p.drawText(textPos, dimensionText(dim));
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawNote(QPainter& p, const SheetNote& note) const
{
	p.save();
	const bool sel = m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size() &&
					 m_notes[m_selectedNoteIndex].id == note.id;
	const bool hover = !sel && m_pickHover.noteIndex >= 0 && m_pickHover.noteIndex < m_notes.size() &&
					   m_notes[m_pickHover.noteIndex].id == note.id;
	p.setPen(resolvePen(note.layerId, note.style, false, sel));
	if (hover)
		p.setPen(QPen(QColor(0, 188, 212), qMax(1.6, p.pen().widthF() * 1.4), Qt::DashLine));
	const QPointF wa = sceneToWidget(note.anchor);
	const QPointF wt = sceneToWidget(note.textPos);
	if (note.kind == SheetNote::Kind::Roughness)
	{
		// 简易粗糙度折线符号
		const QPointF tip = wa;
		p.drawLine(tip, tip + QPointF(-6, 10));
		p.drawLine(tip, tip + QPointF(6, 10));
		p.drawLine(tip + QPointF(-6, 10), tip + QPointF(10, 10));
		p.drawText(tip + QPointF(12, 4), note.text.isEmpty() ? QStringLiteral("Ra%1").arg(note.roughnessRa) : note.text);
	}
	else if (note.kind == SheetNote::Kind::Gdt)
	{
		p.drawLine(wa, wt);
		p.drawEllipse(wa, 3, 3);
		const QString label = note.text.isEmpty() ? note.gdtCode : note.text;
		QFontMetrics fm(p.font());
		const QRectF box(wt, QSizeF(qMax(48.0, fm.horizontalAdvance(label) + 10.0), 16.0));
		p.setBrush(QColor(255, 255, 255, 230));
		p.drawRect(box);
		p.setBrush(Qt::NoBrush);
		p.drawText(box.adjusted(4, 0, -2, 0), Qt::AlignVCenter | Qt::AlignLeft, label);
	}
	else
	{
		p.drawLine(wa, wt);
		p.drawEllipse(wa, 3, 3);
		if (const TextStyle* ts = textStyleById(note.textStyleId))
		{
			QFont f = p.font();
			f.setFamily(ts->fontFamily);
			f.setPointSizeF(qMax(6.0, ts->heightMm * m_zoom * 0.85));
			p.setFont(f);
		}
		p.drawText(wt + QPointF(4, -2), note.text);
	}
	p.restore();
}

void DrawingSheetCanvasWidget::paintEvent(QPaintEvent*)
{
	// 与工艺流程一致：首次有内容时在 paint 里补 fit，避免仅靠 resize
	if (m_needInitialFit && (!isEmpty() || m_paper.visible) && width() >= 10 && height() >= 10)
		fitToView();

	// 拖动/平移期间：贴交互缓存；缩放滚动中：贴旧场景缓存，防抖后再重绘
	const bool interactive = m_panning || m_draggingView;
	if (interactive && !m_interactiveCacheValid)
	{
		m_interactiveCache = QPixmap(size());
		m_interactiveCache.fill(QColor(0xF5, 0xF7, 0xFA));
		if (!m_interactiveCache.isNull())
		{
			QPainter cp(&m_interactiveCache);
			cp.setRenderHint(QPainter::Antialiasing, true);
			paintSheet(cp, false);
			m_interactiveCacheValid = true;
		}
	}

	QPainter p(this);
	p.fillRect(rect(), QColor(0xF5, 0xF7, 0xFA));
	if (interactive && m_interactiveCacheValid && !m_interactiveCache.isNull())
	{
		p.drawPixmap(0, 0, m_interactiveCache);
	}
	else if (m_zoomRepaintPending && m_sceneCacheValid && !m_sceneCache.isNull())
	{
		// 滚轮缩放：先按新变换把旧缓存整体放大，停顿后再走下方正常重绘
		p.drawPixmap(0, 0, m_sceneCache);
	}
	else
	{
		// 非交互/非缩放防抖帧即重建场景缓存：内容任何改动都会落到这条路径
		m_interactiveCacheValid = false;
		m_sceneCache = QPixmap(size());
		m_sceneCache.fill(QColor(0xF5, 0xF7, 0xFA));
		m_sceneCacheValid = false;
		if (!m_sceneCache.isNull())
		{
			QPainter cp(&m_sceneCache);
			cp.setRenderHint(QPainter::Antialiasing, true);
			paintSheet(cp, false);
			m_sceneCacheValid = true;
			p.drawPixmap(0, 0, m_sceneCache);
		}
		else
		{
			p.setRenderHint(QPainter::Antialiasing, true);
			paintSheet(p, false);
		}
		m_zoomRepaintPending = false;
	}
	if (m_detailDragging)
	{
		p.setPen(QPen(QColor(41, 128, 185), 1.2, Qt::DashLine));
		p.setBrush(QColor(41, 128, 185, 40));
		const QRectF r = QRectF(sceneToWidget(m_detailStart), sceneToWidget(m_detailCurrent)).normalized();
		p.drawRect(r);
	}
	if (m_tool == DrawingCanvasTool::ModifyStretch && m_modifyPicking && !m_stretchHasWindow && m_modifyStep >= 1)
	{
		p.setPen(QPen(QColor(230, 126, 34), 1.2, Qt::DashLine));
		p.setBrush(QColor(230, 126, 34, 35));
		const QRectF r = QRectF(sceneToWidget(m_modifyP1), sceneToWidget(m_modifyP2)).normalized();
		p.drawRect(r);
	}
	else if (m_stretchHasWindow)
	{
		p.setPen(QPen(QColor(230, 126, 34), 1.2, Qt::DashLine));
		p.setBrush(QColor(230, 126, 34, 25));
		const QRectF r = QRectF(sceneToWidget(m_stretchWindow.topLeft()), sceneToWidget(m_stretchWindow.bottomRight()))
						 .normalized();
		p.drawRect(r);
	}
	drawPickFeedback(p);
}

void DrawingSheetCanvasWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	m_interactiveCacheValid = false;
	m_sceneCacheValid = false;
	m_zoomRepaintPending = false;
	if (m_needInitialFit && !m_views.isEmpty())
		fitToView();
}

int DrawingSheetCanvasWidget::hitViewIndex(const QPointF& scenePos) const
{
	for (int i = m_views.size() - 1; i >= 0; --i)
	{
		if (!isLayerEditable(m_views[i].layerId))
			continue;
		if (m_views[i].frame.contains(scenePos))
			return i;
	}
	return -1;
}

void DrawingSheetCanvasWidget::moveViewBy(int index, const QPointF& deltaScene)
{
	if (index < 0 || index >= m_views.size())
		return;
	DrawingView& v = m_views[index];
	if (!isLayerEditable(v.layerId))
		return;
	translateViewGeometry(v, deltaScene);
	for (SheetDimension& d : m_dims)
	{
		if (d.anchorViewId != v.id)
			continue;
		d.p1 += deltaScene;
		d.p2 += deltaScene;
		if (d.kind == SheetDimension::Kind::Angle)
			d.p3 += deltaScene;
	}
	for (SheetNote& n : m_notes)
	{
		if (n.anchorViewId != v.id)
			continue;
		n.anchor += deltaScene;
		n.textPos += deltaScene;
	}
	for (SheetHatch& h : m_hatches)
	{
		if (h.anchorViewId != v.id)
			continue;
		for (QPointF& p : h.boundary)
			p += deltaScene;
	}
	// 子视图标记在父视图上：父视图移动时平移 mark
	for (DrawingView& child : m_views)
	{
		if (!child.hasMark || child.parentViewId != v.id)
			continue;
		child.markP1 += deltaScene;
		child.markP2 += deltaScene;
	}
	if (m_projectionPinned && v.kind == QLatin1String("front"))
	{
		const int top = findViewIndexByKind(QStringLiteral("top"));
		const int right = findViewIndexByKind(QStringLiteral("right"));
		if (top >= 0 && top != index)
		{
			translateViewGeometry(m_views[top], QPointF(deltaScene.x(), 0));
			for (SheetHatch& h : m_hatches)
			{
				if (h.anchorViewId != m_views[top].id)
					continue;
				for (QPointF& p : h.boundary)
					p += QPointF(deltaScene.x(), 0);
			}
		}
		if (right >= 0 && right != index)
		{
			translateViewGeometry(m_views[right], QPointF(0, deltaScene.y()));
			for (SheetHatch& h : m_hatches)
			{
				if (h.anchorViewId != m_views[right].id)
					continue;
				for (QPointF& p : h.boundary)
					p += QPointF(0, deltaScene.y());
			}
		}
	}
	rebuildProjectionGuides();
	emit sheetChanged();
	update();
}

QString DrawingSheetCanvasWidget::inferAnchorViewId(const QPointF& scenePos) const
{
	const int idx = hitViewIndex(scenePos);
	return idx >= 0 ? m_views[idx].id : QString();
}

bool DrawingSheetCanvasWidget::addDetailView(const QString& parentViewId, const QRectF& regionScene, double scale)
{
	int parentIdx = -1;
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id == parentViewId)
		{
			parentIdx = i;
			break;
		}
	}
	if (parentIdx < 0)
		return false;
	const double useScale = scale > 1e-6 ? scale : m_detailScale;
	const DrawingView& parent = m_views[parentIdx];
	const QRectF region = regionScene.normalized();
	if (region.width() < 1e-3 || region.height() < 1e-3)
		return false;

	auto clipScale = [&](const QVector<Polyline2d>& src) { return clipScalePolylines(src, region, useScale); };

	DrawingView detail;
	detail.id = QStringLiteral("detail_%1").arg(m_nextDetailId++);
	const QString letter = nextMarkLetter();
	detail.markLetter = letter;
	detail.hasMark = true;
	detail.markP1 = region.topLeft();
	detail.markP2 = region.bottomRight();
	detail.title = QStringLiteral("局部%1 ×%2").arg(letter).arg(useScale, 0, 'f', 1);
	detail.kind = QStringLiteral("detail");
	detail.parentViewId = parentViewId;
	detail.contentScale = useScale;
	detail.visible = clipScale(parent.visible);
	detail.hidden = clipScale(parent.hidden);
	const QRectF localBox = boundsOfPolylines(detail.visible, detail.hidden);
	detail.visible = offsetPolylines(detail.visible, -localBox.topLeft());
	detail.hidden = offsetPolylines(detail.hidden, -localBox.topLeft());
	const QPointF origin(contentBounds().right() + 40.0, contentBounds().top());
	detail.visible = offsetPolylines(detail.visible, origin);
	detail.hidden = offsetPolylines(detail.hidden, origin);
	detail.frame = QRectF(origin, localBox.size()).adjusted(-8, -18, 8, 8);
	detail.layerId = m_currentLayerId;
	m_views.push_back(detail);
	emit sheetChanged();
	fitToView();
	return true;
}

bool DrawingSheetCanvasWidget::refreshDetailViewFromMark(int viewIndex)
{
	if (viewIndex < 0 || viewIndex >= m_views.size())
		return false;
	DrawingView& detail = m_views[viewIndex];
	if (detail.kind != QLatin1String("detail") || !detail.hasMark)
		return false;
	int parentIdx = -1;
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id == detail.parentViewId)
		{
			parentIdx = i;
			break;
		}
	}
	if (parentIdx < 0)
		return false;
	const DrawingView& parent = m_views[parentIdx];
	const QRectF region = QRectF(detail.markP1, detail.markP2).normalized();
	if (region.width() < 1e-3 || region.height() < 1e-3)
		return false;
	const double useScale = detail.contentScale > 1e-6 ? detail.contentScale : m_detailScale;
	const QPointF frameOrigin = detail.frame.topLeft() + QPointF(8, 18);
	detail.visible = clipScalePolylines(parent.visible, region, useScale);
	detail.hidden = clipScalePolylines(parent.hidden, region, useScale);
	const QRectF localBox = boundsOfPolylines(detail.visible, detail.hidden);
	detail.visible = offsetPolylines(detail.visible, -localBox.topLeft() + frameOrigin);
	detail.hidden = offsetPolylines(detail.hidden, -localBox.topLeft() + frameOrigin);
	detail.frame = QRectF(frameOrigin, localBox.size()).adjusted(-8, -18, 8, 8);
	detail.title = QStringLiteral("局部%1 ×%2").arg(detail.markLetter).arg(useScale, 0, 'f', 1);
	update();
	return true;
}

bool DrawingSheetCanvasWidget::removeView(const QString& viewId)
{
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id != viewId)
			continue;
		if (!isLayerEditable(m_views[i].layerId))
		{
			emit statusMessage(QStringLiteral("图层已锁定"));
			return false;
		}
		m_views.removeAt(i);
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::renameView(const QString& viewId, const QString& title)
{
	const QString t = title.trimmed();
	if (t.isEmpty())
		return false;
	for (DrawingView& v : m_views)
	{
		if (v.id != viewId)
			continue;
		v.title = t;
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::setDetailViewScale(const QString& viewId, double scale)
{
	if (scale < 1.5 || scale > 10.0)
		return false;
	for (DrawingView& v : m_views)
	{
		if (v.id != viewId || v.kind != QLatin1String("detail"))
			continue;
		if (!isLayerEditable(v.layerId))
		{
			emit statusMessage(QStringLiteral("图层已锁定"));
			return false;
		}
		const double old = v.contentScale > 1e-9 ? v.contentScale : 1.0;
		const double ratio = scale / old;
		const QPointF origin = v.frame.topLeft() + QPointF(8, 18);
		v.visible = scalePolylinesAbout(v.visible, origin, ratio);
		v.hidden = scalePolylinesAbout(v.hidden, origin, ratio);
		v.contentScale = scale;
		v.title = QStringLiteral("局部 ×%1").arg(scale, 0, 'f', 1);
		const QRectF localBox = boundsOfPolylines(v.visible, v.hidden);
		v.frame = localBox.adjusted(-8, -18, 8, 8);
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

QVector<DrawingSheetCanvasWidget::DrawingView> DrawingSheetCanvasWidget::detailViews() const
{
	QVector<DrawingView> out;
	for (const DrawingView& v : m_views)
	{
		if (v.kind == QLatin1String("detail"))
			out.push_back(v);
	}
	return out;
}

bool DrawingSheetCanvasWidget::resolveCircleDim(int entityId, QPointF& center, QPointF& rim, double& radius) const
{
	if (const SkCircle* c = m_sketch.document().findCircle(entityId))
	{
		const SkPoint* cen = m_sketch.document().findPoint(c->center);
		if (!cen)
			return false;
		center = SheetSketchAdapter::toScene(cen->p);
		radius = c->radius;
		rim = QPointF(center.x() + radius, center.y());
		return true;
	}
	if (const SkArc* arc = m_sketch.document().findArc(entityId))
	{
		const SkPoint* s = m_sketch.document().findPoint(arc->pStart);
		const SkPoint* m = m_sketch.document().findPoint(arc->pMid);
		const SkPoint* e = m_sketch.document().findPoint(arc->pEnd);
		if (!s || !m || !e)
			return false;
		SkVec2 cen;
		if (!sketchCircumcenter(s->p, m->p, e->p, cen, radius))
			return false;
		center = SheetSketchAdapter::toScene(cen);
		rim = SheetSketchAdapter::toScene(s->p);
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::resolveHlrCircleNear(const QPointF& scenePos, QPointF& center, QPointF& rim,
												   double& radius) const
{
	const double tol = snapTolMm() * 3.0;
	double bestDist = 1e100;
	QVector<QPointF> bestPts;
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		auto consider = [&](const QVector<Polyline2d>& polys) {
			for (const Polyline2d& poly : polys)
			{
				if (poly.points.size() < 3)
					continue;
				double minD = 1e100;
				for (const QPointF& p : poly.points)
					minD = qMin(minD, QLineF(p, scenePos).length());
				for (int i = 1; i < poly.points.size(); ++i)
				{
					QLineF seg(poly.points[i - 1], poly.points[i]);
					const QPointF ab = seg.p2() - seg.p1();
					const double len2 = QPointF::dotProduct(ab, ab);
					double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - seg.p1(), ab) / len2 : 0.0;
					t = qBound(0.0, t, 1.0);
					minD = qMin(minD, QLineF(seg.p1() + ab * t, scenePos).length());
				}
				if (minD > tol || minD >= bestDist)
					continue;
				QPointF c;
				double r = 0, err = 0;
				if (!fitCircle2d(poly.points, c, r, err))
					continue;
				bestDist = minD;
				bestPts = poly.points;
				center = c;
				radius = r;
			}
		};
		consider(v.visible);
		consider(v.hidden);
	}
	if (bestPts.isEmpty())
		return false;
	QLineF ray(center, scenePos);
	if (ray.length() < 1e-6)
		ray = QLineF(center, bestPts.constFirst());
	ray.setLength(radius);
	rim = ray.p2();
	return true;
}

int DrawingSheetCanvasWidget::hitDimensionIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm();
	for (int i = m_dims.size() - 1; i >= 0; --i)
	{
		const SheetDimension& d = m_dims[i];
		if (!isLayerEditable(d.layerId))
			continue;
		if (d.kind == SheetDimension::Kind::Linear)
		{
			QLineF base(d.p1, d.p2);
			if (base.length() < 1e-9)
				continue;
			base.setLength(1.0);
			const QPointF dir = base.p2() - base.p1();
			const QPointF n(-dir.y(), dir.x());
			double off = d.textOffset.y();
			if (std::abs(off) < 1e-6)
				off = -12.0;
			const QLineF dimLine(d.p1 + n * off, d.p2 + n * off);
			const QPointF a = dimLine.p1();
			const QPointF b = dimLine.p2();
			const QPointF ab = b - a;
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - a, ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			const QPointF proj = a + ab * t;
			if (QLineF(proj, scenePos).length() <= tol)
				return i;
		}
		else
		{
			if (QLineF(d.p1, scenePos).length() <= tol || QLineF(d.p2, scenePos).length() <= tol ||
				QLineF(d.p2 + d.textOffset, scenePos).length() <= tol * 2)
				return i;
			QLineF ray(d.p1, d.p2);
			const QPointF a = ray.p1();
			const QPointF b = ray.p2();
			const QPointF ab = b - a;
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - a, ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			if (QLineF(a + ab * t, scenePos).length() <= tol)
				return i;
		}
	}
	return -1;
}

int DrawingSheetCanvasWidget::hitNoteIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm() * 2.0;
	for (int i = m_notes.size() - 1; i >= 0; --i)
	{
		const SheetNote& n = m_notes[i];
		if (!isLayerEditable(n.layerId))
			continue;
		if (QLineF(n.textPos, scenePos).length() <= tol || QLineF(n.anchor, scenePos).length() <= tol)
			return i;
		QLineF ray(n.anchor, n.textPos);
		const QPointF ab = ray.p2() - ray.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - ray.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		if (QLineF(ray.p1() + ab * t, scenePos).length() <= tol)
			return i;
	}
	return -1;
}

void DrawingSheetCanvasWidget::mousePressEvent(QMouseEvent* event)
{
	const QPointF widgetPos = event->pos();
	const QPointF scenePos = widgetToScene(widgetPos);
	const QVector<QPointF> extraSnap = collectViewSnapPoints();

	if (event->button() == Qt::MiddleButton ||
		(event->button() == Qt::LeftButton && (event->modifiers() & Qt::AltModifier)))
	{
		m_panning = true;
		m_lastWidgetPos = widgetPos;
		event->accept();
		return;
	}

	if (event->button() == Qt::RightButton && isSketchTool(m_tool))
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		m_sketch.press(scenePos, true, snapTolMm(), extraSnap, m_currentLayerId);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::RightButton &&
		(m_tool == DrawingCanvasTool::DimContinuous || m_tool == DrawingCanvasTool::DimBaseline))
	{
		m_dimPicking = false;
		m_dimPickStep = 0;
		m_chainParentId.clear();
		emit statusMessage(QStringLiteral("尺寸链结束（%1 段）").arg(m_chainCount));
		m_chainCount = 0;
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && isSketchTool(m_tool))
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		m_sketch.press(sp, false, snapTolMm(), extraSnap, m_currentLayerId);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::LinearDim)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		QPointF sp = SheetSketchAdapter::toScene(snapped);
		const QPointF* orthoRef = (m_dimPicking && m_dimPickStep == 1) ? &m_dimP1 : nullptr;
		sp = snapScenePoint(sp, orthoRef);
		if (!m_dimPicking)
		{
			m_dimP1 = sp;
			m_dimP2 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("尺寸：再点第二点（可吸附轮廓）"));
		}
		else if (m_dimPickStep == 1)
		{
			m_dimP2 = sp;
			m_dimPickStep = 2;
			emit statusMessage(QStringLiteral("尺寸：再点放置尺寸线"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Linear;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			QLineF base(m_dimP1, m_dimP2);
			if (base.length() > 1e-9)
			{
				base.setLength(1.0);
				const QPointF dir = base.p2() - base.p1();
				const QPointF n(-dir.y(), dir.x());
				const QPointF mid = (m_dimP1 + m_dimP2) * 0.5;
				dim.textOffset = QPointF(0.0, QPointF::dotProduct(sp - mid, n));
				if (std::abs(dim.textOffset.y()) < 1.0)
					dim.textOffset.setY(-12.0);
			}
			dim.anchorViewId = inferAnchorViewId((m_dimP1 + m_dimP2) * 0.5);
			dim.layerId = m_currentLayerId;
			if (!m_dimStyles.isEmpty())
				dim.styleId = m_currentDimStyleId;
			assignDimEdgeKeys(dim);
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加线性尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton &&
		(m_tool == DrawingCanvasTool::DimContinuous || m_tool == DrawingCanvasTool::DimBaseline))
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const QPointF sp = snapScenePoint(scenePos, m_dimPicking ? &m_dimP1 : nullptr);
		if (!m_dimPicking)
		{
			m_dimP1 = sp;
			m_dimP2 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			m_chainParentId.clear();
			m_chainCount = 0;
			emit statusMessage(QStringLiteral("尺寸链：再点下一点"));
			event->accept();
			return;
		}
		if (m_tool == DrawingCanvasTool::DimContinuous)
		{
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Linear;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = sp;
			dim.textOffset = QPointF(0.0, -12.0);
			dim.anchorViewId = inferAnchorViewId((dim.p1 + dim.p2) * 0.5);
			dim.layerId = m_currentLayerId;
			dim.styleId = m_currentDimStyleId;
			if (m_chainParentId.isEmpty())
				m_chainParentId = dim.id;
			else
				dim.chainParentId = m_chainParentId;
			assignDimEdgeKeys(dim);
			m_dims.push_back(dim);
			m_dimP1 = sp;
			++m_chainCount;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已加连续段，继续点或右键结束（%1）").arg(m_chainCount));
			update();
			event->accept();
			return;
		}
		// DimBaseline
		if (m_dimPickStep == 1)
		{
			m_dimP2 = sp;
			m_dimPickStep = 2;
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Linear;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			dim.textOffset = QPointF(0.0, m_chainBaseOffset);
			dim.anchorViewId = inferAnchorViewId((dim.p1 + dim.p2) * 0.5);
			dim.layerId = m_currentLayerId;
			dim.styleId = m_currentDimStyleId;
			m_chainParentId = dim.id;
			assignDimEdgeKeys(dim);
			m_dims.push_back(dim);
			m_chainCount = 1;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("基线：再点下一点延伸，右键结束"));
			update();
			event->accept();
			return;
		}
		SheetDimension dim;
		dim.kind = SheetDimension::Kind::Linear;
		dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
		dim.p1 = m_dimP1;
		dim.p2 = sp;
		m_chainBaseOffset -= 8.0;
		dim.textOffset = QPointF(0.0, m_chainBaseOffset);
		dim.anchorViewId = inferAnchorViewId((dim.p1 + dim.p2) * 0.5);
		dim.layerId = m_currentLayerId;
		dim.styleId = m_currentDimStyleId;
		dim.chainParentId = m_chainParentId;
		assignDimEdgeKeys(dim);
		m_dims.push_back(dim);
		++m_chainCount;
		emit sheetChanged();
		emit statusMessage(QStringLiteral("已加基线段，继续点或右键结束（%1）").arg(m_chainCount));
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton &&
		(m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter))
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		if (!m_dimPicking)
		{
			QPointF c, rim;
			double r = 0;
			const int ent = hitSketchEntity(scenePos, false);
			bool ok = (ent >= 0 && resolveCircleDim(ent, c, rim, r));
			if (ok)
				m_dimEntityId = ent;
			else
			{
				m_dimEntityId = -1;
				ok = resolveHlrCircleNear(scenePos, c, rim, r);
			}
			if (ok)
			{
				m_dimP1 = c;
				QLineF ray(c, scenePos);
				if (ray.length() < 1e-6)
					ray = QLineF(c, rim);
				ray.setLength(r);
				m_dimP2 = ray.p2();
				m_dimPicking = true;
				m_dimPickStep = 1;
				emit statusMessage(QStringLiteral("再点文字位置"));
			}
			else
			{
				const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
				m_dimP1 = SheetSketchAdapter::toScene(snapped);
				m_dimP2 = m_dimP1;
				m_dimPicking = true;
				m_dimPickStep = 10;
				m_dimEntityId = -1;
				emit statusMessage(QStringLiteral("未拟合到圆：再点圆周一点"));
			}
		}
		else if (m_dimPickStep == 10)
		{
			const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
			m_dimP2 = SheetSketchAdapter::toScene(snapped);
			if (QLineF(m_dimP1, m_dimP2).length() < 1e-6)
			{
				emit statusMessage(QStringLiteral("半径过小，请重选圆周点"));
				event->accept();
				return;
			}
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("再点文字位置"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = (m_tool == DrawingCanvasTool::DimDiameter) ? SheetDimension::Kind::Diameter
																 : SheetDimension::Kind::Radius;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			dim.sketchEntityId = m_dimEntityId;
			dim.textOffset = scenePos - m_dimP2;
			dim.anchorViewId = inferAnchorViewId(m_dimP1);
			dim.layerId = m_currentLayerId;
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			m_dimEntityId = -1;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::DimAngle)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_dimPicking)
		{
			m_dimP1 = m_dimP2 = m_dimP3 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("角度：再点第一边上一点"));
		}
		else if (m_dimPickStep == 1)
		{
			m_dimP2 = sp;
			m_dimPickStep = 2;
			emit statusMessage(QStringLiteral("角度：再点第二边上一点"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Angle;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			dim.p3 = sp;
			dim.anchorViewId = inferAnchorViewId(m_dimP1);
			dim.layerId = m_currentLayerId;
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加角度尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::NoteLeader)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_notePicking)
		{
			m_noteAnchor = sp;
			m_dimP2 = sp;
			m_notePicking = true;
			emit statusMessage(QStringLiteral("文字：再点文字位置"));
		}
		else
		{
			bool ok = false;
			const QString text = QInputDialog::getText(this, QStringLiteral("引线文字"), QStringLiteral("内容"),
													  QLineEdit::Normal, QStringLiteral("注"), &ok);
			if (ok && !text.trimmed().isEmpty())
			{
				SheetNote note;
				note.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
				note.anchor = m_noteAnchor;
				note.textPos = sp;
				note.text = text.trimmed();
				note.anchorViewId = inferAnchorViewId(m_noteAnchor);
				note.layerId = m_currentLayerId;
				m_notes.push_back(note);
				emit sheetChanged();
				emit statusMessage(QStringLiteral("已添加文字"));
			}
			m_notePicking = false;
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::MatchProp)
	{
		if (!m_hasMatchStyle)
		{
			selectAtScene(scenePos);
			if (!matchPropFromSelection())
				emit statusMessage(QStringLiteral("请先选中源对象"));
			update();
			event->accept();
			return;
		}
		selectAtScene(scenePos);
		if (applyStyleToSelection(m_matchStyle, m_matchLayerId))
			emit statusMessage(QStringLiteral("已匹配特性"));
		else
			emit statusMessage(QStringLiteral("未命中目标"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyErase)
	{
		selectAtScene(scenePos);
		eraseSelection();
		emit statusMessage(QStringLiteral("已删除"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyTrim)
	{
		if (trimSketchAt(scenePos))
			emit statusMessage(QStringLiteral("已修剪"));
		else
			emit statusMessage(QStringLiteral("未命中可修剪线段"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyOffset)
	{
		bool ok = false;
		const double dist = QInputDialog::getDouble(this, QStringLiteral("偏移"), QStringLiteral("距离 mm"), 5.0, -1e4,
													1e4, 2, &ok);
		if (ok && offsetSketchAt(scenePos, dist))
			emit statusMessage(QStringLiteral("已偏移"));
		else if (ok)
			emit statusMessage(QStringLiteral("偏移失败（需选中线段）"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyFillet)
	{
		bool ok = false;
		const double r =
			QInputDialog::getDouble(this, QStringLiteral("圆角"), QStringLiteral("半径 mm"), 3.0, 0.1, 1e4, 2, &ok);
		if (ok && filletSketchAt(scenePos, r))
			emit statusMessage(QStringLiteral("已圆角"));
		else if (ok)
			emit statusMessage(QStringLiteral("圆角失败（需两条相交线附近）"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyChamfer)
	{
		bool ok = false;
		const double d =
			QInputDialog::getDouble(this, QStringLiteral("倒角"), QStringLiteral("距离 mm"), 3.0, 0.1, 1e4, 2, &ok);
		if (ok && chamferSketchAt(scenePos, d))
			emit statusMessage(QStringLiteral("已倒角"));
		else if (ok)
			emit statusMessage(QStringLiteral("倒角失败（需两条相交线附近）"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyExtend)
	{
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		if (!m_extendPicking)
		{
			m_modifyP1 = sp;
			m_extendPicking = true;
			emit statusMessage(QStringLiteral("延伸：再点要延伸的线段"));
		}
		else
		{
			if (extendSketchAt(m_modifyP1, sp))
				emit statusMessage(QStringLiteral("已延伸"));
			else
				emit statusMessage(QStringLiteral("延伸失败"));
			m_extendPicking = false;
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyArray)
	{
		const bool hasSel = m_selectedViewIndex >= 0 || m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 ||
							m_selectedBlockRefIndex >= 0;
		if (!hasSel)
			selectAtScene(scenePos);
		bool ok = false;
		const int cols = QInputDialog::getInt(this, QStringLiteral("矩形阵列"), QStringLiteral("列数"), 3, 1, 50, 1, &ok);
		if (!ok)
		{
			event->accept();
			return;
		}
		const int rows = QInputDialog::getInt(this, QStringLiteral("矩形阵列"), QStringLiteral("行数"), 2, 1, 50, 1, &ok);
		if (!ok)
		{
			event->accept();
			return;
		}
		const double dx =
			QInputDialog::getDouble(this, QStringLiteral("矩形阵列"), QStringLiteral("列距 mm"), 40.0, 0.1, 1e5, 2, &ok);
		if (!ok)
		{
			event->accept();
			return;
		}
		const double dy =
			QInputDialog::getDouble(this, QStringLiteral("矩形阵列"), QStringLiteral("行距 mm"), 40.0, 0.1, 1e5, 2, &ok);
		if (ok && arraySelectionRect(cols, rows, dx, dy))
			emit statusMessage(QStringLiteral("已阵列"));
		else if (ok)
			emit statusMessage(QStringLiteral("请先选中视图/尺寸/注释/块"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyPolarArray)
	{
		const bool hasSel = m_selectedViewIndex >= 0 || m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 ||
							m_selectedBlockRefIndex >= 0;
		if (!hasSel)
			selectAtScene(scenePos);
		const QPointF pivot = snapScenePoint(scenePos, nullptr);
		bool ok = false;
		const int count =
			QInputDialog::getInt(this, QStringLiteral("环形阵列"), QStringLiteral("数量"), 6, 2, 36, 1, &ok);
		if (!ok)
		{
			event->accept();
			return;
		}
		const double ang = QInputDialog::getDouble(this, QStringLiteral("环形阵列"), QStringLiteral("总角度 °"), 360.0,
												   -3600.0, 3600.0, 1, &ok);
		if (ok && arraySelectionPolar(count, ang / count, pivot))
			emit statusMessage(QStringLiteral("已环形阵列"));
		else if (ok)
			emit statusMessage(QStringLiteral("请先选中视图/尺寸/注释/块"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyBreak)
	{
		if (breakSketchAt(scenePos))
			emit statusMessage(QStringLiteral("已打断"));
		else
			emit statusMessage(QStringLiteral("未命中可打断线段"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyJoin)
	{
		if (joinSketchAt(scenePos))
			emit statusMessage(QStringLiteral("已合并"));
		else
			emit statusMessage(QStringLiteral("合并失败（需共线近端点）"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ModifyStretch)
	{
		const QPointF sp = snapScenePoint(scenePos, m_modifyStep >= 1 ? &m_modifyP1 : nullptr);
		if (!m_stretchHasWindow)
		{
			if (!m_modifyPicking)
			{
				m_modifyP1 = sp;
				m_modifyP2 = sp;
				m_modifyPicking = true;
				m_modifyStep = 1;
				emit statusMessage(QStringLiteral("拉伸：再点窗口对角"));
			}
			else
			{
				m_stretchWindow = QRectF(m_modifyP1, sp).normalized();
				m_stretchHasWindow = true;
				m_modifyPicking = false;
				m_modifyStep = 0;
				emit statusMessage(QStringLiteral("拉伸：再点基点"));
			}
			update();
			event->accept();
			return;
		}
		if (!m_modifyPicking)
		{
			m_modifyP1 = sp;
			m_modifyPicking = true;
			m_modifyStep = 1;
			emit statusMessage(QStringLiteral("拉伸：再点目标点"));
			event->accept();
			return;
		}
		m_modifyP2 = sp;
		if (stretchSketchWindow(m_stretchWindow, m_modifyP2 - m_modifyP1))
			emit statusMessage(QStringLiteral("已拉伸"));
		m_modifyPicking = false;
		m_modifyStep = 0;
		m_stretchHasWindow = false;
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::SymRoughness)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		bool ok = false;
		const double ra = QInputDialog::getDouble(this, QStringLiteral("粗糙度"), QStringLiteral("Ra"), 3.2, 0.025,
												  100.0, 3, &ok);
		if (!ok)
		{
			event->accept();
			return;
		}
		SheetNote n;
		n.kind = SheetNote::Kind::Roughness;
		n.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
		n.anchor = sp;
		n.textPos = sp + QPointF(8.0, -6.0);
		n.roughnessRa = ra;
		n.text = QStringLiteral("Ra%1").arg(ra, 0, 'g', 3);
		n.layerId = m_currentLayerId;
		m_notes.push_back(n);
		emit sheetChanged();
		emit statusMessage(QStringLiteral("已添加粗糙度"));
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::SymGdt)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const QPointF sp = snapScenePoint(scenePos, m_notePicking ? &m_noteAnchor : nullptr);
		if (!m_notePicking)
		{
			m_noteAnchor = sp;
			m_dimP2 = sp;
			m_notePicking = true;
			emit statusMessage(QStringLiteral("形位公差：再点框位置"));
			event->accept();
			return;
		}
		bool ok = false;
		const QStringList codes{QStringLiteral("位置度"), QStringLiteral("平行度"), QStringLiteral("平面度"),
								QStringLiteral("垂直度"), QStringLiteral("圆度")};
		const QString code = QInputDialog::getItem(this, QStringLiteral("形位公差"), QStringLiteral("代号"), codes, 0,
												   false, &ok);
		if (!ok)
		{
			m_notePicking = false;
			event->accept();
			return;
		}
		const QString tol = QInputDialog::getText(this, QStringLiteral("形位公差"), QStringLiteral("公差值"),
												  QLineEdit::Normal, QStringLiteral("0.05"), &ok);
		if (!ok)
		{
			m_notePicking = false;
			event->accept();
			return;
		}
		SheetNote n;
		n.kind = SheetNote::Kind::Gdt;
		n.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
		n.anchor = m_noteAnchor;
		n.textPos = sp;
		n.gdtCode = code;
		n.text = QStringLiteral("%1|%2").arg(code, tol);
		n.layerId = m_currentLayerId;
		m_notes.push_back(n);
		m_notePicking = false;
		emit sheetChanged();
		emit statusMessage(QStringLiteral("已添加形位公差框"));
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ExplodeBlock)
	{
		selectAtScene(scenePos);
		explodeSelectedBlock();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::InsertBlock)
	{
		if (m_pendingInsertBlockId.isEmpty())
		{
			emit statusMessage(QStringLiteral("请先从 Ribbon 选择要插入的块"));
			event->accept();
			return;
		}
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		if (insertBlock(m_pendingInsertBlockId, sp))
			emit statusMessage(QStringLiteral("已插入块"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::ProjectionGuide)
	{
		if (beginGuideTipDrag(scenePos))
			emit statusMessage(QStringLiteral("拖动投影线端点（吸附视图框边）"));
		else if (removeProjectionGuideAt(scenePos))
			;
		else
			emit statusMessage(QStringLiteral("未命中投影线"));
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::MText)
	{
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		bool ok = false;
		const QString text = QInputDialog::getMultiLineText(this, QStringLiteral("多行文字"), QStringLiteral("内容"),
															QStringLiteral("文字"), &ok);
		if (ok && !text.trimmed().isEmpty())
		{
			SheetNote note;
			note.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
			note.anchor = sp;
			note.textPos = sp + QPointF(8, -4);
			note.text = text;
			note.layerId = m_currentLayerId;
			if (!m_textStyles.isEmpty())
				note.textStyleId = m_textStyles.first().id;
			m_notes.push_back(note);
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加多行文字"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton &&
		(m_tool == DrawingCanvasTool::ModifyMove || m_tool == DrawingCanvasTool::ModifyCopy ||
		 m_tool == DrawingCanvasTool::ModifyRotate || m_tool == DrawingCanvasTool::ModifyMirror ||
		 m_tool == DrawingCanvasTool::ModifyScale))
	{
		const QPointF sp = snapScenePoint(scenePos, m_modifyStep >= 1 ? &m_modifyP1 : nullptr);
		const bool hasSel = m_selectedViewIndex >= 0 || m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 ||
							m_selectedSketchId >= 0 || m_selectedHatchIndex >= 0 || m_selectedBlockRefIndex >= 0;
		if (!hasSel)
		{
			selectAtScene(scenePos);
			const bool ok = m_selectedViewIndex >= 0 || m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 ||
							m_selectedSketchId >= 0 || m_selectedHatchIndex >= 0 || m_selectedBlockRefIndex >= 0;
			emit statusMessage(ok ? QStringLiteral("已选中，再点基点") : QStringLiteral("未命中对象"));
			update();
			event->accept();
			return;
		}
		if (!m_modifyPicking)
		{
			m_modifyP1 = sp;
			m_modifyPicking = true;
			m_modifyStep = 1;
			emit statusMessage(m_tool == DrawingCanvasTool::ModifyMirror ? QStringLiteral("再点镜像轴第二点")
																		: QStringLiteral("再点目标点"));
			event->accept();
			return;
		}
		m_modifyP2 = sp;
		if (m_tool == DrawingCanvasTool::ModifyCopy)
			duplicateSelection();
		if (m_tool == DrawingCanvasTool::ModifyMove || m_tool == DrawingCanvasTool::ModifyCopy)
			transformSelection(m_modifyP2 - m_modifyP1);
		else if (m_tool == DrawingCanvasTool::ModifyRotate)
		{
			const QPointF d = m_modifyP2 - m_modifyP1;
			rotateSelection(m_modifyP1, std::atan2(d.y(), d.x()));
		}
		else if (m_tool == DrawingCanvasTool::ModifyMirror)
			mirrorSelection(QLineF(m_modifyP1, m_modifyP2));
		else if (m_tool == DrawingCanvasTool::ModifyScale)
		{
			const double base = 20.0;
			const double len = QLineF(m_modifyP1, m_modifyP2).length();
			scaleSelection(m_modifyP1, len / base);
		}
		m_modifyPicking = false;
		m_modifyStep = 0;
		emit statusMessage(QStringLiteral("修改完成"));
		event->accept();
		return;
	}

	if (event->button() == Qt::RightButton && m_tool == DrawingCanvasTool::HatchPick)
	{
		if (m_hatchPickPts.size() >= 3)
		{
			SheetHatch h;
			h.id = QStringLiteral("h_%1").arg(m_nextHatchId++);
			h.boundary = m_hatchPickPts;
			h.layerId = m_currentLayerId;
			bool ok = false;
			const QStringList patterns{QStringLiteral("SOLID"), QStringLiteral("ANSI31")};
			const QString pat = QInputDialog::getItem(this, QStringLiteral("填充"), QStringLiteral("图案"), patterns, 0,
													 false, &ok);
			if (ok)
				h.pattern = pat;
			m_hatches.push_back(h);
			m_hatchPickPts.clear();
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加填充"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::HatchPick)
	{
		m_hatchPickPts.push_back(snapScenePoint(scenePos, m_hatchPickPts.isEmpty() ? nullptr : &m_hatchPickPts.last()));
		emit statusMessage(QStringLiteral("填充点 %1（右键完成）").arg(m_hatchPickPts.size()));
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::TextNote)
	{
		const QPointF sp = snapScenePoint(scenePos, nullptr);
		bool ok = false;
		const QString text = QInputDialog::getText(this, QStringLiteral("单行文字"), QStringLiteral("内容"),
												  QLineEdit::Normal, QStringLiteral("文字"), &ok);
		if (ok && !text.trimmed().isEmpty())
		{
			SheetNote note;
			note.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
			note.anchor = sp;
			note.textPos = sp + QPointF(8, -4);
			note.text = text.trimmed();
			note.layerId = m_currentLayerId;
			if (!m_textStyles.isEmpty())
				note.textStyleId = m_textStyles.first().id;
			m_notes.push_back(note);
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加文字"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::SelectEntity)
	{
		selectAtScene(scenePos);
		emit statusMessage(m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 || m_selectedSketchId >= 0 ||
								   m_selectedViewIndex >= 0 || m_selectedHatchIndex >= 0 || m_selectedBlockRefIndex >= 0
							   ? QStringLiteral("已选中，按 Delete 删除")
							   : QStringLiteral("未命中"));
		emitSelectionChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::DetailRegion)
	{
		m_detailDragging = true;
		m_detailStart = m_detailCurrent = scenePos;
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::PanSelect)
	{
		if (beginGuideTipDrag(scenePos))
		{
			emit statusMessage(QStringLiteral("拖动投影线端点"));
			event->accept();
			return;
		}
		if (beginSectionMarkDrag(scenePos))
		{
			emit statusMessage(QStringLiteral("拖动剖切/局部符号端点"));
			event->accept();
			return;
		}
		const int hit = hitViewIndex(scenePos);
		if (hit >= 0)
		{
			if (event->modifiers() & Qt::ControlModifier)
			{
				m_selectedDimIndex = -1;
				m_selectedNoteIndex = -1;
				m_selectedSketchId = -1;
				m_selectedHatchIndex = -1;
				m_selectedBlockRefIndex = -1;
				toggleViewSelection(hit);
			}
			else
			{
				clearSelection();
				setViewSelection({hit}, hit);
			}
			m_draggingView = true;
			m_dragViewIndex = hit;
			m_lastWidgetPos = widgetPos;
			update();
			event->accept();
			return;
		}
		clearSelection();
		selectAtScene(scenePos);
		update();
	}

	QWidget::mousePressEvent(event);
}

void DrawingSheetCanvasWidget::mouseMoveEvent(QMouseEvent* event)
{
	const QPointF widgetPos = event->pos();
	const QPointF scenePos = widgetToScene(widgetPos);
	if (m_panning)
	{
		m_panOffset += widgetPos - m_lastWidgetPos;
		m_lastWidgetPos = widgetPos;
		emit viewChanged(m_zoom);
		update();
		event->accept();
		return;
	}
	if (m_guideTipDragging)
	{
		updateGuideTipDrag(scenePos);
		event->accept();
		return;
	}
	if (m_sectionMarkDragging)
	{
		updateSectionMarkDrag(scenePos);
		event->accept();
		return;
	}
	if (m_draggingView && m_dragViewIndex >= 0)
	{
		const QPointF deltaWidget = widgetPos - m_lastWidgetPos;
		m_lastWidgetPos = widgetPos;
		const QPointF delta = deltaWidget / qMax(0.001, m_zoom);
		QVector<int> moveIdxs = m_selectedViewIndices;
		if (moveIdxs.isEmpty())
			moveIdxs.push_back(m_dragViewIndex);
		else if (!moveIdxs.contains(m_dragViewIndex))
			moveIdxs.push_back(m_dragViewIndex);
		for (int i : moveIdxs)
		{
			QPointF d = delta;
			if (m_projectionDragLock)
				d = constrainViewDragDelta(i, d);
			moveViewBy(i, d);
		}
		event->accept();
		return;
	}
	if (m_detailDragging)
	{
		m_detailCurrent = scenePos;
		update();
		event->accept();
		return;
	}
	if (isSketchTool(m_tool))
	{
		m_sketch.move(scenePos, snapTolMm(), collectViewSnapPoints());
		updatePickFeedback(scenePos);
		update();
		event->accept();
		return;
	}

	const QPointF* orthoRef = nullptr;
	if (m_dimPicking && m_dimPickStep >= 1)
		orthoRef = &m_dimP1;
	else if (m_modifyPicking && m_modifyStep >= 1)
		orthoRef = &m_modifyP1;
	else if (m_notePicking)
		orthoRef = &m_noteAnchor;

	if (m_dimPicking && isDimTool(m_tool))
	{
		const SheetSnapResult snap = snapSceneResult(scenePos, orthoRef);
		const QPointF sp = snap.snapped ? snap.pos : scenePos;
		if (m_tool == DrawingCanvasTool::LinearDim && m_dimPickStep == 1)
			m_dimP2 = sp;
		else if (m_tool == DrawingCanvasTool::DimContinuous || m_tool == DrawingCanvasTool::DimBaseline)
			m_dimP2 = sp;
		else if (m_tool == DrawingCanvasTool::DimAngle)
		{
			if (m_dimPickStep == 1)
				m_dimP2 = sp;
			else
				m_dimP3 = sp;
		}
		else if (m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter)
		{
			if (m_dimPickStep == 10)
				m_dimP2 = sp;
			else if (m_dimPickStep == 1 && m_dimEntityId < 0)
			{
				// 已有圆心：沿半径方向跟手
				QLineF ray(m_dimP1, sp);
				if (ray.length() > 1e-6)
				{
					ray.setLength(QLineF(m_dimP1, m_dimP2).length());
					if (ray.length() < 1e-6)
						ray.setLength(QLineF(m_dimP1, sp).length());
					m_dimP2 = ray.p2();
				}
			}
			else if (m_dimPickStep == 1)
			{
				QLineF ray(m_dimP1, sp);
				const double r = QLineF(m_dimP1, m_dimP2).length();
				if (ray.length() > 1e-6 && r > 1e-6)
				{
					ray.setLength(r);
					m_dimP2 = ray.p2();
				}
			}
		}
		updatePickFeedback(scenePos);
		update();
		event->accept();
		return;
	}
	if (m_notePicking)
	{
		const SheetSnapResult snap = snapSceneResult(scenePos, orthoRef);
		m_dimP2 = snap.snapped ? snap.pos : scenePos;
		updatePickFeedback(scenePos);
		update();
		event->accept();
		return;
	}
	if (m_modifyPicking && m_modifyStep >= 1)
	{
		const SheetSnapResult snap = snapSceneResult(scenePos, &m_modifyP1);
		m_modifyP2 = snap.snapped ? snap.pos : scenePos;
		updatePickFeedback(scenePos);
		update();
		event->accept();
		return;
	}

	updatePickFeedback(scenePos);
	if (isPickFeedbackTool(m_tool))
	{
		update();
		event->accept();
		return;
	}
	QWidget::mouseMoveEvent(event);
}

void DrawingSheetCanvasWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (m_panning && (event->button() == Qt::MiddleButton || event->button() == Qt::LeftButton))
	{
		m_panning = false;
		m_interactiveCacheValid = false;
		event->accept();
		return;
	}
	if (m_guideTipDragging && event->button() == Qt::LeftButton)
	{
		endGuideTipDrag();
		event->accept();
		return;
	}
	if (m_sectionMarkDragging && event->button() == Qt::LeftButton)
	{
		endSectionMarkDrag();
		event->accept();
		return;
	}
	if (m_draggingView && event->button() == Qt::LeftButton)
	{
		m_draggingView = false;
		m_dragViewIndex = -1;
		m_interactiveCacheValid = false;
		event->accept();
		return;
	}
	if (m_detailDragging && event->button() == Qt::LeftButton)
	{
		m_detailDragging = false;
		const QRectF region = QRectF(m_detailStart, m_detailCurrent).normalized();
		int parentIdx = hitViewIndex(region.center());
		if (parentIdx < 0)
			parentIdx = hitViewIndex(m_detailStart);
		if (parentIdx >= 0)
		{
			addDetailView(m_views[parentIdx].id, region, m_detailScale);
			emit statusMessage(QStringLiteral("已添加局部放大视图"));
		}
		else
		{
			emit statusMessage(QStringLiteral("未命中父视图"));
		}
		update();
		event->accept();
		return;
	}
	QWidget::mouseReleaseEvent(event);
}

void DrawingSheetCanvasWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (event->button() != Qt::LeftButton)
	{
		QWidget::mouseDoubleClickEvent(event);
		return;
	}
	const int idx = hitDimensionIndex(widgetToScene(event->pos()));
	if (idx < 0)
	{
		QWidget::mouseDoubleClickEvent(event);
		return;
	}
	if (!isLayerEditable(m_dims[idx].layerId))
	{
		emit statusMessage(QStringLiteral("图层已锁定"));
		event->accept();
		return;
	}
	SheetDimension& dim = m_dims[idx];
	bool ok = false;
	const double cur = dimensionValue(dim);
	const double v = QInputDialog::getDouble(this, QStringLiteral("编辑尺寸"), QStringLiteral("显示值"), cur, -1e9,
											 1e9, 3, &ok);
	if (ok)
	{
		dim.overrideValue = v;
		const bool showTol = QInputDialog::getInt(this, QStringLiteral("公差"), QStringLiteral("显示公差？0=否 1=是"),
												  dim.showTolerance || dim.tolOverride ? 1 : 0, 0, 1, 1, &ok) != 0;
		if (ok && showTol)
		{
			const double tp = QInputDialog::getDouble(this, QStringLiteral("公差"), QStringLiteral("上偏差"),
													 dim.tolPlus, 0.0, 1e3, 3, &ok);
			if (!ok)
			{
				emit sheetChanged();
				update();
				event->accept();
				return;
			}
			const double tm = QInputDialog::getDouble(this, QStringLiteral("公差"), QStringLiteral("下偏差"),
													 dim.tolMinus, 0.0, 1e3, 3, &ok);
			if (ok)
			{
				dim.tolOverride = true;
				dim.showTolerance = true;
				dim.tolPlus = tp;
				dim.tolMinus = tm;
			}
		}
		else if (ok && !showTol)
		{
			dim.tolOverride = true;
			dim.showTolerance = false;
		}
		emit sheetChanged();
		update();
	}
	event->accept();
}

void DrawingSheetCanvasWidget::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Escape)
	{
		m_sketch.cancelTool();
		m_dimPicking = false;
		m_dimPickStep = 0;
		m_notePicking = false;
		m_modifyPicking = false;
		m_modifyStep = 0;
		m_hatchPickPts.clear();
		clearSelection();
		update();
		event->accept();
		return;
	}
	if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
	{
		eraseSelection();
		emit statusMessage(QStringLiteral("已删除"));
		event->accept();
		return;
	}
	QWidget::keyPressEvent(event);
}

void DrawingSheetCanvasWidget::wheelEvent(QWheelEvent* event)
{
	const QPointF widgetPoint = event->position();
	const QPointF scenePoint = widgetToScene(widgetPoint);
	m_zoom *= (event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15);
	clampZoom();
	m_panOffset = widgetPoint - scenePoint * m_zoom;
	emit viewChanged(m_zoom);
	// 滚动中只重贴旧缓存，停顿 ~90ms 后再重绘，避免复杂视图逐帧卡
	m_zoomRepaintPending = true;
	if (!m_zoomDebounceTimer)
	{
		m_zoomDebounceTimer = new QTimer(this);
		m_zoomDebounceTimer->setSingleShot(true);
		connect(m_zoomDebounceTimer, &QTimer::timeout, this, [this]() {
			m_zoomRepaintPending = false;
			m_sceneCacheValid = false;
			update();
		});
	}
	m_zoomDebounceTimer->start(90);
	update();
	event->accept();
}

QJsonObject DrawingSheetCanvasWidget::toJson() const
{
	QJsonObject root;
	root.insert(QStringLiteral("version"), 9);
	root.insert(QStringLiteral("backendId"), m_backendId);
	root.insert(QStringLiteral("projection"),
				m_projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("thirdAngle")
																	: QStringLiteral("firstAngle"));
	root.insert(QStringLiteral("currentLayerId"), m_currentLayerId);
	QJsonArray layersArr;
	for (const SheetLayer& L : m_layers)
	{
		QJsonObject lo;
		lo.insert(QStringLiteral("id"), L.id);
		lo.insert(QStringLiteral("name"), L.name);
		lo.insert(QStringLiteral("visible"), L.visible);
		lo.insert(QStringLiteral("locked"), L.locked);
		lo.insert(QStringLiteral("frozen"), L.frozen);
		lo.insert(QStringLiteral("plottable"), L.plottable);
		lo.insert(QStringLiteral("color"), L.color.name(QColor::HexRgb));
		lo.insert(QStringLiteral("lineType"), lineTypeToString(L.lineType));
		lo.insert(QStringLiteral("lineWidthMm"), L.lineWidthMm);
		layersArr.append(lo);
	}
	root.insert(QStringLiteral("layers"), layersArr);
	QJsonObject paper;
	QString sizeStr = QStringLiteral("A4");
	switch (m_paper.size)
	{
	case DrawingPaperSize::A3:
		sizeStr = QStringLiteral("A3");
		break;
	case DrawingPaperSize::A2:
		sizeStr = QStringLiteral("A2");
		break;
	case DrawingPaperSize::A1:
		sizeStr = QStringLiteral("A1");
		break;
	case DrawingPaperSize::A0:
		sizeStr = QStringLiteral("A0");
		break;
	case DrawingPaperSize::Custom:
		sizeStr = QStringLiteral("custom");
		break;
	case DrawingPaperSize::A4:
	default:
		sizeStr = QStringLiteral("A4");
		break;
	}
	paper.insert(QStringLiteral("size"), sizeStr);
	paper.insert(QStringLiteral("landscape"), m_paper.landscape);
	paper.insert(QStringLiteral("customWidthMm"), m_paper.customWidthMm);
	paper.insert(QStringLiteral("customHeightMm"), m_paper.customHeightMm);
	paper.insert(QStringLiteral("sheetScale"), m_paper.sheetScale);
	paper.insert(QStringLiteral("ltScale"), m_paper.ltScale);
	paper.insert(QStringLiteral("title"), m_paper.title);
	paper.insert(QStringLiteral("drawingNo"), m_paper.drawingNo);
	paper.insert(QStringLiteral("material"), m_paper.material);
	paper.insert(QStringLiteral("scaleText"), m_paper.scaleText);
	paper.insert(QStringLiteral("date"), m_paper.date);
	paper.insert(QStringLiteral("visible"), m_paper.visible);
	root.insert(QStringLiteral("paper"), paper);
	root.insert(QStringLiteral("currentDimStyleId"), m_currentDimStyleId);
	root.insert(QStringLiteral("ctbEnabled"), m_ctbEnabled);
	QJsonArray viewsArr;
	for (const DrawingView& v : m_views)
	{
		QJsonObject vo;
		vo.insert(QStringLiteral("id"), v.id);
		vo.insert(QStringLiteral("title"), v.title);
		vo.insert(QStringLiteral("kind"), v.kind);
		vo.insert(QStringLiteral("parentViewId"), v.parentViewId);
		vo.insert(QStringLiteral("contentScale"), v.contentScale);
		vo.insert(QStringLiteral("layerId"), v.layerId);
		vo.insert(QStringLiteral("style"), styleToJson(v.style));
		vo.insert(QStringLiteral("markLetter"), v.markLetter);
		vo.insert(QStringLiteral("hasMark"), v.hasMark);
		vo.insert(QStringLiteral("markP1"), QJsonArray{v.markP1.x(), v.markP1.y()});
		vo.insert(QStringLiteral("markP2"), QJsonArray{v.markP2.x(), v.markP2.y()});
		vo.insert(QStringLiteral("frame"), QJsonArray{v.frame.x(), v.frame.y(), v.frame.width(), v.frame.height()});
		QJsonArray vis;
		for (const Polyline2d& poly : v.visible)
			vis.append(pointsToXyArray(poly.points));
		QJsonArray hid;
		for (const Polyline2d& poly : v.hidden)
			hid.append(pointsToXyArray(poly.points));
		vo.insert(QStringLiteral("visible"), vis);
		vo.insert(QStringLiteral("hidden"), hid);
		viewsArr.append(vo);
	}
	root.insert(QStringLiteral("views"), viewsArr);
	QJsonArray dimsArr;
	for (const SheetDimension& d : m_dims)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), d.id);
		QString kind = QStringLiteral("linear");
		if (d.kind == SheetDimension::Kind::Radius)
			kind = QStringLiteral("radius");
		else if (d.kind == SheetDimension::Kind::Diameter)
			kind = QStringLiteral("diameter");
		else if (d.kind == SheetDimension::Kind::Angle)
			kind = QStringLiteral("angle");
		o.insert(QStringLiteral("kind"), kind);
		o.insert(QStringLiteral("p1"), QJsonArray{d.p1.x(), d.p1.y()});
		o.insert(QStringLiteral("p2"), QJsonArray{d.p2.x(), d.p2.y()});
		o.insert(QStringLiteral("p3"), QJsonArray{d.p3.x(), d.p3.y()});
		o.insert(QStringLiteral("textOffset"), QJsonArray{d.textOffset.x(), d.textOffset.y()});
		o.insert(QStringLiteral("anchorViewId"), d.anchorViewId);
		o.insert(QStringLiteral("layerId"), d.layerId);
		o.insert(QStringLiteral("style"), styleToJson(d.style));
		o.insert(QStringLiteral("styleId"), d.styleId);
		o.insert(QStringLiteral("sketchEntityId"), d.sketchEntityId);
		if (std::isfinite(d.overrideValue))
			o.insert(QStringLiteral("overrideValue"), d.overrideValue);
		o.insert(QStringLiteral("tolOverride"), d.tolOverride);
		o.insert(QStringLiteral("showTolerance"), d.showTolerance);
		o.insert(QStringLiteral("tolPlus"), d.tolPlus);
		o.insert(QStringLiteral("tolMinus"), d.tolMinus);
		o.insert(QStringLiteral("anchorEdgeKey"), d.anchorEdgeKey);
		o.insert(QStringLiteral("chainParentId"), d.chainParentId);
		dimsArr.append(o);
	}
	root.insert(QStringLiteral("dimensions"), dimsArr);
	QJsonArray notesArr;
	for (const SheetNote& n : m_notes)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), n.id);
		QString nk = QStringLiteral("leader");
		if (n.kind == SheetNote::Kind::Roughness)
			nk = QStringLiteral("roughness");
		else if (n.kind == SheetNote::Kind::Gdt)
			nk = QStringLiteral("gdt");
		o.insert(QStringLiteral("noteKind"), nk);
		o.insert(QStringLiteral("anchor"), QJsonArray{n.anchor.x(), n.anchor.y()});
		o.insert(QStringLiteral("textPos"), QJsonArray{n.textPos.x(), n.textPos.y()});
		o.insert(QStringLiteral("text"), n.text);
		o.insert(QStringLiteral("gdtCode"), n.gdtCode);
		o.insert(QStringLiteral("roughnessRa"), n.roughnessRa);
		o.insert(QStringLiteral("anchorViewId"), n.anchorViewId);
		o.insert(QStringLiteral("layerId"), n.layerId);
		o.insert(QStringLiteral("style"), styleToJson(n.style));
		o.insert(QStringLiteral("textStyleId"), n.textStyleId);
		notesArr.append(o);
	}
	root.insert(QStringLiteral("notes"), notesArr);
	QJsonArray hatchArr;
	for (const SheetHatch& h : m_hatches)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), h.id);
		o.insert(QStringLiteral("boundary"), pointsToXyArray(h.boundary));
		o.insert(QStringLiteral("pattern"), h.pattern);
		o.insert(QStringLiteral("scale"), h.scale);
		o.insert(QStringLiteral("angleDeg"), h.angleDeg);
		o.insert(QStringLiteral("layerId"), h.layerId);
		o.insert(QStringLiteral("style"), styleToJson(h.style));
		o.insert(QStringLiteral("anchorViewId"), h.anchorViewId);
		hatchArr.append(o);
	}
	root.insert(QStringLiteral("hatches"), hatchArr);
	root.insert(QStringLiteral("projectionDragLock"), m_projectionDragLock);
	root.insert(QStringLiteral("projectionPinned"), m_projectionPinned);
	root.insert(QStringLiteral("halfSection"), m_halfSection);
	QJsonArray guidesArr;
	for (const SheetProjectionGuide& g : m_projectionGuides)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), g.id);
		o.insert(QStringLiteral("fromViewId"), g.fromViewId);
		o.insert(QStringLiteral("toViewId"), g.toViewId);
		o.insert(QStringLiteral("axis"), g.axis == SheetProjectionGuide::Axis::Vertical ? QStringLiteral("V")
																					   : QStringLiteral("H"));
		o.insert(QStringLiteral("visible"), g.visible);
		o.insert(QStringLiteral("layerId"), g.layerId);
		o.insert(QStringLiteral("tipsCustom"), g.tipsCustom);
		o.insert(QStringLiteral("tipA"), QJsonArray{g.tipA.x(), g.tipA.y()});
		o.insert(QStringLiteral("tipB"), QJsonArray{g.tipB.x(), g.tipB.y()});
		guidesArr.append(o);
	}
	root.insert(QStringLiteral("projectionGuides"), guidesArr);
	QJsonArray dimStylesArr;
	for (const DimStyle& s : m_dimStyles)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), s.id);
		o.insert(QStringLiteral("name"), s.name);
		o.insert(QStringLiteral("textHeightMm"), s.textHeightMm);
		o.insert(QStringLiteral("arrowSizeMm"), s.arrowSizeMm);
		o.insert(QStringLiteral("precision"), s.precision);
		o.insert(QStringLiteral("tolPlus"), s.tolPlus);
		o.insert(QStringLiteral("tolMinus"), s.tolMinus);
		o.insert(QStringLiteral("showTolerance"), s.showTolerance);
		dimStylesArr.append(o);
	}
	root.insert(QStringLiteral("dimStyles"), dimStylesArr);
	QJsonArray textStylesArr;
	for (const TextStyle& s : m_textStyles)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), s.id);
		o.insert(QStringLiteral("name"), s.name);
		o.insert(QStringLiteral("heightMm"), s.heightMm);
		o.insert(QStringLiteral("fontFamily"), s.fontFamily);
		textStylesArr.append(o);
	}
	root.insert(QStringLiteral("textStyles"), textStylesArr);
	QJsonArray blockDefsArr;
	for (const SheetBlockDef& d : m_blockDefs)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), d.id);
		o.insert(QStringLiteral("name"), d.name);
		o.insert(QStringLiteral("base"), QJsonArray{d.base.x(), d.base.y()});
		QJsonArray g;
		for (const Polyline2d& poly : d.geometry)
			g.append(pointsToXyArray(poly.points));
		o.insert(QStringLiteral("geometry"), g);
		QJsonArray gs;
		for (const SheetEntityStyle& s : d.geometryStyles)
			gs.append(styleToJson(s));
		o.insert(QStringLiteral("geometryStyles"), gs);
		QJsonArray attrs;
		for (const SheetBlockDef::AttrDef& a : d.attrDefs)
		{
			QJsonObject ao;
			ao.insert(QStringLiteral("tag"), a.tag);
			ao.insert(QStringLiteral("prompt"), a.prompt);
			ao.insert(QStringLiteral("defaultValue"), a.defaultValue);
			ao.insert(QStringLiteral("position"), QJsonArray{a.position.x(), a.position.y()});
			attrs.append(ao);
		}
		o.insert(QStringLiteral("attrDefs"), attrs);
		blockDefsArr.append(o);
	}
	root.insert(QStringLiteral("blockDefs"), blockDefsArr);
	QJsonArray blockRefsArr;
	for (const SheetBlockRef& r : m_blockRefs)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), r.id);
		o.insert(QStringLiteral("defId"), r.defId);
		o.insert(QStringLiteral("insert"), QJsonArray{r.insert.x(), r.insert.y()});
		o.insert(QStringLiteral("scale"), r.scale);
		o.insert(QStringLiteral("rotationDeg"), r.rotationDeg);
		o.insert(QStringLiteral("layerId"), r.layerId);
		o.insert(QStringLiteral("style"), styleToJson(r.style));
		QJsonObject av;
		for (auto it = r.attrValues.constBegin(); it != r.attrValues.constEnd(); ++it)
			av.insert(it.key(), it.value());
		o.insert(QStringLiteral("attrValues"), av);
		blockRefsArr.append(o);
	}
	root.insert(QStringLiteral("blockRefs"), blockRefsArr);
	QJsonArray ctbArr;
	for (const CtbEntry& e : m_ctbTable)
	{
		QJsonObject o;
		o.insert(QStringLiteral("aci"), e.aci);
		o.insert(QStringLiteral("widthMm"), e.widthMm);
		ctbArr.append(o);
	}
	root.insert(QStringLiteral("ctbTable"), ctbArr);
	QJsonObject sketchLayers;
	const QHash<int, QString> el = m_sketch.entityLayers();
	for (auto it = el.constBegin(); it != el.constEnd(); ++it)
		sketchLayers.insert(QString::number(it.key()), it.value());
	root.insert(QStringLiteral("sketchLayers"), sketchLayers);
	const QByteArray sketchUtf8 = m_sketch.toJsonUtf8();
	if (!sketchUtf8.isEmpty())
	{
		const QJsonDocument sd = QJsonDocument::fromJson(sketchUtf8);
		if (sd.isObject())
			root.insert(QStringLiteral("sketch"), sd.object());
	}
	return root;
}

bool DrawingSheetCanvasWidget::fromJson(const QJsonObject& root)
{
	m_views.clear();
	m_dims.clear();
	m_notes.clear();
	m_hatches.clear();
	m_blockDefs.clear();
	m_blockRefs.clear();
	m_dimStyles.clear();
	m_textStyles.clear();
	m_sketch.clear();
	m_layers.clear();
	clearSelection();
	m_backendId = root.value(QStringLiteral("backendId")).toString();
	const QString proj = root.value(QStringLiteral("projection")).toString();
	m_projection = (proj == QLatin1String("thirdAngle")) ? DrawingProjectionMethod::ThirdAngle
														 : DrawingProjectionMethod::FirstAngle;
	m_nextLayerSeq = 1;
	for (const QJsonValue& lv : root.value(QStringLiteral("layers")).toArray())
	{
		const QJsonObject lo = lv.toObject();
		SheetLayer L;
		L.id = lo.value(QStringLiteral("id")).toString();
		L.name = lo.value(QStringLiteral("name")).toString(L.id);
		L.visible = lo.value(QStringLiteral("visible")).toBool(true);
		L.locked = lo.value(QStringLiteral("locked")).toBool(false);
		L.frozen = lo.value(QStringLiteral("frozen")).toBool(false);
		L.plottable = lo.value(QStringLiteral("plottable")).toBool(true);
		const QString colorName = lo.value(QStringLiteral("color")).toString();
		if (!colorName.isEmpty())
			L.color = QColor(colorName);
		L.lineType = lineTypeFromString(lo.value(QStringLiteral("lineType")).toString());
		L.lineWidthMm = lo.value(QStringLiteral("lineWidthMm")).toDouble(0.35);
		if (!(L.lineWidthMm > 1e-6))
			L.lineWidthMm = 0.35;
		if (L.id.isEmpty())
			continue;
		m_layers.push_back(L);
		if (L.id.startsWith(QLatin1String("L")))
		{
			bool ok = false;
			const int n = L.id.mid(1).toInt(&ok);
			if (ok && n >= m_nextLayerSeq)
				m_nextLayerSeq = n + 1;
		}
	}
	ensureDefaultLayer();
	m_currentLayerId = root.value(QStringLiteral("currentLayerId")).toString(defaultLayerId());
	if (layerIndex(m_currentLayerId) < 0)
		m_currentLayerId = defaultLayerId();
	if (root.contains(QStringLiteral("paper")))
	{
		const QJsonObject po = root.value(QStringLiteral("paper")).toObject();
		const QString sz = po.value(QStringLiteral("size")).toString(QStringLiteral("A4"));
		if (sz == QLatin1String("A3"))
			m_paper.size = DrawingPaperSize::A3;
		else if (sz == QLatin1String("A2"))
			m_paper.size = DrawingPaperSize::A2;
		else if (sz == QLatin1String("A1"))
			m_paper.size = DrawingPaperSize::A1;
		else if (sz == QLatin1String("A0"))
			m_paper.size = DrawingPaperSize::A0;
		else if (sz == QLatin1String("custom") || sz == QLatin1String("Custom"))
			m_paper.size = DrawingPaperSize::Custom;
		else
			m_paper.size = DrawingPaperSize::A4;
		m_paper.landscape = po.value(QStringLiteral("landscape")).toBool(true);
		m_paper.customWidthMm = po.value(QStringLiteral("customWidthMm")).toDouble(297.0);
		m_paper.customHeightMm = po.value(QStringLiteral("customHeightMm")).toDouble(210.0);
		m_paper.sheetScale = po.value(QStringLiteral("sheetScale")).toDouble(1.0);
		if (!(m_paper.sheetScale > 1e-12))
			m_paper.sheetScale = 1.0;
		m_paper.ltScale = po.value(QStringLiteral("ltScale")).toDouble(1.0);
		if (!(m_paper.ltScale > 1e-6))
			m_paper.ltScale = 1.0;
		m_paper.title = po.value(QStringLiteral("title")).toString();
		m_paper.drawingNo = po.value(QStringLiteral("drawingNo")).toString();
		m_paper.material = po.value(QStringLiteral("material")).toString();
		m_paper.scaleText = po.value(QStringLiteral("scaleText")).toString(QStringLiteral("1:1"));
		if (m_paper.scaleText.isEmpty() || !po.contains(QStringLiteral("sheetScale")))
			syncScaleTextFromSheetScale();
		m_paper.date = po.value(QStringLiteral("date")).toString();
		m_paper.visible = po.value(QStringLiteral("visible")).toBool(true);
	}
	m_currentDimStyleId = root.value(QStringLiteral("currentDimStyleId")).toString(QStringLiteral("Standard"));
	m_ctbEnabled = root.value(QStringLiteral("ctbEnabled")).toBool(false);
	for (const QJsonValue& vv : root.value(QStringLiteral("views")).toArray())
	{
		const QJsonObject vo = vv.toObject();
		DrawingView view;
		view.id = vo.value(QStringLiteral("id")).toString();
		view.title = vo.value(QStringLiteral("title")).toString(view.id);
		view.kind = vo.value(QStringLiteral("kind")).toString(QStringLiteral("front"));
		view.parentViewId = vo.value(QStringLiteral("parentViewId")).toString();
		view.contentScale = vo.value(QStringLiteral("contentScale")).toDouble(1.0);
		view.layerId = vo.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(view.layerId) < 0)
			view.layerId = defaultLayerId();
		if (vo.contains(QStringLiteral("style")))
			view.style = styleFromJson(vo.value(QStringLiteral("style")).toObject());
		view.markLetter = vo.value(QStringLiteral("markLetter")).toString();
		view.hasMark = vo.value(QStringLiteral("hasMark")).toBool(false);
		const QJsonArray mp1 = vo.value(QStringLiteral("markP1")).toArray();
		const QJsonArray mp2 = vo.value(QStringLiteral("markP2")).toArray();
		if (mp1.size() >= 2)
			view.markP1 = QPointF(mp1.at(0).toDouble(), mp1.at(1).toDouble());
		if (mp2.size() >= 2)
			view.markP2 = QPointF(mp2.at(0).toDouble(), mp2.at(1).toDouble());
		const QJsonArray fr = vo.value(QStringLiteral("frame")).toArray();
		if (fr.size() >= 4)
			view.frame = QRectF(fr.at(0).toDouble(), fr.at(1).toDouble(), fr.at(2).toDouble(), fr.at(3).toDouble());
		for (const QJsonValue& pv : vo.value(QStringLiteral("visible")).toArray())
		{
			Polyline2d poly;
			poly.points = xyArrayToPoints(pv.toArray());
			if (poly.points.size() >= 2)
				view.visible.push_back(poly);
		}
		for (const QJsonValue& pv : vo.value(QStringLiteral("hidden")).toArray())
		{
			Polyline2d poly;
			poly.points = xyArrayToPoints(pv.toArray());
			if (poly.points.size() >= 2)
				view.hidden.push_back(poly);
		}
		m_views.push_back(view);
	}
	for (const QJsonValue& dv : root.value(QStringLiteral("dimensions")).toArray())
	{
		const QJsonObject o = dv.toObject();
		SheetDimension dim;
		dim.id = o.value(QStringLiteral("id")).toString();
		const QString kind = o.value(QStringLiteral("kind")).toString(QStringLiteral("linear"));
		if (kind == QLatin1String("radius"))
			dim.kind = SheetDimension::Kind::Radius;
		else if (kind == QLatin1String("diameter"))
			dim.kind = SheetDimension::Kind::Diameter;
		else if (kind == QLatin1String("angle"))
			dim.kind = SheetDimension::Kind::Angle;
		else
			dim.kind = SheetDimension::Kind::Linear;
		const QJsonArray a = o.value(QStringLiteral("p1")).toArray();
		const QJsonArray b = o.value(QStringLiteral("p2")).toArray();
		const QJsonArray c = o.value(QStringLiteral("p3")).toArray();
		if (a.size() >= 2)
			dim.p1 = QPointF(a.at(0).toDouble(), a.at(1).toDouble());
		if (b.size() >= 2)
			dim.p2 = QPointF(b.at(0).toDouble(), b.at(1).toDouble());
		if (c.size() >= 2)
			dim.p3 = QPointF(c.at(0).toDouble(), c.at(1).toDouble());
		const QJsonArray to = o.value(QStringLiteral("textOffset")).toArray();
		if (to.size() >= 2)
			dim.textOffset = QPointF(to.at(0).toDouble(), to.at(1).toDouble());
		dim.anchorViewId = o.value(QStringLiteral("anchorViewId")).toString();
		dim.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(dim.layerId) < 0)
			dim.layerId = defaultLayerId();
		if (o.contains(QStringLiteral("style")))
			dim.style = styleFromJson(o.value(QStringLiteral("style")).toObject());
		dim.styleId = o.value(QStringLiteral("styleId")).toString(QStringLiteral("Standard"));
		dim.sketchEntityId = o.value(QStringLiteral("sketchEntityId")).toInt(-1);
		if (o.contains(QStringLiteral("overrideValue")))
			dim.overrideValue = o.value(QStringLiteral("overrideValue")).toDouble();
		dim.tolOverride = o.value(QStringLiteral("tolOverride")).toBool(false);
		dim.showTolerance = o.value(QStringLiteral("showTolerance")).toBool(false);
		dim.tolPlus = o.value(QStringLiteral("tolPlus")).toDouble(0.0);
		dim.tolMinus = o.value(QStringLiteral("tolMinus")).toDouble(0.0);
		dim.anchorEdgeKey = o.value(QStringLiteral("anchorEdgeKey")).toString();
		dim.chainParentId = o.value(QStringLiteral("chainParentId")).toString();
		m_dims.push_back(dim);
	}
	for (const QJsonValue& nv : root.value(QStringLiteral("notes")).toArray())
	{
		const QJsonObject o = nv.toObject();
		SheetNote note;
		note.id = o.value(QStringLiteral("id")).toString();
		const QString nk = o.value(QStringLiteral("noteKind")).toString(QStringLiteral("leader"));
		if (nk == QLatin1String("roughness"))
			note.kind = SheetNote::Kind::Roughness;
		else if (nk == QLatin1String("gdt"))
			note.kind = SheetNote::Kind::Gdt;
		else
			note.kind = SheetNote::Kind::Leader;
		const QJsonArray a = o.value(QStringLiteral("anchor")).toArray();
		const QJsonArray t = o.value(QStringLiteral("textPos")).toArray();
		if (a.size() >= 2)
			note.anchor = QPointF(a.at(0).toDouble(), a.at(1).toDouble());
		if (t.size() >= 2)
			note.textPos = QPointF(t.at(0).toDouble(), t.at(1).toDouble());
		note.text = o.value(QStringLiteral("text")).toString();
		note.gdtCode = o.value(QStringLiteral("gdtCode")).toString();
		note.roughnessRa = o.value(QStringLiteral("roughnessRa")).toDouble(3.2);
		note.anchorViewId = o.value(QStringLiteral("anchorViewId")).toString();
		note.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(note.layerId) < 0)
			note.layerId = defaultLayerId();
		if (o.contains(QStringLiteral("style")))
			note.style = styleFromJson(o.value(QStringLiteral("style")).toObject());
		note.textStyleId = o.value(QStringLiteral("textStyleId")).toString(QStringLiteral("Standard"));
		m_notes.push_back(note);
	}
	for (const QJsonValue& hv : root.value(QStringLiteral("hatches")).toArray())
	{
		const QJsonObject o = hv.toObject();
		SheetHatch h;
		h.id = o.value(QStringLiteral("id")).toString();
		h.boundary = xyArrayToPoints(o.value(QStringLiteral("boundary")).toArray());
		h.pattern = o.value(QStringLiteral("pattern")).toString(QStringLiteral("SOLID"));
		h.scale = o.value(QStringLiteral("scale")).toDouble(1.0);
		h.angleDeg = o.value(QStringLiteral("angleDeg")).toDouble(0.0);
		h.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (o.contains(QStringLiteral("style")))
			h.style = styleFromJson(o.value(QStringLiteral("style")).toObject());
		h.anchorViewId = o.value(QStringLiteral("anchorViewId")).toString();
		m_hatches.push_back(h);
	}
	m_projectionDragLock = root.value(QStringLiteral("projectionDragLock")).toBool(true);
	m_projectionPinned = root.value(QStringLiteral("projectionPinned")).toBool(false);
	m_halfSection = root.value(QStringLiteral("halfSection")).toBool(false);
	m_projectionGuides.clear();
	for (const QJsonValue& gv : root.value(QStringLiteral("projectionGuides")).toArray())
	{
		const QJsonObject o = gv.toObject();
		SheetProjectionGuide g;
		g.id = o.value(QStringLiteral("id")).toString();
		g.fromViewId = o.value(QStringLiteral("fromViewId")).toString();
		g.toViewId = o.value(QStringLiteral("toViewId")).toString();
		g.axis = o.value(QStringLiteral("axis")).toString() == QLatin1String("V")
					 ? SheetProjectionGuide::Axis::Vertical
					 : SheetProjectionGuide::Axis::Horizontal;
		g.visible = o.value(QStringLiteral("visible")).toBool(true);
		g.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		g.tipsCustom = o.value(QStringLiteral("tipsCustom")).toBool(false);
		const QJsonArray ta = o.value(QStringLiteral("tipA")).toArray();
		const QJsonArray tb = o.value(QStringLiteral("tipB")).toArray();
		if (ta.size() >= 2)
			g.tipA = QPointF(ta.at(0).toDouble(), ta.at(1).toDouble());
		if (tb.size() >= 2)
			g.tipB = QPointF(tb.at(0).toDouble(), tb.at(1).toDouble());
		m_projectionGuides.push_back(g);
	}
	if (m_projectionGuides.isEmpty())
		rebuildProjectionGuides();
	for (const QJsonValue& sv : root.value(QStringLiteral("dimStyles")).toArray())
	{
		const QJsonObject o = sv.toObject();
		DimStyle s;
		s.id = o.value(QStringLiteral("id")).toString(QStringLiteral("Standard"));
		s.name = o.value(QStringLiteral("name")).toString(s.id);
		s.textHeightMm = o.value(QStringLiteral("textHeightMm")).toDouble(3.5);
		s.arrowSizeMm = o.value(QStringLiteral("arrowSizeMm")).toDouble(2.5);
		s.precision = o.value(QStringLiteral("precision")).toInt(2);
		s.tolPlus = o.value(QStringLiteral("tolPlus")).toDouble(0.0);
		s.tolMinus = o.value(QStringLiteral("tolMinus")).toDouble(0.0);
		s.showTolerance = o.value(QStringLiteral("showTolerance")).toBool(false);
		m_dimStyles.push_back(s);
	}
	for (const QJsonValue& sv : root.value(QStringLiteral("textStyles")).toArray())
	{
		const QJsonObject o = sv.toObject();
		TextStyle s;
		s.id = o.value(QStringLiteral("id")).toString(QStringLiteral("Standard"));
		s.name = o.value(QStringLiteral("name")).toString(s.id);
		s.heightMm = o.value(QStringLiteral("heightMm")).toDouble(3.5);
		s.fontFamily = o.value(QStringLiteral("fontFamily")).toString(QStringLiteral("Microsoft YaHei"));
		m_textStyles.push_back(s);
	}
	for (const QJsonValue& bv : root.value(QStringLiteral("blockDefs")).toArray())
	{
		const QJsonObject o = bv.toObject();
		SheetBlockDef d;
		d.id = o.value(QStringLiteral("id")).toString();
		d.name = o.value(QStringLiteral("name")).toString(d.id);
		const QJsonArray b = o.value(QStringLiteral("base")).toArray();
		if (b.size() >= 2)
			d.base = QPointF(b.at(0).toDouble(), b.at(1).toDouble());
		for (const QJsonValue& pv : o.value(QStringLiteral("geometry")).toArray())
		{
			Polyline2d poly;
			poly.points = xyArrayToPoints(pv.toArray());
			if (poly.points.size() >= 2)
				d.geometry.push_back(poly);
		}
		for (const QJsonValue& sv : o.value(QStringLiteral("geometryStyles")).toArray())
			d.geometryStyles.push_back(styleFromJson(sv.toObject()));
		for (const QJsonValue& av : o.value(QStringLiteral("attrDefs")).toArray())
		{
			const QJsonObject ao = av.toObject();
			SheetBlockDef::AttrDef a;
			a.tag = ao.value(QStringLiteral("tag")).toString();
			a.prompt = ao.value(QStringLiteral("prompt")).toString();
			a.defaultValue = ao.value(QStringLiteral("defaultValue")).toString();
			const QJsonArray pos = ao.value(QStringLiteral("position")).toArray();
			if (pos.size() >= 2)
				a.position = QPointF(pos.at(0).toDouble(), pos.at(1).toDouble());
			if (!a.tag.isEmpty())
				d.attrDefs.push_back(a);
		}
		m_blockDefs.push_back(d);
	}
	for (const QJsonValue& bv : root.value(QStringLiteral("blockRefs")).toArray())
	{
		const QJsonObject o = bv.toObject();
		SheetBlockRef r;
		r.id = o.value(QStringLiteral("id")).toString();
		r.defId = o.value(QStringLiteral("defId")).toString();
		const QJsonArray ins = o.value(QStringLiteral("insert")).toArray();
		if (ins.size() >= 2)
			r.insert = QPointF(ins.at(0).toDouble(), ins.at(1).toDouble());
		r.scale = o.value(QStringLiteral("scale")).toDouble(1.0);
		r.rotationDeg = o.value(QStringLiteral("rotationDeg")).toDouble(0.0);
		r.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (o.contains(QStringLiteral("style")))
			r.style = styleFromJson(o.value(QStringLiteral("style")).toObject());
		const QJsonObject av = o.value(QStringLiteral("attrValues")).toObject();
		for (auto it = av.begin(); it != av.end(); ++it)
			r.attrValues.insert(it.key(), it.value().toString());
		m_blockRefs.push_back(r);
	}
	m_ctbTable.clear();
	for (const QJsonValue& cv : root.value(QStringLiteral("ctbTable")).toArray())
	{
		const QJsonObject o = cv.toObject();
		CtbEntry e;
		e.aci = o.value(QStringLiteral("aci")).toInt();
		e.widthMm = o.value(QStringLiteral("widthMm")).toDouble();
		if (e.aci >= 0 && e.widthMm > 0)
			m_ctbTable.push_back(e);
	}
	ensureDefaultStyles();
	if (root.contains(QStringLiteral("sketch")))
	{
		const QJsonObject so = root.value(QStringLiteral("sketch")).toObject();
		m_sketch.fromJsonUtf8(QJsonDocument(so).toJson(QJsonDocument::Compact));
	}
	QHash<int, QString> sketchMap;
	const QJsonObject sl = root.value(QStringLiteral("sketchLayers")).toObject();
	for (auto it = sl.begin(); it != sl.end(); ++it)
	{
		bool ok = false;
		const int id = it.key().toInt(&ok);
		if (!ok)
			continue;
		QString lid = it.value().toString(defaultLayerId());
		if (layerIndex(lid) < 0)
			lid = defaultLayerId();
		sketchMap.insert(id, lid);
	}
	m_sketch.setEntityLayers(sketchMap);
	m_needInitialFit = true;
	emit layersChanged();
	emit sheetChanged();
	fitToView();
	update();
	return true;
}

void DrawingSheetCanvasWidget::setViewCatalog(const QVector<ViewTemplate>& catalog)
{
	m_viewCatalog = catalog;
}

bool DrawingSheetCanvasWidget::addCatalogViewAt(const QString& kind, const QPointF& sceneTopLeft)
{
	const ViewTemplate* found = nullptr;
	for (const ViewTemplate& t : m_viewCatalog)
	{
		if (t.kind == kind)
		{
			found = &t;
			break;
		}
	}
	if (!found || (found->visible.isEmpty() && found->hidden.isEmpty()))
		return false;

	QVector<Polyline2d> lVis, lHid;
	double w = 0, h = 0;
	normalizeViewLocal(found->visible, found->hidden, lVis, lHid, w, h);
	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	if (std::abs(s - 1.0) > 1e-12)
	{
		for (Polyline2d& poly : lVis)
			for (QPointF& p : poly.points)
				p = QPointF(p.x() * s, p.y() * s);
		for (Polyline2d& poly : lHid)
			for (QPointF& p : poly.points)
				p = QPointF(p.x() * s, p.y() * s);
		w *= s;
		h *= s;
	}
	DrawingView view;
	view.id = QStringLiteral("%1_%2").arg(kind).arg(m_nextCatalogViewId++);
	view.title = found->title.isEmpty() ? kind : found->title;
	view.kind = kind;
	view.visible = offsetPolylines(lVis, sceneTopLeft);
	view.hidden = offsetPolylines(lHid, sceneTopLeft);
	view.frame = QRectF(sceneTopLeft.x(), sceneTopLeft.y(), w, h).adjusted(-8, -18, 8, 8);
	view.layerId = m_currentLayerId;
	m_views.push_back(view);
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已添加%1").arg(view.title));
	update();
	return true;
}

void DrawingSheetCanvasWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		event->acceptProposedAction();
}

void DrawingSheetCanvasWidget::dragMoveEvent(QDragMoveEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		event->acceptProposedAction();
}

void DrawingSheetCanvasWidget::dropEvent(QDropEvent* event)
{
	if (!event->mimeData() || !event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		return;
	const QJsonObject o =
		QJsonDocument::fromJson(event->mimeData()->data(QString::fromLatin1(drawingViewMimeType()))).object();
	const QString kind = o.value(QStringLiteral("kind")).toString();
	if (kind.isEmpty())
		return;
	const QPointF scenePos = widgetToScene(event->pos());
	if (addCatalogViewAt(kind, scenePos))
		event->acceptProposedAction();
}

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
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& sectionHid)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> topLVis, topLHid, frontLVis, frontLHid, rightLVis, rightLHid;
	QVector<DrawingSheetCanvasWidget::Polyline2d> isoLVis, isoLHid, secLVis, secLHid;
	double topW = 0, topH = 0, frontW = 0, frontH = 0, rightW = 0, rightH = 0;
	double isoW = 0, isoH = 0, secW = 0, secH = 0;
	normalizeViewLocal(topVis, topHid, topLVis, topLHid, topW, topH);
	normalizeViewLocal(frontVis, frontHid, frontLVis, frontLHid, frontW, frontH);
	normalizeViewLocal(rightVis, rightHid, rightLVis, rightLHid, rightW, rightH);
	if (hasIso)
		normalizeViewLocal(isoVis, isoHid, isoLVis, isoLHid, isoW, isoH);
	if (hasSection)
		normalizeViewLocal(sectionVis, sectionHid, secLVis, secLHid, secW, secH);

	const double extent = qMax(qMax(topW, topH), qMax(qMax(frontW, frontH), qMax(rightW, rightH)));
	const double gap = qMax(30.0, extent * 0.2);
	const double col0W = qMax(topW, frontW);

	QVector<DrawingSheetCanvasWidget::DrawingView> views;
	if (method == DrawingProjectionMethod::FirstAngle)
	{
		// 俯上 / 正左下 / 右右下
		views.push_back(placeView(QStringLiteral("top"), QStringLiteral("俯视图"), QStringLiteral("top"), QPointF(0, 0),
								  topW, topH, topLVis, topLHid));
		views.push_back(placeView(QStringLiteral("front"), QStringLiteral("正视图"), QStringLiteral("front"),
								  QPointF(0, topH + gap), frontW, frontH, frontLVis, frontLHid));
		views.push_back(placeView(QStringLiteral("right"), QStringLiteral("右视图"), QStringLiteral("right"),
								  QPointF(col0W + gap, topH + gap), rightW, rightH, rightLVis, rightLHid));
	}
	else
	{
		// 第三角：正上 / 俯下 / 右右上
		views.push_back(placeView(QStringLiteral("front"), QStringLiteral("正视图"), QStringLiteral("front"),
								  QPointF(0, 0), frontW, frontH, frontLVis, frontLHid));
		views.push_back(placeView(QStringLiteral("top"), QStringLiteral("俯视图"), QStringLiteral("top"),
								  QPointF(0, frontH + gap), topW, topH, topLVis, topLHid));
		views.push_back(placeView(QStringLiteral("right"), QStringLiteral("右视图"), QStringLiteral("right"),
								  QPointF(qMax(frontW, topW) + gap, 0), rightW, rightH, rightLVis, rightLHid));
	}

	QRectF used;
	for (const auto& v : views)
		used = used.isValid() ? used.united(v.frame) : v.frame;
	double nextX = used.right() + gap;
	double nextY = used.top();
	if (hasIso)
	{
		views.push_back(placeView(QStringLiteral("iso"), QStringLiteral("轴测图"), QStringLiteral("iso"),
								  QPointF(nextX, nextY), isoW, isoH, isoLVis, isoLHid));
		nextY += isoH + gap;
	}
	if (hasSection)
	{
		views.push_back(placeView(QStringLiteral("section"), QStringLiteral("剖视图"), QStringLiteral("section"),
								  QPointF(nextX, nextY), secW, secH, secLVis, secLHid));
	}
	return views;
}
