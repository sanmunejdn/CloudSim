/// @file DrawingExport.cpp
/// @brief SVG / ASCII DXF 导出（插件内自写，不链 OCC/dxflib）

#include "DrawingExport.h"

#include <QFile>
#include <QLineF>
#include <QTextStream>
#include <QtMath>

#include <cmath>

namespace drawing_export
{
namespace
{
double dimValue(const DrawingSheetCanvasWidget::SheetDimension& d)
{
	if (std::isfinite(d.overrideValue))
		return d.overrideValue;
	const double r = QLineF(d.p1, d.p2).length();
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		return 2.0 * r;
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius)
		return r;
	return r;
}

QString dimText(const DrawingSheetCanvasWidget::SheetDimension& d)
{
	const double v = dimValue(d);
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius)
		return QStringLiteral("R%1").arg(v, 0, 'f', 2);
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		return QStringLiteral("Ø%1").arg(v, 0, 'f', 2);
	return QString::number(v, 'f', 2);
}

QRectF boundsOf(const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
				const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
				const QVector<SheetSketchPolyline>& sketch)
{
	QRectF box;
	auto addPt = [&](const QPointF& p) {
		if (!box.isValid())
			box = QRectF(p, QSizeF(1, 1));
		else
			box = box.united(QRectF(p, QSizeF(0, 0)));
	};
	for (const auto& v : views)
	{
		addPt(v.frame.topLeft());
		addPt(v.frame.bottomRight());
		for (const auto& poly : v.visible)
			for (const QPointF& p : poly.points)
				addPt(p);
		for (const auto& poly : v.hidden)
			for (const QPointF& p : poly.points)
				addPt(p);
	}
	for (const auto& d : dims)
	{
		addPt(d.p1);
		addPt(d.p2);
	}
	for (const auto& poly : sketch)
		for (const QPointF& p : poly.points)
			addPt(p);
	if (!box.isValid())
		box = QRectF(0, 0, 100, 100);
	return box.adjusted(-20, -20, 20, 20);
}

QString esc(const QString& s)
{
	QString o = s;
	o.replace(QLatin1Char('&'), QStringLiteral("&amp;"));
	o.replace(QLatin1Char('<'), QStringLiteral("&lt;"));
	o.replace(QLatin1Char('>'), QStringLiteral("&gt;"));
	o.replace(QLatin1Char('"'), QStringLiteral("&quot;"));
	return o;
}

} // namespace

bool writeSvg(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<SheetSketchPolyline>& sketch)
{
	const QRectF box = boundsOf(views, dims, sketch);
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts.setRealNumberPrecision(4);
	ts << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	ts << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << box.width() << "\" height=\"" << box.height()
	   << "\" viewBox=\"" << box.x() << " " << box.y() << " " << box.width() << " " << box.height() << "\">\n";
	ts << "<rect x=\"" << box.x() << "\" y=\"" << box.y() << "\" width=\"" << box.width() << "\" height=\""
	   << box.height() << "\" fill=\"#F5F7FA\"/>\n";

	auto emitPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, const QString& stroke,
						 const QString& dash) {
		for (const auto& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			ts << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"0.8\"";
			if (!dash.isEmpty())
				ts << " stroke-dasharray=\"" << dash << "\"";
			ts << " points=\"";
			for (const QPointF& p : poly.points)
				ts << p.x() << "," << p.y() << " ";
			ts << "\"/>\n";
		}
	};

	for (const auto& v : views)
	{
		ts << "<g id=\"" << esc(v.id) << "\">\n";
		ts << "<rect x=\"" << v.frame.x() << "\" y=\"" << v.frame.y() << "\" width=\"" << v.frame.width()
		   << "\" height=\"" << v.frame.height() << "\" fill=\"none\" stroke=\"#5A6478\" stroke-width=\"1\"/>\n";
		ts << "<text x=\"" << (v.frame.x() + 4) << "\" y=\"" << (v.frame.y() + 12) << "\" font-size=\"10\" fill=\"#323C50\">"
		   << esc(v.title) << "</text>\n";
		emitPolys(v.hidden, QStringLiteral("#8C919B"), QStringLiteral("4 3"));
		emitPolys(v.visible, QStringLiteral("#141820"), QString());
		ts << "</g>\n";
	}

	for (const auto& poly : sketch)
	{
		if (poly.points.size() < 2)
			continue;
		ts << "<polyline fill=\"none\" stroke=\"#1E78B4\" stroke-width=\"0.9\"";
		if (poly.construction)
			ts << " stroke-dasharray=\"3 2\"";
		ts << " points=\"";
		for (const QPointF& p : poly.points)
			ts << p.x() << "," << p.y() << " ";
		ts << "\"/>\n";
	}

	for (const auto& d : dims)
	{
		if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Linear)
		{
			QLineF base(d.p1, d.p2);
			QPointF a = d.p1, b = d.p2;
			QPointF mid = (a + b) * 0.5;
			if (base.length() > 1e-9)
			{
				base.setLength(1.0);
				const QPointF dir = base.p2() - base.p1();
				const QPointF n(-dir.y(), dir.x());
				double off = d.textOffset.y();
				if (std::abs(off) < 1e-6)
					off = -12.0;
				a = d.p1 + n * off;
				b = d.p2 + n * off;
				mid = (a + b) * 0.5;
			}
			ts << "<line x1=\"" << a.x() << "\" y1=\"" << a.y() << "\" x2=\"" << b.x() << "\" y2=\"" << b.y()
			   << "\" stroke=\"#C0392B\" stroke-width=\"0.7\"/>\n";
			ts << "<text x=\"" << mid.x() << "\" y=\"" << mid.y() << "\" font-size=\"9\" fill=\"#C0392B\">"
			   << esc(dimText(d)) << "</text>\n";
		}
		else
		{
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p2.x() << "\" y2=\"" << d.p2.y()
			   << "\" stroke=\"#C0392B\" stroke-width=\"0.7\"/>\n";
			const QPointF t = d.p2 + d.textOffset;
			ts << "<text x=\"" << t.x() << "\" y=\"" << t.y() << "\" font-size=\"9\" fill=\"#C0392B\">" << esc(dimText(d))
			   << "</text>\n";
		}
	}
	ts << "</svg>\n";
	return true;
}

bool writeDxf(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<SheetSketchPolyline>& sketch)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts.setRealNumberPrecision(6);

	auto entLine = [&](double x1, double y1, double x2, double y2, int color) {
		ts << "0\nLINE\n8\n0\n62\n" << color << "\n10\n" << x1 << "\n20\n" << y1 << "\n30\n0\n11\n" << x2 << "\n21\n"
		   << y2 << "\n31\n0\n";
	};
	auto entText = [&](double x, double y, const QString& text, int color) {
		ts << "0\nTEXT\n8\n0\n62\n" << color << "\n10\n" << x << "\n20\n" << y << "\n30\n0\n40\n3\n1\n" << text
		   << "\n";
	};

	ts << "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n0\nENDSEC\n";
	ts << "0\nSECTION\n2\nENTITIES\n";

	auto emitPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, int color) {
		for (const auto& poly : polys)
		{
			for (int i = 1; i < poly.points.size(); ++i)
			{
				entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(),
						color);
			}
		}
	};

	for (const auto& v : views)
	{
		entLine(v.frame.left(), v.frame.top(), v.frame.right(), v.frame.top(), 8);
		entLine(v.frame.right(), v.frame.top(), v.frame.right(), v.frame.bottom(), 8);
		entLine(v.frame.right(), v.frame.bottom(), v.frame.left(), v.frame.bottom(), 8);
		entLine(v.frame.left(), v.frame.bottom(), v.frame.left(), v.frame.top(), 8);
		emitPolys(v.hidden, 8);
		emitPolys(v.visible, 7);
	}
	for (const auto& poly : sketch)
	{
		for (int i = 1; i < poly.points.size(); ++i)
			entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), 5);
	}
	for (const auto& d : dims)
	{
		if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Linear)
		{
			QLineF base(d.p1, d.p2);
			QPointF a = d.p1, b = d.p2, mid = (a + b) * 0.5;
			if (base.length() > 1e-9)
			{
				base.setLength(1.0);
				const QPointF dir = base.p2() - base.p1();
				const QPointF n(-dir.y(), dir.x());
				double off = d.textOffset.y();
				if (std::abs(off) < 1e-6)
					off = -12.0;
				a = d.p1 + n * off;
				b = d.p2 + n * off;
				mid = (a + b) * 0.5;
			}
			entLine(a.x(), a.y(), b.x(), b.y(), 1);
			entText(mid.x(), mid.y(), dimText(d), 1);
		}
		else
		{
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), 1);
			const QPointF t = d.p2 + d.textOffset;
			entText(t.x(), t.y(), dimText(d), 1);
		}
	}

	ts << "0\nENDSEC\n0\nEOF\n";
	return true;
}

} // namespace drawing_export
