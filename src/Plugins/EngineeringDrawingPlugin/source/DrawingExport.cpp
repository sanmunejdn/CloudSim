/// @file DrawingExport.cpp
/// @brief SVG / ASCII DXF 导出（插件内自写，不链 OCC/dxflib）

#include "DrawingExport.h"

#include <QColor>
#include <QFile>
#include <QHash>
#include <QLineF>
#include <QSet>
#include <QStringList>
#include <QTextStream>
#include <QtMath>

#include <cmath>
#include <limits>

namespace drawing_export
{
namespace
{
double dimValue(const DrawingSheetCanvasWidget::SheetDimension& d)
{
	if (std::isfinite(d.overrideValue))
		return d.overrideValue;
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
	{
		const QPointF a = d.p2 - d.p1;
		const QPointF b = d.p3 - d.p1;
		const double la = std::hypot(a.x(), a.y());
		const double lb = std::hypot(b.x(), b.y());
		if (la < 1e-9 || lb < 1e-9)
			return 0.0;
		double c = (a.x() * b.x() + a.y() * b.y()) / (la * lb);
		c = qBound(-1.0, c, 1.0);
		return std::acos(c) * 180.0 / 3.141592653589793;
	}
	const double r = QLineF(d.p1, d.p2).length();
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		return 2.0 * r;
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius)
		return r;
	return r;
}

bool fitClosedCirclePoly(const QVector<QPointF>& pts, QPointF& center, double& radius)
{
	if (pts.size() < 5)
		return false;
	double sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumX3 = 0, sumY3 = 0, sumX2Y = 0, sumXY2 = 0;
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
	}
	const double C = n * sumX2 - sumX * sumX;
	const double D = n * sumXY - sumX * sumY;
	const double E = n * sumY2 - sumY * sumY;
	const double G = 0.5 * (n * sumX3 + n * sumXY2 - sumX * (sumX2 + sumY2));
	const double H = 0.5 * (n * sumY3 + n * sumX2Y - sumY * (sumX2 + sumY2));
	const double denom = C * E - D * D;
	if (std::abs(denom) < 1e-12)
		return false;
	center = QPointF((G * E - D * H) / denom, (C * H - D * G) / denom);
	double sumR = 0, maxErr = 0;
	for (const QPointF& p : pts)
		sumR += QLineF(center, p).length();
	radius = sumR / n;
	if (!(radius > 1e-6))
		return false;
	for (const QPointF& p : pts)
		maxErr = qMax(maxErr, std::abs(QLineF(center, p).length() - radius));
	if (maxErr > qMax(0.03 * radius, 0.25))
		return false;
	return QLineF(pts.first(), pts.last()).length() <= qMax(0.04 * radius, 0.3);
}

QString dimText(const DrawingSheetCanvasWidget::SheetDimension& d)
{
	const double v = dimValue(d);
	QString base;
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius)
		base = QStringLiteral("R%1").arg(v, 0, 'f', 2);
	else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		base = QStringLiteral("Ø%1").arg(v, 0, 'f', 2);
	else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		base = QStringLiteral("%1°").arg(v, 0, 'f', 1);
	else
		base = QString::number(v, 'f', 2);
	if (d.tolOverride && d.showTolerance)
		base += QStringLiteral(" +%1/-%2").arg(d.tolPlus, 0, 'f', 2).arg(d.tolMinus, 0, 'f', 2);
	return base;
}

QSizeF paperSizeMm(const DrawingSheetCanvasWidget::SheetPaper& paper)
{
	double w = 210.0, h = 297.0;
	switch (paper.size)
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
		w = qMax(10.0, paper.customWidthMm);
		h = qMax(10.0, paper.customHeightMm);
		break;
	case DrawingPaperSize::A4:
	default:
		w = 210.0;
		h = 297.0;
		break;
	}
	if (paper.size != DrawingPaperSize::Custom && paper.landscape)
		qSwap(w, h);
	return QSizeF(w, h);
}

QRectF boundsOf(const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
				const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
				const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
				const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper)
{
	QRectF box;
	auto addPt = [&](const QPointF& p) {
		if (!box.isValid())
			box = QRectF(p, QSizeF(1, 1));
		else
			box = box.united(QRectF(p, QSizeF(0, 0)));
	};
	if (paper.visible)
	{
		const QSizeF s = paperSizeMm(paper);
		addPt(QPointF(0, 0));
		addPt(QPointF(s.width(), s.height()));
	}
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
		if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
			addPt(d.p3);
	}
	for (const auto& n : notes)
	{
		addPt(n.anchor);
		addPt(n.textPos);
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

void emitPaperSvg(QTextStream& ts, const DrawingSheetCanvasWidget::SheetPaper& paper, DrawingProjectionMethod projection)
{
	if (!paper.visible)
		return;
	const QSizeF s = paperSizeMm(paper);
	ts << "<rect x=\"0\" y=\"0\" width=\"" << s.width() << "\" height=\"" << s.height()
	   << "\" fill=\"none\" stroke=\"#28303C\" stroke-width=\"1.2\"/>\n";
	ts << "<rect x=\"8\" y=\"8\" width=\"" << (s.width() - 16) << "\" height=\"" << (s.height() - 16)
	   << "\" fill=\"none\" stroke=\"#3C4655\" stroke-width=\"0.6\"/>\n";
	const double tw = qMin(s.width() * 0.42, 180.0);
	const double th = qMin(s.height() * 0.18, 42.0);
	const double bx = s.width() - 8 - tw;
	const double by = s.height() - 8 - th;
	ts << "<rect x=\"" << bx << "\" y=\"" << by << "\" width=\"" << tw << "\" height=\"" << th
	   << "\" fill=\"#FFFFFF\" stroke=\"#28303C\" stroke-width=\"0.8\"/>\n";
	const QString proj =
		projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("第三角") : QStringLiteral("第一角");
	const QString title = paper.title.isEmpty() ? QStringLiteral("未命名") : paper.title;
	ts << "<text x=\"" << (bx + 4) << "\" y=\"" << (by + th * 0.35) << "\" font-size=\"9\" fill=\"#1E2330\">"
	   << esc(title) << "</text>\n";
	ts << "<text x=\"" << (bx + 4) << "\" y=\"" << (by + th * 0.8) << "\" font-size=\"8\" fill=\"#1E2330\">比例 "
	   << esc(paper.scaleText) << "</text>\n";
	ts << "<text x=\"" << (bx + tw * 0.55) << "\" y=\"" << (by + th * 0.35) << "\" font-size=\"8\" fill=\"#1E2330\">"
	   << esc(proj) << "</text>\n";
	ts << "<text x=\"" << (bx + tw * 0.55) << "\" y=\"" << (by + th * 0.8) << "\" font-size=\"8\" fill=\"#1E2330\">"
	   << esc(paper.date.isEmpty() ? QStringLiteral("-") : paper.date) << "  mm</text>\n";
}

bool layerVisible(const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers, const QString& layerId)
{
	const QString id = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	for (const auto& L : layers)
	{
		if (L.id == id)
			return L.visible && !L.frozen && L.plottable;
	}
	return true;
}

const DrawingSheetCanvasWidget::SheetLayer* findLayer(const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
													 const QString& layerId)
{
	const QString id = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	for (const auto& L : layers)
	{
		if (L.id == id)
			return &L;
	}
	return nullptr;
}

QString layerNameOf(const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers, const QString& layerId)
{
	const auto* L = findLayer(layers, layerId);
	if (!L)
		return QStringLiteral("0");
	return L->name.isEmpty() ? L->id : L->name;
}

QString sanitizeDxfLayerName(QString name)
{
	name = name.trimmed();
	if (name.isEmpty())
		name = QStringLiteral("0");
	for (QChar& c : name)
	{
		if (c == QLatin1Char('<') || c == QLatin1Char('>') || c == QLatin1Char('/') || c == QLatin1Char('\\') ||
			c == QLatin1Char('"') || c == QLatin1Char(';') || c == QLatin1Char('?') || c == QLatin1Char('*') ||
			c == QLatin1Char('|') || c == QLatin1Char('=') || c == QLatin1Char('`') || c.isSpace())
			c = QLatin1Char('_');
	}
	return name.left(31);
}

QString svgDashFor(SheetLineType t, bool forceDashed)
{
	if (forceDashed)
		t = SheetLineType::Dashed;
	switch (t)
	{
	case SheetLineType::Dashed:
		return QStringLiteral("4 3");
	case SheetLineType::Center:
		return QStringLiteral("8 2 1.5 2");
	case SheetLineType::DashDot:
		return QStringLiteral("6 2 1.2 2 1.2 2");
	case SheetLineType::Continuous:
	default:
		return {};
	}
}

QString dxfLinetypeName(SheetLineType t, bool forceDashed = false)
{
	if (forceDashed)
		t = SheetLineType::Dashed;
	switch (t)
	{
	case SheetLineType::Dashed:
		return QStringLiteral("DASHED");
	case SheetLineType::Center:
		return QStringLiteral("CENTER");
	case SheetLineType::DashDot:
		return QStringLiteral("DASHDOT");
	case SheetLineType::Continuous:
	default:
		return QStringLiteral("CONTINUOUS");
	}
}

int colorToAci(const QColor& c)
{
	if (!c.isValid())
		return 7;
	if (c.lightness() < 40)
		return 250;
	struct Entry
	{
		int aci;
		int r, g, b;
	};
	static const Entry kTable[] = {
		{1, 255, 0, 0},		{2, 255, 255, 0},	{3, 0, 255, 0},		{4, 0, 255, 255},
		{5, 0, 0, 255},		{6, 255, 0, 255},	{7, 255, 255, 255}, {8, 128, 128, 128},
		{9, 192, 192, 192},
	};
	int best = 7;
	int bestD = 1 << 30;
	for (const Entry& e : kTable)
	{
		const int dr = c.red() - e.r;
		const int dg = c.green() - e.g;
		const int db = c.blue() - e.b;
		const int d = dr * dr + dg * dg + db * db;
		if (d < bestD)
		{
			bestD = d;
			best = e.aci;
		}
	}
	return best;
}

QString strokeCss(const DrawingSheetCanvasWidget::SheetLayer* L)
{
	if (!L)
		return QStringLiteral("#1E2330");
	return L->color.name(QColor::HexRgb);
}

double strokeWidthMm(const DrawingSheetCanvasWidget::SheetLayer* L)
{
	if (!L || !(L->lineWidthMm > 1e-6))
		return 0.35;
	return L->lineWidthMm;
}

struct DxfUserLayer
{
	QString name;
	int color = 7;
	QString ltype;
};

} // namespace

bool writeSvg(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
			  const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper,
			  DrawingProjectionMethod projection, const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
			  const QHash<int, QString>& sketchLayers)
{
	const QRectF box = boundsOf(views, dims, notes, sketch, paper);
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
	emitPaperSvg(ts, paper, projection);

	auto emitPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, const QString& stroke, double width,
						 const QString& dash) {
		for (const auto& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			QPointF c;
			double r = 0;
			if (fitClosedCirclePoly(poly.points, c, r))
			{
				ts << "<circle cx=\"" << c.x() << "\" cy=\"" << c.y() << "\" r=\"" << r << "\" fill=\"none\" stroke=\""
				   << stroke << "\" stroke-width=\"" << width << "\"";
				if (!dash.isEmpty())
					ts << " stroke-dasharray=\"" << dash << "\"";
				ts << "/>\n";
				continue;
			}
			ts << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << width << "\"";
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
		if (!layerVisible(layers, v.layerId))
			continue;
		const auto* L = findLayer(layers, v.layerId);
		const QString stroke = strokeCss(L);
		const double w = strokeWidthMm(L);
		const SheetLineType lt = L ? L->lineType : SheetLineType::Continuous;
		ts << "<g id=\"" << esc(v.id) << "\">\n";
		ts << "<rect x=\"" << v.frame.x() << "\" y=\"" << v.frame.y() << "\" width=\"" << v.frame.width()
		   << "\" height=\"" << v.frame.height() << "\" fill=\"none\" stroke=\"#5A6478\" stroke-width=\"1\"/>\n";
		ts << "<text x=\"" << (v.frame.x() + 4) << "\" y=\"" << (v.frame.y() + 12)
		   << "\" font-size=\"10\" fill=\"#323C50\">" << esc(v.title) << "</text>\n";
		emitPolys(v.hidden, stroke, qMax(0.2, w * 0.85), svgDashFor(lt, true));
		emitPolys(v.visible, stroke, w, svgDashFor(lt, false));
		ts << "</g>\n";
	}

	for (const auto& poly : sketch)
	{
		if (poly.points.size() < 2)
			continue;
		const QString lid = sketchLayers.value(poly.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
			continue;
		const auto* L = findLayer(layers, lid);
		const SheetLineType lt = L ? L->lineType : SheetLineType::Continuous;
		DrawingSheetCanvasWidget::Polyline2d one;
		one.points = poly.points;
		emitPolys({one}, strokeCss(L), strokeWidthMm(L), svgDashFor(lt, poly.construction));
	}

	for (const auto& d : dims)
	{
		if (!layerVisible(layers, d.layerId))
			continue;
		const auto* L = findLayer(layers, d.layerId);
		const QString stroke = strokeCss(L);
		const double w = strokeWidthMm(L);
		const QString dash = svgDashFor(L ? L->lineType : SheetLineType::Continuous, false);
		const QString dashAttr = dash.isEmpty() ? QString() : QStringLiteral(" stroke-dasharray=\"%1\"").arg(dash);
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
			   << "\" stroke=\"" << stroke << "\" stroke-width=\"" << w << "\"" << dashAttr << "/>\n";
			ts << "<text x=\"" << mid.x() << "\" y=\"" << mid.y() << "\" font-size=\"9\" fill=\"" << stroke << "\">"
			   << esc(dimText(d)) << "</text>\n";
		}
		else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		{
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p2.x() << "\" y2=\"" << d.p2.y()
			   << "\" stroke=\"" << stroke << "\" stroke-width=\"" << w << "\"" << dashAttr << "/>\n";
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p3.x() << "\" y2=\"" << d.p3.y()
			   << "\" stroke=\"" << stroke << "\" stroke-width=\"" << w << "\"" << dashAttr << "/>\n";
			const QPointF mid = d.p1 + ((d.p2 - d.p1) + (d.p3 - d.p1)) * 0.15;
			ts << "<text x=\"" << mid.x() << "\" y=\"" << mid.y() << "\" font-size=\"9\" fill=\"" << stroke << "\">"
			   << esc(dimText(d)) << "</text>\n";
		}
		else
		{
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p2.x() << "\" y2=\"" << d.p2.y()
			   << "\" stroke=\"" << stroke << "\" stroke-width=\"" << w << "\"" << dashAttr << "/>\n";
			const QPointF t = d.p2 + d.textOffset;
			ts << "<text x=\"" << t.x() << "\" y=\"" << t.y() << "\" font-size=\"9\" fill=\"" << stroke << "\">"
			   << esc(dimText(d)) << "</text>\n";
		}
	}

	for (const auto& n : notes)
	{
		if (!layerVisible(layers, n.layerId))
			continue;
		const auto* L = findLayer(layers, n.layerId);
		const QString stroke = strokeCss(L);
		const double w = strokeWidthMm(L);
		const QString dash = svgDashFor(L ? L->lineType : SheetLineType::Continuous, false);
		const QString dashAttr = dash.isEmpty() ? QString() : QStringLiteral(" stroke-dasharray=\"%1\"").arg(dash);
		ts << "<line x1=\"" << n.anchor.x() << "\" y1=\"" << n.anchor.y() << "\" x2=\"" << n.textPos.x() << "\" y2=\""
		   << n.textPos.y() << "\" stroke=\"" << stroke << "\" stroke-width=\"" << w << "\"" << dashAttr << "/>\n";
		ts << "<text x=\"" << n.textPos.x() << "\" y=\"" << n.textPos.y() << "\" font-size=\"9\" fill=\"" << stroke
		   << "\">" << esc(n.text) << "</text>\n";
	}

	ts << "</svg>\n";
	return true;
}

bool writeDxf(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
			  const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper,
			  DrawingProjectionMethod projection, const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
			  const QHash<int, QString>& sketchLayers,
			  const QVector<DrawingSheetCanvasWidget::SheetHatch>& hatches,
			  const QVector<DrawingSheetCanvasWidget::SheetBlockDef>& blockDefs,
			  const QVector<DrawingSheetCanvasWidget::SheetBlockRef>& blockRefs,
			  const QVector<DxfSketchCircle>& sketchCircles, const QVector<DxfSketchArc>& sketchArcs)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts.setRealNumberPrecision(6);

	auto entLine = [&](double x1, double y1, double x2, double y2, int color, const QString& layer,
					   const QString& ltype) {
		ts << "0\nLINE\n8\n" << layer << "\n6\n" << ltype << "\n62\n" << color << "\n10\n" << x1 << "\n20\n" << y1
		   << "\n30\n0\n11\n" << x2 << "\n21\n" << y2 << "\n31\n0\n";
	};
	auto entText = [&](double x, double y, const QString& text, int color, const QString& layer) {
		ts << "0\nTEXT\n8\n" << layer << "\n62\n" << color << "\n10\n" << x << "\n20\n" << y << "\n30\n0\n40\n3\n1\n"
		   << text << "\n";
	};

	QVector<DxfUserLayer> userLayers;
	for (const auto& L : layers)
	{
		if (!L.visible)
			continue;
		DxfUserLayer u;
		u.name = sanitizeDxfLayerName(L.name);
		u.color = colorToAci(L.color);
		u.ltype = dxfLinetypeName(L.lineType);
		bool exists = false;
		for (const auto& e : userLayers)
		{
			if (e.name == u.name)
			{
				exists = true;
				break;
			}
		}
		if (!exists)
			userLayers.push_back(u);
	}
	const int layerCount = 3 + userLayers.size();

	ts << "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n0\nENDSEC\n";
	ts << "0\nSECTION\n2\nTABLES\n";

	ts << "0\nTABLE\n2\nLTYPE\n70\n4\n";
	ts << "0\nLTYPE\n2\nCONTINUOUS\n70\n0\n3\nSolid line\n72\n65\n73\n0\n40\n0.0\n";
	ts << "0\nLTYPE\n2\nDASHED\n70\n0\n3\nDashed\n72\n65\n73\n2\n40\n0.75\n49\n0.5\n49\n-0.25\n";
	ts << "0\nLTYPE\n2\nCENTER\n70\n0\n3\nCenter\n72\n65\n73\n4\n40\n2.0\n49\n1.25\n49\n-0.25\n49\n0.25\n49\n-0.25\n";
	ts << "0\nLTYPE\n2\nDASHDOT\n70\n0\n3\nDash dot\n72\n65\n73\n4\n40\n1.0\n49\n0.5\n49\n-0.25\n49\n0.0\n49\n-0.25\n";
	ts << "0\nENDTAB\n";

	ts << "0\nTABLE\n2\nLAYER\n70\n" << layerCount << "\n";
	auto writeLayer = [&](const QString& name, int color, const QString& ltype) {
		ts << "0\nLAYER\n2\n" << name << "\n70\n0\n62\n" << color << "\n6\n" << ltype << "\n";
	};
	writeLayer(QStringLiteral("VISIBLE"), 7, QStringLiteral("CONTINUOUS"));
	writeLayer(QStringLiteral("HIDDEN"), 8, QStringLiteral("DASHED"));
	writeLayer(QStringLiteral("FRAME"), 8, QStringLiteral("CONTINUOUS"));
	for (const DxfUserLayer& u : userLayers)
		writeLayer(u.name, u.color, u.ltype);
	ts << "0\nENDTAB\n0\nENDSEC\n";

	ts << "0\nSECTION\n2\nBLOCKS\n";
	for (const auto& def : blockDefs)
	{
		const QString bname = sanitizeDxfLayerName(def.name.isEmpty() ? def.id : def.name);
		ts << "0\nBLOCK\n8\n0\n2\n" << bname << "\n70\n0\n10\n" << def.base.x() << "\n20\n" << def.base.y()
		   << "\n30\n0\n";
		for (const auto& poly : def.geometry)
		{
			for (int i = 1; i < poly.points.size(); ++i)
			{
				entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), 7,
						QStringLiteral("0"), QStringLiteral("CONTINUOUS"));
			}
		}
		for (const auto& a : def.attrDefs)
		{
			const QPointF p = def.base + a.position;
			ts << "0\nATTDEF\n8\n0\n10\n" << p.x() << "\n20\n" << p.y() << "\n30\n0\n40\n3.5\n1\n"
			   << a.defaultValue << "\n2\n" << a.tag << "\n3\n" << a.prompt << "\n70\n0\n";
		}
		ts << "0\nENDBLK\n";
	}
	ts << "0\nENDSEC\n";

	ts << "0\nSECTION\n2\nENTITIES\n";

	if (paper.visible)
	{
		const QSizeF s = paperSizeMm(paper);
		entLine(0, 0, s.width(), 0, 8, QStringLiteral("FRAME"), QStringLiteral("CONTINUOUS"));
		entLine(s.width(), 0, s.width(), s.height(), 8, QStringLiteral("FRAME"), QStringLiteral("CONTINUOUS"));
		entLine(s.width(), s.height(), 0, s.height(), 8, QStringLiteral("FRAME"), QStringLiteral("CONTINUOUS"));
		entLine(0, s.height(), 0, 0, 8, QStringLiteral("FRAME"), QStringLiteral("CONTINUOUS"));
		const QString proj =
			projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("第三角") : QStringLiteral("第一角");
		const QString title = paper.title.isEmpty() ? QStringLiteral("未命名") : paper.title;
		entText(s.width() - 160, 20, title, 8, QStringLiteral("FRAME"));
		entText(s.width() - 160, 12,
				QStringLiteral("比例 %1 / %2 / %3")
					.arg(paper.scaleText, proj, paper.date.isEmpty() ? QStringLiteral("-") : paper.date),
				8, QStringLiteral("FRAME"));
	}

	auto emitPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, int color, const QString& layer,
						 const QString& ltype) {
		for (const auto& poly : polys)
		{
			QPointF c;
			double r = 0;
			if (fitClosedCirclePoly(poly.points, c, r))
			{
				ts << "0\nCIRCLE\n8\n" << layer << "\n62\n" << color << "\n6\n" << ltype << "\n10\n" << c.x()
				   << "\n20\n" << c.y() << "\n30\n0\n40\n" << r << "\n";
				continue;
			}
			for (int i = 1; i < poly.points.size(); ++i)
			{
				entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), color,
						layer, ltype);
			}
		}
	};

	for (const auto& v : views)
	{
		if (!layerVisible(layers, v.layerId))
			continue;
		const auto* L = findLayer(layers, v.layerId);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, v.layerId));
		const int aci = L ? colorToAci(L->color) : 7;
		const SheetLineType lt = L ? L->lineType : SheetLineType::Continuous;
		entLine(v.frame.left(), v.frame.top(), v.frame.right(), v.frame.top(), 8, QStringLiteral("FRAME"),
				QStringLiteral("CONTINUOUS"));
		entLine(v.frame.right(), v.frame.top(), v.frame.right(), v.frame.bottom(), 8, QStringLiteral("FRAME"),
				QStringLiteral("CONTINUOUS"));
		entLine(v.frame.right(), v.frame.bottom(), v.frame.left(), v.frame.bottom(), 8, QStringLiteral("FRAME"),
				QStringLiteral("CONTINUOUS"));
		entLine(v.frame.left(), v.frame.bottom(), v.frame.left(), v.frame.top(), 8, QStringLiteral("FRAME"),
				QStringLiteral("CONTINUOUS"));
		emitPolys(v.hidden, aci, dxfLayer, dxfLinetypeName(lt, true));
		emitPolys(v.visible, aci, dxfLayer, dxfLinetypeName(lt, false));
	}
	QSet<int> skipIds;
	for (const DxfSketchCircle& c : sketchCircles)
	{
		if (!(c.radius > 1e-9))
			continue;
		skipIds.insert(c.entityId);
		const QString lid = sketchLayers.value(c.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
			continue;
		const auto* L = findLayer(layers, lid);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, lid));
		const int aci = L ? colorToAci(L->color) : 5;
		ts << "0\nCIRCLE\n8\n" << dxfLayer << "\n62\n" << aci << "\n10\n" << c.center.x() << "\n20\n" << c.center.y()
		   << "\n30\n0\n40\n" << c.radius << "\n";
	}
	for (const DxfSketchArc& a : sketchArcs)
	{
		if (!(a.radius > 1e-9))
			continue;
		skipIds.insert(a.entityId);
		const QString lid = sketchLayers.value(a.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
			continue;
		const auto* L = findLayer(layers, lid);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, lid));
		const int aci = L ? colorToAci(L->color) : 5;
		ts << "0\nARC\n8\n" << dxfLayer << "\n62\n" << aci << "\n10\n" << a.center.x() << "\n20\n" << a.center.y()
		   << "\n30\n0\n40\n" << a.radius << "\n50\n" << a.startDeg << "\n51\n" << a.endDeg << "\n";
	}
	for (const auto& poly : sketch)
	{
		if (skipIds.contains(poly.entityId))
			continue;
		const QString lid = sketchLayers.value(poly.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
			continue;
		const auto* L = findLayer(layers, lid);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, lid));
		const int aci = L ? colorToAci(L->color) : 5;
		const SheetLineType lt = L ? L->lineType : SheetLineType::Continuous;
		for (int i = 1; i < poly.points.size(); ++i)
			entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), aci,
					dxfLayer, dxfLinetypeName(lt, poly.construction));
	}
	for (const auto& d : dims)
	{
		if (!layerVisible(layers, d.layerId))
			continue;
		const auto* L = findLayer(layers, d.layerId);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, d.layerId));
		const int aci = L ? colorToAci(L->color) : 1;
		const QString ltype = dxfLinetypeName(L ? L->lineType : SheetLineType::Continuous);
		if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Linear)
		{
			QLineF base(d.p1, d.p2);
			QPointF a = d.p1, b = d.p2, mid = (a + b) * 0.5;
			double off = d.textOffset.y();
			if (std::abs(off) < 1e-6)
				off = -12.0;
			if (base.length() > 1e-9)
			{
				base.setLength(1.0);
				const QPointF dir = base.p2() - base.p1();
				const QPointF n(-dir.y(), dir.x());
				a = d.p1 + n * off;
				b = d.p2 + n * off;
				mid = (a + b) * 0.5;
			}
			// 写出 DIMENSION（线性对齐），便于 AutoCAD 识别
			ts << "0\nDIMENSION\n8\n" << dxfLayer << "\n62\n" << aci << "\n"
			   << "10\n" << mid.x() << "\n20\n" << mid.y() << "\n30\n0\n"
			   << "11\n" << mid.x() << "\n21\n" << mid.y() << "\n31\n0\n"
			   << "70\n1\n" // aligned
			   << "13\n" << d.p1.x() << "\n23\n" << d.p1.y() << "\n33\n0\n"
			   << "14\n" << d.p2.x() << "\n24\n" << d.p2.y() << "\n34\n0\n"
			   << "1\n" << dimText(d) << "\n";
			entLine(a.x(), a.y(), b.x(), b.y(), aci, dxfLayer, ltype);
		}
		else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		{
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), aci, dxfLayer, ltype);
			entLine(d.p1.x(), d.p1.y(), d.p3.x(), d.p3.y(), aci, dxfLayer, ltype);
			const QPointF mid = d.p1 + ((d.p2 - d.p1) + (d.p3 - d.p1)) * 0.15;
			entText(mid.x(), mid.y(), dimText(d), aci, dxfLayer);
		}
		else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius ||
				 d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		{
			// 半径/直径：DIMENSION(径向) + 辅助线/文字，AC1009 兼容
			const QPointF mid = (d.p1 + d.p2) * 0.5;
			ts << "0\nDIMENSION\n8\n" << dxfLayer << "\n62\n" << aci << "\n"
			   << "10\n" << mid.x() << "\n20\n" << mid.y() << "\n30\n0\n"
			   << "11\n" << d.p2.x() << "\n21\n" << d.p2.y() << "\n31\n0\n"
			   << "70\n4\n" // radius
			   << "15\n" << d.p1.x() << "\n25\n" << d.p1.y() << "\n35\n0\n"
			   << "40\n" << QLineF(d.p1, d.p2).length() << "\n"
			   << "1\n" << dimText(d) << "\n";
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), aci, dxfLayer, ltype);
			const QPointF t = d.p2 + d.textOffset;
			entText(t.x(), t.y(), dimText(d), aci, dxfLayer);
		}
		else
		{
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), aci, dxfLayer, ltype);
			const QPointF t = d.p2 + d.textOffset;
			entText(t.x(), t.y(), dimText(d), aci, dxfLayer);
		}
	}
	for (const auto& r : blockRefs)
	{
		QString bname;
		for (const auto& def : blockDefs)
		{
			if (def.id == r.defId)
			{
				bname = sanitizeDxfLayerName(def.name.isEmpty() ? def.id : def.name);
				break;
			}
		}
		if (bname.isEmpty())
			continue;
		if (!layerVisible(layers, r.layerId))
			continue;
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, r.layerId));
		const auto* L = findLayer(layers, r.layerId);
		const int aci = L ? colorToAci(L->color) : 7;
		ts << "0\nINSERT\n8\n" << dxfLayer << "\n62\n" << aci << "\n2\n" << bname << "\n10\n" << r.insert.x()
		   << "\n20\n" << r.insert.y() << "\n30\n0\n41\n" << r.scale << "\n42\n" << r.scale << "\n50\n"
		   << r.rotationDeg << "\n66\n1\n";
		const DrawingSheetCanvasWidget::SheetBlockDef* defPtr = nullptr;
		for (const auto& def : blockDefs)
		{
			if (def.id == r.defId)
			{
				defPtr = &def;
				break;
			}
		}
		if (defPtr)
		{
			int row = 0;
			for (const auto& a : defPtr->attrDefs)
			{
				const QString val = r.attrValues.value(a.tag, a.defaultValue);
				QPointF ap = r.insert + a.position;
				if (a.position.isNull())
					ap = r.insert + QPointF(6.0, -6.0 - row * 5.0);
				ts << "0\nATTRIB\n8\n" << dxfLayer << "\n10\n" << ap.x() << "\n20\n" << ap.y()
				   << "\n30\n0\n40\n3.5\n1\n" << val << "\n2\n" << a.tag << "\n70\n0\n";
				++row;
			}
		}
		ts << "0\nSEQEND\n";
	}
	for (const auto& n : notes)
	{
		if (!layerVisible(layers, n.layerId))
			continue;
		const auto* L = findLayer(layers, n.layerId);
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, n.layerId));
		const int aci = L ? colorToAci(L->color) : 4;
		const QString ltype = dxfLinetypeName(L ? L->lineType : SheetLineType::Continuous);
		entLine(n.anchor.x(), n.anchor.y(), n.textPos.x(), n.textPos.y(), aci, dxfLayer, ltype);
		entText(n.textPos.x(), n.textPos.y(), n.text, aci, dxfLayer);
	}
	for (const auto& h : hatches)
	{
		if (h.boundary.size() < 3 || !layerVisible(layers, h.layerId))
			continue;
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, h.layerId));
		const auto* L = findLayer(layers, h.layerId);
		const int aci = L ? colorToAci(L->color) : 3;
		ts << "0\nHATCH\n8\n" << dxfLayer << "\n62\n" << aci << "\n70\n1\n71\n0\n91\n1\n92\n1\n93\n"
		   << h.boundary.size() << "\n";
		for (const QPointF& pt : h.boundary)
			ts << "10\n" << pt.x() << "\n20\n" << pt.y() << "\n";
		ts << "97\n0\n75\n0\n76\n1\n98\n1\n10\n" << h.boundary.first().x() << "\n20\n" << h.boundary.first().y()
		   << "\n";
	}

	ts << "0\nENDSEC\n0\nEOF\n";
	return true;
}

} // namespace drawing_export
