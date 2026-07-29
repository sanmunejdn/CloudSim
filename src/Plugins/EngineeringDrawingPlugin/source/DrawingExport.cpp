/// @file DrawingExport.cpp
/// @brief SVG / ASCII DXF 导出（插件内自写，不链 OCC/dxflib）

#include "DrawingExport.h"

#include <QFile>
#include <QHash>
#include <QLineF>
#include <QStringList>
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

QString dimText(const DrawingSheetCanvasWidget::SheetDimension& d)
{
	const double v = dimValue(d);
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Radius)
		return QStringLiteral("R%1").arg(v, 0, 'f', 2);
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Diameter)
		return QStringLiteral("Ø%1").arg(v, 0, 'f', 2);
	if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		return QStringLiteral("%1°").arg(v, 0, 'f', 1);
	return QString::number(v, 'f', 2);
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
			return L.visible;
	}
	return true;
}

QString layerNameOf(const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers, const QString& layerId)
{
	const QString id = layerId.isEmpty() ? QStringLiteral("L0") : layerId;
	for (const auto& L : layers)
	{
		if (L.id == id)
			return L.name.isEmpty() ? id : L.name;
	}
	return QStringLiteral("0");
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
		if (!layerVisible(layers, v.layerId))
			continue;
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
		const QString lid = sketchLayers.value(poly.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
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
		if (!layerVisible(layers, d.layerId))
			continue;
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
		else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		{
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p2.x() << "\" y2=\"" << d.p2.y()
			   << "\" stroke=\"#C0392B\" stroke-width=\"0.7\"/>\n";
			ts << "<line x1=\"" << d.p1.x() << "\" y1=\"" << d.p1.y() << "\" x2=\"" << d.p3.x() << "\" y2=\"" << d.p3.y()
			   << "\" stroke=\"#C0392B\" stroke-width=\"0.7\"/>\n";
			const QPointF mid = d.p1 + ((d.p2 - d.p1) + (d.p3 - d.p1)) * 0.15;
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
	for (const auto& n : notes)
	{
		if (!layerVisible(layers, n.layerId))
			continue;
		ts << "<line x1=\"" << n.anchor.x() << "\" y1=\"" << n.anchor.y() << "\" x2=\"" << n.textPos.x() << "\" y2=\""
		   << n.textPos.y() << "\" stroke=\"#34495E\" stroke-width=\"0.6\"/>\n";
		ts << "<text x=\"" << n.textPos.x() << "\" y=\"" << n.textPos.y() << "\" font-size=\"9\" fill=\"#34495E\">"
		   << esc(n.text) << "</text>\n";
	}
	ts << "</svg>\n";
	return true;
}

bool writeDxf(const QString& path, const QVector<DrawingSheetCanvasWidget::DrawingView>& views,
			  const QVector<DrawingSheetCanvasWidget::SheetDimension>& dims,
			  const QVector<DrawingSheetCanvasWidget::SheetNote>& notes,
			  const QVector<SheetSketchPolyline>& sketch, const DrawingSheetCanvasWidget::SheetPaper& paper,
			  DrawingProjectionMethod projection, const QVector<DrawingSheetCanvasWidget::SheetLayer>& layers,
			  const QHash<int, QString>& sketchLayers)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts.setRealNumberPrecision(6);

	auto entLine = [&](double x1, double y1, double x2, double y2, int color, const QString& layer) {
		ts << "0\nLINE\n8\n" << layer << "\n62\n" << color << "\n10\n" << x1 << "\n20\n" << y1 << "\n30\n0\n11\n" << x2
		   << "\n21\n" << y2 << "\n31\n0\n";
	};
	auto entText = [&](double x, double y, const QString& text, int color, const QString& layer) {
		ts << "0\nTEXT\n8\n" << layer << "\n62\n" << color << "\n10\n" << x << "\n20\n" << y << "\n30\n0\n40\n3\n1\n"
		   << text << "\n";
	};

	QStringList userLayerNames;
	for (const auto& L : layers)
	{
		if (!L.visible)
			continue;
		const QString n = sanitizeDxfLayerName(L.name);
		if (!userLayerNames.contains(n))
			userLayerNames.push_back(n);
	}
	const int layerCount = 3 + userLayerNames.size();

	ts << "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n0\nENDSEC\n";
	ts << "0\nSECTION\n2\nTABLES\n";
	ts << "0\nTABLE\n2\nLAYER\n70\n" << layerCount << "\n";
	auto writeLayer = [&](const QString& name, int color) {
		ts << "0\nLAYER\n2\n" << name << "\n70\n0\n62\n" << color << "\n6\nCONTINUOUS\n";
	};
	writeLayer(QStringLiteral("VISIBLE"), 7);
	writeLayer(QStringLiteral("HIDDEN"), 8);
	writeLayer(QStringLiteral("FRAME"), 8);
	for (const QString& n : userLayerNames)
		writeLayer(n, 7);
	ts << "0\nENDTAB\n0\nENDSEC\n";
	ts << "0\nSECTION\n2\nENTITIES\n";

	if (paper.visible)
	{
		const QSizeF s = paperSizeMm(paper);
		entLine(0, 0, s.width(), 0, 8, QStringLiteral("FRAME"));
		entLine(s.width(), 0, s.width(), s.height(), 8, QStringLiteral("FRAME"));
		entLine(s.width(), s.height(), 0, s.height(), 8, QStringLiteral("FRAME"));
		entLine(0, s.height(), 0, 0, 8, QStringLiteral("FRAME"));
		const QString proj =
			projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("第三角") : QStringLiteral("第一角");
		const QString title = paper.title.isEmpty() ? QStringLiteral("未命名") : paper.title;
		entText(s.width() - 160, 20, title, 8, QStringLiteral("FRAME"));
		entText(s.width() - 160, 12,
				QStringLiteral("比例 %1 / %2 / %3")
					.arg(paper.scaleText, proj, paper.date.isEmpty() ? QStringLiteral("-") : paper.date),
				8, QStringLiteral("FRAME"));
	}

	auto emitPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, int color, const QString& layer) {
		for (const auto& poly : polys)
		{
			for (int i = 1; i < poly.points.size(); ++i)
			{
				entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), color,
						layer);
			}
		}
	};

	for (const auto& v : views)
	{
		if (!layerVisible(layers, v.layerId))
			continue;
		entLine(v.frame.left(), v.frame.top(), v.frame.right(), v.frame.top(), 8, QStringLiteral("FRAME"));
		entLine(v.frame.right(), v.frame.top(), v.frame.right(), v.frame.bottom(), 8, QStringLiteral("FRAME"));
		entLine(v.frame.right(), v.frame.bottom(), v.frame.left(), v.frame.bottom(), 8, QStringLiteral("FRAME"));
		entLine(v.frame.left(), v.frame.bottom(), v.frame.left(), v.frame.top(), 8, QStringLiteral("FRAME"));
		emitPolys(v.hidden, 8, QStringLiteral("HIDDEN"));
		emitPolys(v.visible, 7, QStringLiteral("VISIBLE"));
	}
	for (const auto& poly : sketch)
	{
		const QString lid = sketchLayers.value(poly.entityId, QStringLiteral("L0"));
		if (!layerVisible(layers, lid))
			continue;
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, lid));
		for (int i = 1; i < poly.points.size(); ++i)
			entLine(poly.points[i - 1].x(), poly.points[i - 1].y(), poly.points[i].x(), poly.points[i].y(), 5,
					dxfLayer);
	}
	for (const auto& d : dims)
	{
		if (!layerVisible(layers, d.layerId))
			continue;
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, d.layerId));
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
			entLine(a.x(), a.y(), b.x(), b.y(), 1, dxfLayer);
			entText(mid.x(), mid.y(), dimText(d), 1, dxfLayer);
		}
		else if (d.kind == DrawingSheetCanvasWidget::SheetDimension::Kind::Angle)
		{
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), 1, dxfLayer);
			entLine(d.p1.x(), d.p1.y(), d.p3.x(), d.p3.y(), 1, dxfLayer);
			const QPointF mid = d.p1 + ((d.p2 - d.p1) + (d.p3 - d.p1)) * 0.15;
			entText(mid.x(), mid.y(), dimText(d), 1, dxfLayer);
		}
		else
		{
			entLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), 1, dxfLayer);
			const QPointF t = d.p2 + d.textOffset;
			entText(t.x(), t.y(), dimText(d), 1, dxfLayer);
		}
	}
	for (const auto& n : notes)
	{
		if (!layerVisible(layers, n.layerId))
			continue;
		const QString dxfLayer = sanitizeDxfLayerName(layerNameOf(layers, n.layerId));
		entLine(n.anchor.x(), n.anchor.y(), n.textPos.x(), n.textPos.y(), 4, dxfLayer);
		entText(n.textPos.x(), n.textPos.y(), n.text, 4, dxfLayer);
	}

	ts << "0\nENDSEC\n0\nEOF\n";
	return true;
}

} // namespace drawing_export
