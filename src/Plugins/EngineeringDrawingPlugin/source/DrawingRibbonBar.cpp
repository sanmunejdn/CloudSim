/// @file DrawingRibbonBar.cpp
/// @brief 工程图模式条：卡片分组 + 青绿强调（对齐几何建模 Ribbon）

#include "DrawingRibbonBar.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QScrollArea>
#include <QSizePolicy>
#include <QToolButton>
#include <QVariant>
#include <QVBoxLayout>

namespace
{
constexpr QRgb kAccent = 0xff0f766e;
constexpr QRgb kAccentSoft = 0xff14b8a6;

QIcon makeGlyphIcon(const QString& kind, bool dark)
{
	QPixmap pm(40, 40);
	pm.fill(Qt::transparent);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	const QColor ink = dark ? QColor(0xf4, 0xf4, 0xf5) : QColor(0x18, 0x18, 0x1b);
	p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	p.setBrush(Qt::NoBrush);

	if (kind == QLatin1String("pan"))
	{
		p.drawRoundedRect(QRectF(11, 11, 18, 18), 3, 3);
		p.drawLine(20, 8, 20, 14);
		p.drawLine(20, 26, 20, 32);
		p.drawLine(8, 20, 14, 20);
		p.drawLine(26, 20, 32, 20);
	}
	else if (kind == QLatin1String("detail"))
	{
		p.drawEllipse(QRectF(8, 8, 18, 18));
		p.setPen(QPen(ink, 2.4, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(24, 24, 32, 32);
	}
	else if (kind == QLatin1String("line"))
	{
		p.setPen(QPen(ink, 2.4, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(8, 30, 32, 10);
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(8, 30), 3, 3);
		p.drawEllipse(QPointF(32, 10), 3, 3);
	}
	else if (kind == QLatin1String("arc"))
	{
		p.drawArc(QRectF(7, 7, 26, 26), 20 * 16, 140 * 16);
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(11, 28), 2.5, 2.5);
		p.drawEllipse(QPointF(20, 9), 2.5, 2.5);
		p.drawEllipse(QPointF(29, 28), 2.5, 2.5);
	}
	else if (kind == QLatin1String("circle"))
	{
		p.drawEllipse(QRectF(8, 8, 24, 24));
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(20, 20), 2.5, 2.5);
	}
	else if (kind == QLatin1String("rect"))
	{
		p.drawRoundedRect(QRectF(8, 11, 24, 18), 2, 2);
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(8, 11), 2.2, 2.2);
		p.drawEllipse(QPointF(32, 11), 2.2, 2.2);
		p.drawEllipse(QPointF(32, 29), 2.2, 2.2);
		p.drawEllipse(QPointF(8, 29), 2.2, 2.2);
	}
	else if (kind == QLatin1String("spline"))
	{
		p.setPen(QPen(ink, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		QPainterPath path;
		path.moveTo(7, 28);
		path.cubicTo(12, 8, 18, 32, 22, 14);
		path.cubicTo(26, 4, 30, 22, 33, 18);
		p.drawPath(path);
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(7, 28), 2.4, 2.4);
		p.drawEllipse(QPointF(16, 16), 2.4, 2.4);
		p.drawEllipse(QPointF(26, 12), 2.4, 2.4);
		p.drawEllipse(QPointF(33, 18), 2.4, 2.4);
	}
	else if (kind == QLatin1String("select"))
	{
		QPainterPath arrow;
		arrow.moveTo(10, 8);
		arrow.lineTo(10, 28);
		arrow.lineTo(16, 22);
		arrow.lineTo(22, 34);
		arrow.lineTo(26, 32);
		arrow.lineTo(20, 20);
		arrow.lineTo(28, 20);
		arrow.closeSubpath();
		p.setBrush(ink);
		p.setPen(Qt::NoPen);
		p.drawPath(arrow);
	}
	else if (kind == QLatin1String("dimLen"))
	{
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(8, 28, 32, 12);
		p.drawLine(10, 22, 16, 30);
		p.drawLine(24, 10, 30, 18);
	}
	else if (kind == QLatin1String("dimRad"))
	{
		p.drawEllipse(QRectF(8, 8, 24, 24));
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(20, 20, 30, 12);
	}
	else if (kind == QLatin1String("dimDia"))
	{
		p.drawEllipse(QRectF(8, 8, 24, 24));
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(12, 28, 28, 12);
	}
	return QIcon(pm);
}

QToolButton* makeRibbonButton(QWidget* parent, const QString& text, const QString& kind, bool compact = true)
{
	auto* btn = new QToolButton(parent);
	btn->setObjectName(QStringLiteral("RibbonBtn"));
	btn->setProperty("glyphKind", kind);
	btn->setProperty("btnRole", QStringLiteral("draw"));
	btn->setProperty("compact", compact);
	btn->setAutoRaise(false);
	btn->setCursor(Qt::PointingHandCursor);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setToolTip(text);
	btn->setCheckable(true);
	if (compact)
	{
		btn->setToolButtonStyle(Qt::ToolButtonIconOnly);
		btn->setIconSize(QSize(20, 20));
		btn->setFixedSize(34, 34);
		btn->setText(QString());
	}
	else
	{
		btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
		btn->setIconSize(QSize(22, 22));
		btn->setText(text);
		btn->setMinimumWidth(44);
		btn->setFixedHeight(58);
	}
	return btn;
}

QWidget* makeGroup(QWidget* parent, const QString& title, QHBoxLayout*& outButtons)
{
	auto* group = new QWidget(parent);
	group->setObjectName(QStringLiteral("RibbonGroup"));
	auto* lay = new QVBoxLayout(group);
	lay->setContentsMargins(6, 3, 6, 2);
	lay->setSpacing(1);

	auto* buttonsHost = new QWidget(group);
	buttonsHost->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
	outButtons = new QHBoxLayout(buttonsHost);
	outButtons->setContentsMargins(0, 0, 0, 0);
	outButtons->setSpacing(2);
	lay->addWidget(buttonsHost, 0, Qt::AlignVCenter);

	auto* label = new QLabel(title, group);
	label->setAlignment(Qt::AlignHCenter);
	label->setObjectName(QStringLiteral("RibbonGroupTitle"));
	label->setFixedHeight(12);
	lay->addWidget(label, 0, Qt::AlignHCenter);
	return group;
}

void addCompactSep(QHBoxLayout* lay)
{
	auto* sep = new QFrame;
	sep->setObjectName(QStringLiteral("RibbonSep"));
	sep->setFrameShape(QFrame::VLine);
	sep->setFixedWidth(1);
	sep->setFixedHeight(28);
	lay->addWidget(sep);
}

void bindTool(QToolButton* btn, DrawingCanvasTool tool, DrawingRibbonBar* bar, QButtonGroup* group)
{
	group->addButton(btn);
	btn->setProperty("canvasTool", static_cast<int>(tool));
	QObject::connect(btn, &QToolButton::clicked, bar, [bar, tool]() { emit bar->toolRequested(tool); });
}
} // namespace

DrawingRibbonBar::DrawingRibbonBar(QWidget* parent) : QWidget(parent)
{
	setObjectName(QStringLiteral("DrawingRibbonBar"));
	setAttribute(Qt::WA_StyledBackground, true);
	setFixedHeight(72);
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

	auto* outer = new QHBoxLayout(this);
	outer->setContentsMargins(0, 0, 0, 0);
	outer->setSpacing(0);

	auto* scroll = new QScrollArea(this);
	scroll->setObjectName(QStringLiteral("RibbonScroll"));
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	scroll->setFocusPolicy(Qt::NoFocus);
	outer->addWidget(scroll);

	auto* host = new QWidget;
	host->setObjectName(QStringLiteral("RibbonHost"));
	scroll->setWidget(host);

	auto* root = new QHBoxLayout(host);
	root->setContentsMargins(6, 4, 8, 4);
	root->setSpacing(6);

	m_tools = new QButtonGroup(this);
	m_tools->setExclusive(true);

	QHBoxLayout* viewBtns = nullptr;
	QWidget* view = makeGroup(host, QStringLiteral("视图"), viewBtns);
	m_lblView = view->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnPan = makeRibbonButton(view, QStringLiteral("选择/拖视图"), QStringLiteral("pan"));
	m_btnDetail = makeRibbonButton(view, QStringLiteral("局部放大"), QStringLiteral("detail"));
	bindTool(m_btnPan, DrawingCanvasTool::PanSelect, this, m_tools);
	bindTool(m_btnDetail, DrawingCanvasTool::DetailRegion, this, m_tools);
	viewBtns->addWidget(m_btnPan);
	viewBtns->addWidget(m_btnDetail);
	root->addWidget(view);

	QHBoxLayout* sketchBtns = nullptr;
	QWidget* sketch = makeGroup(host, QStringLiteral("绘图"), sketchBtns);
	m_lblSketch = sketch->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnLine = makeRibbonButton(sketch, QStringLiteral("直线"), QStringLiteral("line"));
	m_btnArc = makeRibbonButton(sketch, QStringLiteral("圆弧"), QStringLiteral("arc"));
	m_btnCircle = makeRibbonButton(sketch, QStringLiteral("圆"), QStringLiteral("circle"));
	m_btnRect = makeRibbonButton(sketch, QStringLiteral("矩形"), QStringLiteral("rect"));
	m_btnSpline = makeRibbonButton(sketch, QStringLiteral("样条"), QStringLiteral("spline"));
	m_btnSelect = makeRibbonButton(sketch, QStringLiteral("选择图元"), QStringLiteral("select"));
	bindTool(m_btnLine, DrawingCanvasTool::SketchLine, this, m_tools);
	bindTool(m_btnArc, DrawingCanvasTool::SketchArc, this, m_tools);
	bindTool(m_btnCircle, DrawingCanvasTool::SketchCircle, this, m_tools);
	bindTool(m_btnRect, DrawingCanvasTool::SketchRect, this, m_tools);
	bindTool(m_btnSpline, DrawingCanvasTool::SketchSpline, this, m_tools);
	bindTool(m_btnSelect, DrawingCanvasTool::SelectEntity, this, m_tools);
	sketchBtns->addWidget(m_btnLine);
	sketchBtns->addWidget(m_btnArc);
	sketchBtns->addWidget(m_btnCircle);
	sketchBtns->addWidget(m_btnRect);
	sketchBtns->addWidget(m_btnSpline);
	addCompactSep(sketchBtns);
	sketchBtns->addWidget(m_btnSelect);
	root->addWidget(sketch);

	QHBoxLayout* markBtns = nullptr;
	QWidget* marks = makeGroup(host, QStringLiteral("标注"), markBtns);
	m_lblMarks = marks->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnDimLinear = makeRibbonButton(marks, QStringLiteral("线性尺寸"), QStringLiteral("dimLen"));
	m_btnDimRadius = makeRibbonButton(marks, QStringLiteral("半径尺寸"), QStringLiteral("dimRad"));
	m_btnDimDiameter = makeRibbonButton(marks, QStringLiteral("直径尺寸"), QStringLiteral("dimDia"));
	bindTool(m_btnDimLinear, DrawingCanvasTool::LinearDim, this, m_tools);
	bindTool(m_btnDimRadius, DrawingCanvasTool::DimRadius, this, m_tools);
	bindTool(m_btnDimDiameter, DrawingCanvasTool::DimDiameter, this, m_tools);
	markBtns->addWidget(m_btnDimLinear);
	markBtns->addWidget(m_btnDimRadius);
	markBtns->addWidget(m_btnDimDiameter);
	root->addWidget(marks);

	root->addStretch(1);
	m_btnPan->setChecked(true);
	applyTheme(false);
	applyLanguage(true);
}

void DrawingRibbonBar::setActiveTool(DrawingCanvasTool tool)
{
	if (!m_tools)
		return;
	for (QAbstractButton* b : m_tools->buttons())
	{
		if (b->property("canvasTool").toInt() == static_cast<int>(tool))
		{
			b->setChecked(true);
			return;
		}
	}
}

void DrawingRibbonBar::applyTheme(bool dark)
{
	m_dark = dark;
	const QString sheet =
		dark ? QStringLiteral("#DrawingRibbonBar {"
							  "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
							  "    stop:0 #27272a, stop:1 #18181b);"
							  "  border-bottom: 2px solid #0f766e;"
							  "}"
							  "#RibbonScroll, #RibbonHost { background: transparent; }"
							  "#RibbonGroup {"
							  "  background-color: #27272a;"
							  "  border: 1px solid #3f3f46;"
							  "  border-radius: 6px;"
							  "}"
							  "QFrame#RibbonSep { background: #3f3f46; border: none; }"
							  "QLabel#RibbonGroupTitle {"
							  "  color: #a1a1aa;"
							  "  font-size: 9px;"
							  "  font-weight: 600;"
							  "}"
							  "QToolButton#RibbonBtn {"
							  "  background-color: #3f3f46;"
							  "  color: #f4f4f5;"
							  "  border: 1px solid #52525b;"
							  "  border-radius: 5px;"
							  "  padding: 1px;"
							  "}"
							  "QToolButton#RibbonBtn:hover {"
							  "  background-color: #52525b;"
							  "  border-color: #14b8a6;"
							  "}"
							  "QToolButton#RibbonBtn:checked {"
							  "  background-color: #115e59;"
							  "  border: 2px solid #2dd4bf;"
							  "}")
			 : QStringLiteral("#DrawingRibbonBar {"
							  "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
							  "    stop:0 #ffffff, stop:1 #f4f4f5);"
							  "  border-bottom: 2px solid #0f766e;"
							  "}"
							  "#RibbonScroll, #RibbonHost { background: transparent; }"
							  "#RibbonGroup {"
							  "  background-color: #ffffff;"
							  "  border: 1px solid #d4d4d8;"
							  "  border-radius: 6px;"
							  "}"
							  "QFrame#RibbonSep { background: #d4d4d8; border: none; }"
							  "QLabel#RibbonGroupTitle {"
							  "  color: #71717a;"
							  "  font-size: 9px;"
							  "  font-weight: 600;"
							  "}"
							  "QToolButton#RibbonBtn {"
							  "  background-color: #f4f4f5;"
							  "  color: #18181b;"
							  "  border: 1px solid #d4d4d8;"
							  "  border-radius: 5px;"
							  "  padding: 1px;"
							  "}"
							  "QToolButton#RibbonBtn:hover {"
							  "  background-color: #ecfdf5;"
							  "  border-color: #14b8a6;"
							  "}"
							  "QToolButton#RibbonBtn:checked {"
							  "  background-color: #ccfbf1;"
							  "  border: 2px solid #0f766e;"
							  "}");
	setStyleSheet(sheet);
	rebuildIcons(dark);
}

void DrawingRibbonBar::rebuildIcons(bool dark)
{
	for (QToolButton* btn : findChildren<QToolButton*>(QStringLiteral("RibbonBtn")))
	{
		const QString kind = btn->property("glyphKind").toString();
		if (!kind.isEmpty())
			btn->setIcon(makeGlyphIcon(kind, dark));
	}
}

void DrawingRibbonBar::setBtnText(QToolButton* btn, const QString& text)
{
	if (!btn)
		return;
	btn->setToolTip(text);
	if (!btn->property("compact").toBool())
		btn->setText(text);
}

void DrawingRibbonBar::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	auto tr = [useChinese](const QString& en, const QString& zh) { return useChinese ? zh : en; };
	if (m_lblView)
		m_lblView->setText(tr(QStringLiteral("View"), QStringLiteral("视图")));
	if (m_lblSketch)
		m_lblSketch->setText(tr(QStringLiteral("Draw"), QStringLiteral("绘图")));
	if (m_lblMarks)
		m_lblMarks->setText(tr(QStringLiteral("Annotate"), QStringLiteral("标注")));
	setBtnText(m_btnPan, tr(QStringLiteral("Select/Pan"), QStringLiteral("选择/拖视图")));
	setBtnText(m_btnDetail, tr(QStringLiteral("Detail"), QStringLiteral("局部放大")));
	setBtnText(m_btnLine, tr(QStringLiteral("Line"), QStringLiteral("直线")));
	setBtnText(m_btnArc, tr(QStringLiteral("Arc"), QStringLiteral("圆弧")));
	setBtnText(m_btnCircle, tr(QStringLiteral("Circle"), QStringLiteral("圆")));
	setBtnText(m_btnRect, tr(QStringLiteral("Rectangle"), QStringLiteral("矩形")));
	setBtnText(m_btnSpline, tr(QStringLiteral("Spline"), QStringLiteral("样条")));
	setBtnText(m_btnSelect, tr(QStringLiteral("Select Entity"), QStringLiteral("选择图元")));
	setBtnText(m_btnDimLinear, tr(QStringLiteral("Linear Dim"), QStringLiteral("线性尺寸")));
	setBtnText(m_btnDimRadius, tr(QStringLiteral("Radius"), QStringLiteral("半径尺寸")));
	setBtnText(m_btnDimDiameter, tr(QStringLiteral("Diameter"), QStringLiteral("直径尺寸")));
}
