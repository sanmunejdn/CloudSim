/// @file GeometricModelingRibbonBar.cpp
/// @brief CAD 模式条：卡片分组 + 青绿强调（与灰阶宿主工具栏区分）

#include "GeometricModelingRibbonBar.h"
#include "GeomodelingI18n.h"

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
// 工程青绿：浅色/深色共用同一强调，避免紫粉 AI 默认色
constexpr QRgb kAccent = 0xff0f766e;	  // teal-700
constexpr QRgb kAccentSoft = 0xff14b8a6; // teal-500

QIcon makeGlyphIcon(const QString& kind, bool dark, bool accentFill = false)
{
	QPixmap pm(40, 40);
	pm.fill(Qt::transparent);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	const QColor ink = accentFill ? QColor(kAccentSoft) : (dark ? QColor(0xf4, 0xf4, 0xf5) : QColor(0x18, 0x18, 0x1b));
	const QColor fillSoft = dark ? QColor(0x3f, 0x3f, 0x46) : QColor(0xe4, 0xe4, 0xe7);
	const QColor hole = dark ? QColor(0x27, 0x27, 0x2a) : QColor(0xfa, 0xfa, 0xfa);
	p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	p.setBrush(Qt::NoBrush);

	if (kind == QLatin1String("sketch"))
	{
		p.setBrush(fillSoft);
		p.drawRoundedRect(QRectF(7, 7, 26, 26), 3, 3);
		p.setBrush(Qt::NoBrush);
		p.drawLine(10, 28, 30, 10);
		p.drawEllipse(QPointF(12, 12), 2.2, 2.2);
		p.drawEllipse(QPointF(28, 28), 2.2, 2.2);
	}
	else if (kind == QLatin1String("solve"))
	{
		p.drawEllipse(QRectF(8, 8, 24, 24));
		p.setPen(QPen(QColor(kAccentSoft), 2.2, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(16, 12, 16, 22);
		p.drawLine(16, 22, 24, 18);
	}
	else if (kind == QLatin1String("pad"))
	{
		p.setBrush(QColor(kAccent));
		p.setPen(Qt::NoPen);
		p.drawRoundedRect(QRectF(9, 18, 22, 12), 2, 2);
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		p.setBrush(Qt::NoBrush);
		p.drawLine(9, 18, 14, 10);
		p.drawLine(31, 18, 36, 10);
		p.drawLine(14, 10, 36, 10);
	}
	else if (kind == QLatin1String("pocket"))
	{
		p.drawRoundedRect(QRectF(7, 9, 26, 22), 2, 2);
		p.setBrush(hole);
		p.drawRoundedRect(QRectF(12, 14, 16, 12), 1, 1);
	}
	else if (kind == QLatin1String("sweep"))
	{
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		p.drawArc(QRectF(6, 10, 28, 20), 200 * 16, 140 * 16);
		p.setBrush(QColor(kAccent));
		p.setPen(Qt::NoPen);
		p.drawRoundedRect(QRectF(8, 22, 10, 8), 1, 1);
	}
	else if (kind == QLatin1String("sweepCut"))
	{
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		p.drawArc(QRectF(6, 10, 28, 20), 200 * 16, 140 * 16);
		p.setBrush(hole);
		p.setPen(QPen(ink, 1.5));
		p.drawRoundedRect(QRectF(8, 22, 10, 8), 1, 1);
	}
	else if (kind == QLatin1String("rebuild"))
	{
		p.drawArc(QRectF(8, 8, 24, 24), 45 * 16, 270 * 16);
		p.setBrush(ink);
		p.setPen(Qt::NoPen);
		const QPointF tip(30, 10);
		QPolygonF arrow;
		arrow << tip << QPointF(24, 9) << QPointF(29, 16);
		p.drawPolygon(arrow);
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
	else if (kind == QLatin1String("endSketch"))
	{
		p.setPen(QPen(dark ? QColor(0xfa, 0xa2, 0xa2) : QColor(0xdc, 0x26, 0x26), 2.2, Qt::SolidLine, Qt::RoundCap));
		p.drawRoundedRect(QRectF(8, 8, 24, 24), 3, 3);
		p.drawLine(13, 13, 27, 27);
		p.drawLine(27, 13, 13, 27);
	}
	else if (kind == QLatin1String("undo"))
	{
		p.drawArc(QRectF(9, 10, 20, 20), 40 * 16, 200 * 16);
		p.setBrush(ink);
		p.setPen(Qt::NoPen);
		QPolygonF a;
		a << QPointF(9, 14) << QPointF(4, 18) << QPointF(12, 20);
		p.drawPolygon(a);
	}
	else if (kind == QLatin1String("redo"))
	{
		p.drawArc(QRectF(11, 10, 20, 20), -40 * 16, -200 * 16);
		p.setBrush(ink);
		p.setPen(Qt::NoPen);
		QPolygonF a;
		a << QPointF(31, 14) << QPointF(36, 18) << QPointF(28, 20);
		p.drawPolygon(a);
	}
	else if (kind == QLatin1String("dimLen"))
	{
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(8, 28, 32, 12);
		p.drawLine(10, 22, 16, 30);
		p.drawLine(24, 10, 30, 18);
	}
	else if (kind == QLatin1String("dimDist"))
	{
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(10, 28), 3, 3);
		p.drawEllipse(QPointF(30, 12), 3, 3);
		p.setPen(QPen(ink, 1.8, Qt::DashLine, Qt::RoundCap));
		p.setBrush(Qt::NoBrush);
		p.drawLine(12, 26, 28, 14);
	}
	else if (kind == QLatin1String("dimRad"))
	{
		p.drawEllipse(QRectF(8, 8, 24, 24));
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(20, 20, 30, 12);
	}
	else if (kind == QLatin1String("dimAng"))
	{
		p.drawLine(8, 30, 20, 10);
		p.drawLine(20, 10, 32, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawArc(QRectF(12, 8, 16, 16), 200 * 16, 140 * 16);
	}
	else if (kind == QLatin1String("dimArcR"))
	{
		p.drawArc(QRectF(7, 7, 26, 26), 30 * 16, 120 * 16);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(20, 20, 28, 12);
	}
	else if (kind == QLatin1String("constr"))
	{
		p.setPen(QPen(ink, 2.0, Qt::DashLine, Qt::RoundCap));
		p.drawLine(8, 30, 32, 10);
	}
	else if (kind == QLatin1String("geomH"))
	{
		p.drawLine(8, 20, 32, 20);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(14, 14, 26, 14);
	}
	else if (kind == QLatin1String("geomV"))
	{
		p.drawLine(20, 8, 20, 32);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(14, 14, 14, 26);
	}
	else if (kind == QLatin1String("geomCoin"))
	{
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(16, 20), 4, 4);
		p.setBrush(ink);
		p.drawEllipse(QPointF(24, 20), 4, 4);
	}
	else if (kind == QLatin1String("geomPar"))
	{
		p.drawLine(10, 28, 22, 10);
		p.drawLine(18, 30, 30, 12);
	}
	else if (kind == QLatin1String("geomPerp"))
	{
		p.drawLine(10, 28, 28, 10);
		p.drawLine(18, 10, 30, 22);
	}
	else if (kind == QLatin1String("geomEq"))
	{
		p.drawLine(8, 26, 20, 14);
		p.drawLine(20, 26, 32, 14);
		p.setPen(QPen(QColor(kAccentSoft), 1.6, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(10, 30, 18, 30);
		p.drawLine(22, 30, 30, 30);
	}
	else if (kind == QLatin1String("geomFix"))
	{
		p.setBrush(QColor(kAccentSoft));
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(20, 18), 4, 4);
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap));
		p.setBrush(Qt::NoBrush);
		p.drawLine(20, 22, 20, 32);
		p.drawLine(14, 32, 26, 32);
	}
	else if (kind == QLatin1String("trim"))
	{
		p.drawLine(8, 28, 32, 12);
		p.setPen(QPen(QColor(0xdc, 0x26, 0x26), 2.2, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(18, 14, 26, 26);
	}
	else if (kind == QLatin1String("mirror"))
	{
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::DashLine, Qt::RoundCap));
		p.drawLine(20, 8, 20, 32);
		p.setPen(QPen(ink, 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(8, 26, 16, 14);
		p.drawLine(24, 14, 32, 26);
	}
	else if (kind == QLatin1String("delete"))
	{
		p.setPen(QPen(QColor(0xdc, 0x26, 0x26), 2.2, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(12, 12, 28, 28);
		p.drawLine(28, 12, 12, 28);
	}
	return QIcon(pm);
}


QToolButton* makeRibbonButton(QWidget* parent, const QString& text, const QString& kind, const QString& role,
							  bool compact = false)
{
	auto* btn = new QToolButton(parent);
	btn->setObjectName(QStringLiteral("RibbonBtn"));
	btn->setProperty("glyphKind", kind);
	btn->setProperty("btnRole", role);
	btn->setProperty("compact", compact);
	btn->setAutoRaise(false);
	btn->setCursor(Qt::PointingHandCursor);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setToolTip(text);
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
		btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
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
} // namespace

GeometricModelingRibbonBar::GeometricModelingRibbonBar(QWidget* parent) : QWidget(parent)
{
	setObjectName(QStringLiteral("GeometricModelingRibbonBar"));
	setAttribute(Qt::WA_StyledBackground, true);
	setFixedHeight(88);
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

	m_drawTools = new QButtonGroup(this);
	m_drawTools->setExclusive(true);

	QHBoxLayout* sketchBtns = nullptr;
	QWidget* sketch = makeGroup(host, QStringLiteral("草图"), sketchBtns);
	m_lblSketch = sketch->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnNewSketch = makeRibbonButton(sketch, QStringLiteral("新建"), QStringLiteral("sketch"), QStringLiteral("normal"));
	auto* newSketch = m_btnNewSketch;
	m_btnLine = makeRibbonButton(sketch, QStringLiteral("直线"), QStringLiteral("line"), QStringLiteral("draw"), true);
	m_btnArc = makeRibbonButton(sketch, QStringLiteral("圆弧"), QStringLiteral("arc"), QStringLiteral("draw"), true);
	m_btnCircle = makeRibbonButton(sketch, QStringLiteral("圆"), QStringLiteral("circle"), QStringLiteral("draw"), true);
	m_btnRect = makeRibbonButton(sketch, QStringLiteral("矩形"), QStringLiteral("rect"), QStringLiteral("draw"), true);
	m_btnSpline = makeRibbonButton(sketch, QStringLiteral("样条"), QStringLiteral("spline"), QStringLiteral("draw"), true);
	m_btnConstr = makeRibbonButton(sketch, QStringLiteral("构造线"), QStringLiteral("constr"), QStringLiteral("draw"), true);
	m_btnTrim = makeRibbonButton(sketch, QStringLiteral("修剪"), QStringLiteral("trim"), QStringLiteral("draw"), true);
	m_btnMirror = makeRibbonButton(sketch, QStringLiteral("镜像"), QStringLiteral("mirror"), QStringLiteral("draw"), true);
	m_btnDelete = makeRibbonButton(sketch, QStringLiteral("删除"), QStringLiteral("delete"), QStringLiteral("draw"), true);
	m_btnEndSketch = makeRibbonButton(sketch, QStringLiteral("结束"), QStringLiteral("endSketch"), QStringLiteral("danger"));
	auto* endSketch = m_btnEndSketch;
	for (QToolButton* b :
		 {m_btnLine, m_btnArc, m_btnCircle, m_btnRect, m_btnSpline, m_btnConstr, m_btnTrim, m_btnMirror, m_btnDelete})
	{
		b->setCheckable(true);
		m_drawTools->addButton(b);
	}
	sketchBtns->addWidget(newSketch);
	addCompactSep(sketchBtns);
	sketchBtns->addWidget(m_btnLine);
	sketchBtns->addWidget(m_btnArc);
	sketchBtns->addWidget(m_btnCircle);
	sketchBtns->addWidget(m_btnRect);
	sketchBtns->addWidget(m_btnSpline);
	addCompactSep(sketchBtns);
	sketchBtns->addWidget(m_btnConstr);
	sketchBtns->addWidget(m_btnTrim);
	sketchBtns->addWidget(m_btnMirror);
	sketchBtns->addWidget(m_btnDelete);
	addCompactSep(sketchBtns);
	sketchBtns->addWidget(endSketch);
	connect(newSketch, &QToolButton::clicked, this, &GeometricModelingRibbonBar::newSketchRequested);
	connect(m_btnLine, &QToolButton::clicked, this, &GeometricModelingRibbonBar::lineToolRequested);
	connect(m_btnArc, &QToolButton::clicked, this, &GeometricModelingRibbonBar::arcToolRequested);
	connect(m_btnCircle, &QToolButton::clicked, this, &GeometricModelingRibbonBar::circleToolRequested);
	connect(m_btnRect, &QToolButton::clicked, this, &GeometricModelingRibbonBar::rectToolRequested);
	connect(m_btnSpline, &QToolButton::clicked, this, &GeometricModelingRibbonBar::splineToolRequested);
	connect(m_btnConstr, &QToolButton::clicked, this, &GeometricModelingRibbonBar::constructionToolRequested);
	connect(m_btnTrim, &QToolButton::clicked, this, &GeometricModelingRibbonBar::trimToolRequested);
	connect(m_btnMirror, &QToolButton::clicked, this, &GeometricModelingRibbonBar::mirrorToolRequested);
	connect(m_btnDelete, &QToolButton::clicked, this, &GeometricModelingRibbonBar::deleteToolRequested);
	connect(endSketch, &QToolButton::clicked, this,
			[this]()
			{
				clearToolChecks();
				emit endSketchRequested();
			});
	root->addWidget(sketch);

	QHBoxLayout* markBtns = nullptr;
	QWidget* marks = makeGroup(host, QStringLiteral("标注"), markBtns);
	m_lblMarks = marks->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnDimLen = makeRibbonButton(marks, QStringLiteral("长度"), QStringLiteral("dimLen"), QStringLiteral("draw"), true);
	m_btnDimDist = makeRibbonButton(marks, QStringLiteral("距离"), QStringLiteral("dimDist"), QStringLiteral("draw"), true);
	m_btnDimRad = makeRibbonButton(marks, QStringLiteral("半径"), QStringLiteral("dimRad"), QStringLiteral("draw"), true);
	m_btnDimAng = makeRibbonButton(marks, QStringLiteral("角度"), QStringLiteral("dimAng"), QStringLiteral("draw"), true);
	m_btnDimArcR = makeRibbonButton(marks, QStringLiteral("弧半径"), QStringLiteral("dimArcR"), QStringLiteral("draw"), true);
	m_btnGeomH = makeRibbonButton(marks, QStringLiteral("水平"), QStringLiteral("geomH"), QStringLiteral("draw"), true);
	m_btnGeomV = makeRibbonButton(marks, QStringLiteral("竖直"), QStringLiteral("geomV"), QStringLiteral("draw"), true);
	m_btnGeomCoin = makeRibbonButton(marks, QStringLiteral("重合"), QStringLiteral("geomCoin"), QStringLiteral("draw"), true);
	m_btnGeomPar = makeRibbonButton(marks, QStringLiteral("平行"), QStringLiteral("geomPar"), QStringLiteral("draw"), true);
	m_btnGeomPerp = makeRibbonButton(marks, QStringLiteral("垂直"), QStringLiteral("geomPerp"), QStringLiteral("draw"), true);
	m_btnGeomEq = makeRibbonButton(marks, QStringLiteral("等长"), QStringLiteral("geomEq"), QStringLiteral("draw"), true);
	m_btnGeomFix = makeRibbonButton(marks, QStringLiteral("固定"), QStringLiteral("geomFix"), QStringLiteral("draw"), true);
	for (QToolButton* b : {m_btnDimLen, m_btnDimDist, m_btnDimRad, m_btnDimAng, m_btnDimArcR})
	{
		b->setCheckable(true);
		m_drawTools->addButton(b);
		markBtns->addWidget(b);
	}
	addCompactSep(markBtns);
	for (QToolButton* b : {m_btnGeomH, m_btnGeomV, m_btnGeomCoin, m_btnGeomPar, m_btnGeomPerp, m_btnGeomEq, m_btnGeomFix})
	{
		b->setCheckable(true);
		m_drawTools->addButton(b);
		markBtns->addWidget(b);
	}
	connect(m_btnDimLen, &QToolButton::clicked, this, &GeometricModelingRibbonBar::dimLengthRequested);
	connect(m_btnDimDist, &QToolButton::clicked, this, &GeometricModelingRibbonBar::dimDistanceRequested);
	connect(m_btnDimRad, &QToolButton::clicked, this, &GeometricModelingRibbonBar::dimRadiusRequested);
	connect(m_btnDimAng, &QToolButton::clicked, this, &GeometricModelingRibbonBar::dimAngleRequested);
	connect(m_btnDimArcR, &QToolButton::clicked, this, &GeometricModelingRibbonBar::dimArcRadiusRequested);
	connect(m_btnGeomH, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomHorizontalRequested);
	connect(m_btnGeomV, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomVerticalRequested);
	connect(m_btnGeomCoin, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomCoincidentRequested);
	connect(m_btnGeomPar, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomParallelRequested);
	connect(m_btnGeomPerp, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomPerpendicularRequested);
	connect(m_btnGeomEq, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomEqualLengthRequested);
	connect(m_btnGeomFix, &QToolButton::clicked, this, &GeometricModelingRibbonBar::geomFixRequested);
	root->addWidget(marks);

	QHBoxLayout* featBtns = nullptr;
	QWidget* feat = makeGroup(host, QStringLiteral("特征"), featBtns);
	m_lblFeat = feat->findChild<QLabel*>(QStringLiteral("RibbonGroupTitle"));
	m_btnSolve = makeRibbonButton(feat, QStringLiteral("求解"), QStringLiteral("solve"), QStringLiteral("normal"), true);
	m_btnPad = makeRibbonButton(feat, QStringLiteral("拉伸"), QStringLiteral("pad"), QStringLiteral("primary"));
	m_btnPocket = makeRibbonButton(feat, QStringLiteral("切除"), QStringLiteral("pocket"), QStringLiteral("normal"), true);
	m_btnSweep = makeRibbonButton(feat, QStringLiteral("扫描"), QStringLiteral("sweep"), QStringLiteral("normal"));
	m_btnSweepCut = makeRibbonButton(feat, QStringLiteral("扫描切除"), QStringLiteral("sweepCut"), QStringLiteral("normal"), true);
	m_btnRebuild = makeRibbonButton(feat, QStringLiteral("重建"), QStringLiteral("rebuild"), QStringLiteral("normal"), true);
	m_btnUndo = makeRibbonButton(feat, QStringLiteral("撤销"), QStringLiteral("undo"), QStringLiteral("normal"), true);
	m_btnRedo = makeRibbonButton(feat, QStringLiteral("重做"), QStringLiteral("redo"), QStringLiteral("normal"), true);
	auto* solve = m_btnSolve;
	auto* pad = m_btnPad;
	auto* pocket = m_btnPocket;
	auto* sweep = m_btnSweep;
	auto* sweepCut = m_btnSweepCut;
	auto* rebuild = m_btnRebuild;
	auto* undo = m_btnUndo;
	auto* redo = m_btnRedo;
	featBtns->addWidget(solve);
	featBtns->addWidget(pad);
	featBtns->addWidget(pocket);
	featBtns->addWidget(sweep);
	featBtns->addWidget(sweepCut);
	featBtns->addWidget(rebuild);
	addCompactSep(featBtns);
	featBtns->addWidget(undo);
	featBtns->addWidget(redo);
	connect(solve, &QToolButton::clicked, this, &GeometricModelingRibbonBar::solveRequested);
	connect(pad, &QToolButton::clicked, this,
			[this]()
			{
				clearToolChecks();
				emit padRequested();
			});
	connect(pocket, &QToolButton::clicked, this,
			[this]()
			{
				clearToolChecks();
				emit pocketRequested();
			});
	connect(sweep, &QToolButton::clicked, this,
			[this]()
			{
				clearToolChecks();
				emit sweepRequested();
			});
	connect(sweepCut, &QToolButton::clicked, this,
			[this]()
			{
				clearToolChecks();
				emit sweepCutRequested();
			});
	connect(rebuild, &QToolButton::clicked, this, &GeometricModelingRibbonBar::rebuildRequested);
	connect(undo, &QToolButton::clicked, this, &GeometricModelingRibbonBar::undoRequested);
	connect(redo, &QToolButton::clicked, this, &GeometricModelingRibbonBar::redoRequested);
	root->addWidget(feat);

	root->addStretch(1);
	applyTheme(false);
	applyLanguage(true);
}

void GeometricModelingRibbonBar::clearToolChecks()
{
	if (!m_drawTools)
		return;
	const bool was = m_drawTools->exclusive();
	m_drawTools->setExclusive(false);
	for (QAbstractButton* b : m_drawTools->buttons())
		b->setChecked(false);
	m_drawTools->setExclusive(was);
}

void GeometricModelingRibbonBar::applyTheme(bool dark)
{
	m_dark = dark;
	const QString sheet = dark ? QStringLiteral(
									 "#GeometricModelingRibbonBar {"
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
									 "  font-size: 10px;"
									 "  font-weight: 600;"
									 "}"
									 "QToolButton#RibbonBtn:hover {"
									 "  background-color: #52525b;"
									 "  border-color: #14b8a6;"
									 "}"
									 "QToolButton#RibbonBtn:pressed { background-color: #0f766e; }"
									 "QToolButton#RibbonBtn:checked {"
									 "  background-color: #115e59;"
									 "  border: 2px solid #2dd4bf;"
									 "  color: #ccfbf1;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"primary\"] {"
									 "  background-color: #0f766e;"
									 "  border-color: #14b8a6;"
									 "  color: #ffffff;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"primary\"]:hover { background-color: #0d9488; }"
									 "QToolButton#RibbonBtn[btnRole=\"danger\"] {"
									 "  background-color: #3f1d1d;"
									 "  border-color: #7f1d1d;"
									 "  color: #fecaca;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"danger\"]:hover { background-color: #7f1d1d; }")
							   : QStringLiteral(
									 "#GeometricModelingRibbonBar {"
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
									 "  font-size: 10px;"
									 "  font-weight: 600;"
									 "}"
									 "QToolButton#RibbonBtn:hover {"
									 "  background-color: #ecfdf5;"
									 "  border-color: #14b8a6;"
									 "}"
									 "QToolButton#RibbonBtn:pressed { background-color: #ccfbf1; }"
									 "QToolButton#RibbonBtn:checked {"
									 "  background-color: #ccfbf1;"
									 "  border: 2px solid #0f766e;"
									 "  color: #115e59;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"primary\"] {"
									 "  background-color: #0f766e;"
									 "  border-color: #0d9488;"
									 "  color: #ffffff;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"primary\"]:hover { background-color: #0d9488; }"
									 "QToolButton#RibbonBtn[btnRole=\"danger\"] {"
									 "  background-color: #fef2f2;"
									 "  border-color: #fca5a5;"
									 "  color: #b91c1c;"
									 "}"
									 "QToolButton#RibbonBtn[btnRole=\"danger\"]:hover { background-color: #fee2e2; }");
	setStyleSheet(sheet);
	rebuildIcons(dark);
}

void GeometricModelingRibbonBar::rebuildIcons(bool dark)
{
	const auto buttons = findChildren<QToolButton*>(QStringLiteral("RibbonBtn"));
	for (QToolButton* btn : buttons)
	{
		const QString kind = btn->property("glyphKind").toString();
		const QString role = btn->property("btnRole").toString();
		if (kind.isEmpty())
			continue;
		const bool accent = (role == QLatin1String("primary"));
		btn->setIcon(makeGlyphIcon(kind, dark, accent));
	}
}


void GeometricModelingRibbonBar::setBtnText(QToolButton* btn, const QString& text)
{
	if (!btn)
		return;
	btn->setToolTip(text);
	if (!btn->property("compact").toBool())
		btn->setText(text);
}

void GeometricModelingRibbonBar::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	auto tr = [useChinese](const QString& en, const QString& zh) { return gmTr(useChinese, en, zh); };
	if (m_lblSketch)
		m_lblSketch->setText(tr(QStringLiteral("Sketch"), QStringLiteral("草图")));
	if (m_lblMarks)
		m_lblMarks->setText(tr(QStringLiteral("Annotate"), QStringLiteral("标注")));
	if (m_lblFeat)
		m_lblFeat->setText(tr(QStringLiteral("Features"), QStringLiteral("特征")));

	setBtnText(m_btnNewSketch, tr(QStringLiteral("New"), QStringLiteral("新建")));
	setBtnText(m_btnEndSketch, tr(QStringLiteral("Exit Sketch"), QStringLiteral("结束")));
	setBtnText(m_btnLine, tr(QStringLiteral("Line"), QStringLiteral("直线")));
	setBtnText(m_btnArc, tr(QStringLiteral("Arc"), QStringLiteral("圆弧")));
	setBtnText(m_btnCircle, tr(QStringLiteral("Circle"), QStringLiteral("圆")));
	setBtnText(m_btnRect, tr(QStringLiteral("Rectangle"), QStringLiteral("矩形")));
	setBtnText(m_btnSpline, tr(QStringLiteral("Spline"), QStringLiteral("样条")));
	setBtnText(m_btnConstr, tr(QStringLiteral("Construction"), QStringLiteral("构造线")));
	setBtnText(m_btnTrim, tr(QStringLiteral("Trim"), QStringLiteral("修剪")));
	setBtnText(m_btnMirror, tr(QStringLiteral("Mirror"), QStringLiteral("镜像")));
	setBtnText(m_btnDelete, tr(QStringLiteral("Delete"), QStringLiteral("删除")));
	setBtnText(m_btnDimLen, tr(QStringLiteral("Length"), QStringLiteral("长度")));
	setBtnText(m_btnDimDist, tr(QStringLiteral("Distance"), QStringLiteral("距离")));
	setBtnText(m_btnDimRad, tr(QStringLiteral("Radius"), QStringLiteral("半径")));
	setBtnText(m_btnDimAng, tr(QStringLiteral("Angle"), QStringLiteral("角度")));
	setBtnText(m_btnDimArcR, tr(QStringLiteral("Arc Radius"), QStringLiteral("弧半径")));
	setBtnText(m_btnGeomH, tr(QStringLiteral("Horizontal"), QStringLiteral("水平")));
	setBtnText(m_btnGeomV, tr(QStringLiteral("Vertical"), QStringLiteral("竖直")));
	setBtnText(m_btnGeomCoin, tr(QStringLiteral("Coincident"), QStringLiteral("重合")));
	setBtnText(m_btnGeomPar, tr(QStringLiteral("Parallel"), QStringLiteral("平行")));
	setBtnText(m_btnGeomPerp, tr(QStringLiteral("Perpendicular"), QStringLiteral("垂直")));
	setBtnText(m_btnGeomEq, tr(QStringLiteral("Equal"), QStringLiteral("等长")));
	setBtnText(m_btnGeomFix, tr(QStringLiteral("Fix"), QStringLiteral("固定")));
	setBtnText(m_btnSolve, tr(QStringLiteral("Solve"), QStringLiteral("求解")));
	setBtnText(m_btnPad, tr(QStringLiteral("Pad"), QStringLiteral("拉伸")));
	setBtnText(m_btnPocket, tr(QStringLiteral("Pocket"), QStringLiteral("切除")));
	setBtnText(m_btnSweep, tr(QStringLiteral("Sweep"), QStringLiteral("扫描")));
	setBtnText(m_btnSweepCut, tr(QStringLiteral("Sweep Cut"), QStringLiteral("扫描切除")));
	setBtnText(m_btnRebuild, tr(QStringLiteral("Rebuild"), QStringLiteral("重建")));
	setBtnText(m_btnUndo, tr(QStringLiteral("Undo"), QStringLiteral("撤销")));
	setBtnText(m_btnRedo, tr(QStringLiteral("Redo"), QStringLiteral("重做")));
}
