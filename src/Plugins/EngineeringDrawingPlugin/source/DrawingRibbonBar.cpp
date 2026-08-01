/// @file DrawingRibbonBar.cpp
/// @brief 工程图模式条：工具分组 + 出图/图幅/导出

#include "DrawingRibbonBar.h"

#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDate>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
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
	else if (kind == QLatin1String("fit"))
	{
		p.drawRect(QRectF(8, 10, 24, 20));
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(12, 14, 16, 14);
		p.drawLine(12, 14, 12, 18);
		p.drawLine(28, 26, 24, 26);
		p.drawLine(28, 26, 28, 22);
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
	else if (kind == QLatin1String("dimAng"))
	{
		p.drawLine(8, 30, 20, 10);
		p.drawLine(20, 10, 32, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawArc(QRectF(12, 8, 16, 16), 200 * 16, 140 * 16);
	}
	else if (kind == QLatin1String("note"))
	{
		p.drawLine(10, 28, 22, 14);
		p.drawRect(QRectF(20, 8, 12, 10));
		p.drawLine(22, 12, 30, 12);
	}
	else if (kind == QLatin1String("move"))
	{
		p.drawLine(10, 20, 30, 20);
		p.drawLine(26, 16, 30, 20);
		p.drawLine(26, 24, 30, 20);
	}
	else if (kind == QLatin1String("copy"))
	{
		p.drawRect(QRectF(10, 12, 12, 12));
		p.drawRect(QRectF(18, 16, 12, 12));
	}
	else if (kind == QLatin1String("rotate"))
	{
		p.drawArc(QRectF(10, 10, 20, 20), 40 * 16, 260 * 16);
		p.drawLine(28, 14, 30, 10);
	}
	else if (kind == QLatin1String("mirror"))
	{
		p.drawLine(20, 8, 20, 32);
		p.drawLine(10, 12, 18, 20);
		p.drawLine(30, 12, 22, 20);
	}
	else if (kind == QLatin1String("erase"))
	{
		p.drawLine(12, 12, 28, 28);
		p.drawLine(28, 12, 12, 28);
	}
	else if (kind == QLatin1String("match"))
	{
		p.drawRect(QRectF(8, 10, 10, 14));
		p.drawRect(QRectF(22, 10, 10, 14));
		p.setPen(QPen(QColor(kAccentSoft), 2.0));
		p.drawLine(18, 17, 22, 17);
	}
	else if (kind == QLatin1String("hatch"))
	{
		p.drawRect(QRectF(8, 10, 24, 20));
		p.drawLine(8, 14, 32, 14);
		p.drawLine(8, 20, 32, 20);
		p.drawLine(8, 26, 32, 26);
	}
	else if (kind == QLatin1String("text"))
	{
		p.drawText(QRectF(8, 8, 24, 24), Qt::AlignCenter, QStringLiteral("A"));
	}
	else if (kind == QLatin1String("alignL"))
	{
		p.drawLine(10, 10, 10, 30);
		p.drawRect(QRectF(12, 12, 16, 6));
		p.drawRect(QRectF(12, 22, 10, 6));
	}
	else if (kind == QLatin1String("alignHC"))
	{
		p.drawLine(20, 8, 20, 32);
		p.drawRect(QRectF(12, 12, 16, 6));
		p.drawRect(QRectF(14, 22, 12, 6));
	}
	else if (kind == QLatin1String("alignR"))
	{
		p.drawLine(30, 10, 30, 30);
		p.drawRect(QRectF(12, 12, 16, 6));
		p.drawRect(QRectF(18, 22, 10, 6));
	}
	else if (kind == QLatin1String("alignT"))
	{
		p.drawLine(10, 10, 30, 10);
		p.drawRect(QRectF(12, 12, 6, 16));
		p.drawRect(QRectF(22, 12, 6, 10));
	}
	else if (kind == QLatin1String("alignVC"))
	{
		p.drawLine(8, 20, 32, 20);
		p.drawRect(QRectF(12, 12, 6, 16));
		p.drawRect(QRectF(22, 14, 6, 12));
	}
	else if (kind == QLatin1String("alignB"))
	{
		p.drawLine(10, 30, 30, 30);
		p.drawRect(QRectF(12, 12, 6, 16));
		p.drawRect(QRectF(22, 18, 6, 10));
	}
	else if (kind == QLatin1String("alignProj"))
	{
		p.drawRect(QRectF(8, 8, 12, 12));
		p.drawRect(QRectF(8, 22, 12, 10));
		p.drawRect(QRectF(22, 8, 10, 12));
		p.setPen(QPen(QColor(kAccentSoft), 1.5, Qt::DashLine));
		p.drawLine(14, 8, 14, 32);
		p.drawLine(8, 14, 32, 14);
	}
	else if (kind == QLatin1String("trim"))
	{
		p.drawLine(8, 28, 28, 8);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(22, 8, 32, 18);
	}
	else if (kind == QLatin1String("offset"))
	{
		p.drawLine(10, 28, 28, 10);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::DashLine, Qt::RoundCap));
		p.drawLine(14, 32, 32, 14);
	}
	else if (kind == QLatin1String("scale"))
	{
		p.drawRect(QRectF(10, 18, 10, 10));
		p.drawRect(QRectF(18, 8, 14, 14));
	}
	else if (kind == QLatin1String("fillet"))
	{
		p.drawLine(10, 28, 10, 14);
		p.drawLine(10, 28, 26, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawArc(QRectF(10, 14, 14, 14), 180 * 16, 90 * 16);
	}
	else if (kind == QLatin1String("chamfer"))
	{
		p.drawLine(10, 28, 10, 14);
		p.drawLine(10, 28, 26, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(10, 18, 18, 28);
	}
	else if (kind == QLatin1String("extend"))
	{
		p.drawLine(8, 12, 32, 12);
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(12, 28, 28, 12);
		p.drawLine(24, 16, 28, 12);
		p.drawLine(24, 16, 26, 20);
	}
	else if (kind == QLatin1String("array"))
	{
		p.drawRect(QRectF(8, 8, 8, 8));
		p.drawRect(QRectF(20, 8, 8, 8));
		p.drawRect(QRectF(8, 20, 8, 8));
		p.setPen(QPen(QColor(kAccentSoft), 1.5));
		p.drawRect(QRectF(20, 20, 8, 8));
	}
	else if (kind == QLatin1String("polarArray"))
	{
		p.drawEllipse(QRectF(10, 10, 20, 20));
		p.setBrush(ink);
		p.setPen(Qt::NoPen);
		p.drawEllipse(QPointF(20, 12), 2.2, 2.2);
		p.drawEllipse(QPointF(28, 20), 2.2, 2.2);
		p.drawEllipse(QPointF(20, 28), 2.2, 2.2);
		p.setBrush(QColor(kAccentSoft));
		p.drawEllipse(QPointF(20, 20), 2.5, 2.5);
	}
	else if (kind == QLatin1String("dimCont"))
	{
		p.drawLine(8, 28, 18, 12);
		p.drawLine(18, 12, 32, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0));
		p.drawLine(10, 22, 16, 30);
		p.drawLine(20, 14, 28, 26);
	}
	else if (kind == QLatin1String("dimBase"))
	{
		p.drawLine(8, 30, 32, 10);
		p.setPen(QPen(QColor(kAccentSoft), 1.6));
		p.drawLine(10, 24, 20, 30);
		p.drawLine(12, 18, 24, 26);
		p.drawLine(14, 12, 28, 22);
	}
	else if (kind == QLatin1String("break"))
	{
		p.drawLine(8, 28, 18, 12);
		p.drawLine(22, 28, 32, 12);
		p.setPen(QPen(QColor(kAccentSoft), 2.0));
		p.drawLine(16, 18, 24, 22);
	}
	else if (kind == QLatin1String("join"))
	{
		p.drawLine(8, 28, 18, 14);
		p.drawLine(22, 14, 32, 28);
		p.setPen(QPen(QColor(kAccentSoft), 2.0));
		p.drawLine(16, 16, 24, 16);
	}
	else if (kind == QLatin1String("stretch"))
	{
		p.drawRect(QRectF(8, 10, 14, 14));
		p.setPen(QPen(QColor(kAccentSoft), 2.0));
		p.drawLine(18, 24, 30, 30);
		p.drawLine(26, 28, 30, 30);
		p.drawLine(28, 26, 30, 30);
	}
	else if (kind == QLatin1String("rough"))
	{
		p.drawLine(20, 8, 12, 28);
		p.drawLine(20, 8, 28, 28);
		p.drawLine(12, 28, 30, 28);
	}
	else if (kind == QLatin1String("gdt"))
	{
		p.drawRect(QRectF(10, 14, 20, 12));
		p.drawLine(10, 14, 10, 8);
		p.drawLine(10, 8, 6, 8);
	}
	else if (kind == QLatin1String("explode"))
	{
		p.drawRect(QRectF(12, 12, 16, 16));
		p.setPen(QPen(QColor(kAccentSoft), 2.0, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(10, 10, 6, 6);
		p.drawLine(30, 10, 34, 6);
		p.drawLine(10, 30, 6, 34);
		p.drawLine(30, 30, 34, 34);
	}
	else if (kind == QLatin1String("projGuide"))
	{
		p.drawRect(QRectF(8, 8, 12, 12));
		p.drawRect(QRectF(20, 20, 12, 12));
		p.setPen(QPen(QColor(kAccentSoft), 1.6, Qt::DashDotLine));
		p.drawLine(14, 8, 14, 32);
	}
	return QIcon(pm);
}

QToolButton* makeRibbonButton(QWidget* parent, const QString& text, const QString& kind, bool checkable = true)
{
	auto* btn = new QToolButton(parent);
	btn->setObjectName(QStringLiteral("RibbonBtn"));
	btn->setProperty("glyphKind", kind);
	btn->setProperty("btnRole", QStringLiteral("draw"));
	btn->setProperty("compact", true);
	btn->setAutoRaise(false);
	btn->setCursor(Qt::PointingHandCursor);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setToolTip(text);
	btn->setCheckable(checkable);
	btn->setToolButtonStyle(Qt::ToolButtonIconOnly);
	btn->setIconSize(QSize(18, 18));
	btn->setFixedSize(28, 28);
	btn->setText(QString());
	return btn;
}

QPushButton* makeActionButton(QWidget* parent, const QString& text, bool accent = false)
{
	auto* btn = new QPushButton(text, parent);
	btn->setObjectName(accent ? QStringLiteral("RibbonAccentBtn") : QStringLiteral("RibbonActionBtn"));
	btn->setCursor(Qt::PointingHandCursor);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setFixedHeight(24);
	btn->setMinimumWidth(accent ? 64 : 44);
	return btn;
}

QDoubleSpinBox* makeCompactSpin(QWidget* parent, double value, double minV, double maxV, int decimals = 2,
								int maxWidth = 68)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setObjectName(QStringLiteral("RibbonSpin"));
	s->setRange(minV, maxV);
	s->setDecimals(decimals);
	s->setSingleStep(1.0);
	s->setValue(value);
	s->setFixedHeight(24);
	s->setMaximumWidth(maxWidth);
	s->setButtonSymbols(QAbstractSpinBox::NoButtons);
	return s;
}

struct RibbonRows
{
	QHBoxLayout* top = nullptr;
	QHBoxLayout* bottom = nullptr;
};

QWidget* makeGroup(QWidget* parent, const QString& title, RibbonRows& outRows, QLabel** outTitle = nullptr)
{
	auto* group = new QWidget(parent);
	group->setObjectName(QStringLiteral("RibbonGroup"));
	auto* lay = new QVBoxLayout(group);
	lay->setContentsMargins(4, 2, 4, 1);
	lay->setSpacing(1);

	auto* buttonsHost = new QWidget(group);
	buttonsHost->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
	auto* rows = new QVBoxLayout(buttonsHost);
	rows->setContentsMargins(0, 0, 0, 0);
	rows->setSpacing(2);

	outRows.top = new QHBoxLayout;
	outRows.top->setContentsMargins(0, 0, 0, 0);
	outRows.top->setSpacing(2);
	outRows.top->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	outRows.bottom = new QHBoxLayout;
	outRows.bottom->setContentsMargins(0, 0, 0, 0);
	outRows.bottom->setSpacing(2);
	outRows.bottom->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	rows->addLayout(outRows.top);
	rows->addLayout(outRows.bottom);
	lay->addWidget(buttonsHost, 1, Qt::AlignVCenter);

	auto* label = new QLabel(title, group);
	label->setObjectName(QStringLiteral("RibbonGroupTitle"));
	label->setAlignment(Qt::AlignHCenter);
	label->setFixedHeight(12);
	lay->addWidget(label, 0, Qt::AlignHCenter);
	if (outTitle)
		*outTitle = label;
	return group;
}

void addCompactSep(QHBoxLayout* lay)
{
	auto* sep = new QFrame;
	sep->setObjectName(QStringLiteral("RibbonSep"));
	sep->setFrameShape(QFrame::VLine);
	sep->setFixedWidth(1);
	sep->setFixedHeight(22);
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
	root->setContentsMargins(4, 2, 6, 2);
	root->setSpacing(4);

	m_tools = new QButtonGroup(this);
	m_tools->setExclusive(true);

	RibbonRows viewRows;
	QWidget* view = makeGroup(host, QStringLiteral("视图"), viewRows, &m_lblView);
	m_btnPan = makeRibbonButton(view, QStringLiteral("选择/拖视图"), QStringLiteral("pan"));
	m_btnDetail = makeRibbonButton(view, QStringLiteral("局部放大"), QStringLiteral("detail"));
	m_btnFitWindow = makeRibbonButton(view, QStringLiteral("适应窗口"), QStringLiteral("fit"), false);
	m_btnAlignLeft = makeRibbonButton(view, QStringLiteral("左对齐"), QStringLiteral("alignL"), false);
	m_btnAlignHCenter = makeRibbonButton(view, QStringLiteral("水平居中"), QStringLiteral("alignHC"), false);
	m_btnAlignRight = makeRibbonButton(view, QStringLiteral("右对齐"), QStringLiteral("alignR"), false);
	m_btnAlignTop = makeRibbonButton(view, QStringLiteral("顶对齐"), QStringLiteral("alignT"), false);
	m_btnAlignVCenter = makeRibbonButton(view, QStringLiteral("垂直居中"), QStringLiteral("alignVC"), false);
	m_btnAlignBottom = makeRibbonButton(view, QStringLiteral("底对齐"), QStringLiteral("alignB"), false);
	m_btnAlignProj = makeRibbonButton(view, QStringLiteral("投影中心对齐"), QStringLiteral("alignProj"), false);
	m_gridCheck = new QCheckBox(QStringLiteral("网格"), view);
	m_gridCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_gridCheck->setChecked(true);
	bindTool(m_btnPan, DrawingCanvasTool::PanSelect, this, m_tools);
	bindTool(m_btnDetail, DrawingCanvasTool::DetailRegion, this, m_tools);
	viewRows.top->addWidget(m_btnPan);
	viewRows.top->addWidget(m_btnDetail);
	viewRows.top->addWidget(m_btnFitWindow);
	viewRows.top->addWidget(m_gridCheck);
	viewRows.bottom->addWidget(m_btnAlignLeft);
	viewRows.bottom->addWidget(m_btnAlignHCenter);
	viewRows.bottom->addWidget(m_btnAlignRight);
	viewRows.bottom->addWidget(m_btnAlignTop);
	viewRows.bottom->addWidget(m_btnAlignVCenter);
	viewRows.bottom->addWidget(m_btnAlignBottom);
	viewRows.bottom->addWidget(m_btnAlignProj);
	root->addWidget(view);

	RibbonRows sketchRows;
	QWidget* sketch = makeGroup(host, QStringLiteral("绘图"), sketchRows, &m_lblSketch);
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
	sketchRows.top->addWidget(m_btnLine);
	sketchRows.top->addWidget(m_btnArc);
	sketchRows.top->addWidget(m_btnCircle);
	sketchRows.bottom->addWidget(m_btnRect);
	sketchRows.bottom->addWidget(m_btnSpline);
	sketchRows.bottom->addWidget(m_btnSelect);
	root->addWidget(sketch);

	RibbonRows markRows;
	QWidget* marks = makeGroup(host, QStringLiteral("标注"), markRows, &m_lblMarks);
	m_btnDimLinear = makeRibbonButton(marks, QStringLiteral("线性尺寸"), QStringLiteral("dimLen"));
	m_btnDimRadius = makeRibbonButton(marks, QStringLiteral("半径尺寸"), QStringLiteral("dimRad"));
	m_btnDimDiameter = makeRibbonButton(marks, QStringLiteral("直径尺寸"), QStringLiteral("dimDia"));
	m_btnDimAngle = makeRibbonButton(marks, QStringLiteral("角度尺寸"), QStringLiteral("dimAng"));
	m_btnDimCont = makeRibbonButton(marks, QStringLiteral("连续尺寸"), QStringLiteral("dimCont"));
	m_btnDimBase = makeRibbonButton(marks, QStringLiteral("基线尺寸"), QStringLiteral("dimBase"));
	m_btnNote = makeRibbonButton(marks, QStringLiteral("引线文字"), QStringLiteral("note"));
	m_btnHatch = makeRibbonButton(marks, QStringLiteral("填充"), QStringLiteral("hatch"));
	m_btnText = makeRibbonButton(marks, QStringLiteral("单行文字"), QStringLiteral("text"));
	m_btnMText = makeRibbonButton(marks, QStringLiteral("多行文字"), QStringLiteral("mtext"));
	m_btnRoughness = makeRibbonButton(marks, QStringLiteral("粗糙度"), QStringLiteral("rough"));
	m_btnGdt = makeRibbonButton(marks, QStringLiteral("形位公差"), QStringLiteral("gdt"));
	bindTool(m_btnDimLinear, DrawingCanvasTool::LinearDim, this, m_tools);
	bindTool(m_btnDimRadius, DrawingCanvasTool::DimRadius, this, m_tools);
	bindTool(m_btnDimDiameter, DrawingCanvasTool::DimDiameter, this, m_tools);
	bindTool(m_btnDimAngle, DrawingCanvasTool::DimAngle, this, m_tools);
	bindTool(m_btnDimCont, DrawingCanvasTool::DimContinuous, this, m_tools);
	bindTool(m_btnDimBase, DrawingCanvasTool::DimBaseline, this, m_tools);
	bindTool(m_btnNote, DrawingCanvasTool::NoteLeader, this, m_tools);
	bindTool(m_btnHatch, DrawingCanvasTool::HatchPick, this, m_tools);
	bindTool(m_btnText, DrawingCanvasTool::TextNote, this, m_tools);
	bindTool(m_btnMText, DrawingCanvasTool::MText, this, m_tools);
	bindTool(m_btnRoughness, DrawingCanvasTool::SymRoughness, this, m_tools);
	bindTool(m_btnGdt, DrawingCanvasTool::SymGdt, this, m_tools);
	markRows.top->addWidget(m_btnDimLinear);
	markRows.top->addWidget(m_btnDimRadius);
	markRows.top->addWidget(m_btnDimDiameter);
	markRows.top->addWidget(m_btnDimAngle);
	markRows.top->addWidget(m_btnDimCont);
	markRows.top->addWidget(m_btnDimBase);
	markRows.bottom->addWidget(m_btnNote);
	markRows.bottom->addWidget(m_btnHatch);
	markRows.bottom->addWidget(m_btnText);
	markRows.bottom->addWidget(m_btnMText);
	markRows.bottom->addWidget(m_btnRoughness);
	markRows.bottom->addWidget(m_btnGdt);
	root->addWidget(marks);

	RibbonRows modifyRows;
	QWidget* modify = makeGroup(host, QStringLiteral("修改"), modifyRows, &m_lblModify);
	m_btnMatch = makeRibbonButton(modify, QStringLiteral("匹配特性"), QStringLiteral("match"));
	m_btnMove = makeRibbonButton(modify, QStringLiteral("移动"), QStringLiteral("move"));
	m_btnCopy = makeRibbonButton(modify, QStringLiteral("复制"), QStringLiteral("copy"));
	m_btnTrim = makeRibbonButton(modify, QStringLiteral("修剪"), QStringLiteral("trim"));
	m_btnOffset = makeRibbonButton(modify, QStringLiteral("偏移"), QStringLiteral("offset"));
	m_btnScale = makeRibbonButton(modify, QStringLiteral("缩放"), QStringLiteral("scale"));
	m_btnRotate = makeRibbonButton(modify, QStringLiteral("旋转"), QStringLiteral("rotate"));
	m_btnMirror = makeRibbonButton(modify, QStringLiteral("镜像"), QStringLiteral("mirror"));
	m_btnErase = makeRibbonButton(modify, QStringLiteral("删除"), QStringLiteral("erase"));
	m_btnFillet = makeRibbonButton(modify, QStringLiteral("圆角"), QStringLiteral("fillet"));
	m_btnChamfer = makeRibbonButton(modify, QStringLiteral("倒角"), QStringLiteral("chamfer"));
	m_btnExtend = makeRibbonButton(modify, QStringLiteral("延伸"), QStringLiteral("extend"));
	m_btnArray = makeRibbonButton(modify, QStringLiteral("阵列"), QStringLiteral("array"));
	m_btnPolarArray = makeRibbonButton(modify, QStringLiteral("环阵"), QStringLiteral("polarArray"));
	m_btnBreak = makeRibbonButton(modify, QStringLiteral("打断"), QStringLiteral("break"));
	m_btnJoin = makeRibbonButton(modify, QStringLiteral("合并"), QStringLiteral("join"));
	m_btnStretch = makeRibbonButton(modify, QStringLiteral("拉伸"), QStringLiteral("stretch"));
	m_btnExplode = makeRibbonButton(modify, QStringLiteral("炸开"), QStringLiteral("explode"));
	m_btnProjGuide = makeRibbonButton(modify, QStringLiteral("投影线"), QStringLiteral("projGuide"));
	bindTool(m_btnMatch, DrawingCanvasTool::MatchProp, this, m_tools);
	bindTool(m_btnMove, DrawingCanvasTool::ModifyMove, this, m_tools);
	bindTool(m_btnCopy, DrawingCanvasTool::ModifyCopy, this, m_tools);
	bindTool(m_btnTrim, DrawingCanvasTool::ModifyTrim, this, m_tools);
	bindTool(m_btnOffset, DrawingCanvasTool::ModifyOffset, this, m_tools);
	bindTool(m_btnScale, DrawingCanvasTool::ModifyScale, this, m_tools);
	bindTool(m_btnRotate, DrawingCanvasTool::ModifyRotate, this, m_tools);
	bindTool(m_btnMirror, DrawingCanvasTool::ModifyMirror, this, m_tools);
	bindTool(m_btnErase, DrawingCanvasTool::ModifyErase, this, m_tools);
	bindTool(m_btnFillet, DrawingCanvasTool::ModifyFillet, this, m_tools);
	bindTool(m_btnChamfer, DrawingCanvasTool::ModifyChamfer, this, m_tools);
	bindTool(m_btnExtend, DrawingCanvasTool::ModifyExtend, this, m_tools);
	bindTool(m_btnArray, DrawingCanvasTool::ModifyArray, this, m_tools);
	bindTool(m_btnPolarArray, DrawingCanvasTool::ModifyPolarArray, this, m_tools);
	bindTool(m_btnBreak, DrawingCanvasTool::ModifyBreak, this, m_tools);
	bindTool(m_btnJoin, DrawingCanvasTool::ModifyJoin, this, m_tools);
	bindTool(m_btnStretch, DrawingCanvasTool::ModifyStretch, this, m_tools);
	bindTool(m_btnExplode, DrawingCanvasTool::ExplodeBlock, this, m_tools);
	bindTool(m_btnProjGuide, DrawingCanvasTool::ProjectionGuide, this, m_tools);
	modifyRows.top->addWidget(m_btnMatch);
	modifyRows.top->addWidget(m_btnMove);
	modifyRows.top->addWidget(m_btnCopy);
	modifyRows.top->addWidget(m_btnTrim);
	modifyRows.top->addWidget(m_btnOffset);
	modifyRows.top->addWidget(m_btnScale);
	modifyRows.top->addWidget(m_btnExtend);
	modifyRows.top->addWidget(m_btnBreak);
	modifyRows.bottom->addWidget(m_btnRotate);
	modifyRows.bottom->addWidget(m_btnMirror);
	modifyRows.bottom->addWidget(m_btnErase);
	modifyRows.bottom->addWidget(m_btnFillet);
	modifyRows.bottom->addWidget(m_btnChamfer);
	modifyRows.bottom->addWidget(m_btnArray);
	modifyRows.bottom->addWidget(m_btnPolarArray);
	modifyRows.bottom->addWidget(m_btnJoin);
	modifyRows.bottom->addWidget(m_btnStretch);
	modifyRows.bottom->addWidget(m_btnExplode);
	modifyRows.bottom->addWidget(m_btnProjGuide);
	root->addWidget(modify);

	RibbonRows snapRows;
	QWidget* snap = makeGroup(host, QStringLiteral("捕捉"), snapRows, &m_lblSnap);
	m_snapEnd = new QCheckBox(QStringLiteral("端点"), snap);
	m_snapMid = new QCheckBox(QStringLiteral("中点"), snap);
	m_snapInt = new QCheckBox(QStringLiteral("交点"), snap);
	m_snapCen = new QCheckBox(QStringLiteral("圆心"), snap);
	m_snapPerp = new QCheckBox(QStringLiteral("垂足"), snap);
	m_snapNear = new QCheckBox(QStringLiteral("最近点"), snap);
	m_snapPolar = new QCheckBox(QStringLiteral("极轴"), snap);
	m_orthoCheck = new QCheckBox(QStringLiteral("正交"), snap);
	for (QCheckBox* c :
		 {m_snapEnd, m_snapMid, m_snapInt, m_snapCen, m_snapPerp, m_snapNear, m_snapPolar, m_orthoCheck})
	{
		c->setObjectName(QStringLiteral("RibbonCheck"));
		c->setChecked(c == m_snapEnd || c == m_snapMid || c == m_snapInt || c == m_snapCen);
		connect(c, &QCheckBox::toggled, this, [this](bool) { emitSnapFlags(); });
	}
	snapRows.top->addWidget(m_snapEnd);
	snapRows.top->addWidget(m_snapMid);
	snapRows.top->addWidget(m_snapInt);
	snapRows.top->addWidget(m_snapCen);
	snapRows.bottom->addWidget(m_snapPerp);
	snapRows.bottom->addWidget(m_snapNear);
	snapRows.bottom->addWidget(m_snapPolar);
	snapRows.bottom->addWidget(m_orthoCheck);
	root->addWidget(snap);

	RibbonRows outRows;
	QWidget* out = makeGroup(host, QStringLiteral("出图"), outRows, &m_lblOut);
	m_generateBtn = makeActionButton(out, QStringLiteral("生成图纸"), true);
	m_angleCombo = new QComboBox(out);
	m_angleCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_angleCombo->setFixedHeight(24);
	m_angleCombo->setMinimumWidth(72);
	m_angleCombo->setMaximumWidth(88);
	m_angleCombo->addItem(QStringLiteral("第一角法"), 0);
	m_angleCombo->addItem(QStringLiteral("第三角法"), 1);
	m_isoCheck = new QCheckBox(QStringLiteral("轴测"), out);
	m_isoCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_isoCheck->setToolTip(QStringLiteral("等轴测（当前仅一种）"));
	m_coarseViewCheck = new QCheckBox(QStringLiteral("快速预览"), out);
	m_coarseViewCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_coarseViewCheck->setChecked(false);
	m_coarseViewCheck->setToolTip(QStringLiteral("网格 HLR 预览（圆呈多边形）；正式出图请关闭"));
	m_projDragLock = new QCheckBox(QStringLiteral("投影约束"), out);
	m_projDragLock->setObjectName(QStringLiteral("RibbonCheck"));
	m_projDragLock->setChecked(true);
	m_projDragLock->setToolTip(QStringLiteral("拖俯/右视图时锁定投影轴"));
	m_projPinned = new QCheckBox(QStringLiteral("钉住投影"), out);
	m_projPinned->setObjectName(QStringLiteral("RibbonCheck"));
	m_projPinned->setToolTip(QStringLiteral("移动正视时俯视/右视跟随"));
	m_projGuideVisible = new QCheckBox(QStringLiteral("显示投影线"), out);
	m_projGuideVisible->setObjectName(QStringLiteral("RibbonCheck"));
	m_projGuideVisible->setChecked(true);
	m_sectionCheck = new QCheckBox(QStringLiteral("剖视"), out);
	m_sectionCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_halfSectionCheck = new QCheckBox(QStringLiteral("半剖"), out);
	m_halfSectionCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_halfSectionCheck->setToolTip(QStringLiteral("正视左半保留（表达层裁剪）"));
	m_sectionPlaneCombo = new QComboBox(out);
	m_sectionPlaneCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_sectionPlaneCombo->setFixedHeight(24);
	m_sectionPlaneCombo->setMinimumWidth(80);
	m_sectionPlaneCombo->setMaximumWidth(100);
	m_sectionPlaneCombo->addItem(QStringLiteral("正视中面"), 0);
	m_sectionPlaneCombo->addItem(QStringLiteral("俯视中面"), 1);
	m_sectionPlaneCombo->addItem(QStringLiteral("右视中面"), 2);
	m_sectionPlaneCombo->addItem(QStringLiteral("自定义平面"), 3);
	m_secOxLabel = new QLabel(QStringLiteral("原点"), out);
	m_secOxLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_secOx = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 48);
	m_secOy = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 48);
	m_secOz = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 48);
	m_secNxLabel = new QLabel(QStringLiteral("法向"), out);
	m_secNxLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_secNx = makeCompactSpin(out, 0.0, -1e3, 1e3, 3, 44);
	m_secNy = makeCompactSpin(out, 1.0, -1e3, 1e3, 3, 44);
	m_secNz = makeCompactSpin(out, 0.0, -1e3, 1e3, 3, 44);
	outRows.top->addWidget(m_generateBtn);
	outRows.top->addWidget(m_angleCombo);
	outRows.top->addWidget(m_isoCheck);
	outRows.top->addWidget(m_coarseViewCheck);
	outRows.top->addWidget(m_sectionCheck);
	outRows.top->addWidget(m_halfSectionCheck);
	outRows.bottom->addWidget(m_projDragLock);
	outRows.bottom->addWidget(m_projPinned);
	outRows.bottom->addWidget(m_projGuideVisible);
	outRows.bottom->addWidget(m_sectionPlaneCombo);
	outRows.bottom->addWidget(m_secOxLabel);
	outRows.bottom->addWidget(m_secOx);
	outRows.bottom->addWidget(m_secOy);
	outRows.bottom->addWidget(m_secOz);
	outRows.bottom->addWidget(m_secNxLabel);
	outRows.bottom->addWidget(m_secNx);
	outRows.bottom->addWidget(m_secNy);
	outRows.bottom->addWidget(m_secNz);
	root->addWidget(out);

	RibbonRows sheetRows;
	QWidget* sheet = makeGroup(host, QStringLiteral("图幅"), sheetRows, &m_lblSheet);
	m_paperCombo = new QComboBox(sheet);
	m_paperCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_paperCombo->setFixedHeight(24);
	m_paperCombo->setMinimumWidth(76);
	m_paperCombo->setMaximumWidth(96);
	m_paperCombo->addItem(QStringLiteral("A4 横向"), QStringLiteral("A4L"));
	m_paperCombo->addItem(QStringLiteral("A4 纵向"), QStringLiteral("A4P"));
	m_paperCombo->addItem(QStringLiteral("A3 横向"), QStringLiteral("A3L"));
	m_paperCombo->addItem(QStringLiteral("A3 纵向"), QStringLiteral("A3P"));
	m_paperCombo->addItem(QStringLiteral("A2 横向"), QStringLiteral("A2L"));
	m_paperCombo->addItem(QStringLiteral("A2 纵向"), QStringLiteral("A2P"));
	m_paperCombo->addItem(QStringLiteral("A1 横向"), QStringLiteral("A1L"));
	m_paperCombo->addItem(QStringLiteral("A1 纵向"), QStringLiteral("A1P"));
	m_paperCombo->addItem(QStringLiteral("A0 横向"), QStringLiteral("A0L"));
	m_paperCombo->addItem(QStringLiteral("A0 纵向"), QStringLiteral("A0P"));
	m_paperCombo->addItem(QStringLiteral("自定义"), QStringLiteral("CUSTOM"));
	m_paperWLabel = new QLabel(QStringLiteral("宽"), sheet);
	m_paperWLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_paperWSpin = makeCompactSpin(sheet, 297.0, 10.0, 5000.0, 1, 72);
	m_paperWSpin->setSuffix(QStringLiteral("mm"));
	m_paperHLabel = new QLabel(QStringLiteral("高"), sheet);
	m_paperHLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_paperHSpin = makeCompactSpin(sheet, 210.0, 10.0, 5000.0, 1, 72);
	m_paperHSpin->setSuffix(QStringLiteral("mm"));
	m_scaleCombo = new QComboBox(sheet);
	m_scaleCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_scaleCombo->setFixedHeight(24);
	m_scaleCombo->setMinimumWidth(60);
	m_scaleCombo->setMaximumWidth(76);
	m_scaleCombo->addItem(QStringLiteral("1:1"), 1.0);
	m_scaleCombo->addItem(QStringLiteral("1:2"), 0.5);
	m_scaleCombo->addItem(QStringLiteral("1:5"), 0.2);
	m_scaleCombo->addItem(QStringLiteral("1:10"), 0.1);
	m_scaleCombo->addItem(QStringLiteral("2:1"), 2.0);
	m_scaleCombo->addItem(QStringLiteral("5:1"), 5.0);
	m_scaleCombo->addItem(QStringLiteral("自定义"), 0.0);
	m_scaleSpin = makeCompactSpin(sheet, 1.0, 0.01, 100.0, 3, 64);
	m_scaleSpin->setToolTip(QStringLiteral("图面 mm / 模型 mm（0.5 = 1:2）"));
	m_fitPaperBtn = makeActionButton(sheet, QStringLiteral("适应图幅"));
	m_titleEdit = new QLineEdit(sheet);
	m_titleEdit->setObjectName(QStringLiteral("RibbonEdit"));
	m_titleEdit->setPlaceholderText(QStringLiteral("图名"));
	m_titleEdit->setFixedHeight(24);
	m_titleEdit->setMaximumWidth(120);
	m_detailScaleSpin = makeCompactSpin(sheet, 2.0, 1.5, 10.0, 1, 56);
	m_detailScaleSpin->setSingleStep(0.5);
	m_detailScaleSpin->setSuffix(QStringLiteral("×"));
	m_detailScaleSpin->setToolTip(QStringLiteral("局部放大倍率"));
	m_ltScaleLabel = new QLabel(QStringLiteral("LT"), sheet);
	m_ltScaleLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_ltScaleSpin = makeCompactSpin(sheet, 1.0, 0.25, 100.0, 2, 56);
	m_ltScaleSpin->setToolTip(QStringLiteral("线型比例 LTSCALE"));
	sheetRows.top->addWidget(m_paperCombo);
	sheetRows.top->addWidget(m_paperWLabel);
	sheetRows.top->addWidget(m_paperWSpin);
	sheetRows.top->addWidget(m_paperHLabel);
	sheetRows.top->addWidget(m_paperHSpin);
	addCompactSep(sheetRows.top);
	sheetRows.top->addWidget(m_scaleCombo);
	sheetRows.top->addWidget(m_scaleSpin);
	sheetRows.bottom->addWidget(m_fitPaperBtn);
	sheetRows.bottom->addWidget(m_titleEdit);
	sheetRows.bottom->addWidget(m_detailScaleSpin);
	addCompactSep(sheetRows.bottom);
	sheetRows.bottom->addWidget(m_ltScaleLabel);
	sheetRows.bottom->addWidget(m_ltScaleSpin);
	root->addWidget(sheet);

	RibbonRows exportRows;
	QWidget* exportGroup = makeGroup(host, QStringLiteral("导出"), exportRows, &m_lblExport);
	m_svgBtn = makeActionButton(exportGroup, QStringLiteral("SVG"));
	m_dxfBtn = makeActionButton(exportGroup, QStringLiteral("DXF"));
	m_pdfBtn = makeActionButton(exportGroup, QStringLiteral("PDF"));
	m_importDxfBtn = makeActionButton(exportGroup, QStringLiteral("导入DXF"));
	m_printPreviewBtn = makeActionButton(exportGroup, QStringLiteral("打印预览"));
	m_blockBtn = makeActionButton(exportGroup, QStringLiteral("建块"));
	m_insertBlockBtn = makeActionButton(exportGroup, QStringLiteral("插入块"));
	m_dimStyleBtn = makeActionButton(exportGroup, QStringLiteral("标注样式"));
	m_titleAttrBtn = makeActionButton(exportGroup, QStringLiteral("图框属性"));
	m_ctbCheck = new QCheckBox(QStringLiteral("CTB线宽"), exportGroup);
	m_ctbCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_ctbTableBtn = makeActionButton(exportGroup, QStringLiteral("CTB表"));
	m_recalcDimBtn = makeActionButton(exportGroup, QStringLiteral("重算尺寸"));
	exportRows.top->addWidget(m_svgBtn);
	exportRows.top->addWidget(m_dxfBtn);
	exportRows.top->addWidget(m_pdfBtn);
	exportRows.top->addWidget(m_importDxfBtn);
	exportRows.bottom->addWidget(m_printPreviewBtn);
	exportRows.bottom->addWidget(m_blockBtn);
	exportRows.bottom->addWidget(m_insertBlockBtn);
	exportRows.bottom->addWidget(m_dimStyleBtn);
	exportRows.bottom->addWidget(m_titleAttrBtn);
	exportRows.bottom->addWidget(m_ctbCheck);
	exportRows.bottom->addWidget(m_ctbTableBtn);
	exportRows.bottom->addWidget(m_recalcDimBtn);
	root->addWidget(exportGroup);

	root->addStretch(1);
	m_btnPan->setChecked(true);
	updateCustomSectionUi();
	updateCustomPaperUi();
	updateCustomScaleUi();

	connect(m_generateBtn, &QPushButton::clicked, this, &DrawingRibbonBar::generateRequested);
	connect(m_svgBtn, &QPushButton::clicked, this, &DrawingRibbonBar::exportSvgRequested);
	connect(m_dxfBtn, &QPushButton::clicked, this, &DrawingRibbonBar::exportDxfRequested);
	connect(m_pdfBtn, &QPushButton::clicked, this, &DrawingRibbonBar::exportPdfRequested);
	connect(m_importDxfBtn, &QPushButton::clicked, this, &DrawingRibbonBar::importDxfRequested);
	connect(m_printPreviewBtn, &QPushButton::clicked, this, &DrawingRibbonBar::printPreviewRequested);
	connect(m_blockBtn, &QPushButton::clicked, this, &DrawingRibbonBar::createBlockRequested);
	connect(m_insertBlockBtn, &QPushButton::clicked, this, &DrawingRibbonBar::insertBlockRequested);
	connect(m_dimStyleBtn, &QPushButton::clicked, this, &DrawingRibbonBar::dimStyleDialogRequested);
	connect(m_titleAttrBtn, &QPushButton::clicked, this, &DrawingRibbonBar::titleBlockAttrsRequested);
	connect(m_ctbCheck, &QCheckBox::toggled, this, &DrawingRibbonBar::ctbEnabledChanged);
	connect(m_ctbTableBtn, &QPushButton::clicked, this, &DrawingRibbonBar::ctbTableEditRequested);
	connect(m_recalcDimBtn, &QPushButton::clicked, this, &DrawingRibbonBar::recalculateDimsRequested);
	connect(m_projDragLock, &QCheckBox::toggled, this, &DrawingRibbonBar::projectionDragLockChanged);
	connect(m_projPinned, &QCheckBox::toggled, this, &DrawingRibbonBar::projectionPinnedChanged);
	connect(m_projGuideVisible, &QCheckBox::toggled, this, &DrawingRibbonBar::projectionGuidesVisibleChanged);
	connect(m_halfSectionCheck, &QCheckBox::toggled, this, &DrawingRibbonBar::halfSectionChanged);
	connect(m_btnFitWindow, &QToolButton::clicked, this, &DrawingRibbonBar::fitWindowRequested);
	connect(m_fitPaperBtn, &QPushButton::clicked, this, &DrawingRibbonBar::fitPaperRequested);
	connect(m_btnAlignLeft, &QToolButton::clicked, this,
			[this]() { emit viewAlignRequested(ViewAlignMode::Left); });
	connect(m_btnAlignHCenter, &QToolButton::clicked, this,
			[this]() { emit viewAlignRequested(ViewAlignMode::HCenter); });
	connect(m_btnAlignRight, &QToolButton::clicked, this,
			[this]() { emit viewAlignRequested(ViewAlignMode::Right); });
	connect(m_btnAlignTop, &QToolButton::clicked, this, [this]() { emit viewAlignRequested(ViewAlignMode::Top); });
	connect(m_btnAlignVCenter, &QToolButton::clicked, this,
			[this]() { emit viewAlignRequested(ViewAlignMode::VCenter); });
	connect(m_btnAlignBottom, &QToolButton::clicked, this,
			[this]() { emit viewAlignRequested(ViewAlignMode::Bottom); });
	connect(m_btnAlignProj, &QToolButton::clicked, this, &DrawingRibbonBar::projectionAlignRequested);
	connect(m_gridCheck, &QCheckBox::toggled, this, &DrawingRibbonBar::gridVisibleChanged);
	connect(m_detailScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&DrawingRibbonBar::detailScaleChanged);
	connect(m_sectionPlaneCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int) { updateCustomSectionUi(); });
	connect(m_sectionCheck, &QCheckBox::toggled, this, [this](bool) { updateCustomSectionUi(); });
	connect(m_paperCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		updateCustomPaperUi();
		emit sheetSettingsChanged(false);
	});
	connect(m_paperWSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (m_paperCombo && m_paperCombo->currentData().toString() == QLatin1String("CUSTOM"))
			emit sheetSettingsChanged(false);
	});
	connect(m_paperHSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (m_paperCombo && m_paperCombo->currentData().toString() == QLatin1String("CUSTOM"))
			emit sheetSettingsChanged(false);
	});
	connect(m_scaleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		updateCustomScaleUi();
		emit sheetSettingsChanged(true);
	});
	connect(m_scaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (m_scaleCombo && qFuzzyIsNull(m_scaleCombo->currentData().toDouble()))
			emit sheetSettingsChanged(true);
	});
	connect(m_titleEdit, &QLineEdit::editingFinished, this, [this]() { emit sheetSettingsChanged(false); });
	connect(m_ltScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double v) { emit ltScaleChanged(v); });

	applyTheme(false);
	applyLanguage(true);
}

void DrawingRibbonBar::updateCustomSectionUi()
{
	const bool custom = customSection() && includeSection();
	for (QWidget* w : {static_cast<QWidget*>(m_secOxLabel), static_cast<QWidget*>(m_secOx),
					   static_cast<QWidget*>(m_secOy), static_cast<QWidget*>(m_secOz),
					   static_cast<QWidget*>(m_secNxLabel), static_cast<QWidget*>(m_secNx),
					   static_cast<QWidget*>(m_secNy), static_cast<QWidget*>(m_secNz)})
	{
		if (w)
			w->setVisible(custom);
	}
}

void DrawingRibbonBar::updateCustomPaperUi()
{
	const bool custom = m_paperCombo && m_paperCombo->currentData().toString() == QLatin1String("CUSTOM");
	for (QWidget* w : {static_cast<QWidget*>(m_paperWLabel), static_cast<QWidget*>(m_paperWSpin),
					   static_cast<QWidget*>(m_paperHLabel), static_cast<QWidget*>(m_paperHSpin)})
	{
		if (w)
			w->setVisible(custom);
	}
}

void DrawingRibbonBar::updateCustomScaleUi()
{
	const bool custom = m_scaleCombo && qFuzzyIsNull(m_scaleCombo->currentData().toDouble());
	if (m_scaleSpin)
		m_scaleSpin->setVisible(custom);
}

void DrawingRibbonBar::applySheetSettings(DrawingSheetCanvasWidget* canvas, bool rescaleContent)
{
	if (!canvas)
		return;
	auto paper = canvas->paper();
	paper.visible = true;
	paper.date = QDate::currentDate().toString(Qt::ISODate);
	const QString key = m_paperCombo ? m_paperCombo->currentData().toString() : QStringLiteral("A4L");
	auto setIso = [&](DrawingPaperSize size, bool landscape) {
		paper.size = size;
		paper.landscape = landscape;
	};
	if (key == QLatin1String("A4L"))
		setIso(DrawingPaperSize::A4, true);
	else if (key == QLatin1String("A4P"))
		setIso(DrawingPaperSize::A4, false);
	else if (key == QLatin1String("A3L"))
		setIso(DrawingPaperSize::A3, true);
	else if (key == QLatin1String("A3P"))
		setIso(DrawingPaperSize::A3, false);
	else if (key == QLatin1String("A2L"))
		setIso(DrawingPaperSize::A2, true);
	else if (key == QLatin1String("A2P"))
		setIso(DrawingPaperSize::A2, false);
	else if (key == QLatin1String("A1L"))
		setIso(DrawingPaperSize::A1, true);
	else if (key == QLatin1String("A1P"))
		setIso(DrawingPaperSize::A1, false);
	else if (key == QLatin1String("A0L"))
		setIso(DrawingPaperSize::A0, true);
	else if (key == QLatin1String("A0P"))
		setIso(DrawingPaperSize::A0, false);
	else if (key == QLatin1String("CUSTOM"))
	{
		paper.size = DrawingPaperSize::Custom;
		paper.landscape = false;
		paper.customWidthMm = m_paperWSpin ? m_paperWSpin->value() : 297.0;
		paper.customHeightMm = m_paperHSpin ? m_paperHSpin->value() : 210.0;
	}
	else
		setIso(DrawingPaperSize::A4, true);

	if (m_titleEdit)
		paper.title = m_titleEdit->text().trimmed();

	double s = m_scaleCombo ? m_scaleCombo->currentData().toDouble() : 1.0;
	if (qFuzzyIsNull(s))
		s = m_scaleSpin ? m_scaleSpin->value() : 1.0;
	if (!(s > 1e-6))
		s = 1.0;
	// sheetScale 留给 setSheetScale 按旧值比值缩放
	canvas->setPaper(paper);
	canvas->setSheetScale(s, rescaleContent);
	if (!rescaleContent)
		canvas->placeViewsInPaper();
	canvas->setDetailScale(detailScale());
	canvas->setGridVisible(m_gridCheck && m_gridCheck->isChecked());
}

void DrawingRibbonBar::syncFromCanvas(const DrawingSheetCanvasWidget* canvas)
{
	if (!canvas)
		return;
	const auto& paper = canvas->paper();
	QString key = QStringLiteral("A4L");
	if (paper.size == DrawingPaperSize::Custom)
		key = QStringLiteral("CUSTOM");
	else
	{
		const char* sizeTag = "A4";
		switch (paper.size)
		{
		case DrawingPaperSize::A3:
			sizeTag = "A3";
			break;
		case DrawingPaperSize::A2:
			sizeTag = "A2";
			break;
		case DrawingPaperSize::A1:
			sizeTag = "A1";
			break;
		case DrawingPaperSize::A0:
			sizeTag = "A0";
			break;
		default:
			sizeTag = "A4";
			break;
		}
		key = QString::fromLatin1(sizeTag) + (paper.landscape ? QStringLiteral("L") : QStringLiteral("P"));
	}
	if (m_paperCombo)
	{
		const QSignalBlocker b(m_paperCombo);
		const int idx = m_paperCombo->findData(key);
		if (idx >= 0)
			m_paperCombo->setCurrentIndex(idx);
	}
	if (m_paperWSpin)
	{
		const QSignalBlocker b(m_paperWSpin);
		m_paperWSpin->setValue(paper.customWidthMm);
	}
	if (m_paperHSpin)
	{
		const QSignalBlocker b(m_paperHSpin);
		m_paperHSpin->setValue(paper.customHeightMm);
	}
	if (m_titleEdit)
		m_titleEdit->setText(paper.title);

	const double s = paper.sheetScale > 1e-12 ? paper.sheetScale : 1.0;
	int scaleIdx = -1;
	if (m_scaleCombo)
	{
		for (int i = 0; i < m_scaleCombo->count(); ++i)
		{
			const double v = m_scaleCombo->itemData(i).toDouble();
			if (!qFuzzyIsNull(v) && qAbs(v - s) < 1e-6)
			{
				scaleIdx = i;
				break;
			}
		}
		const QSignalBlocker b(m_scaleCombo);
		if (scaleIdx >= 0)
			m_scaleCombo->setCurrentIndex(scaleIdx);
		else
			m_scaleCombo->setCurrentIndex(m_scaleCombo->findData(0.0));
	}
	if (m_scaleSpin)
	{
		const QSignalBlocker b(m_scaleSpin);
		m_scaleSpin->setValue(s);
	}
	if (m_detailScaleSpin)
	{
		const QSignalBlocker b(m_detailScaleSpin);
		m_detailScaleSpin->setValue(canvas->detailScale());
	}
	if (m_ltScaleSpin)
	{
		const QSignalBlocker b(m_ltScaleSpin);
		m_ltScaleSpin->setValue(canvas->ltScale());
	}
	const SheetSnapFlags sf = canvas->snapFlags();
	auto setSnap = [](QCheckBox* c, bool on) {
		if (!c)
			return;
		const QSignalBlocker b(c);
		c->setChecked(on);
	};
	setSnap(m_snapEnd, sf.endpoint);
	setSnap(m_snapMid, sf.midpoint);
	setSnap(m_snapInt, sf.intersection);
	setSnap(m_snapCen, sf.center);
	setSnap(m_snapPerp, sf.perpendicular);
	setSnap(m_snapNear, sf.nearest);
	setSnap(m_snapPolar, sf.polar);
	setSnap(m_orthoCheck, sf.ortho);
	if (m_ctbCheck)
	{
		const QSignalBlocker b(m_ctbCheck);
		m_ctbCheck->setChecked(canvas->ctbEnabled());
	}
	if (m_projDragLock)
	{
		const QSignalBlocker b(m_projDragLock);
		m_projDragLock->setChecked(canvas->projectionDragLock());
	}
	if (m_projPinned)
	{
		const QSignalBlocker b(m_projPinned);
		m_projPinned->setChecked(canvas->projectionPinned());
	}
	if (m_halfSectionCheck)
	{
		const QSignalBlocker b(m_halfSectionCheck);
		m_halfSectionCheck->setChecked(canvas->halfSection());
	}
	updateCustomPaperUi();
	updateCustomScaleUi();
}

void DrawingRibbonBar::emitSnapFlags()
{
	SheetSnapFlags f;
	f.endpoint = m_snapEnd && m_snapEnd->isChecked();
	f.midpoint = m_snapMid && m_snapMid->isChecked();
	f.intersection = m_snapInt && m_snapInt->isChecked();
	f.center = m_snapCen && m_snapCen->isChecked();
	f.perpendicular = m_snapPerp && m_snapPerp->isChecked();
	f.nearest = m_snapNear && m_snapNear->isChecked();
	f.polar = m_snapPolar && m_snapPolar->isChecked();
	f.ortho = m_orthoCheck && m_orthoCheck->isChecked();
	emit snapFlagsChanged(f);
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

bool DrawingRibbonBar::includeIso() const
{
	return m_isoCheck && m_isoCheck->isChecked();
}

bool DrawingRibbonBar::coarseView() const
{
	return m_coarseViewCheck && m_coarseViewCheck->isChecked();
}

bool DrawingRibbonBar::includeSection() const
{
	return m_sectionCheck && m_sectionCheck->isChecked();
}

int DrawingRibbonBar::sectionPlane() const
{
	const int v = m_sectionPlaneCombo ? m_sectionPlaneCombo->currentData().toInt() : 0;
	return (v >= 0 && v <= 2) ? v : 0;
}

bool DrawingRibbonBar::customSection() const
{
	return m_sectionPlaneCombo && m_sectionPlaneCombo->currentData().toInt() == 3;
}

void DrawingRibbonBar::sectionOriginMm(double out[3]) const
{
	out[0] = m_secOx ? m_secOx->value() : 0.0;
	out[1] = m_secOy ? m_secOy->value() : 0.0;
	out[2] = m_secOz ? m_secOz->value() : 0.0;
}

void DrawingRibbonBar::sectionNormal(double out[3]) const
{
	out[0] = m_secNx ? m_secNx->value() : 0.0;
	out[1] = m_secNy ? m_secNy->value() : 1.0;
	out[2] = m_secNz ? m_secNz->value() : 0.0;
}

bool DrawingRibbonBar::thirdAngle() const
{
	return m_angleCombo && m_angleCombo->currentData().toInt() == 1;
}

double DrawingRibbonBar::detailScale() const
{
	return m_detailScaleSpin ? m_detailScaleSpin->value() : 2.0;
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
							  "QLabel#RibbonFieldLabel { color: #a1a1aa; font-size: 11px; }"
							  "QToolButton#RibbonBtn {"
							  "  background-color: #3f3f46; color: #f4f4f5;"
							  "  border: 1px solid #52525b; border-radius: 5px; padding: 1px;"
							  "}"
							  "QToolButton#RibbonBtn:hover { background-color: #52525b; border-color: #14b8a6; }"
							  "QToolButton#RibbonBtn:checked {"
							  "  background-color: #115e59; border: 2px solid #2dd4bf;"
							  "}"
							  "QPushButton#RibbonActionBtn, QPushButton#RibbonAccentBtn {"
							  "  background-color: #3f3f46; color: #f4f4f5;"
							  "  border: 1px solid #52525b; border-radius: 5px; padding: 2px 8px;"
							  "  font-size: 11px;"
							  "}"
							  "QPushButton#RibbonAccentBtn {"
							  "  background-color: #0f766e; border-color: #14b8a6; font-weight: 600;"
							  "}"
							  "QPushButton#RibbonActionBtn:hover, QPushButton#RibbonAccentBtn:hover {"
							  "  border-color: #2dd4bf;"
							  "}"
							  "QComboBox#RibbonCombo, QLineEdit#RibbonEdit, QDoubleSpinBox#RibbonSpin {"
							  "  background: #3f3f46; color: #f4f4f5; border: 1px solid #52525b;"
							  "  border-radius: 4px; padding: 1px 4px; font-size: 11px;"
							  "}"
							  "QCheckBox#RibbonCheck { color: #f4f4f5; font-size: 11px; spacing: 4px; }")
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
							  "QLabel#RibbonFieldLabel { color: #71717a; font-size: 11px; }"
							  "QToolButton#RibbonBtn {"
							  "  background-color: #f4f4f5; color: #18181b;"
							  "  border: 1px solid #d4d4d8; border-radius: 5px; padding: 1px;"
							  "}"
							  "QToolButton#RibbonBtn:hover { background-color: #ecfdf5; border-color: #14b8a6; }"
							  "QToolButton#RibbonBtn:checked {"
							  "  background-color: #ccfbf1; border: 2px solid #0f766e;"
							  "}"
							  "QPushButton#RibbonActionBtn, QPushButton#RibbonAccentBtn {"
							  "  background-color: #f4f4f5; color: #18181b;"
							  "  border: 1px solid #d4d4d8; border-radius: 5px; padding: 2px 8px;"
							  "  font-size: 11px;"
							  "}"
							  "QPushButton#RibbonAccentBtn {"
							  "  background-color: #0f766e; color: #ffffff; border-color: #0f766e; font-weight: 600;"
							  "}"
							  "QPushButton#RibbonActionBtn:hover { background-color: #ecfdf5; border-color: #14b8a6; }"
							  "QPushButton#RibbonAccentBtn:hover { background-color: #0d9488; }"
							  "QComboBox#RibbonCombo, QLineEdit#RibbonEdit, QDoubleSpinBox#RibbonSpin {"
							  "  background: #ffffff; color: #18181b; border: 1px solid #d4d4d8;"
							  "  border-radius: 4px; padding: 1px 4px; font-size: 11px;"
							  "}"
							  "QCheckBox#RibbonCheck { color: #18181b; font-size: 11px; spacing: 4px; }");
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
	if (m_lblModify)
		m_lblModify->setText(tr(QStringLiteral("Modify"), QStringLiteral("修改")));
	if (m_lblSnap)
		m_lblSnap->setText(tr(QStringLiteral("Osnap"), QStringLiteral("捕捉")));
	if (m_lblOut)
		m_lblOut->setText(tr(QStringLiteral("Project"), QStringLiteral("出图")));
	if (m_lblSheet)
		m_lblSheet->setText(tr(QStringLiteral("Sheet"), QStringLiteral("图幅")));
	if (m_lblExport)
		m_lblExport->setText(tr(QStringLiteral("Export"), QStringLiteral("导出")));
	setBtnText(m_btnPan, tr(QStringLiteral("Select/Pan"), QStringLiteral("选择/拖视图")));
	setBtnText(m_btnDetail, tr(QStringLiteral("Detail"), QStringLiteral("局部放大")));
	setBtnText(m_btnFitWindow, tr(QStringLiteral("Fit Window"), QStringLiteral("适应窗口")));
	setBtnText(m_btnAlignLeft, tr(QStringLiteral("Align Left"), QStringLiteral("左对齐")));
	setBtnText(m_btnAlignHCenter, tr(QStringLiteral("Align H-Center"), QStringLiteral("水平居中")));
	setBtnText(m_btnAlignRight, tr(QStringLiteral("Align Right"), QStringLiteral("右对齐")));
	setBtnText(m_btnAlignTop, tr(QStringLiteral("Align Top"), QStringLiteral("顶对齐")));
	setBtnText(m_btnAlignVCenter, tr(QStringLiteral("Align V-Center"), QStringLiteral("垂直居中")));
	setBtnText(m_btnAlignBottom, tr(QStringLiteral("Align Bottom"), QStringLiteral("底对齐")));
	setBtnText(m_btnAlignProj, tr(QStringLiteral("Projection Align"), QStringLiteral("投影中心对齐")));
	setBtnText(m_btnLine, tr(QStringLiteral("Line"), QStringLiteral("直线")));
	setBtnText(m_btnArc, tr(QStringLiteral("Arc"), QStringLiteral("圆弧")));
	setBtnText(m_btnCircle, tr(QStringLiteral("Circle"), QStringLiteral("圆")));
	setBtnText(m_btnRect, tr(QStringLiteral("Rectangle"), QStringLiteral("矩形")));
	setBtnText(m_btnSpline, tr(QStringLiteral("Spline"), QStringLiteral("样条")));
	setBtnText(m_btnSelect, tr(QStringLiteral("Select Entity"), QStringLiteral("选择图元")));
	setBtnText(m_btnDimLinear, tr(QStringLiteral("Linear Dim"), QStringLiteral("线性尺寸")));
	setBtnText(m_btnDimRadius, tr(QStringLiteral("Radius"), QStringLiteral("半径尺寸")));
	setBtnText(m_btnDimDiameter, tr(QStringLiteral("Diameter"), QStringLiteral("直径尺寸")));
	setBtnText(m_btnDimAngle, tr(QStringLiteral("Angle"), QStringLiteral("角度尺寸")));
	setBtnText(m_btnDimCont, tr(QStringLiteral("Continue"), QStringLiteral("连续尺寸")));
	setBtnText(m_btnDimBase, tr(QStringLiteral("Baseline"), QStringLiteral("基线尺寸")));
	setBtnText(m_btnNote, tr(QStringLiteral("Note"), QStringLiteral("引线文字")));
	setBtnText(m_btnHatch, tr(QStringLiteral("Hatch"), QStringLiteral("填充")));
	setBtnText(m_btnText, tr(QStringLiteral("Text"), QStringLiteral("单行文字")));
	setBtnText(m_btnMText, tr(QStringLiteral("MText"), QStringLiteral("多行文字")));
	setBtnText(m_btnMatch, tr(QStringLiteral("Match Prop"), QStringLiteral("匹配特性")));
	setBtnText(m_btnMove, tr(QStringLiteral("Move"), QStringLiteral("移动")));
	setBtnText(m_btnCopy, tr(QStringLiteral("Copy"), QStringLiteral("复制")));
	setBtnText(m_btnTrim, tr(QStringLiteral("Trim"), QStringLiteral("修剪")));
	setBtnText(m_btnOffset, tr(QStringLiteral("Offset"), QStringLiteral("偏移")));
	setBtnText(m_btnScale, tr(QStringLiteral("Scale"), QStringLiteral("缩放")));
	setBtnText(m_btnRotate, tr(QStringLiteral("Rotate"), QStringLiteral("旋转")));
	setBtnText(m_btnMirror, tr(QStringLiteral("Mirror"), QStringLiteral("镜像")));
	setBtnText(m_btnErase, tr(QStringLiteral("Erase"), QStringLiteral("删除")));
	setBtnText(m_btnFillet, tr(QStringLiteral("Fillet"), QStringLiteral("圆角")));
	setBtnText(m_btnChamfer, tr(QStringLiteral("Chamfer"), QStringLiteral("倒角")));
	setBtnText(m_btnExtend, tr(QStringLiteral("Extend"), QStringLiteral("延伸")));
	setBtnText(m_btnArray, tr(QStringLiteral("Array"), QStringLiteral("阵列")));
	setBtnText(m_btnPolarArray, tr(QStringLiteral("Polar"), QStringLiteral("环阵")));
	setBtnText(m_btnBreak, tr(QStringLiteral("Break"), QStringLiteral("打断")));
	setBtnText(m_btnJoin, tr(QStringLiteral("Join"), QStringLiteral("合并")));
	setBtnText(m_btnStretch, tr(QStringLiteral("Stretch"), QStringLiteral("拉伸")));
	setBtnText(m_btnRoughness, tr(QStringLiteral("Roughness"), QStringLiteral("粗糙度")));
	setBtnText(m_btnGdt, tr(QStringLiteral("GD&T"), QStringLiteral("形位公差")));
	setBtnText(m_btnExplode, tr(QStringLiteral("Explode"), QStringLiteral("炸开")));
	setBtnText(m_btnProjGuide, tr(QStringLiteral("Proj Line"), QStringLiteral("投影线")));
	if (m_snapEnd)
		m_snapEnd->setText(tr(QStringLiteral("End"), QStringLiteral("端点")));
	if (m_snapMid)
		m_snapMid->setText(tr(QStringLiteral("Mid"), QStringLiteral("中点")));
	if (m_snapInt)
		m_snapInt->setText(tr(QStringLiteral("Int"), QStringLiteral("交点")));
	if (m_snapCen)
		m_snapCen->setText(tr(QStringLiteral("Cen"), QStringLiteral("圆心")));
	if (m_snapPerp)
		m_snapPerp->setText(tr(QStringLiteral("Perp"), QStringLiteral("垂足")));
	if (m_snapNear)
		m_snapNear->setText(tr(QStringLiteral("Near"), QStringLiteral("最近点")));
	if (m_snapPolar)
		m_snapPolar->setText(tr(QStringLiteral("Polar"), QStringLiteral("极轴")));
	if (m_orthoCheck)
		m_orthoCheck->setText(tr(QStringLiteral("Ortho"), QStringLiteral("正交")));
	if (m_gridCheck)
		m_gridCheck->setText(tr(QStringLiteral("Grid"), QStringLiteral("网格")));
	if (m_generateBtn)
		m_generateBtn->setText(tr(QStringLiteral("Generate"), QStringLiteral("生成图纸")));
	if (m_isoCheck)
	{
		m_isoCheck->setText(tr(QStringLiteral("Iso"), QStringLiteral("轴测")));
		m_isoCheck->setToolTip(tr(QStringLiteral("Isometric only"), QStringLiteral("等轴测（当前仅一种）")));
	}
	if (m_coarseViewCheck)
	{
		m_coarseViewCheck->setText(tr(QStringLiteral("Coarse"), QStringLiteral("快速预览")));
		m_coarseViewCheck->setToolTip(tr(QStringLiteral("Mesh HLR preview"),
										 QStringLiteral("网格 HLR 预览（圆呈多边形）；正式出图请关闭")));
	}
	if (m_sectionCheck)
		m_sectionCheck->setText(tr(QStringLiteral("Section"), QStringLiteral("剖视")));
	if (m_halfSectionCheck)
		m_halfSectionCheck->setText(tr(QStringLiteral("Half"), QStringLiteral("半剖")));
	if (m_projDragLock)
		m_projDragLock->setText(tr(QStringLiteral("Proj Lock"), QStringLiteral("投影约束")));
	if (m_projPinned)
		m_projPinned->setText(tr(QStringLiteral("Pin Proj"), QStringLiteral("钉住投影")));
	if (m_projGuideVisible)
		m_projGuideVisible->setText(tr(QStringLiteral("Guides"), QStringLiteral("显示投影线")));
	if (m_sectionCheck)
		m_sectionCheck->setText(tr(QStringLiteral("Section"), QStringLiteral("剖视")));
	if (m_secOxLabel)
		m_secOxLabel->setText(tr(QStringLiteral("Origin"), QStringLiteral("原点")));
	if (m_secNxLabel)
		m_secNxLabel->setText(tr(QStringLiteral("Normal"), QStringLiteral("法向")));
	if (m_paperWLabel)
		m_paperWLabel->setText(tr(QStringLiteral("W"), QStringLiteral("宽")));
	if (m_paperHLabel)
		m_paperHLabel->setText(tr(QStringLiteral("H"), QStringLiteral("高")));
	if (m_ltScaleLabel)
		m_ltScaleLabel->setText(QStringLiteral("LT"));
	if (m_fitPaperBtn)
		m_fitPaperBtn->setText(tr(QStringLiteral("Fit Sheet"), QStringLiteral("适应图幅")));
	if (m_titleEdit)
		m_titleEdit->setPlaceholderText(tr(QStringLiteral("Title"), QStringLiteral("图名")));
	if (m_svgBtn)
		m_svgBtn->setText(QStringLiteral("SVG"));
	if (m_dxfBtn)
		m_dxfBtn->setText(QStringLiteral("DXF"));
	if (m_pdfBtn)
		m_pdfBtn->setText(QStringLiteral("PDF"));
	if (m_importDxfBtn)
		m_importDxfBtn->setText(tr(QStringLiteral("Import DXF"), QStringLiteral("导入DXF")));
	if (m_printPreviewBtn)
		m_printPreviewBtn->setText(tr(QStringLiteral("Print Preview"), QStringLiteral("打印预览")));
	if (m_blockBtn)
		m_blockBtn->setText(tr(QStringLiteral("Block"), QStringLiteral("建块")));
	if (m_insertBlockBtn)
		m_insertBlockBtn->setText(tr(QStringLiteral("Insert"), QStringLiteral("插入块")));
	if (m_dimStyleBtn)
		m_dimStyleBtn->setText(tr(QStringLiteral("DimStyle"), QStringLiteral("标注样式")));
	if (m_titleAttrBtn)
		m_titleAttrBtn->setText(tr(QStringLiteral("Title Attr"), QStringLiteral("图框属性")));
	if (m_ctbCheck)
		m_ctbCheck->setText(tr(QStringLiteral("CTB"), QStringLiteral("CTB线宽")));
	if (m_ctbTableBtn)
		m_ctbTableBtn->setText(tr(QStringLiteral("CTB Table"), QStringLiteral("CTB表")));
	if (m_recalcDimBtn)
		m_recalcDimBtn->setText(tr(QStringLiteral("Recalc Dim"), QStringLiteral("重算尺寸")));
}
