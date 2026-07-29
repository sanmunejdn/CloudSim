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
	btn->setIconSize(QSize(20, 20));
	btn->setFixedSize(34, 34);
	btn->setText(QString());
	return btn;
}

QPushButton* makeActionButton(QWidget* parent, const QString& text, bool accent = false)
{
	auto* btn = new QPushButton(text, parent);
	btn->setObjectName(accent ? QStringLiteral("RibbonAccentBtn") : QStringLiteral("RibbonActionBtn"));
	btn->setCursor(Qt::PointingHandCursor);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setFixedHeight(28);
	btn->setMinimumWidth(accent ? 72 : 56);
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
	s->setFixedHeight(26);
	s->setMaximumWidth(maxWidth);
	s->setButtonSymbols(QAbstractSpinBox::NoButtons);
	return s;
}

QWidget* makeGroup(QWidget* parent, const QString& title, QHBoxLayout*& outButtons, QLabel** outTitle = nullptr)
{
	auto* group = new QWidget(parent);
	group->setObjectName(QStringLiteral("RibbonGroup"));
	auto* lay = new QVBoxLayout(group);
	lay->setContentsMargins(8, 4, 8, 3);
	lay->setSpacing(2);

	auto* buttonsHost = new QWidget(group);
	buttonsHost->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
	outButtons = new QHBoxLayout(buttonsHost);
	outButtons->setContentsMargins(0, 0, 0, 0);
	outButtons->setSpacing(4);
	outButtons->setAlignment(Qt::AlignVCenter);
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
	setFixedHeight(86);
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
	QWidget* view = makeGroup(host, QStringLiteral("视图"), viewBtns, &m_lblView);
	m_btnPan = makeRibbonButton(view, QStringLiteral("选择/拖视图"), QStringLiteral("pan"));
	m_btnDetail = makeRibbonButton(view, QStringLiteral("局部放大"), QStringLiteral("detail"));
	m_btnFitWindow = makeRibbonButton(view, QStringLiteral("适应窗口"), QStringLiteral("fit"), false);
	m_gridCheck = new QCheckBox(QStringLiteral("网格"), view);
	m_gridCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_gridCheck->setChecked(true);
	bindTool(m_btnPan, DrawingCanvasTool::PanSelect, this, m_tools);
	bindTool(m_btnDetail, DrawingCanvasTool::DetailRegion, this, m_tools);
	viewBtns->addWidget(m_btnPan);
	viewBtns->addWidget(m_btnDetail);
	addCompactSep(viewBtns);
	viewBtns->addWidget(m_btnFitWindow);
	viewBtns->addWidget(m_gridCheck);
	root->addWidget(view);

	QHBoxLayout* sketchBtns = nullptr;
	QWidget* sketch = makeGroup(host, QStringLiteral("绘图"), sketchBtns, &m_lblSketch);
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
	QWidget* marks = makeGroup(host, QStringLiteral("标注"), markBtns, &m_lblMarks);
	m_btnDimLinear = makeRibbonButton(marks, QStringLiteral("线性尺寸"), QStringLiteral("dimLen"));
	m_btnDimRadius = makeRibbonButton(marks, QStringLiteral("半径尺寸"), QStringLiteral("dimRad"));
	m_btnDimDiameter = makeRibbonButton(marks, QStringLiteral("直径尺寸"), QStringLiteral("dimDia"));
	m_btnDimAngle = makeRibbonButton(marks, QStringLiteral("角度尺寸"), QStringLiteral("dimAng"));
	m_btnNote = makeRibbonButton(marks, QStringLiteral("引线文字"), QStringLiteral("note"));
	bindTool(m_btnDimLinear, DrawingCanvasTool::LinearDim, this, m_tools);
	bindTool(m_btnDimRadius, DrawingCanvasTool::DimRadius, this, m_tools);
	bindTool(m_btnDimDiameter, DrawingCanvasTool::DimDiameter, this, m_tools);
	bindTool(m_btnDimAngle, DrawingCanvasTool::DimAngle, this, m_tools);
	bindTool(m_btnNote, DrawingCanvasTool::NoteLeader, this, m_tools);
	markBtns->addWidget(m_btnDimLinear);
	markBtns->addWidget(m_btnDimRadius);
	markBtns->addWidget(m_btnDimDiameter);
	markBtns->addWidget(m_btnDimAngle);
	markBtns->addWidget(m_btnNote);
	root->addWidget(marks);

	QHBoxLayout* outBtns = nullptr;
	QWidget* out = makeGroup(host, QStringLiteral("出图"), outBtns, &m_lblOut);
	m_generateBtn = makeActionButton(out, QStringLiteral("生成图纸"), true);
	m_angleCombo = new QComboBox(out);
	m_angleCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_angleCombo->setFixedHeight(26);
	m_angleCombo->setMinimumWidth(88);
	m_angleCombo->addItem(QStringLiteral("第一角法"), 0);
	m_angleCombo->addItem(QStringLiteral("第三角法"), 1);
	m_isoCheck = new QCheckBox(QStringLiteral("轴测"), out);
	m_isoCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_sectionCheck = new QCheckBox(QStringLiteral("剖视"), out);
	m_sectionCheck->setObjectName(QStringLiteral("RibbonCheck"));
	m_sectionPlaneCombo = new QComboBox(out);
	m_sectionPlaneCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_sectionPlaneCombo->setFixedHeight(26);
	m_sectionPlaneCombo->setMinimumWidth(96);
	m_sectionPlaneCombo->addItem(QStringLiteral("正视中面"), 0);
	m_sectionPlaneCombo->addItem(QStringLiteral("俯视中面"), 1);
	m_sectionPlaneCombo->addItem(QStringLiteral("右视中面"), 2);
	m_sectionPlaneCombo->addItem(QStringLiteral("自定义平面"), 3);
	m_secOxLabel = new QLabel(QStringLiteral("原点"), out);
	m_secOxLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_secOx = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 56);
	m_secOy = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 56);
	m_secOz = makeCompactSpin(out, 0.0, -1e6, 1e6, 2, 56);
	m_secNxLabel = new QLabel(QStringLiteral("法向"), out);
	m_secNxLabel->setObjectName(QStringLiteral("RibbonFieldLabel"));
	m_secNx = makeCompactSpin(out, 0.0, -1e3, 1e3, 3, 52);
	m_secNy = makeCompactSpin(out, 1.0, -1e3, 1e3, 3, 52);
	m_secNz = makeCompactSpin(out, 0.0, -1e3, 1e3, 3, 52);
	outBtns->addWidget(m_generateBtn);
	outBtns->addWidget(m_angleCombo);
	outBtns->addWidget(m_isoCheck);
	outBtns->addWidget(m_sectionCheck);
	outBtns->addWidget(m_sectionPlaneCombo);
	outBtns->addWidget(m_secOxLabel);
	outBtns->addWidget(m_secOx);
	outBtns->addWidget(m_secOy);
	outBtns->addWidget(m_secOz);
	outBtns->addWidget(m_secNxLabel);
	outBtns->addWidget(m_secNx);
	outBtns->addWidget(m_secNy);
	outBtns->addWidget(m_secNz);
	root->addWidget(out);

	QHBoxLayout* sheetBtns = nullptr;
	QWidget* sheet = makeGroup(host, QStringLiteral("图幅"), sheetBtns, &m_lblSheet);
	m_paperCombo = new QComboBox(sheet);
	m_paperCombo->setObjectName(QStringLiteral("RibbonCombo"));
	m_paperCombo->setFixedHeight(26);
	m_paperCombo->setMinimumWidth(92);
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
	m_scaleCombo->setFixedHeight(26);
	m_scaleCombo->setMinimumWidth(72);
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
	m_titleEdit->setFixedHeight(26);
	m_titleEdit->setMaximumWidth(120);
	m_detailScaleSpin = makeCompactSpin(sheet, 2.0, 1.5, 10.0, 1, 56);
	m_detailScaleSpin->setSingleStep(0.5);
	m_detailScaleSpin->setSuffix(QStringLiteral("×"));
	m_detailScaleSpin->setToolTip(QStringLiteral("局部放大倍率"));
	sheetBtns->addWidget(m_paperCombo);
	sheetBtns->addWidget(m_paperWLabel);
	sheetBtns->addWidget(m_paperWSpin);
	sheetBtns->addWidget(m_paperHLabel);
	sheetBtns->addWidget(m_paperHSpin);
	addCompactSep(sheetBtns);
	sheetBtns->addWidget(m_scaleCombo);
	sheetBtns->addWidget(m_scaleSpin);
	sheetBtns->addWidget(m_fitPaperBtn);
	addCompactSep(sheetBtns);
	sheetBtns->addWidget(m_titleEdit);
	sheetBtns->addWidget(m_detailScaleSpin);
	root->addWidget(sheet);

	QHBoxLayout* exportBtns = nullptr;
	QWidget* exportGroup = makeGroup(host, QStringLiteral("导出"), exportBtns, &m_lblExport);
	m_svgBtn = makeActionButton(exportGroup, QStringLiteral("SVG"));
	m_dxfBtn = makeActionButton(exportGroup, QStringLiteral("DXF"));
	m_pdfBtn = makeActionButton(exportGroup, QStringLiteral("PDF"));
	exportBtns->addWidget(m_svgBtn);
	exportBtns->addWidget(m_dxfBtn);
	exportBtns->addWidget(m_pdfBtn);
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
	connect(m_btnFitWindow, &QToolButton::clicked, this, &DrawingRibbonBar::fitWindowRequested);
	connect(m_fitPaperBtn, &QPushButton::clicked, this, &DrawingRibbonBar::fitPaperRequested);
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
	updateCustomPaperUi();
	updateCustomScaleUi();
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
	if (m_lblOut)
		m_lblOut->setText(tr(QStringLiteral("Project"), QStringLiteral("出图")));
	if (m_lblSheet)
		m_lblSheet->setText(tr(QStringLiteral("Sheet"), QStringLiteral("图幅")));
	if (m_lblExport)
		m_lblExport->setText(tr(QStringLiteral("Export"), QStringLiteral("导出")));
	setBtnText(m_btnPan, tr(QStringLiteral("Select/Pan"), QStringLiteral("选择/拖视图")));
	setBtnText(m_btnDetail, tr(QStringLiteral("Detail"), QStringLiteral("局部放大")));
	setBtnText(m_btnFitWindow, tr(QStringLiteral("Fit Window"), QStringLiteral("适应窗口")));
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
	setBtnText(m_btnNote, tr(QStringLiteral("Note"), QStringLiteral("引线文字")));
	if (m_gridCheck)
		m_gridCheck->setText(tr(QStringLiteral("Grid"), QStringLiteral("网格")));
	if (m_generateBtn)
		m_generateBtn->setText(tr(QStringLiteral("Generate"), QStringLiteral("生成图纸")));
	if (m_isoCheck)
		m_isoCheck->setText(tr(QStringLiteral("Iso"), QStringLiteral("轴测")));
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
}
