/// @file ApplicationStyle.cpp
/// @brief 应用样式

#include "ApplicationStyle.h"

#include "ApplicationSettings.h"
#include "UiIconDecorators.h"
#include "UiIcons.h"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPalette>
#include <QPolygonF>
#include <QProxyStyle>
#include <QStyleFactory>
#include <QStyleOption>
#include <QtGlobal>

namespace
{
/// 样式表一旦写过 ::branch 就会吃掉默认三角；用代理绘制保证有子节点可见开合符
class CloudSimTreeBranchStyle final : public QProxyStyle
{
public:
	CloudSimTreeBranchStyle() : QProxyStyle(QStyleFactory::create(QStringLiteral("Fusion"))) {}

	void drawPrimitive(PrimitiveElement element, const QStyleOption* option, QPainter* painter,
					   const QWidget* widget = nullptr) const override
	{
		if (element == PE_IndicatorBranch && option && painter)
		{
			if (!(option->state & State_Children))
			{
				return;
			}
			painter->save();
			painter->setRenderHint(QPainter::Antialiasing, true);
			QColor fill = option->palette.color(QPalette::Text);
			fill.setAlpha(200);
			painter->setPen(Qt::NoPen);
			painter->setBrush(fill);
			const QRectF r = option->rect;
			const qreal cx = r.center().x();
			const qreal cy = r.center().y();
			const qreal s = qMin(r.width(), r.height()) * 0.32;
			QPolygonF poly;
			if (option->state & State_Open)
			{
				poly << QPointF(cx - s, cy - s * 0.55) << QPointF(cx + s, cy - s * 0.55) << QPointF(cx, cy + s * 0.85);
			}
			else
			{
				poly << QPointF(cx - s * 0.55, cy - s) << QPointF(cx + s * 0.85, cy) << QPointF(cx - s * 0.55, cy + s);
			}
			painter->drawPolygon(poly);
			painter->restore();
			return;
		}
		QProxyStyle::drawPrimitive(element, option, painter, widget);
	}
};

QString hex(const QColor& c)
{
	return c.name(QColor::HexRgb);
}

ApplicationStyle::ThemeTokens makeTokens(ApplicationStyle::Theme theme)
{
	ApplicationStyle::ThemeTokens t;
	if (theme == ApplicationStyle::Theme::Dark)
	{
		t.window = QColor(QStringLiteral("#242426"));
		t.panel = QColor(QStringLiteral("#2c2c2e"));
		t.base = QColor(QStringLiteral("#1c1c1e"));
		t.text = QColor(QStringLiteral("#f2f2f7"));
		t.textMuted = QColor(QStringLiteral("#8e8e93"));
		t.accent = QColor(QStringLiteral("#3b8fdd"));
		t.accentHover = QColor(QStringLiteral("#54a3eb"));
		t.accentPressed = QColor(QStringLiteral("#2a74b8"));
		t.selectionBg = QColor(QStringLiteral("#1e3a56"));
		t.selectionText = QColor(QStringLiteral("#d6ebff"));
		t.hoverBg = QColor(QStringLiteral("#3a3a3c"));
		t.danger = QColor(QStringLiteral("#f07178"));
		t.border = QColor(QStringLiteral("#3a3a3c"));
		t.borderStrong = QColor(QStringLiteral("#4a4a4c"));
		t.button = QColor(QStringLiteral("#3a3a3c"));
		t.buttonHover = QColor(QStringLiteral("#48484a"));
	}
	else
	{
		// window 与 base 同色，避免 Tab 下 QWidget 默认 Window 灰底露出横缝
		t.window = QColor(QStringLiteral("#ffffff"));
		t.panel = QColor(QStringLiteral("#f3f4f6"));
		t.base = QColor(QStringLiteral("#ffffff"));
		t.text = QColor(QStringLiteral("#1c1c1e"));
		t.textMuted = QColor(QStringLiteral("#8e8e93"));
		t.accent = QColor(QStringLiteral("#2b79c2"));
		t.accentHover = QColor(QStringLiteral("#3a8fd4"));
		t.accentPressed = QColor(QStringLiteral("#2468a8"));
		t.selectionBg = QColor(QStringLiteral("#e8f0fe"));
		t.selectionText = QColor(QStringLiteral("#1a4f8a"));
		t.hoverBg = QColor(QStringLiteral("#eef1f5"));
		t.danger = QColor(QStringLiteral("#c0392b"));
		// 分区靠面色差，线尽量淡
		t.border = QColor(QStringLiteral("#eceef1"));
		t.borderStrong = QColor(QStringLiteral("#d5d9df"));
		t.button = QColor(QStringLiteral("#f0f1f3"));
		t.buttonHover = QColor(QStringLiteral("#e4e5e7"));
	}
	return t;
}

QPalette makePalette(const ApplicationStyle::ThemeTokens& t, ApplicationStyle::Theme theme)
{
	QPalette p;
	p.setColor(QPalette::Window, t.window);
	p.setColor(QPalette::WindowText, t.text);
	p.setColor(QPalette::Base, t.base);
	p.setColor(QPalette::AlternateBase, t.panel);
	p.setColor(QPalette::ToolTipBase, theme == ApplicationStyle::Theme::Dark ? t.button : t.base);
	p.setColor(QPalette::ToolTipText, t.text);
	p.setColor(QPalette::Text, t.text);
	p.setColor(QPalette::Button, t.button);
	p.setColor(QPalette::ButtonText, t.text);
	p.setColor(QPalette::BrightText, t.danger);
	p.setColor(QPalette::Link, t.accent);
	p.setColor(QPalette::Highlight, t.selectionBg);
	p.setColor(QPalette::HighlightedText, t.selectionText);
	p.setColor(QPalette::Disabled, QPalette::Text, t.textMuted);
	p.setColor(QPalette::Disabled, QPalette::ButtonText, t.textMuted);
	p.setColor(QPalette::Shadow, theme == ApplicationStyle::Theme::Dark ? QColor(0, 0, 0, 80) : QColor(0, 0, 0, 20));
	// Mid 给 palette(mid) 次要文案用，勿填边框色（会几乎看不见）
	p.setColor(QPalette::Mid, t.textMuted);
	p.setColor(QPalette::Light, theme == ApplicationStyle::Theme::Dark ? t.border : t.base);
	p.setColor(QPalette::Midlight, t.panel);
	p.setColor(QPalette::Dark, theme == ApplicationStyle::Theme::Dark ? QColor(QStringLiteral("#161618"))
																	 : t.borderStrong);
	return p;
}

void applyFontConfiguration(QApplication* app)
{
	QFont defaultFont = app->font();
	defaultFont.setFamilies({QStringLiteral("Segoe UI Variable Display"), QStringLiteral("Segoe UI"),
							 QStringLiteral("Microsoft YaHei UI"), QStringLiteral("Microsoft YaHei")});
	defaultFont.setPointSize(10);
	defaultFont.setWeight(QFont::Normal);
	app->setFont(defaultFont);
}

QString generateStyleSheet(const ApplicationStyle::ThemeTokens& t)
{
	const QString radius = QString::number(t.radiusSmPx);
	QString sheet = QStringLiteral(R"(
QMainWindow {
	background-color: {{window}};
}

/* Dock 区间分隔：默认隐线，悬停才显，保留可拖宽度 */
QMainWindow::separator {
	width: 4px;
	height: 4px;
	background-color: transparent;
}
QMainWindow::separator:hover {
	background-color: {{border}};
}

QMenuBar {
	background-color: {{panel}};
	border-bottom: 1px solid transparent;
	padding: 2px 4px;
}

QMenuBar::item {
	background: transparent;
	padding: 6px 12px;
	border-radius: {{radius}}px;
	color: {{text}};
}

QMenuBar::item:selected {
	background-color: {{hover}};
}

QMenuBar::item:pressed {
	background-color: {{buttonHover}};
}

QMenu {
	background-color: {{base}};
	border: 1px solid {{borderStrong}};
	border-radius: 0;
	padding: 4px;
}

QMenu::item {
	padding: 8px 24px 8px 12px;
	border-radius: {{radius}}px;
	color: {{text}};
}

QMenu::item:selected {
	background-color: {{selectionBg}};
	color: {{selectionText}};
}

QMenu::separator {
	height: 1px;
	background-color: {{border}};
	margin: 4px 8px;
}

QDockWidget {
	color: {{text}};
	background-color: {{base}};
}

QDockWidget::title {
	background-color: {{base}};
	padding: 8px 12px;
	font-weight: 500;
	border-bottom: 1px solid transparent;
}

QDockWidget::close-button, QDockWidget::float-button {
	background-color: transparent;
	border: 1px solid transparent;
	border-radius: 6px;
	padding: 5px;
	margin: 2px;
	width: 16px;
	height: 16px;
}

QDockWidget::close-button:hover {
	background-color: {{dangerHoverBg}};
	border-color: {{danger}};
}

QDockWidget::float-button:hover {
	background-color: {{hover}};
	border-color: {{borderStrong}};
}

/* Tab 条与页面同底贯通，只保留选中下划线作指示 */
QTabWidget {
	background-color: {{base}};
	border: none;
}

QTabWidget::pane {
	border: none;
	background-color: {{base}};
	border-radius: 0;
	margin: 0;
	padding: 0;
	top: 0px;
}

QTabWidget::tab-bar {
	alignment: left;
	left: 0px;
}

QTabBar {
	background-color: {{base}};
	border: none;
	qproperty-drawBase: 0;
}

QTabBar::tab {
	background-color: {{base}};
	color: {{muted}};
	padding: 8px 12px;
	margin: 0px;
	margin-right: 2px;
	border: none;
	border-bottom: 2px solid transparent;
	font-weight: 400;
	min-width: 50px;
}

QTabBar::tab:selected {
	background-color: {{base}};
	color: {{text}};
	border: none;
	border-bottom: 2px solid {{accent}};
	font-weight: 500;
}

QTabBar::tab:hover:!selected {
	background-color: {{hover}};
	color: {{text}};
}

QTabBar::tab:only-one {
	min-width: 100px;
}

/* 页根控件跟 pane 同色，挡住默认 Window 灰阶横带 */
QTabWidget > QStackedWidget > QWidget {
	background-color: {{base}};
}

/* 树/列表融进侧栏面，表格仍保留描边以免数据区发糊 */
QTreeWidget, QTreeView {
	background-color: transparent;
	border: none;
	border-radius: 0;
	padding: 2px;
	outline: none;
	selection-background-color: {{selectionBg}};
	selection-color: {{selectionText}};
	alternate-background-color: transparent;
	show-decoration-selected: 1;
}

/* 属性页：斑马线 + 紧凑表头，与侧栏 ThemeTokens 对齐 */
QtTreePropertyBrowser#PropertyBrowser,
QWidget#PropertyBrowser {
	background-color: {{base}};
	border: none;
}

QTreeWidget#PropertyBrowserTree {
	background-color: {{base}};
	padding: 0;
	alternate-background-color: {{panel}};
}

QTreeWidget#PropertyBrowserTree::item {
	padding: 4px 8px;
	min-height: 24px;
	border: none;
}

QTreeWidget#PropertyBrowserTree QHeaderView::section {
	background-color: {{panel}};
	color: {{muted}};
	padding: 7px 8px;
	border: none;
	border-right: 1px solid {{border}};
	border-bottom: 1px solid {{border}};
	font-weight: 500;
}

QTreeWidget#PropertyBrowserTree QHeaderView::section:last {
	border-right: none;
}

QTabWidget#PropertyDockTabs::pane {
	border: none;
	top: 0px;
}

/* 表格外框+网格必须可见；勿与 Tab 去缝的淡 border 混用 */
QTableWidget, QTableView {
	background-color: {{base}};
	border: 1px solid {{borderStrong}};
	border-radius: 0;
	padding: 0;
	outline: none;
	gridline-color: {{borderStrong}};
	selection-background-color: {{selectionBg}};
	selection-color: {{selectionText}};
	alternate-background-color: {{panel}};
	show-decoration-selected: 1;
}

QHeaderView::section {
	background-color: {{panel}};
	color: {{text}};
	padding: 6px 8px;
	border: none;
	border-right: 1px solid {{borderStrong}};
	border-bottom: 1px solid {{borderStrong}};
	font-weight: 500;
}

QHeaderView::section:last {
	border-right: none;
}

/* itemWidget 列表：勿给 item 加 padding/margin，否则 sizeHint 对不齐、高亮裁字 */
QListWidget {
	background-color: transparent;
	border: none;
	border-radius: 0;
	padding: 0;
	outline: none;
	selection-background-color: {{selectionBg}};
	selection-color: {{selectionText}};
	alternate-background-color: transparent;
	show-decoration-selected: 1;
}

QTreeWidget::item, QTreeView::item {
	padding: 3px 8px;
	border-radius: 0;
	margin: 0;
	min-height: 22px;
}

QListWidget::item {
	padding: 0;
	border-radius: 0;
	margin: 0;
}

QTreeWidget::item:selected, QTreeView::item:selected,
QListWidget::item:selected {
	background-color: {{selectionBg}};
	color: {{selectionText}};
}

QTreeWidget::item:hover:!selected, QTreeView::item:hover:!selected,
QListWidget::item:hover:!selected {
	background-color: {{hover}};
}

QTableWidget::item:selected, QTableView::item:selected {
	background-color: {{selectionBg}};
	color: {{selectionText}};
}

QPushButton {
	background-color: {{button}};
	color: {{text}};
	border: 1px solid {{border}};
	border-radius: {{radius}}px;
	/* Qt 的 min-height 不含 padding/border；配合常见 FixedHeight≈30，避免裁底边 */
	padding: 2px 10px;
	font-weight: 500;
	min-width: 36px;
	min-height: 24px;
}

QPushButton:hover {
	background-color: {{buttonHover}};
	border-color: {{borderStrong}};
}

QPushButton:pressed {
	background-color: {{hover}};
}

QPushButton:disabled {
	background-color: {{panel}};
	color: {{muted}};
	border-color: {{border}};
}

QPushButton:focus {
	outline: none;
	border-color: {{accent}};
}

QPushButton[btnRole="primary"] {
	background-color: {{accent}};
	color: #ffffff;
	border: 1px solid {{accentPressed}};
	font-weight: 600;
}

QPushButton[btnRole="primary"]:hover {
	background-color: {{accentHover}};
	border-color: {{accent}};
}

QPushButton[btnRole="primary"]:pressed {
	background-color: {{accentPressed}};
}

QPushButton[btnRole="primary"]:disabled {
	background-color: {{primaryDisabledBg}};
	color: {{primaryDisabledFg}};
	border-color: {{primaryDisabledBg}};
}

QPushButton[btnRole="secondary"] {
	background-color: {{base}};
	color: {{text}};
	border: 1px solid {{borderStrong}};
}

QPushButton[btnRole="danger"] {
	background-color: {{base}};
	color: {{danger}};
	border: 1px solid {{dangerBorder}};
}

QPushButton[btnRole="danger"]:hover {
	background-color: {{dangerHoverBg}};
	border-color: {{danger}};
}

QLineEdit {
	background-color: {{base}};
	color: {{text}};
	border: 1px solid {{borderStrong}};
	border-radius: {{radius}}px;
	padding: 4px 8px;
}

QTextEdit, QPlainTextEdit, QTextBrowser {
	background-color: {{base}};
	color: {{text}};
	border: 1px solid {{border}};
	border-radius: 0;
	padding: 4px 8px;
}

QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QTextBrowser:focus {
	border-color: {{accent}};
}

QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled, QTextBrowser:disabled {
	background-color: {{panel}};
	color: {{muted}};
}

/* --- forms：选择器结构参考 GTRONICK/QSS、BreezeStyleSheets (MIT)，色值仅走 ThemeTokens --- */
QComboBox {
	background-color: {{button}};
	color: {{text}};
	border: 1px solid {{borderStrong}};
	border-radius: {{radius}}px;
	padding: 2px 22px 2px 6px;
	min-width: 72px;
	min-height: 24px;
}

QComboBox:hover {
	border-color: {{accent}};
}

QComboBox:focus {
	border-color: {{accent}};
}

QComboBox:disabled {
	background-color: {{panel}};
	color: {{muted}};
	border-color: {{border}};
}

QComboBox::drop-down {
	subcontrol-origin: padding;
	subcontrol-position: top right;
	width: 20px;
	border: none;
	border-left: 1px solid {{border}};
	background-color: transparent;
}

QComboBox::drop-down:hover {
	background-color: {{hover}};
}

QComboBox::down-arrow {
	width: 0;
	height: 0;
	border-left: 4px solid transparent;
	border-right: 4px solid transparent;
	border-top: 5px solid {{muted}};
	margin-right: 6px;
}

QComboBox::down-arrow:on {
	border-top-color: {{accent}};
}

QComboBox QAbstractItemView {
	background-color: {{base}};
	border: 1px solid {{borderStrong}};
	border-radius: 0;
	padding: 2px;
	outline: 0;
	selection-background-color: {{selectionBg}};
	selection-color: {{selectionText}};
}

QComboBox QAbstractItemView::item {
	min-height: 24px;
	padding: 2px 8px;
}

QScrollArea {
	border: none;
	background-color: {{base}};
}

QScrollBar:vertical {
	background-color: {{panel}};
	width: 10px;
	margin: 0;
	border: none;
}

QScrollBar::handle:vertical {
	background-color: {{borderStrong}};
	min-height: 32px;
	border-radius: 4px;
	margin: 2px;
}

QScrollBar::handle:vertical:hover {
	background-color: {{muted}};
}

QScrollBar::handle:vertical:pressed {
	background-color: {{accent}};
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
	height: 0;
	border: none;
	background: none;
}

QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
	background: none;
}

QScrollBar:horizontal {
	background-color: {{panel}};
	height: 10px;
	margin: 0;
	border: none;
}

QScrollBar::handle:horizontal {
	background-color: {{borderStrong}};
	min-width: 32px;
	border-radius: 4px;
	margin: 2px;
}

QScrollBar::handle:horizontal:hover {
	background-color: {{muted}};
}

QScrollBar::handle:horizontal:pressed {
	background-color: {{accent}};
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
	width: 0;
	border: none;
	background: none;
}

QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
	background: none;
}

QGroupBox {
	background-color: transparent;
	border: 1px solid {{borderStrong}};
	border-radius: 0;
	margin-top: 10px;
	padding-top: 18px;
	font-weight: 500;
	color: {{text}};
}

QGroupBox::title {
	subcontrol-origin: margin;
	subcontrol-position: top left;
	padding: 2px 8px;
	color: {{accent}};
	background-color: {{base}};
	border-radius: 0;
	margin-left: 8px;
	font-weight: 600;
}

QProgressBar {
	background-color: {{panel}};
	border: 1px solid {{borderStrong}};
	border-radius: 6px;
	text-align: center;
	color: {{text}};
	height: 18px;
	min-height: 18px;
}

QProgressBar::chunk {
	background-color: {{accent}};
	border-radius: 5px;
}

QToolBar {
	background-color: {{panel}};
	border-bottom: 1px solid transparent;
	padding: 4px;
	spacing: 4px;
}

QToolButton {
	background-color: transparent;
	color: {{text}};
	border: 1px solid transparent;
	border-radius: {{radius}}px;
	padding: 6px 10px;
}

QToolButton:hover {
	background-color: {{hover}};
	border-color: {{border}};
}

QToolButton:pressed {
	background-color: {{buttonHover}};
}

QToolButton:checked {
	background-color: {{selectionBg}};
	color: {{selectionText}};
	border-color: {{accent}};
}

QStatusBar {
	background-color: {{panel}};
	color: {{muted}};
	border-top: 1px solid transparent;
}

QStatusBar::item {
	border: none;
}

QSplitter::handle {
	background-color: transparent;
}

QSplitter::handle:hover {
	background-color: {{border}};
}

QSplitter::handle:horizontal {
	width: 4px;
}

QSplitter::handle:vertical {
	height: 4px;
}

QSpinBox, QDoubleSpinBox {
	background-color: {{base}};
	color: {{text}};
	border: 1px solid {{borderStrong}};
	border-radius: {{radius}}px;
	padding: 2px 22px 2px 8px;
	min-height: 24px;
}

QSpinBox:hover, QDoubleSpinBox:hover {
	border-color: {{accent}};
}

QSpinBox:focus, QDoubleSpinBox:focus {
	border-color: {{accent}};
}

QSpinBox:disabled, QDoubleSpinBox:disabled {
	background-color: {{panel}};
	color: {{muted}};
	border-color: {{border}};
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
	subcontrol-origin: border;
	subcontrol-position: top right;
	width: 18px;
	border-left: 1px solid {{border}};
	border-bottom: 1px solid {{border}};
	background-color: {{button}};
	border-top-right-radius: {{radius}}px;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
	subcontrol-origin: border;
	subcontrol-position: bottom right;
	width: 18px;
	border-left: 1px solid {{border}};
	background-color: {{button}};
	border-bottom-right-radius: {{radius}}px;
}

QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
	background-color: {{hover}};
}

QSpinBox::up-button:pressed, QDoubleSpinBox::up-button:pressed,
QSpinBox::down-button:pressed, QDoubleSpinBox::down-button:pressed {
	background-color: {{buttonHover}};
}

QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
	width: 0;
	height: 0;
	border-left: 3px solid transparent;
	border-right: 3px solid transparent;
	border-bottom: 4px solid {{muted}};
}

QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
	width: 0;
	height: 0;
	border-left: 3px solid transparent;
	border-right: 3px solid transparent;
	border-top: 4px solid {{muted}};
}

QCheckBox, QRadioButton {
	color: {{text}};
	spacing: 8px;
}

QCheckBox:disabled, QRadioButton:disabled {
	color: {{muted}};
}

QCheckBox::indicator {
	width: 16px;
	height: 16px;
	border: 1px solid {{borderStrong}};
	border-radius: 3px;
	background-color: {{base}};
}

QCheckBox::indicator:hover {
	border-color: {{accent}};
}

QCheckBox::indicator:checked {
	background-color: {{accent}};
	border-color: {{accentPressed}};
}

QCheckBox::indicator:disabled {
	background-color: {{panel}};
	border-color: {{border}};
}

QRadioButton::indicator {
	width: 16px;
	height: 16px;
	border: 1px solid {{borderStrong}};
	border-radius: 8px;
	background-color: {{base}};
}

QRadioButton::indicator:hover {
	border-color: {{accent}};
}

QRadioButton::indicator:checked {
	background-color: {{accent}};
	border-color: {{accentPressed}};
}

QRadioButton::indicator:disabled {
	background-color: {{panel}};
	border-color: {{border}};
}

/* 负 margin 把手会溢出 groove；控件高度不足时上下被裁成扁圆 */
QSlider:horizontal {
	min-height: 22px;
	padding: 2px 0;
}

QSlider:vertical {
	min-width: 22px;
	padding: 0 2px;
}

QSlider::groove:horizontal {
	height: 4px;
	background-color: {{borderStrong}};
	border-radius: 2px;
}

QSlider::groove:vertical {
	width: 4px;
	background-color: {{borderStrong}};
	border-radius: 2px;
}

QSlider::sub-page:horizontal, QSlider::add-page:vertical {
	background-color: {{accent}};
	border-radius: 2px;
}

QSlider::add-page:horizontal, QSlider::sub-page:vertical {
	background-color: {{border}};
	border-radius: 2px;
}

QSlider::handle:horizontal {
	width: 14px;
	height: 14px;
	margin: -5px 0;
	border-radius: 7px;
	background-color: {{base}};
	border: 1px solid {{borderStrong}};
}

QSlider::handle:vertical {
	width: 14px;
	height: 14px;
	margin: 0 -5px;
	border-radius: 7px;
	background-color: {{base}};
	border: 1px solid {{borderStrong}};
}

QSlider::handle:horizontal:hover, QSlider::handle:vertical:hover {
	border-color: {{accent}};
}

QSlider::handle:horizontal:pressed, QSlider::handle:vertical:pressed {
	background-color: {{selectionBg}};
	border-color: {{accent}};
}

QToolTip {
	background-color: {{base}};
	color: {{text}};
	border: 1px solid {{borderStrong}};
	border-radius: {{radius}}px;
	padding: 4px 8px;
	font-size: 11px;
}
)");

	const bool dark = t.base.valueF() < 0.5;
	const QColor primaryDisabledBg =
		dark ? QColor(QStringLiteral("#2c4a66")) : QColor(QStringLiteral("#a0c4e8"));
	const QString primaryDisabledFg = dark ? QStringLiteral("#8a9aaa") : QStringLiteral("#f0f1f3");
	const QString dangerHoverBgHex = dark ? QStringLiteral("#5a3030") : QStringLiteral("#fdecea");
	const QString dangerBorderHex = dark ? QStringLiteral("#8a4040") : QStringLiteral("#e0a0a0");

	sheet.replace(QStringLiteral("{{window}}"), hex(t.window));
	sheet.replace(QStringLiteral("{{panel}}"), hex(t.panel));
	sheet.replace(QStringLiteral("{{base}}"), hex(t.base));
	sheet.replace(QStringLiteral("{{text}}"), hex(t.text));
	sheet.replace(QStringLiteral("{{muted}}"), hex(t.textMuted));
	sheet.replace(QStringLiteral("{{accent}}"), hex(t.accent));
	sheet.replace(QStringLiteral("{{accentHover}}"), hex(t.accentHover));
	sheet.replace(QStringLiteral("{{accentPressed}}"), hex(t.accentPressed));
	sheet.replace(QStringLiteral("{{selectionBg}}"), hex(t.selectionBg));
	sheet.replace(QStringLiteral("{{selectionText}}"), hex(t.selectionText));
	sheet.replace(QStringLiteral("{{hover}}"), hex(t.hoverBg));
	sheet.replace(QStringLiteral("{{danger}}"), hex(t.danger));
	sheet.replace(QStringLiteral("{{border}}"), hex(t.border));
	sheet.replace(QStringLiteral("{{borderStrong}}"), hex(t.borderStrong));
	sheet.replace(QStringLiteral("{{button}}"), hex(t.button));
	sheet.replace(QStringLiteral("{{buttonHover}}"), hex(t.buttonHover));
	sheet.replace(QStringLiteral("{{radius}}"), radius);
	sheet.replace(QStringLiteral("{{dangerHoverBg}}"), dangerHoverBgHex);
	sheet.replace(QStringLiteral("{{dangerBorder}}"), dangerBorderHex);
	sheet.replace(QStringLiteral("{{primaryDisabledBg}}"), hex(primaryDisabledBg));
	sheet.replace(QStringLiteral("{{primaryDisabledFg}}"), primaryDisabledFg);
	return sheet;
}

} // namespace

namespace ApplicationStyle
{
ThemeTokens tokens(Theme theme)
{
	return makeTokens(theme);
}

void applyTheme(QApplication* app, Theme theme)
{
	if (!app)
	{
		return;
	}
	const ThemeTokens t = makeTokens(theme);
	app->setStyle(new CloudSimTreeBranchStyle());
	applyFontConfiguration(app);
	app->setPalette(makePalette(t, theme));
	app->setStyleSheet(generateStyleSheet(t));

	UiIcons::setTheme(theme == Theme::Dark ? UiIcons::Theme::Dark : UiIcons::Theme::Light);
	UiIcons::invalidateCache();
	UiIconDecorators::refreshAllDecorated();
}

Theme loadSavedTheme()
{
	return ApplicationSettings::loadTheme();
}

void saveTheme(Theme theme)
{
	ApplicationSettings::saveTheme(theme);
}

bool usesDarkTheme()
{
	return loadSavedTheme() == Theme::Dark;
}

} // namespace ApplicationStyle
