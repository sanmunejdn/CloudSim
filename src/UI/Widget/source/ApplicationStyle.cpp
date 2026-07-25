/// @file ApplicationStyle.cpp
/// @brief ApplicationStyle 实现

#include "ApplicationStyle.h"

#include "UiIconDecorators.h"
#include "UiIcons.h"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QPalette>
#include <QSettings>
#include <QStyleFactory>

namespace
{
QString themeToString(ApplicationStyle::Theme t)
{
	return t == ApplicationStyle::Theme::Dark ? QStringLiteral("dark") : QStringLiteral("light");
}

ApplicationStyle::Theme themeFromString(const QString& s)
{
	return s.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0 ? ApplicationStyle::Theme::Dark
																	   : ApplicationStyle::Theme::Light;
}

// 亮色主题：带微妙冷色调的专业配色
QPalette makeLightPalette()
{
	QPalette p;
	// 背景色：微妙冷色调
	p.setColor(QPalette::Window, QColor(248, 248, 250));
	p.setColor(QPalette::WindowText, QColor(28, 28, 30));
	p.setColor(QPalette::Base, QColor(255, 255, 255));
	p.setColor(QPalette::AlternateBase, QColor(240, 241, 243));
	p.setColor(QPalette::ToolTipBase, QColor(255, 255, 255));
	p.setColor(QPalette::ToolTipText, QColor(28, 28, 30));
	p.setColor(QPalette::Text, QColor(28, 28, 30));
	p.setColor(QPalette::Button, QColor(240, 241, 243));
	p.setColor(QPalette::ButtonText, QColor(28, 28, 30));
	p.setColor(QPalette::BrightText, QColor(220, 50, 50));
	// 链接和强调色：降低饱和度的蓝色
	p.setColor(QPalette::Link, QColor(0, 102, 204));
	p.setColor(QPalette::Highlight, QColor(0, 102, 204));
	p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
	// 禁用状态
	p.setColor(QPalette::Disabled, QPalette::Text, QColor(142, 142, 147));
	p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(142, 142, 147));
	// 阴影和边框
	p.setColor(QPalette::Shadow, QColor(0, 0, 0, 20));
	p.setColor(QPalette::Mid, QColor(218, 218, 220));
	p.setColor(QPalette::Light, QColor(255, 255, 255));
	p.setColor(QPalette::Midlight, QColor(232, 233, 236));
	p.setColor(QPalette::Dark, QColor(200, 200, 202));
	return p;
}

// 暗色主题：深邃专业的暗色配色
QPalette makeDarkPalette()
{
	QPalette p;
	// 背景色：深邃但不刺眼
	p.setColor(QPalette::Window, QColor(36, 36, 38));
	p.setColor(QPalette::WindowText, QColor(242, 242, 247));
	p.setColor(QPalette::Base, QColor(28, 28, 30));
	p.setColor(QPalette::AlternateBase, QColor(44, 44, 46));
	p.setColor(QPalette::ToolTipBase, QColor(56, 56, 58));
	p.setColor(QPalette::ToolTipText, QColor(242, 242, 247));
	p.setColor(QPalette::Text, QColor(242, 242, 247));
	p.setColor(QPalette::Button, QColor(56, 56, 58));
	p.setColor(QPalette::ButtonText, QColor(242, 242, 247));
	p.setColor(QPalette::BrightText, QColor(255, 100, 100));
	// 链接和强调色：明亮但不刺眼的蓝色
	p.setColor(QPalette::Link, QColor(40, 140, 240));
	p.setColor(QPalette::Highlight, QColor(40, 140, 240));
	p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
	// 禁用状态
	p.setColor(QPalette::Disabled, QPalette::Text, QColor(99, 99, 102));
	p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(99, 99, 102));
	// 阴影和边框
	p.setColor(QPalette::Shadow, QColor(0, 0, 0, 80));
	p.setColor(QPalette::Mid, QColor(68, 68, 70));
	p.setColor(QPalette::Light, QColor(68, 68, 70));
	p.setColor(QPalette::Midlight, QColor(56, 56, 58));
	p.setColor(QPalette::Dark, QColor(22, 22, 24));
	return p;
}

// 应用字体配置：使用系统字体，增加字重层次
void applyFontConfiguration(QApplication* app)
{
	QFont defaultFont = app->font();
	// Windows 11 使用 Segoe UI Variable，Windows 10 使用 Segoe UI
	defaultFont.setFamily(QStringLiteral("Segoe UI Variable Display"));
	defaultFont.setPointSize(10);
	defaultFont.setWeight(QFont::Normal);
	app->setFont(defaultFont);
}

// 生成统一样式表
QString generateStyleSheet(ApplicationStyle::Theme theme)
{
	if (theme == ApplicationStyle::Theme::Dark)
	{
		return QStringLiteral(R"(
/* 暗色主题样式表 */

/* 主窗口 */
QMainWindow {
	background-color: #242426;
}

/* 菜单栏 */
QMenuBar {
	background-color: #2c2c2e;
	border-bottom: 1px solid #38383a;
	padding: 2px 4px;
}

QMenuBar::item {
	background: transparent;
	padding: 6px 12px;
	border-radius: 4px;
	color: #f2f2f7;
}

QMenuBar::item:selected {
	background-color: #3a3a3c;
}

QMenuBar::item:pressed {
	background-color: #48484a;
}

/* 菜单 */
QMenu {
	background-color: #2c2c2e;
	border: 1px solid #48484a;
	border-radius: 0;
	padding: 4px;
}

QMenu::item {
	padding: 8px 24px 8px 12px;
	border-radius: 4px;
	color: #f2f2f7;
}

QMenu::item:selected {
	background-color: #288cf0;
	color: #ffffff;
}

QMenu::separator {
	height: 1px;
	background-color: #48484a;
	margin: 4px 8px;
}

/* 停靠部件 */
QDockWidget {
	titlebar-close-icon: url(close.png);
	titlebar-normal-icon: url(float.png);
	color: #f2f2f7;
}

QDockWidget::title {
	background-color: #3a3a3c;
	padding: 8px 12px;
	font-weight: 500;
	border-bottom: 1px solid #48484a;
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
	background-color: #5c3a3a;
	border-color: #7a4a4a;
}

QDockWidget::float-button:hover {
	background-color: #48484a;
	border-color: #5c5c5e;
}

QDockWidget::close-button:pressed {
	background-color: #6e4444;
}

QDockWidget::float-button:pressed {
	background-color: #555557;
}

/* 标签页：大面板不做圆角（Qt 无法裁切子控件，圆角外会漏直角底） */
QTabWidget::pane {
	border: 1px solid #48484a;
	background-color: #2c2c2e;
	border-radius: 0;
}

QTabBar::tab {
	background-color: transparent;
	color: #8e8e93;
	padding: 8px 12px;
	margin-right: 2px;
	border-bottom: 2px solid transparent;
	font-weight: 400;
	min-width: 50px;
}

QTabBar::tab:selected {
	background-color: #2c2c2e;
	color: #f2f2f7;
	border-bottom: 2px solid #288cf0;
	font-weight: 500;
}

QTabBar::tab:hover:!selected {
	background-color: #3a3a3c;
	color: #f2f2f7;
}

/* 文档标签页特殊样式 */
QTabBar::tab:only-one {
	min-width: 100px;
}

/* 树形部件 */
QTreeWidget, QTreeView, QListWidget, QTableWidget, QTableView {
	background-color: #1c1c1e;
	border: 1px solid #38383a;
	border-radius: 0;
	padding: 4px;
	outline: none;
}

QTreeWidget::item, QTreeView::item {
	padding: 8px 12px;
	border-radius: 4px;
	margin: 1px 0;
}

QTreeWidget::item:selected, QTreeView::item:selected {
	background-color: #288cf0;
	color: #ffffff;
}

QTreeWidget::item:hover, QTreeView::item:hover {
	background-color: #3a3a3c;
}

QTreeWidget::branch, QTreeView::branch {
	background-color: transparent;
}

/* 按钮（默认=次要；btnRole 区分主次） */
QPushButton {
	background-color: #3a3a3c;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 4px;
	padding: 4px 8px;
	font-weight: 500;
	min-width: 40px;
	min-height: 24px;
}

QPushButton:hover {
	background-color: #48484a;
	border-color: #5a5a5c;
}

QPushButton:pressed {
	background-color: #2a2a2c;
}

QPushButton:disabled {
	background-color: #2c2c2e;
	color: #636366;
	border-color: #38383a;
}

QPushButton:focus {
	outline: none;
	border-color: #288cf0;
}

QPushButton[btnRole="primary"] {
	background-color: #288cf0;
	color: #ffffff;
	border: 1px solid #1a7ad8;
	font-weight: 600;
}

QPushButton[btnRole="primary"]:hover {
	background-color: #3a9af5;
	border-color: #288cf0;
}

QPushButton[btnRole="primary"]:pressed {
	background-color: #1a7ad8;
}

QPushButton[btnRole="primary"]:disabled {
	background-color: #2c4a66;
	color: #8a9aaa;
	border-color: #2c4a66;
}

QPushButton[btnRole="secondary"] {
	background-color: #3a3a3c;
	color: #f2f2f7;
	border: 1px solid #5a5a5c;
}

QPushButton[btnRole="danger"] {
	background-color: #3a3a3c;
	color: #ff6b6b;
	border: 1px solid #8a4040;
}

QPushButton[btnRole="danger"]:hover {
	background-color: #5a3030;
	border-color: #ff6b6b;
}

/* 输入框：单行可圆角；多行滚动区保持直角避免视口漏角 */
QLineEdit {
	background-color: #1c1c1e;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 4px;
	padding: 4px 8px;
}

QTextEdit, QPlainTextEdit, QTextBrowser {
	background-color: #1c1c1e;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 0;
	padding: 4px 8px;
}

QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QTextBrowser:focus {
	border-color: #288cf0;
}

QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled, QTextBrowser:disabled {
	background-color: #2c2c2e;
	color: #636366;
}

/* 下拉框：闭合高度贴合字号，弹层项可滚完整显示 */
QComboBox {
	background-color: #3a3a3c;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 4px;
	padding: 2px 6px;
	min-width: 72px;
	min-height: 24px;
	max-height: 26px;
}

QComboBox:hover {
	border-color: #5a5a5c;
}

QComboBox::drop-down {
	subcontrol-origin: padding;
	subcontrol-position: center right;
	width: 18px;
	border-left: 1px solid #48484a;
}

QComboBox QAbstractItemView {
	background-color: #2c2c2e;
	border: 1px solid #48484a;
	border-radius: 0;
	padding: 2px;
	outline: 0;
	selection-background-color: #288cf0;
	selection-color: #ffffff;
}

QComboBox QAbstractItemView::item {
	min-height: 24px;
	padding: 2px 8px;
}

/* 滚动条 */
QScrollBar:vertical {
	background-color: #1c1c1e;
	width: 12px;
	margin: 0;
	border-radius: 6px;
}

QScrollBar::handle:vertical {
	background-color: #48484a;
	min-height: 40px;
	border-radius: 6px;
	margin: 2px;
}

QScrollBar::handle:vertical:hover {
	background-color: #5a5a5c;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
	height: 0;
}

QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
	background: none;
}

QScrollBar:horizontal {
	background-color: #1c1c1e;
	height: 12px;
	margin: 0;
	border-radius: 6px;
}

QScrollBar::handle:horizontal {
	background-color: #48484a;
	min-width: 40px;
	border-radius: 6px;
	margin: 2px;
}

QScrollBar::handle:horizontal:hover {
	background-color: #5a5a5c;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
	width: 0;
}

QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
	background: none;
}

/* 分组框 */
QGroupBox {
	background-color: #2c2c2e;
	border: 1px solid #48484a;
	border-radius: 0;
	margin-top: 12px;
	padding-top: 24px;
	font-weight: 500;
	color: #f2f2f7;
}

QGroupBox::title {
	subcontrol-origin: margin;
	subcontrol-position: top left;
	padding: 4px 12px;
	background-color: #3a3a3c;
	border-radius: 4px;
	margin-left: 12px;
}

/* 进度条 */
QProgressBar {
	background-color: #1c1c1e;
	border: 1px solid #38383a;
	border-radius: 6px;
	text-align: center;
	color: #f2f2f7;
	height: 20px;
}

QProgressBar::chunk {
	background-color: #288cf0;
	border-radius: 5px;
}

/* 工具栏 */
QToolBar {
	background-color: #2c2c2e;
	border-bottom: 1px solid #38383a;
	padding: 4px;
	spacing: 4px;
}

QToolButton {
	background-color: transparent;
	color: #f2f2f7;
	border: 1px solid transparent;
	border-radius: 4px;
	padding: 6px 10px;
}

QToolButton:hover {
	background-color: #3a3a3c;
	border-color: #48484a;
}

QToolButton:pressed {
	background-color: #48484a;
}

QToolButton:checked {
	background-color: #288cf0;
	color: #ffffff;
}

/* 状态栏 */
QStatusBar {
	background-color: #2c2c2e;
	color: #8e8e93;
	border-top: 1px solid #38383a;
}

QStatusBar::item {
	border: none;
}

/* 分割器 */
QSplitter::handle {
	background-color: #48484a;
}

QSplitter::handle:horizontal {
	width: 2px;
}

QSplitter::handle:vertical {
	height: 2px;
}

/* 旋钮 */
QSpinBox, QDoubleSpinBox {
	background-color: #1c1c1e;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 6px;
	padding: 6px 8px;
}

QSpinBox:focus, QDoubleSpinBox:focus {
	border-color: #288cf0;
}

/* 复选框和单选按钮 */
QCheckBox, QRadioButton {
	color: #f2f2f7;
	spacing: 8px;
}

QCheckBox::indicator, QRadioButton::indicator {
	width: 18px;
	height: 18px;
}

/* 工具提示 */
QToolTip {
	background-color: #3a3a3c;
	color: #f2f2f7;
	border: 1px solid #48484a;
	border-radius: 3px;
	padding: 2px 6px;
	font-size: 11px;
}
)");
	}
	else
	{
		// 亮色主题样式表
		return QStringLiteral(R"(
/* 亮色主题样式表 */

/* 主窗口 */
QMainWindow {
	background-color: #f8f8fa;
}

/* 菜单栏 */
QMenuBar {
	background-color: #f0f1f3;
	border-bottom: 1px solid #dadcde;
	padding: 2px 4px;
}

QMenuBar::item {
	background: transparent;
	padding: 6px 12px;
	border-radius: 4px;
	color: #1c1c1e;
}

QMenuBar::item:selected {
	background-color: #e4e5e7;
}

QMenuBar::item:pressed {
	background-color: #dadcde;
}

/* 菜单 */
QMenu {
	background-color: #ffffff;
	border: 1px solid #dadcde;
	border-radius: 0;
	padding: 4px;
	box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

QMenu::item {
	padding: 8px 24px 8px 12px;
	border-radius: 4px;
	color: #1c1c1e;
}

QMenu::item:selected {
	background-color: #0066cc;
	color: #ffffff;
}

QMenu::separator {
	height: 1px;
	background-color: #dadcde;
	margin: 4px 8px;
}

/* 停靠部件 */
QDockWidget {
	color: #1c1c1e;
}

QDockWidget::title {
	background-color: #e8e9ec;
	padding: 8px 12px;
	font-weight: 500;
	border-bottom: 1px solid #dadcde;
}

QDockWidget::close-button, QDockWidget::float-button {
	background-color: transparent;
	border: 1px solid transparent;
	border-radius: 6px;
	padding: 5px;
	margin: 2px;
	width: 16px;
	height: 16px;
	subcontrol-origin: margin;
	subcontrol-position: center right;
}

QDockWidget::close-button:hover {
	background-color: #fde8e8;
	border-color: #f0b4b4;
}

QDockWidget::float-button:hover {
	background-color: #e5ebf1;
	border-color: #c9d3dd;
}

QDockWidget::close-button:pressed {
	background-color: #f8d0d0;
}

QDockWidget::float-button:pressed {
	background-color: #d8e0e8;
}

/* 标签页：大面板不做圆角（Qt 无法裁切子控件，圆角外会漏直角底） */
QTabWidget::pane {
	border: 1px solid #dadcde;
	background-color: #ffffff;
	border-radius: 0;
}

QTabBar::tab {
	background-color: transparent;
	color: #8e8e93;
	padding: 8px 12px;
	margin-right: 2px;
	border-bottom: 2px solid transparent;
	font-weight: 400;
	min-width: 50px;
}

QTabBar::tab:selected {
	background-color: #ffffff;
	color: #1c1c1e;
	border-bottom: 2px solid #0066cc;
	font-weight: 500;
}

QTabBar::tab:hover:!selected {
	background-color: #e8e9ec;
	color: #1c1c1e;
}

/* 文档标签页特殊样式 */
QTabBar::tab:only-one {
	min-width: 100px;
}

/* 树形部件 */
QTreeWidget, QTreeView, QListWidget, QTableWidget, QTableView {
	background-color: #ffffff;
	border: 1px solid #dadcde;
	border-radius: 0;
	padding: 4px;
	outline: none;
}

QTreeWidget::item, QTreeView::item {
	padding: 8px 12px;
	border-radius: 4px;
	margin: 1px 0;
}

QTreeWidget::item:selected, QTreeView::item:selected {
	background-color: #0066cc;
	color: #ffffff;
}

QTreeWidget::item:hover, QTreeView::item:hover {
	background-color: #e8e9ec;
}

QTreeWidget::branch, QTreeView::branch {
	background-color: transparent;
}

/* 按钮（默认=次要；btnRole 区分主次） */
QPushButton {
	background-color: #f0f1f3;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 4px;
	padding: 4px 8px;
	font-weight: 500;
	min-width: 40px;
	min-height: 24px;
}

QPushButton:hover {
	background-color: #e4e5e7;
	border-color: #c8c8ca;
}

QPushButton:pressed {
	background-color: #dadcde;
}

QPushButton:disabled {
	background-color: #f0f1f3;
	color: #8e8e93;
	border-color: #dadcde;
}

QPushButton:focus {
	outline: none;
	border-color: #0066cc;
}

QPushButton[btnRole="primary"] {
	background-color: #0066cc;
	color: #ffffff;
	border: 1px solid #0055aa;
	font-weight: 600;
}

QPushButton[btnRole="primary"]:hover {
	background-color: #0077e0;
	border-color: #0066cc;
}

QPushButton[btnRole="primary"]:pressed {
	background-color: #0055aa;
}

QPushButton[btnRole="primary"]:disabled {
	background-color: #a0c4e8;
	color: #f0f1f3;
	border-color: #a0c4e8;
}

QPushButton[btnRole="secondary"] {
	background-color: #ffffff;
	color: #1c1c1e;
	border: 1px solid #c8c8ca;
}

QPushButton[btnRole="danger"] {
	background-color: #ffffff;
	color: #c62828;
	border: 1px solid #e0a0a0;
}

QPushButton[btnRole="danger"]:hover {
	background-color: #fdecea;
	border-color: #c62828;
}

/* 输入框：单行可圆角；多行滚动区保持直角避免视口漏角 */
QLineEdit {
	background-color: #ffffff;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 4px;
	padding: 4px 8px;
}

QTextEdit, QPlainTextEdit, QTextBrowser {
	background-color: #ffffff;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 0;
	padding: 4px 8px;
}

QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QTextBrowser:focus {
	border-color: #0066cc;
}

QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled, QTextBrowser:disabled {
	background-color: #f0f1f3;
	color: #8e8e93;
}

/* 下拉框：闭合高度贴合字号，弹层项可滚完整显示 */
QComboBox {
	background-color: #f0f1f3;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 4px;
	padding: 2px 6px;
	min-width: 72px;
	min-height: 24px;
	max-height: 26px;
}

QComboBox:hover {
	border-color: #c8c8ca;
}

QComboBox::drop-down {
	subcontrol-origin: padding;
	subcontrol-position: center right;
	width: 18px;
	border-left: 1px solid #dadcde;
}

QComboBox QAbstractItemView {
	background-color: #ffffff;
	border: 1px solid #dadcde;
	border-radius: 0;
	padding: 2px;
	outline: 0;
	selection-background-color: #0066cc;
	selection-color: #ffffff;
}

QComboBox QAbstractItemView::item {
	min-height: 24px;
	padding: 2px 8px;
}

/* 滚动条 */
QScrollBar:vertical {
	background-color: #f8f8fa;
	width: 12px;
	margin: 0;
	border-radius: 6px;
}

QScrollBar::handle:vertical {
	background-color: #c8c8ca;
	min-height: 40px;
	border-radius: 6px;
	margin: 2px;
}

QScrollBar::handle:vertical:hover {
	background-color: #a8a8aa;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
	height: 0;
}

QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
	background: none;
}

QScrollBar:horizontal {
	background-color: #f8f8fa;
	height: 12px;
	margin: 0;
	border-radius: 6px;
}

QScrollBar::handle:horizontal {
	background-color: #c8c8ca;
	min-width: 40px;
	border-radius: 6px;
	margin: 2px;
}

QScrollBar::handle:horizontal:hover {
	background-color: #a8a8aa;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
	width: 0;
}

QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
	background: none;
}

/* 分组框 */
QGroupBox {
	background-color: #ffffff;
	border: 1px solid #dadcde;
	border-radius: 0;
	margin-top: 12px;
	padding-top: 24px;
	font-weight: 500;
	color: #1c1c1e;
}

QGroupBox::title {
	subcontrol-origin: margin;
	subcontrol-position: top left;
	padding: 4px 12px;
	background-color: #e8e9ec;
	border-radius: 4px;
	margin-left: 12px;
}

/* 进度条 */
QProgressBar {
	background-color: #f0f1f3;
	border: 1px solid #dadcde;
	border-radius: 6px;
	text-align: center;
	color: #1c1c1e;
	height: 20px;
}

QProgressBar::chunk {
	background-color: #0066cc;
	border-radius: 5px;
}

/* 工具栏 */
QToolBar {
	background-color: #f0f1f3;
	border-bottom: 1px solid #dadcde;
	padding: 4px;
	spacing: 4px;
}

QToolButton {
	background-color: transparent;
	color: #1c1c1e;
	border: 1px solid transparent;
	border-radius: 4px;
	padding: 6px 10px;
}

QToolButton:hover {
	background-color: #e4e5e7;
	border-color: #dadcde;
}

QToolButton:pressed {
	background-color: #dadcde;
}

QToolButton:checked {
	background-color: #0066cc;
	color: #ffffff;
}

/* 状态栏 */
QStatusBar {
	background-color: #f0f1f3;
	color: #8e8e93;
	border-top: 1px solid #dadcde;
}

QStatusBar::item {
	border: none;
}

/* 分割器 */
QSplitter::handle {
	background-color: #dadcde;
}

QSplitter::handle:horizontal {
	width: 2px;
}

QSplitter::handle:vertical {
	height: 2px;
}

/* 旋钮 */
QSpinBox, QDoubleSpinBox {
	background-color: #ffffff;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 6px;
	padding: 6px 8px;
}

QSpinBox:focus, QDoubleSpinBox:focus {
	border-color: #0066cc;
}

/* 复选框和单选按钮 */
QCheckBox, QRadioButton {
	color: #1c1c1e;
	spacing: 8px;
}

QCheckBox::indicator, QRadioButton::indicator {
	width: 18px;
	height: 18px;
}

/* 工具提示 */
QToolTip {
	background-color: #ffffff;
	color: #1c1c1e;
	border: 1px solid #dadcde;
	border-radius: 3px;
	padding: 2px 6px;
	font-size: 11px;
}
)");
	}
}

} // namespace

namespace ApplicationStyle
{
void applyTheme(QApplication* app, Theme theme)
{
	if (!app)
	{
		return;
	}
	app->setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
	// 应用字体配置
	applyFontConfiguration(app);
	// 应用调色板
	app->setPalette(theme == Theme::Dark ? makeDarkPalette() : makeLightPalette());
	// 应用样式表
	app->setStyleSheet(generateStyleSheet(theme));

	UiIcons::setTheme(theme == Theme::Dark ? UiIcons::Theme::Dark : UiIcons::Theme::Light);
	UiIcons::invalidateCache();
	UiIconDecorators::refreshAllDecorated();
}

Theme loadSavedTheme()
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	return themeFromString(settings.value(QStringLiteral("theme"), QStringLiteral("light")).toString());
}

void saveTheme(Theme theme)
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	settings.setValue(QStringLiteral("theme"), themeToString(theme));
}

bool usesDarkTheme()
{
	return loadSavedTheme() == Theme::Dark;
}

} // namespace ApplicationStyle
