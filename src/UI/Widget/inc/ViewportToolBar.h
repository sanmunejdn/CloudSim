#ifndef WIDGET_VIEWPORTTOOLBAR_H
#define WIDGET_VIEWPORTTOOLBAR_H

/// @file ViewportToolBar.h
/// @brief 3D 视口顶部浮动按钮（视角工具 + 侧栏抽屉切换）

#include <QObject>

class QToolButton;
class QWidget;

/// 3D 视口顶部浮动按钮（视角工具 + 侧栏抽屉切换）
class ViewportToolBar : public QObject
{
	Q_OBJECT
public:
	explicit ViewportToolBar(QWidget* host);

	void setDarkTheme(bool dark);
	void setUseChinese(bool useChinese);
	void refreshChrome();
	void showButtons();
	/// 与 MainWindow 侧栏显隐同步（checked = 面板可见）
	void setSidePanelToggleState(bool leftVisible, bool rightVisible);

signals:
	void focusRequested();
	void wireframeToggled(bool on);
	void screenshotRequested();
	void leftPanelVisibilityToggled(bool visible);
	void rightPanelVisibilityToggled(bool visible);

protected:
	bool eventFilter(QObject* obj, QEvent* ev) override;

private:
	void applyButtonStyle();
	void reposition();
	void raiseButtons();
	void updateSidePanelButtonTips();
	void updateLeftPanelChrome(bool visible);
	void updateRightPanelChrome(bool visible);

	QWidget* m_host = nullptr;
	bool m_darkTheme = false;
	bool m_useChinese = true;
	bool m_wireframeOn = false;
	QToolButton* m_focusBtn = nullptr;
	QToolButton* m_wireBtn = nullptr;
	QToolButton* m_captureBtn = nullptr;
	QToolButton* m_leftPanelBtn = nullptr;
	QToolButton* m_rightPanelBtn = nullptr;
	QWidget* m_actionTip = nullptr;
};

#endif // WIDGET_VIEWPORTTOOLBAR_H
