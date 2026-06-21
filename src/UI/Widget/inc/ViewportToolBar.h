#pragma once

#include <QObject>

class QToolButton;
class QWidget;

/// 3D 视口顶部浮动按钮（无容器，三个按钮直接叠在视口上）
class ViewportToolBar : public QObject
{
	Q_OBJECT
public:
	explicit ViewportToolBar(QWidget* host);

	void setDarkTheme(bool dark);
	void refreshChrome();
	void showButtons();

signals:
	void focusRequested();
	void wireframeToggled(bool on);
	void screenshotRequested();

protected:
	bool eventFilter(QObject* obj, QEvent* ev) override;

private:
	void applyButtonStyle();
	void reposition();
	void raiseButtons();

	QWidget* m_host = nullptr;
	bool m_darkTheme = false;
	bool m_wireframeOn = false;
	QToolButton* m_focusBtn = nullptr;
	QToolButton* m_wireBtn = nullptr;
	QToolButton* m_captureBtn = nullptr;
	QWidget* m_actionTip = nullptr;
};
