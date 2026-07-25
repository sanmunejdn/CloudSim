#ifndef WIDGET_STYLEDDOCKTITLEBAR_H
#define WIDGET_STYLEDDOCKTITLEBAR_H

/// @file StyledDockTitleBar.h
/// @brief 自定义 Dock 标题栏：可读标题 + 美化后的悬浮/关闭按钮

#include <QWidget>

class QDockWidget;
class QLabel;
class QToolButton;

class StyledDockTitleBar final : public QWidget
{
	Q_OBJECT

public:
	explicit StyledDockTitleBar(QDockWidget* dock);

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void paintEvent(QPaintEvent* event) override;

private:
	void syncTitle();
	void syncFloatButton();
	void toggleFloating();
	void requestClose();

	QDockWidget* m_dock = nullptr;
	QLabel* m_title = nullptr;
	QToolButton* m_floatBtn = nullptr;
	QToolButton* m_closeBtn = nullptr;
	QPoint m_pressGlobal;
	bool m_dragging = false;
};

void applyStyledDockTitleBar(QDockWidget* dock);

#endif
