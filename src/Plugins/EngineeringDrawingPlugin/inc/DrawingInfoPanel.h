#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGINFOPANEL_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGINFOPANEL_H

/// @file DrawingInfoPanel.h
/// @brief 右侧说明面板

#include <QWidget>

class QLabel;

class DrawingInfoPanel final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingInfoPanel(QWidget* parent = nullptr);
	void applyLanguage(bool useChinese);

private:
	QLabel* m_label = nullptr;
};

#endif
