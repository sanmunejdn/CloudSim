#ifndef WIDGET_VIEWPRESETOVERLAY_H
#define WIDGET_VIEWPRESETOVERLAY_H

/// @file ViewPresetOverlay.h
/// @brief 3D 视口右上角视角预设浮层

#include "../../OsgWidgetCore/inc/OsgScene.h"

#include <QWidget>

class QGridLayout;
class QLabel;
class QToolButton;

/// 3D 视口右上角视角预设浮层
class ViewPresetOverlay : public QWidget
{
	Q_OBJECT
public:
	explicit ViewPresetOverlay(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setDarkTheme(bool dark);

signals:
	void presetRequested(OsgScene::CameraViewPreset preset);

private:
	void refreshButtonLabels();
	void applyPanelStyle();
	QToolButton* addPresetButton(QGridLayout* grid, int row, int col, int rowSpan, int colSpan, const QString& labelEn,
								 const QString& labelZh, OsgScene::CameraViewPreset preset, bool emphasize = false);

	bool m_useChinese = true;
	bool m_darkTheme = true;
	QLabel* m_titleLabel = nullptr;
	QToolButton* m_isoButton = nullptr;
};

#endif // WIDGET_VIEWPRESETOVERLAY_H
