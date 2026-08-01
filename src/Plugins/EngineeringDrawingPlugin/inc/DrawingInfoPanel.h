#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGINFOPANEL_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGINFOPANEL_H

/// @file DrawingInfoPanel.h
/// @brief 右侧特性面板：图层/色/线型/线宽（ByLayer 或覆盖）

#include "DrawingSheetCanvasWidget.h"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

class DrawingInfoPanel final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingInfoPanel(QWidget* parent = nullptr);
	void bindCanvas(DrawingSheetCanvasWidget* canvas);
	void applyLanguage(bool useChinese);

private:
	void refreshFromSelection();
	void applyUiToSelection();

	QPointer<DrawingSheetCanvasWidget> m_canvas;
	bool m_useChinese = true;
	bool m_busy = false;
	QLabel* m_title = nullptr;
	QLabel* m_hint = nullptr;
	QComboBox* m_layerCombo = nullptr;
	QCheckBox* m_colorByLayer = nullptr;
	QCheckBox* m_colorByBlock = nullptr;
	QPushButton* m_colorBtn = nullptr;
	QCheckBox* m_ltByLayer = nullptr;
	QCheckBox* m_ltByBlock = nullptr;
	QComboBox* m_lineTypeCombo = nullptr;
	QCheckBox* m_lwByLayer = nullptr;
	QCheckBox* m_lwByBlock = nullptr;
	QDoubleSpinBox* m_widthSpin = nullptr;
	QCheckBox* m_showTol = nullptr;
	QDoubleSpinBox* m_tolPlus = nullptr;
	QDoubleSpinBox* m_tolMinus = nullptr;
	QPushButton* m_matchBtn = nullptr;
};

#endif
