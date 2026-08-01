#ifndef ENGINEERINGDRAWINGPLUGIN_DIMSTYLEDIALOG_H
#define ENGINEERINGDRAWINGPLUGIN_DIMSTYLEDIALOG_H

/// @file DimStyleDialog.h
/// @brief 标注样式编辑对话框

#include "DrawingSheetCanvasWidget.h"

#include <QDialog>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;

class DimStyleDialog final : public QDialog
{
	Q_OBJECT

public:
	explicit DimStyleDialog(DrawingSheetCanvasWidget* canvas, QWidget* parent = nullptr);

private:
	void loadFromCanvas();
	void applyToCanvas();

	DrawingSheetCanvasWidget* m_canvas = nullptr;
	QComboBox* m_styleCombo = nullptr;
	QDoubleSpinBox* m_textH = nullptr;
	QDoubleSpinBox* m_arrow = nullptr;
	QSpinBox* m_prec = nullptr;
	QCheckBox* m_showTol = nullptr;
	QDoubleSpinBox* m_tolPlus = nullptr;
	QDoubleSpinBox* m_tolMinus = nullptr;
};

#endif
