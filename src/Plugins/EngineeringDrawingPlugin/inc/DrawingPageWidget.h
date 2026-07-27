#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGPAGEWIDGET_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGPAGEWIDGET_H

/// @file DrawingPageWidget.h
/// @brief 中央工程图页：出图选项 + 画布

#include "DrawingSheetCanvasWidget.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QPushButton;

class DrawingPageWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingPageWidget(QWidget* parent = nullptr);

	DrawingSheetCanvasWidget* canvas() const { return m_canvas; }
	QPushButton* generateButton() const { return m_generateBtn; }

	bool includeIso() const;
	bool includeSection() const;
	int sectionPlane() const;
	bool thirdAngle() const;

	void applyLanguage(bool useChinese);

signals:
	void generateRequested();
	void exportSvgRequested();
	void exportDxfRequested();

private:
	DrawingSheetCanvasWidget* m_canvas = nullptr;
	QPushButton* m_generateBtn = nullptr;
	QCheckBox* m_gridCheck = nullptr;
	QCheckBox* m_isoCheck = nullptr;
	QCheckBox* m_sectionCheck = nullptr;
	QComboBox* m_sectionPlaneCombo = nullptr;
	QComboBox* m_angleCombo = nullptr;
	QPushButton* m_fitBtn = nullptr;
	QPushButton* m_svgBtn = nullptr;
	QPushButton* m_dxfBtn = nullptr;
};

#endif
