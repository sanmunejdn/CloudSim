#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H

/// @file DrawingRibbonBar.h
/// @brief 工程图 Ribbon：视图/绘图/标注 + 出图/图幅/导出

#include "DrawingSheetCanvasWidget.h"

#include <QWidget>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QToolButton;

class DrawingRibbonBar final : public QWidget
{
	Q_OBJECT

public:
	explicit DrawingRibbonBar(QWidget* parent = nullptr);

	void setActiveTool(DrawingCanvasTool tool);

	QPushButton* generateButton() const { return m_generateBtn; }

	bool includeIso() const;
	bool includeSection() const;
	int sectionPlane() const;
	bool customSection() const;
	void sectionOriginMm(double out[3]) const;
	void sectionNormal(double out[3]) const;
	bool thirdAngle() const;
	double detailScale() const;

	/// 把当前图幅/比例/图名写到画布
	void applySheetSettings(DrawingSheetCanvasWidget* canvas, bool rescaleContent = false);
	/// 从画布回填图幅 UI（切页/加载后）
	void syncFromCanvas(const DrawingSheetCanvasWidget* canvas);

public slots:
	void applyTheme(bool dark);
	void applyLanguage(bool useChinese);

signals:
	void toolRequested(DrawingCanvasTool tool);
	void generateRequested();
	void exportSvgRequested();
	void exportDxfRequested();
	void exportPdfRequested();
	void fitWindowRequested();
	void fitPaperRequested();
	void gridVisibleChanged(bool visible);
	void detailScaleChanged(double scale);
	/// rescaleContent：改比例时为 true，仅改图幅时为 false
	void sheetSettingsChanged(bool rescaleContent);

private:
	void rebuildIcons(bool dark);
	void setBtnText(QToolButton* btn, const QString& text);
	void updateCustomSectionUi();
	void updateCustomPaperUi();
	void updateCustomScaleUi();

	bool m_dark = false;
	bool m_useChinese = true;
	QLabel* m_lblView = nullptr;
	QLabel* m_lblSketch = nullptr;
	QLabel* m_lblMarks = nullptr;
	QLabel* m_lblOut = nullptr;
	QLabel* m_lblSheet = nullptr;
	QLabel* m_lblExport = nullptr;
	QButtonGroup* m_tools = nullptr;
	QToolButton* m_btnPan = nullptr;
	QToolButton* m_btnDetail = nullptr;
	QToolButton* m_btnFitWindow = nullptr;
	QCheckBox* m_gridCheck = nullptr;
	QToolButton* m_btnLine = nullptr;
	QToolButton* m_btnArc = nullptr;
	QToolButton* m_btnCircle = nullptr;
	QToolButton* m_btnRect = nullptr;
	QToolButton* m_btnSpline = nullptr;
	QToolButton* m_btnSelect = nullptr;
	QToolButton* m_btnDimLinear = nullptr;
	QToolButton* m_btnDimRadius = nullptr;
	QToolButton* m_btnDimDiameter = nullptr;
	QToolButton* m_btnDimAngle = nullptr;
	QToolButton* m_btnNote = nullptr;

	QPushButton* m_generateBtn = nullptr;
	QComboBox* m_angleCombo = nullptr;
	QCheckBox* m_isoCheck = nullptr;
	QCheckBox* m_sectionCheck = nullptr;
	QComboBox* m_sectionPlaneCombo = nullptr;
	QLabel* m_secOxLabel = nullptr;
	QDoubleSpinBox* m_secOx = nullptr;
	QDoubleSpinBox* m_secOy = nullptr;
	QDoubleSpinBox* m_secOz = nullptr;
	QLabel* m_secNxLabel = nullptr;
	QDoubleSpinBox* m_secNx = nullptr;
	QDoubleSpinBox* m_secNy = nullptr;
	QDoubleSpinBox* m_secNz = nullptr;

	QComboBox* m_paperCombo = nullptr;
	QLabel* m_paperWLabel = nullptr;
	QDoubleSpinBox* m_paperWSpin = nullptr;
	QLabel* m_paperHLabel = nullptr;
	QDoubleSpinBox* m_paperHSpin = nullptr;
	QComboBox* m_scaleCombo = nullptr;
	QDoubleSpinBox* m_scaleSpin = nullptr;
	QPushButton* m_fitPaperBtn = nullptr;
	QLineEdit* m_titleEdit = nullptr;
	QDoubleSpinBox* m_detailScaleSpin = nullptr;

	QPushButton* m_svgBtn = nullptr;
	QPushButton* m_dxfBtn = nullptr;
	QPushButton* m_pdfBtn = nullptr;
};

#endif
