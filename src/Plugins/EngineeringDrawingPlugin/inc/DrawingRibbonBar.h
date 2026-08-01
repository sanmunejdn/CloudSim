#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGRIBBONBAR_H

/// @file DrawingRibbonBar.h
/// @brief 工程图 Ribbon：视图/绘图/标注/修改/捕捉 + 出图/图幅/导出

#include "DrawingSheetCanvasWidget.h"
#include "SheetSnapEngine.h"

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
	/// PolyAlgo 快速预览（默认关；正式出图请关闭）
	bool coarseView() const;
	bool includeSection() const;
	int sectionPlane() const;
	bool customSection() const;
	void sectionOriginMm(double out[3]) const;
	void sectionNormal(double out[3]) const;
	bool thirdAngle() const;
	double detailScale() const;

	void applySheetSettings(DrawingSheetCanvasWidget* canvas, bool rescaleContent = false);
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
	void importDxfRequested();
	void printPreviewRequested();
	void createBlockRequested();
	void insertBlockRequested();
	void dimStyleDialogRequested();
	void titleBlockAttrsRequested();
	void ctbEnabledChanged(bool enabled);
	void ctbTableEditRequested();
	void recalculateDimsRequested();
	void projectionDragLockChanged(bool on);
	void projectionPinnedChanged(bool on);
	void projectionGuidesVisibleChanged(bool visible);
	void halfSectionChanged(bool on);
	void fitWindowRequested();
	void fitPaperRequested();
	void viewAlignRequested(ViewAlignMode mode);
	void projectionAlignRequested();
	void gridVisibleChanged(bool visible);
	void detailScaleChanged(double scale);
	void sheetSettingsChanged(bool rescaleContent);
	void snapFlagsChanged(SheetSnapFlags flags);
	void ltScaleChanged(double scale);

private:
	void rebuildIcons(bool dark);
	void setBtnText(QToolButton* btn, const QString& text);
	void updateCustomSectionUi();
	void updateCustomPaperUi();
	void updateCustomScaleUi();
	void emitSnapFlags();

	bool m_dark = false;
	bool m_useChinese = true;
	QLabel* m_lblView = nullptr;
	QLabel* m_lblSketch = nullptr;
	QLabel* m_lblMarks = nullptr;
	QLabel* m_lblModify = nullptr;
	QLabel* m_lblSnap = nullptr;
	QLabel* m_lblOut = nullptr;
	QLabel* m_lblSheet = nullptr;
	QLabel* m_lblExport = nullptr;
	QButtonGroup* m_tools = nullptr;
	QToolButton* m_btnPan = nullptr;
	QToolButton* m_btnDetail = nullptr;
	QToolButton* m_btnFitWindow = nullptr;
	QToolButton* m_btnAlignLeft = nullptr;
	QToolButton* m_btnAlignHCenter = nullptr;
	QToolButton* m_btnAlignRight = nullptr;
	QToolButton* m_btnAlignTop = nullptr;
	QToolButton* m_btnAlignVCenter = nullptr;
	QToolButton* m_btnAlignBottom = nullptr;
	QToolButton* m_btnAlignProj = nullptr;
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
	QToolButton* m_btnDimCont = nullptr;
	QToolButton* m_btnDimBase = nullptr;
	QToolButton* m_btnNote = nullptr;
	QToolButton* m_btnHatch = nullptr;
	QToolButton* m_btnText = nullptr;
	QToolButton* m_btnMText = nullptr;
	QToolButton* m_btnMatch = nullptr;
	QToolButton* m_btnMove = nullptr;
	QToolButton* m_btnCopy = nullptr;
	QToolButton* m_btnRotate = nullptr;
	QToolButton* m_btnMirror = nullptr;
	QToolButton* m_btnErase = nullptr;
	QToolButton* m_btnTrim = nullptr;
	QToolButton* m_btnOffset = nullptr;
	QToolButton* m_btnScale = nullptr;
	QToolButton* m_btnFillet = nullptr;
	QToolButton* m_btnChamfer = nullptr;
	QToolButton* m_btnExtend = nullptr;
	QToolButton* m_btnArray = nullptr;
	QToolButton* m_btnPolarArray = nullptr;
	QToolButton* m_btnBreak = nullptr;
	QToolButton* m_btnJoin = nullptr;
	QToolButton* m_btnStretch = nullptr;
	QToolButton* m_btnRoughness = nullptr;
	QToolButton* m_btnGdt = nullptr;
	QToolButton* m_btnExplode = nullptr;
	QToolButton* m_btnProjGuide = nullptr;
	QCheckBox* m_snapEnd = nullptr;
	QCheckBox* m_snapMid = nullptr;
	QCheckBox* m_snapInt = nullptr;
	QCheckBox* m_snapCen = nullptr;
	QCheckBox* m_snapPerp = nullptr;
	QCheckBox* m_snapNear = nullptr;
	QCheckBox* m_snapPolar = nullptr;
	QCheckBox* m_orthoCheck = nullptr;

	QPushButton* m_generateBtn = nullptr;
	QComboBox* m_angleCombo = nullptr;
	QCheckBox* m_isoCheck = nullptr;
	QCheckBox* m_coarseViewCheck = nullptr;
	QCheckBox* m_sectionCheck = nullptr;
	QCheckBox* m_halfSectionCheck = nullptr;
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
	QLabel* m_ltScaleLabel = nullptr;
	QDoubleSpinBox* m_ltScaleSpin = nullptr;
	QPushButton* m_fitPaperBtn = nullptr;
	QLineEdit* m_titleEdit = nullptr;
	QDoubleSpinBox* m_detailScaleSpin = nullptr;

	QPushButton* m_svgBtn = nullptr;
	QPushButton* m_dxfBtn = nullptr;
	QPushButton* m_pdfBtn = nullptr;
	QPushButton* m_importDxfBtn = nullptr;
	QPushButton* m_printPreviewBtn = nullptr;
	QPushButton* m_blockBtn = nullptr;
	QPushButton* m_insertBlockBtn = nullptr;
	QPushButton* m_dimStyleBtn = nullptr;
	QPushButton* m_titleAttrBtn = nullptr;
	QCheckBox* m_ctbCheck = nullptr;
	QPushButton* m_ctbTableBtn = nullptr;
	QPushButton* m_recalcDimBtn = nullptr;
	QCheckBox* m_projDragLock = nullptr;
	QCheckBox* m_projPinned = nullptr;
	QCheckBox* m_projGuideVisible = nullptr;
};

#endif
