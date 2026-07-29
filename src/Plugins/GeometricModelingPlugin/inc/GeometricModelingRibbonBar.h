#ifndef GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGRIBBONBAR_H
#define GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGRIBBONBAR_H

/// @file GeometricModelingRibbonBar.h
/// @brief 几何建模模式 Ribbon：强分组、可勾选绘制/尺寸工具、主题跟随宿主

#include <QWidget>

class QToolButton;
class QButtonGroup;
class QLabel;

class GeometricModelingRibbonBar : public QWidget
{
	Q_OBJECT
public:
	explicit GeometricModelingRibbonBar(QWidget* parent = nullptr);

public slots:
	void applyTheme(bool dark);
	void applyLanguage(bool useChinese);

signals:
	void newSketchRequested();
	void datumPlaneRequested();
	void endSketchRequested();
	void lineToolRequested();
	void arcToolRequested();
	void circleToolRequested();
	void rectToolRequested();
	void ellipseToolRequested();
	void polygonToolRequested();
	void slotToolRequested();
	void splineToolRequested();
	void dimLengthRequested();
	void dimDistanceRequested();
	void dimRadiusRequested();
	void dimAngleRequested();
	void dimArcRadiusRequested();
	void constructionToolRequested();
	void geomHorizontalRequested();
	void geomVerticalRequested();
	void geomCoincidentRequested();
	void geomParallelRequested();
	void geomPerpendicularRequested();
	void geomEqualLengthRequested();
	void geomTangentRequested();
	void geomSymmetricRequested();
	void geomMidpointRequested();
	void geomFixRequested();
	void geomFixOriginRequested();
	void trimToolRequested();
	void mirrorToolRequested();
	void deleteToolRequested();
	void projectEdgesRequested();
	void convertEntitiesRequested();
	void offsetRequested();
	void solveRequested();
	void padRequested();
	void pocketRequested();
	void sweepRequested();
	void sweepCutRequested();
	void filletRequested();
	void chamferRequested();
	void revolveRequested();
	void revolveCutRequested();
	void linearPatternRequested();
	void mirror3dRequested();
	void loftRequested();
	void loftCutRequested();
	void shellRequested();
	void draftRequested();
	void rebuildRequested();
	void undoRequested();
	void redoRequested();

private:
	void rebuildIcons(bool dark);
	void clearToolChecks();
	void setBtnText(QToolButton* btn, const QString& text);

	bool m_dark = false;
	bool m_useChinese = true;
	QLabel* m_lblSketch = nullptr;
	QLabel* m_lblMarks = nullptr;
	QLabel* m_lblFeat = nullptr;
	QToolButton* m_btnNewSketch = nullptr;
	QToolButton* m_btnDatumPlane = nullptr;
	QToolButton* m_btnEndSketch = nullptr;
	QButtonGroup* m_drawTools = nullptr;
	QToolButton* m_btnLine = nullptr;
	QToolButton* m_btnArc = nullptr;
	QToolButton* m_btnCircle = nullptr;
	QToolButton* m_btnRect = nullptr;
	QToolButton* m_btnEllipse = nullptr;
	QToolButton* m_btnPolygon = nullptr;
	QToolButton* m_btnSlot = nullptr;
	QToolButton* m_btnSpline = nullptr;
	QToolButton* m_btnConstr = nullptr;
	QToolButton* m_btnProjectEdges = nullptr;
	QToolButton* m_btnConvertEntities = nullptr;
	QToolButton* m_btnOffset = nullptr;
	QToolButton* m_btnDimLen = nullptr;
	QToolButton* m_btnDimDist = nullptr;
	QToolButton* m_btnDimRad = nullptr;
	QToolButton* m_btnDimAng = nullptr;
	QToolButton* m_btnDimArcR = nullptr;
	QToolButton* m_btnGeomH = nullptr;
	QToolButton* m_btnGeomV = nullptr;
	QToolButton* m_btnGeomCoin = nullptr;
	QToolButton* m_btnGeomPar = nullptr;
	QToolButton* m_btnGeomPerp = nullptr;
	QToolButton* m_btnGeomEq = nullptr;
	QToolButton* m_btnGeomTan = nullptr;
	QToolButton* m_btnGeomSym = nullptr;
	QToolButton* m_btnGeomMid = nullptr;
	QToolButton* m_btnGeomFix = nullptr;
	QToolButton* m_btnGeomFixOrigin = nullptr;
	QToolButton* m_btnTrim = nullptr;
	QToolButton* m_btnMirror = nullptr;
	QToolButton* m_btnDelete = nullptr;
	QToolButton* m_btnSolve = nullptr;
	QToolButton* m_btnPad = nullptr;
	QToolButton* m_btnPocket = nullptr;
	QToolButton* m_btnSweep = nullptr;
	QToolButton* m_btnSweepCut = nullptr;
	QToolButton* m_btnFillet = nullptr;
	QToolButton* m_btnChamfer = nullptr;
	QToolButton* m_btnRevolve = nullptr;
	QToolButton* m_btnRevolveCut = nullptr;
	QToolButton* m_btnPattern = nullptr;
	QToolButton* m_btnMirror3d = nullptr;
	QToolButton* m_btnLoft = nullptr;
	QToolButton* m_btnLoftCut = nullptr;
	QToolButton* m_btnShell = nullptr;
	QToolButton* m_btnDraft = nullptr;
	QToolButton* m_btnRebuild = nullptr;
	QToolButton* m_btnUndo = nullptr;
	QToolButton* m_btnRedo = nullptr;
};

#endif
