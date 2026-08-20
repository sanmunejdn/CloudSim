#ifndef POINTCLOUDPLUGIN_TUBULARGRINDINGDOCKWIDGET_H
#define POINTCLOUDPLUGIN_TUBULARGRINDINGDOCKWIDGET_H

/// @file TubularGrindingDockWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TubularGrindingDockWidget 接口

#include "PluginPointCloudTypes.h"

#include <QWidget>

class IPluginHostContext;
class IPluginDocument;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QSpinBox;
class QPushButton;
class QLabel;
class QTextEdit;
class QGroupBox;
class QTabWidget;
class QShowEvent;

class TubularGrindingDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit TubularGrindingDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void applyLanguage();
	void refreshMeshList();

protected:
	void showEvent(QShowEvent* event) override;

private slots:
	void onMeshSelectionChanged();
	void onCenterlineMethodChanged();
	void onResetSessionClicked();
	void onCenterlineClicked();
	void onTemplateClicked();
	void onProjectClicked();
	void onFpfhPartitionClicked();

private:
	QString i18n(const QString& en, const QString& zh) const;
	IPluginDocument* activeDoc() const;
	class IPluginPointCloudHost* pointCloudHost() const;
	std::string selectedMeshBackendId() const;
	bool selectedSourceIsPointCloud() const;
	PluginTubularGrindingCenterlineMethod selectedCenterlineMethod() const;

	PluginTubularGrindingParams buildParams() const;
	void appendLog(const QString& line);
	void resetSessionUi();
	void updateButtonStates();
	void ensureSession();
	void runStage(PluginTubularGrindingStage stage);
	void refreshSummary(const PluginTubularGrindingReport& report);
	void syncCenterlineMethodForSource();
	void updateCenterlineParamVisibility();

	void addParamRow(QFormLayout* form, QLabel*& labelOut, QWidget* editor, QLabel*& hintOut, const QString& labelText,
					 const QString& hintText, QWidget** outFieldWrap = nullptr);

	void setParamRowVisible(QLabel* label, QWidget* fieldWrap, bool visible);

	IPluginHostContext* m_host = nullptr;
	bool m_busy = false;

	QGroupBox* m_rootGroup = nullptr;
	QLabel* m_meshLabel = nullptr;
	QComboBox* m_meshCombo = nullptr;
	QTabWidget* m_paramTabs = nullptr;

	// 中心线提取
	QLabel* m_centerlineMethodLabel = nullptr;
	QComboBox* m_centerlineMethodCombo = nullptr;
	QLabel* m_centerlineMethodHint = nullptr;
	QWidget* m_centerlineMethodRow = nullptr;

	QLabel* m_sectionSpacingLabel = nullptr;
	QDoubleSpinBox* m_sectionSpacingSpin = nullptr;
	QLabel* m_sectionSpacingHint = nullptr;
	QWidget* m_sectionSpacingRow = nullptr;

	QLabel* m_centerlineIterLabel = nullptr;
	QSpinBox* m_centerlineIterSpin = nullptr;
	QLabel* m_centerlineIterHint = nullptr;
	QWidget* m_centerlineIterRow = nullptr;

	QLabel* m_laplacianLambdaLabel = nullptr;
	QDoubleSpinBox* m_laplacianLambdaSpin = nullptr;
	QLabel* m_laplacianLambdaHint = nullptr;
	QWidget* m_laplacianLambdaRow = nullptr;

	QLabel* m_laplacianAttractionLabel = nullptr;
	QDoubleSpinBox* m_laplacianAttractionSpin = nullptr;
	QLabel* m_laplacianAttractionHint = nullptr;
	QWidget* m_laplacianAttractionRow = nullptr;

	QLabel* m_otSampleRateLabel = nullptr;
	QDoubleSpinBox* m_otSampleRateSpin = nullptr;
	QLabel* m_otSampleRateHint = nullptr;
	QWidget* m_otSampleRateRow = nullptr;

	QLabel* m_otCostBetaLabel = nullptr;
	QDoubleSpinBox* m_otCostBetaSpin = nullptr;
	QLabel* m_otCostBetaHint = nullptr;
	QWidget* m_otCostBetaRow = nullptr;

	QLabel* m_otcPreStepsLabel = nullptr;
	QSpinBox* m_otcPreStepsSpin = nullptr;
	QLabel* m_otcPreStepsHint = nullptr;
	QWidget* m_otcPreStepsRow = nullptr;

	QLabel* m_otcOuterLoopsLabel = nullptr;
	QSpinBox* m_otcOuterLoopsSpin = nullptr;
	QLabel* m_otcOuterLoopsHint = nullptr;
	QWidget* m_otcOuterLoopsRow = nullptr;

	QLabel* m_otLcOuterMaxItersLabel = nullptr;
	QSpinBox* m_otLcOuterMaxItersSpin = nullptr;
	QLabel* m_otLcOuterMaxItersHint = nullptr;
	QWidget* m_otLcOuterMaxItersRow = nullptr;

	QLabel* m_minRootsLabel = nullptr;
	QSpinBox* m_minRootsSpin = nullptr;
	QLabel* m_minRootsHint = nullptr;
	QWidget* m_minRootsRow = nullptr;

	// 轨迹与投影
	QLabel* m_templateLabel = nullptr;
	QComboBox* m_templateCombo = nullptr;
	QLabel* m_templateHint = nullptr;
	QLabel* m_helicalCoilsLabel = nullptr;
	QSpinBox* m_helicalCoilsSpin = nullptr;
	QLabel* m_helicalCoilsHint = nullptr;
	QLabel* m_projectionDistLabel = nullptr;
	QDoubleSpinBox* m_projectionDistSpin = nullptr;
	QLabel* m_projectionDistHint = nullptr;

	// FPFH Mesh 区域划分
	QLabel* m_fpfhFeatureVoxelLabel = nullptr;
	QDoubleSpinBox* m_fpfhFeatureVoxelSpin = nullptr;
	QLabel* m_fpfhFeatureVoxelHint = nullptr;
	QLabel* m_fpfhMaxSamplePointsLabel = nullptr;
	QSpinBox* m_fpfhMaxSamplePointsSpin = nullptr;
	QLabel* m_fpfhMaxSamplePointsHint = nullptr;
	QLabel* m_fpfhNeighborsLabel = nullptr;
	QSpinBox* m_fpfhNeighborsSpin = nullptr;
	QLabel* m_fpfhNeighborsHint = nullptr;
	QLabel* m_fpfhSaliencyNeighborsLabel = nullptr;
	QSpinBox* m_fpfhSaliencyNeighborsSpin = nullptr;
	QLabel* m_fpfhSaliencyNeighborsHint = nullptr;
	QLabel* m_fpfhKeypointCountLabel = nullptr;
	QSpinBox* m_fpfhKeypointCountSpin = nullptr;
	QLabel* m_fpfhKeypointCountHint = nullptr;
	QLabel* m_fpfhKeypointMinSepLabel = nullptr;
	QDoubleSpinBox* m_fpfhKeypointMinSepSpin = nullptr;
	QLabel* m_fpfhKeypointMinSepHint = nullptr;
	QLabel* m_fpfhRegionGrowDistLabel = nullptr;
	QDoubleSpinBox* m_fpfhRegionGrowDistSpin = nullptr;
	QLabel* m_fpfhRegionGrowDistHint = nullptr;
	QLabel* m_fpfhRegionGrowAngleLabel = nullptr;
	QDoubleSpinBox* m_fpfhRegionGrowAngleSpin = nullptr;
	QLabel* m_fpfhRegionGrowAngleHint = nullptr;
	QLabel* m_fpfhMinRegionFacesLabel = nullptr;
	QSpinBox* m_fpfhMinRegionFacesSpin = nullptr;
	QLabel* m_fpfhMinRegionFacesHint = nullptr;
	QPushButton* m_fpfhPartitionBtn = nullptr;

	QPushButton* m_centerlineBtn = nullptr;
	QPushButton* m_templateBtn = nullptr;
	QPushButton* m_projectBtn = nullptr;
	QPushButton* m_resetBtn = nullptr;
	QTextEdit* m_log = nullptr;
	QLabel* m_summaryLabel = nullptr;

	PluginTubularGrindingSessionId m_sessionId;
	PluginTubularGrindingStage m_lastStage = PluginTubularGrindingStage::None;
	std::string m_meshBackendId;
};

#endif // POINTCLOUDPLUGIN_TUBULARGRINDINGDOCKWIDGET_H
