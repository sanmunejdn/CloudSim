#pragma once

#include "PluginPointCloudTypes.h"

#include <QWidget>

class QShowEvent;

class IPluginHostContext;
class IPluginPointCloudHost;
class IPluginDocument;
class QListWidget;
class QLabel;
class QProgressBar;
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QPushButton;
class QGroupBox;
class QTextEdit;
class QCheckBox;

class PointCloudDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit PointCloudDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void applyLanguage();
	void refreshDocumentLabel();
	void refreshPointCloudList();
	void refreshSelectionInfo();

	void triggerImport();
	void triggerVoxelDownsample();
	void triggerPoissonReconstruct();
	void triggerExportMesh();
	void triggerMeshSimplify();
	void triggerMeshSmoothLaplacian();
	void triggerSurfaceReconstruct();

protected:
	void showEvent(QShowEvent* event) override;

private slots:
	void onImportClicked();
	void onRefreshListClicked();
	void onSelectionChanged();
	void onVoxelDownsampleClicked();
	void onRandomDownsampleClicked();
	void onBoxCropClicked();
	void onSphereCropClicked();
	void onPolylineCropClicked();
	void onRemoveOutliersClicked();
	void onSmoothClicked();
	void onNormalsPcaClicked();
	void onNormalsOrientClicked();
	void onIcpClicked();
	void onReconstructPoissonAutoClicked();
	void onReconstructScaleSpaceClicked();
	void onExportMeshClicked();
	void onCoarseRegisterScanToTemplateClicked();
	void onFineRegisterScanToTemplateClicked();
	void onUpdateTemplateBrepClicked();
	void onPickTemplateFaceClicked();
	void onClearSelectedFacesClicked();
	void onMeshSimplifyClicked();
	void onMeshSmoothLaplacianClicked();
	void onMeshSmoothImplicitClicked();
	void onMeshRepairClicked();
	void onMeshRemeshClicked();
	void onSurfaceReconstructClicked();
	void onSurfaceReconstructResetSessionClicked();

private:
	QString i18n(const QString& en, const QString& zh) const;
	QString formatInfo(const PluginPointCloudInfo& info, const PluginPointCloudMeasure* measure) const;
	void setBusy(bool busy);
	std::string selectedBackendId() const;
	std::string selectedMeshBackendId() const;
	std::string selectedMeshTargetId() const;
	void refreshMeshExportList(const std::string& preferBackendId = std::string());
	void refreshMeshInfo();
	void refreshSurfaceReconstructSummary(const PluginMeshSurfaceReconstructReport& report);
	PluginMeshSurfaceReconstructParams buildSurfaceReconParams() const;
	void ensureSurfaceReconSession();
	void resetSurfaceReconSessionUi();
	void updateSurfaceReconButtonStates();
	void appendSurfaceReconLog(const QString& line);
	void runSurfaceReconStage(PluginMeshSurfaceReconstructStage stage);
	IPluginDocument* activeDoc() const;
	IPluginPointCloudHost* pointCloudHost() const;
	void runFinished(bool ok, const QString& error, const PluginPointCloudJobResult& result);
	PluginPointCloudTemplateBrepUpdateParams buildTemplateBrepParams() const;
	void runTemplateBrepRegistration(PluginPointCloudTemplateBrepRegistrationStage stage);
	std::vector<int> selectedFaceIndices() const;
	void addSelectedFaceIndex(int faceIndex);

	IPluginHostContext* m_host = nullptr;
	bool m_useChinese = true;
	QGroupBox* m_docGroup = nullptr;
	QGroupBox* m_listGroup = nullptr;
	QGroupBox* m_infoGroup = nullptr;
	QGroupBox* m_downGroup = nullptr;
	QGroupBox* m_cropGroup = nullptr;
	QGroupBox* m_preGroup = nullptr;
	QGroupBox* m_icpGroup = nullptr;
	QGroupBox* m_reconGroup = nullptr;
	QLabel* m_docLabel = nullptr;
	QListWidget* m_list = nullptr;
	QLabel* m_infoLabel = nullptr;
	QLabel* m_voxelLabel = nullptr;
	QLabel* m_randomLabel = nullptr;
	QLabel* m_prefilterLabel = nullptr;
	QLabel* m_meshExportLabel = nullptr;
	QLabel* m_icpTargetLabel = nullptr;
	QDoubleSpinBox* m_voxelSpin = nullptr;
	QDoubleSpinBox* m_randomSpin = nullptr;
	QDoubleSpinBox* m_prefilterSpin = nullptr;
	QComboBox* m_icpTargetCombo = nullptr;
	QComboBox* m_meshExportCombo = nullptr;
	QPushButton* m_importBtn = nullptr;
	QPushButton* m_refreshBtn = nullptr;
	QPushButton* m_voxelBtn = nullptr;
	QPushButton* m_randomBtn = nullptr;
	QPushButton* m_boxCropBtn = nullptr;
	QPushButton* m_sphereCropBtn = nullptr;
	QPushButton* m_polylineCropBtn = nullptr;
	QComboBox* m_polylineCropModeCombo = nullptr;
	QPushButton* m_outlierBtn = nullptr;
	QPushButton* m_smoothBtn = nullptr;
	QPushButton* m_pcaBtn = nullptr;
	QPushButton* m_orientBtn = nullptr;
	QPushButton* m_icpBtn = nullptr;
	QPushButton* m_poissonBtn = nullptr;
	QPushButton* m_scaleBtn = nullptr;
	QPushButton* m_exportMeshBtn = nullptr;
	QGroupBox* m_reGroup = nullptr;
	QComboBox* m_templateBrepCombo = nullptr;
	QPushButton* m_pickFaceBtn = nullptr;
	QListWidget* m_selectedFacesList = nullptr;
	QPushButton* m_clearFacesBtn = nullptr;
	QPushButton* m_coarseMatchBtn = nullptr;
	QPushButton* m_fineMatchBtn = nullptr;
	QPushButton* m_refactorBtn = nullptr;
	QLabel* m_matchStatusLabel = nullptr;
	QDoubleSpinBox* m_faceBandSpin = nullptr;
	QDoubleSpinBox* m_reMinPointsSpin = nullptr;
	QDoubleSpinBox* m_reNormalSpin = nullptr;
	QDoubleSpinBox* m_reMaxDevSpin = nullptr;
	QLabel* m_templateBrepLabel = nullptr;
	QLabel* m_faceBandLabel = nullptr;
	QLabel* m_reMinPtsLabel = nullptr;
	QLabel* m_reNormalLabel = nullptr;
	QLabel* m_reMaxDevLabel = nullptr;
	QSpinBox* m_maxAssignPointsSpin = nullptr;
	QSpinBox* m_bsplineUvGridUSpin = nullptr;
	QSpinBox* m_bsplineUvGridVSpin = nullptr;
	QSpinBox* m_bsplinePoleSmoothSpin = nullptr;
	QLabel* m_maxAssignPointsLabel = nullptr;
	QLabel* m_bsplineUvGridLabel = nullptr;
	QLabel* m_bsplinePoleSmoothLabel = nullptr;
	QProgressBar* m_progress = nullptr;
	QLabel* m_statusLabel = nullptr;
	QWidget* m_scrollContent = nullptr;
	bool m_busy = false;

	// 网格后处理 UI
	QGroupBox* m_meshPostGroup = nullptr;
	QComboBox* m_meshTargetCombo = nullptr;
	QLabel* m_meshTargetLabel = nullptr;
	QLabel* m_simplifyTargetLabel = nullptr;
	QSpinBox* m_simplifyTargetSpin = nullptr;
	QLabel* m_simplifyQualityLabel = nullptr;
	QDoubleSpinBox* m_simplifyQualitySpin = nullptr;
	QPushButton* m_simplifyBtn = nullptr;
	QLabel* m_smoothIterLabel = nullptr;
	QSpinBox* m_smoothIterSpin = nullptr;
	QPushButton* m_smoothLaplacianBtn = nullptr;
	QPushButton* m_smoothImplicitBtn = nullptr;
	QPushButton* m_repairBtn = nullptr;
	QLabel* m_remeshEdgeLabel = nullptr;
	QDoubleSpinBox* m_remeshEdgeSpin = nullptr;
	QPushButton* m_remeshBtn = nullptr;
	QLabel* m_meshInfoLabel = nullptr;

	// 曲面重构 UI
	QGroupBox* m_surfaceReconGroup = nullptr;
	QLabel* m_normalSmoothIterLabel = nullptr;
	QSpinBox* m_normalSmoothIterSpin = nullptr;
	QLabel* m_featureThresholdLabel = nullptr;
	QDoubleSpinBox* m_featureThresholdSpin = nullptr;
	QLabel* m_surfaceReconPreprocessSectionLabel = nullptr;
	QLabel* m_surfaceReconPartitionSectionLabel = nullptr;
	QLabel* m_surfaceReconSampleSectionLabel = nullptr;
	QLabel* m_surfaceReconFitSectionLabel = nullptr;
	QLabel* m_surfaceReconBlendSectionLabel = nullptr;
	QLabel* m_patchCountLabel = nullptr;
	QSpinBox* m_patchCountSpin = nullptr;
	QLabel* m_partitionNormalSmoothLabel = nullptr;
	QSpinBox* m_partitionNormalSmoothSpin = nullptr;
	QLabel* m_featureAnglePercentileLabel = nullptr;
	QDoubleSpinBox* m_featureAnglePercentileSpin = nullptr;
	QLabel* m_samplesPerEdgeLabel = nullptr;
	QSpinBox* m_samplesPerEdgeSpin = nullptr;
	QLabel* m_uvSpacingLabel = nullptr;
	QDoubleSpinBox* m_uvSpacingSpin = nullptr;
	QLabel* m_minSamplesLabel = nullptr;
	QSpinBox* m_minSamplesSpin = nullptr;
	QLabel* m_maxSamplesLabel = nullptr;
	QSpinBox* m_maxSamplesSpin = nullptr;
	QLabel* m_maxFitGridLabel = nullptr;
	QSpinBox* m_maxFitGridSpin = nullptr;
	QLabel* m_fitUvSpacingLabel = nullptr;
	QDoubleSpinBox* m_fitUvSpacingSpin = nullptr;
	QLabel* m_sampleRateLabel = nullptr;
	QDoubleSpinBox* m_sampleRateSpin = nullptr;
	QLabel* m_ctrlPtDensityLabel = nullptr;
	QDoubleSpinBox* m_ctrlPtDensitySpin = nullptr;
	QLabel* m_nurbsFitModeLabel = nullptr;
	QComboBox* m_nurbsFitModeCombo = nullptr;
	QLabel* m_fairingEpsilonLabel = nullptr;
	QDoubleSpinBox* m_fairingEpsilonSpin = nullptr;
	QLabel* m_fairingMaxIterLabel = nullptr;
	QSpinBox* m_fairingMaxIterSpin = nullptr;
	QCheckBox* m_runVcgRepairCheck = nullptr;
	QCheckBox* m_runIsotropicRemeshCheck = nullptr;
	QLabel* m_remeshTargetEdgeLabel = nullptr;
	QDoubleSpinBox* m_remeshTargetEdgeSpin = nullptr;
	QLabel* m_remeshIterLabel = nullptr;
	QSpinBox* m_remeshIterSpin = nullptr;
	QLabel* m_blendStripWidthLabel = nullptr;
	QDoubleSpinBox* m_blendStripWidthSpin = nullptr;
	QLabel* m_tessellateDeflectionLabel = nullptr;
	QDoubleSpinBox* m_tessellateDeflectionSpin = nullptr;
	QPushButton* m_surfaceReconPreprocessBtn = nullptr;
	QPushButton* m_surfaceReconPartitionBtn = nullptr;
	QPushButton* m_surfaceReconSampleBtn = nullptr;
	QPushButton* m_surfaceReconFitBtn = nullptr;
	QPushButton* m_surfaceReconBoundaryBtn = nullptr;
	QPushButton* m_surfaceReconJunctionBtn = nullptr;
	QPushButton* m_surfaceReconFairBtn = nullptr;
	QPushButton* m_surfaceReconAssembleBtn = nullptr;
	QPushButton* m_surfaceReconBtn = nullptr;
	QPushButton* m_surfaceReconResetBtn = nullptr;
	QCheckBox* m_exportPreprocessedMeshCheck = nullptr;
	QTextEdit* m_surfaceReconLog = nullptr;
	QLabel* m_surfaceReconSummaryLabel = nullptr;
	PluginMeshSurfaceReconstructSessionId m_surfaceReconSessionId;
	PluginMeshSurfaceReconstructStage m_surfaceReconLastStage = PluginMeshSurfaceReconstructStage::None;
	std::string m_surfaceReconMeshBackendId;
};
