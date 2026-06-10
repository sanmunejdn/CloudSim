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
	void onRemoveOutliersClicked();
	void onSmoothClicked();
	void onNormalsPcaClicked();
	void onNormalsOrientClicked();
	void onIcpClicked();
	void onReconstructPoissonAutoClicked();
	void onReconstructScaleSpaceClicked();
	void onExportMeshClicked();
	void onRegisterScanToTemplateClicked();
	void onUpdateTemplateBrepClicked();
	void onPickTemplateFaceClicked();
	void onClearSelectedFacesClicked();
	void onMeshSimplifyClicked();
	void onMeshSmoothLaplacianClicked();
	void onMeshSmoothImplicitClicked();
	void onMeshRepairClicked();
	void onMeshRemeshClicked();

private:
	QString i18n(const QString& en, const QString& zh) const;
	QString formatInfo(const PluginPointCloudInfo& info, const PluginPointCloudMeasure* measure) const;
	void setBusy(bool busy);
	std::string selectedBackendId() const;
	std::string selectedMeshBackendId() const;
	std::string selectedMeshTargetId() const;
	void refreshMeshExportList(const std::string& preferBackendId = std::string());
	void refreshMeshInfo();
	IPluginDocument* activeDoc() const;
	IPluginPointCloudHost* pointCloudHost() const;
	void runFinished(bool ok, const QString& error, const PluginPointCloudJobResult& result);
	PluginPointCloudTemplateBrepUpdateParams buildTemplateBrepParams() const;
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
	QPushButton* m_matchBtn = nullptr;
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
};
