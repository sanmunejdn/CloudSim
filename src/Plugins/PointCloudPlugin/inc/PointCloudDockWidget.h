#pragma once

#include "PluginPointCloudTypes.h"

#include <QWidget>

class IPluginHostContext;
class IPluginPointCloudHost;
class IPluginDocument;
class QListWidget;
class QLabel;
class QProgressBar;
class QDoubleSpinBox;
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

private:
	QString i18n(const QString& en, const QString& zh) const;
	QString formatInfo(const PluginPointCloudInfo& info, const PluginPointCloudMeasure* measure) const;
	void setBusy(bool busy);
	std::string selectedBackendId() const;
	std::string selectedMeshBackendId() const;
	void refreshMeshExportList(const std::string& preferBackendId = std::string());
	IPluginDocument* activeDoc() const;
	IPluginPointCloudHost* pointCloudHost() const;
	void runFinished(bool ok, const QString& error, const PluginPointCloudJobResult& result);

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
	QProgressBar* m_progress = nullptr;
	QLabel* m_statusLabel = nullptr;
	QWidget* m_scrollContent = nullptr;
	bool m_busy = false;
};
