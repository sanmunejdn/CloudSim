#include "PointCloudDockWidget.h"

#include "IPluginDocument.h"
#include "IPluginHostContext.h"
#include "IPluginPointCloudHost.h"
#include "PluginPointCloudTypes.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

PointCloudDockWidget::PointCloudDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
	, m_useChinese(host ? host->useChinese() : true)
{
	auto* outer = new QVBoxLayout(this);
	outer->setContentsMargins(0, 0, 0, 0);
	auto* scroll = new QScrollArea(this);
	scroll->setWidgetResizable(true);
	m_scrollContent = new QWidget(scroll);
	auto* layout = new QVBoxLayout(m_scrollContent);
	layout->setContentsMargins(12, 12, 12, 12);
	layout->setSpacing(8);
	scroll->setWidget(m_scrollContent);
	outer->addWidget(scroll);

	m_docGroup = new QGroupBox(m_scrollContent);
	auto* docLayout = new QVBoxLayout(m_docGroup);
	m_docLabel = new QLabel(m_docGroup);
	docLayout->addWidget(m_docLabel);
	auto* importRow = new QHBoxLayout;
	m_importBtn = new QPushButton(m_docGroup);
	m_refreshBtn = new QPushButton(m_docGroup);
	importRow->addWidget(m_importBtn);
	importRow->addWidget(m_refreshBtn);
	docLayout->addLayout(importRow);
	layout->addWidget(m_docGroup);

	m_listGroup = new QGroupBox(m_scrollContent);
	auto* listLayout = new QVBoxLayout(m_listGroup);
	m_list = new QListWidget(m_listGroup);
	listLayout->addWidget(m_list);
	layout->addWidget(m_listGroup);

	m_infoGroup = new QGroupBox(m_scrollContent);
	auto* infoLayout = new QVBoxLayout(m_infoGroup);
	m_infoLabel = new QLabel(m_infoGroup);
	m_infoLabel->setWordWrap(true);
	infoLayout->addWidget(m_infoLabel);
	layout->addWidget(m_infoGroup);

	m_downGroup = new QGroupBox(m_scrollContent);
	auto* downLayout = new QVBoxLayout(m_downGroup);
	auto* voxelRow = new QHBoxLayout;
	m_voxelLabel = new QLabel(m_downGroup);
	m_voxelSpin = new QDoubleSpinBox(m_downGroup);
	m_voxelSpin->setRange(0.01, 1000.0);
	m_voxelSpin->setValue(2.0);
	m_voxelSpin->setDecimals(2);
	m_voxelBtn = new QPushButton(m_downGroup);
	voxelRow->addWidget(m_voxelLabel);
	voxelRow->addWidget(m_voxelSpin);
	voxelRow->addWidget(m_voxelBtn);
	downLayout->addLayout(voxelRow);
	auto* randomRow = new QHBoxLayout;
	m_randomLabel = new QLabel(m_downGroup);
	m_randomSpin = new QDoubleSpinBox(m_downGroup);
	m_randomSpin->setRange(0.01, 1.0);
	m_randomSpin->setValue(0.5);
	m_randomSpin->setDecimals(2);
	m_randomBtn = new QPushButton(m_downGroup);
	randomRow->addWidget(m_randomLabel);
	randomRow->addWidget(m_randomSpin);
	randomRow->addWidget(m_randomBtn);
	downLayout->addLayout(randomRow);
	layout->addWidget(m_downGroup);

	m_cropGroup = new QGroupBox(m_scrollContent);
	auto* cropLayout = new QHBoxLayout(m_cropGroup);
	m_boxCropBtn = new QPushButton(m_cropGroup);
	m_sphereCropBtn = new QPushButton(m_cropGroup);
	cropLayout->addWidget(m_boxCropBtn);
	cropLayout->addWidget(m_sphereCropBtn);
	layout->addWidget(m_cropGroup);

	m_preGroup = new QGroupBox(m_scrollContent);
	auto* preLayout = new QHBoxLayout(m_preGroup);
	m_outlierBtn = new QPushButton(m_preGroup);
	m_smoothBtn = new QPushButton(m_preGroup);
	m_pcaBtn = new QPushButton(m_preGroup);
	m_orientBtn = new QPushButton(m_preGroup);
	preLayout->addWidget(m_outlierBtn);
	preLayout->addWidget(m_smoothBtn);
	preLayout->addWidget(m_pcaBtn);
	preLayout->addWidget(m_orientBtn);
	layout->addWidget(m_preGroup);

	m_icpGroup = new QGroupBox(m_scrollContent);
	auto* icpLayout = new QVBoxLayout(m_icpGroup);
	auto* icpRow = new QHBoxLayout;
	m_icpTargetLabel = new QLabel(m_icpGroup);
	m_icpTargetCombo = new QComboBox(m_icpGroup);
	m_icpBtn = new QPushButton(m_icpGroup);
	icpRow->addWidget(m_icpTargetLabel);
	icpRow->addWidget(m_icpTargetCombo, 1);
	icpRow->addWidget(m_icpBtn);
	icpLayout->addLayout(icpRow);
	layout->addWidget(m_icpGroup);

	m_reconGroup = new QGroupBox(m_scrollContent);
	auto* reconLayout = new QVBoxLayout(m_reconGroup);
	auto* prefilterRow = new QHBoxLayout;
	m_prefilterLabel = new QLabel(m_reconGroup);
	m_prefilterSpin = new QDoubleSpinBox(m_reconGroup);
	m_prefilterSpin->setRange(0.0, 100.0);
	m_prefilterSpin->setValue(1.0);
	m_prefilterSpin->setDecimals(2);
	m_poissonBtn = new QPushButton(m_reconGroup);
	m_scaleBtn = new QPushButton(m_reconGroup);
	prefilterRow->addWidget(m_prefilterLabel);
	prefilterRow->addWidget(m_prefilterSpin);
	reconLayout->addLayout(prefilterRow);
	auto* reconRow = new QHBoxLayout;
	reconRow->addWidget(m_poissonBtn);
	reconRow->addWidget(m_scaleBtn);
	reconLayout->addLayout(reconRow);
	auto* exportRow = new QHBoxLayout;
	m_meshExportLabel = new QLabel(m_reconGroup);
	m_meshExportCombo = new QComboBox(m_reconGroup);
	m_exportMeshBtn = new QPushButton(m_reconGroup);
	exportRow->addWidget(m_meshExportLabel);
	exportRow->addWidget(m_meshExportCombo, 1);
	exportRow->addWidget(m_exportMeshBtn);
	reconLayout->addLayout(exportRow);
	layout->addWidget(m_reconGroup);

	m_statusLabel = new QLabel(m_scrollContent);
	layout->addWidget(m_statusLabel);
	m_progress = new QProgressBar(m_scrollContent);
	m_progress->setRange(0, 100);
	m_progress->setValue(0);
	layout->addWidget(m_progress);
	layout->addStretch(1);

	connect(m_importBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onImportClicked);
	connect(m_refreshBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onRefreshListClicked);
	connect(m_list, &QListWidget::currentRowChanged, this, &PointCloudDockWidget::onSelectionChanged);
	connect(m_voxelBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onVoxelDownsampleClicked);
	connect(m_randomBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onRandomDownsampleClicked);
	connect(m_boxCropBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onBoxCropClicked);
	connect(m_sphereCropBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onSphereCropClicked);
	connect(m_outlierBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onRemoveOutliersClicked);
	connect(m_smoothBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onSmoothClicked);
	connect(m_pcaBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onNormalsPcaClicked);
	connect(m_orientBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onNormalsOrientClicked);
	connect(m_icpBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onIcpClicked);
	connect(m_poissonBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onReconstructPoissonAutoClicked);
	connect(m_scaleBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onReconstructScaleSpaceClicked);
	connect(m_exportMeshBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onExportMeshClicked);

	applyLanguage();
	refreshDocumentLabel();
	refreshPointCloudList();
}

QString PointCloudDockWidget::i18n(const QString& en, const QString& zh) const
{
	return m_useChinese ? zh : en;
}

void PointCloudDockWidget::applyLanguage()
{
	if (m_host)
	{
		m_useChinese = m_host->useChinese();
	}
	m_docGroup->setTitle(i18n(QStringLiteral("Document && Import"), QStringLiteral("文档与导入")));
	m_listGroup->setTitle(i18n(QStringLiteral("Point clouds"), QStringLiteral("点云对象")));
	m_infoGroup->setTitle(i18n(QStringLiteral("Selection info"), QStringLiteral("选中对象信息")));
	m_downGroup->setTitle(i18n(QStringLiteral("Downsample"), QStringLiteral("下采样")));
	m_cropGroup->setTitle(i18n(QStringLiteral("Crop"), QStringLiteral("裁剪")));
	m_preGroup->setTitle(i18n(QStringLiteral("Preprocess"), QStringLiteral("预处理")));
	m_icpGroup->setTitle(i18n(QStringLiteral("Registration"), QStringLiteral("配准")));
	m_reconGroup->setTitle(i18n(QStringLiteral("Reconstruct mesh"), QStringLiteral("重建网格")));
	m_importBtn->setText(i18n(QStringLiteral("Import PLY/XYZ..."), QStringLiteral("导入 PLY/XYZ…")));
	m_refreshBtn->setText(i18n(QStringLiteral("Refresh list"), QStringLiteral("刷新列表")));
	m_voxelLabel->setText(i18n(QStringLiteral("Voxel (mm):"), QStringLiteral("体素(mm):")));
	m_randomLabel->setText(i18n(QStringLiteral("Retain ratio:"), QStringLiteral("保留比:")));
	m_prefilterLabel->setText(i18n(QStringLiteral("Prefilter voxel (mm):"), QStringLiteral("预滤波体素(mm):")));
	m_icpTargetLabel->setText(i18n(QStringLiteral("Target:"), QStringLiteral("目标点云:")));
	m_voxelBtn->setText(i18n(QStringLiteral("Voxel downsample"), QStringLiteral("体素下采样")));
	m_randomBtn->setText(i18n(QStringLiteral("Random downsample"), QStringLiteral("随机下采样")));
	m_boxCropBtn->setText(i18n(QStringLiteral("Crop to bbox"), QStringLiteral("按包围盒裁剪")));
	m_sphereCropBtn->setText(i18n(QStringLiteral("Crop sphere (r=50)"), QStringLiteral("球裁剪 (r=50)")));
	m_outlierBtn->setText(i18n(QStringLiteral("Remove outliers"), QStringLiteral("离群移除")));
	m_smoothBtn->setText(i18n(QStringLiteral("Bilateral smooth"), QStringLiteral("双边平滑")));
	m_pcaBtn->setText(i18n(QStringLiteral("Normals PCA"), QStringLiteral("法线 PCA")));
	m_orientBtn->setText(i18n(QStringLiteral("Orient MST"), QStringLiteral("MST 定向")));
	m_icpBtn->setText(i18n(QStringLiteral("ICP register"), QStringLiteral("ICP 配准")));
	m_poissonBtn->setText(i18n(QStringLiteral("Poisson Auto"), QStringLiteral("Poisson Auto")));
	m_scaleBtn->setText(i18n(QStringLiteral("Scale-space"), QStringLiteral("Scale-space")));
	m_meshExportLabel->setText(i18n(QStringLiteral("Mesh:"), QStringLiteral("网格:")));
	m_exportMeshBtn->setText(i18n(QStringLiteral("Export PLY..."), QStringLiteral("导出 PLY…")));
	if (!m_busy)
	{
		m_statusLabel->setText(i18n(QStringLiteral("Ready"), QStringLiteral("就绪")));
	}
	refreshDocumentLabel();
	refreshPointCloudList();
}

QString PointCloudDockWidget::formatInfo(
	const PluginPointCloudInfo& info,
	const PluginPointCloudMeasure* measure) const
{
	QString text = i18n(QStringLiteral("Points: %1"), QStringLiteral("点数: %1"))
					   .arg(static_cast<qulonglong>(info.pointCount));
	if (info.bounds.valid)
	{
		text += i18n(QStringLiteral("\nBBox min: (%1, %2, %3)"), QStringLiteral("\n包围盒 min: (%1, %2, %3)"))
					.arg(info.bounds.minMm.x, 0, 'f', 2)
					.arg(info.bounds.minMm.y, 0, 'f', 2)
					.arg(info.bounds.minMm.z, 0, 'f', 2);
		text += i18n(QStringLiteral("\nBBox max: (%1, %2, %3)"), QStringLiteral("\n包围盒 max: (%1, %2, %3)"))
					.arg(info.bounds.maxMm.x, 0, 'f', 2)
					.arg(info.bounds.maxMm.y, 0, 'f', 2)
					.arg(info.bounds.maxMm.z, 0, 'f', 2);
	}
	text += info.hasPerVertexColors
		? i18n(QStringLiteral("\nVertex colors: yes"), QStringLiteral("\n顶点色: 有"))
		: i18n(QStringLiteral("\nVertex colors: no"), QStringLiteral("\n顶点色: 无"));
	text += info.hasPointNormals
		? i18n(QStringLiteral("\nNormals: yes"), QStringLiteral("\n法线: 有"))
		: i18n(QStringLiteral("\nNormals: no"), QStringLiteral("\n法线: 无"));
	if (measure)
	{
		text += i18n(QStringLiteral("\nCentroid: (%1, %2, %3) mm"), QStringLiteral("\n质心: (%1, %2, %3) mm"))
					.arg(measure->centroidMm.x, 0, 'f', 2)
					.arg(measure->centroidMm.y, 0, 'f', 2)
					.arg(measure->centroidMm.z, 0, 'f', 2);
		text += i18n(QStringLiteral("\nAvg spacing: %1 mm"), QStringLiteral("\n平均间距: %1 mm"))
					.arg(measure->averageSpacingMm, 0, 'f', 3);
	}
	return text;
}

void PointCloudDockWidget::refreshDocumentLabel()
{
	if (!m_docLabel || !m_host)
	{
		return;
	}
	if (const IPluginDocument* doc = m_host->activeDocument())
	{
		m_docLabel->setText(i18n(QStringLiteral("Active document: %1"), QStringLiteral("活动文档: %1"))
							  .arg(QString::fromStdString(doc->documentLabel())));
	}
	else
	{
		m_docLabel->setText(i18n(QStringLiteral("No active document"), QStringLiteral("无活动文档")));
	}
}

void PointCloudDockWidget::refreshPointCloudList()
{
	if (!m_list || !m_host)
	{
		return;
	}
	const std::string prev = selectedBackendId();
	m_list->clear();
	m_icpTargetCombo->clear();
	IPluginDocument* doc = activeDoc();
	if (!doc)
	{
		refreshSelectionInfo();
		return;
	}
	for (const std::string& id : doc->backendIds())
	{
		if (doc->backendClassName(id) != "PointCloudBackendData")
		{
			continue;
		}
		PluginPointCloudInfo info;
		if (!doc->queryPointCloudInfo(id, info))
		{
			continue;
		}
		const QString label = i18n(QStringLiteral("%1 (%2 pts)"), QStringLiteral("%1 (%2 点)"))
								  .arg(QString::fromStdString(doc->backendDisplayName(id)))
								  .arg(static_cast<qulonglong>(info.pointCount));
		auto* item = new QListWidgetItem(label, m_list);
		item->setData(Qt::UserRole, QString::fromStdString(id));
		m_icpTargetCombo->addItem(label, QString::fromStdString(id));
	}
	for (int i = 0; i < m_list->count(); ++i)
	{
		if (m_list->item(i)->data(Qt::UserRole).toString().toStdString() == prev)
		{
			m_list->setCurrentRow(i);
			break;
		}
	}
	refreshSelectionInfo();
	refreshMeshExportList();
}

void PointCloudDockWidget::refreshSelectionInfo()
{
	if (!m_infoLabel)
	{
		return;
	}
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!doc || id.empty())
	{
		m_infoLabel->setText(i18n(QStringLiteral("Select a point cloud"), QStringLiteral("请选择点云")));
		return;
	}
	PluginPointCloudInfo info;
	PluginPointCloudMeasure measure;
	if (!doc->queryPointCloudInfo(id, info))
	{
		m_infoLabel->setText(i18n(QStringLiteral("Failed to query point cloud"), QStringLiteral("无法查询点云")));
		return;
	}
	const bool hasMeasure = doc->measurePointCloud(id, measure);
	m_infoLabel->setText(formatInfo(info, hasMeasure ? &measure : nullptr));
}

IPluginDocument* PointCloudDockWidget::activeDoc() const
{
	return m_host ? m_host->activeDocument() : nullptr;
}

IPluginPointCloudHost* PointCloudDockWidget::pointCloudHost() const
{
	return m_host ? m_host->pointCloudHost() : nullptr;
}

std::string PointCloudDockWidget::selectedBackendId() const
{
	if (!m_list || m_list->currentRow() < 0)
	{
		return std::string();
	}
	const QListWidgetItem* item = m_list->currentItem();
	return item ? item->data(Qt::UserRole).toString().toStdString() : std::string();
}

std::string PointCloudDockWidget::selectedMeshBackendId() const
{
	if (!m_meshExportCombo || m_meshExportCombo->currentIndex() < 0)
	{
		return std::string();
	}
	return m_meshExportCombo->currentData().toString().toStdString();
}

void PointCloudDockWidget::refreshMeshExportList(const std::string& preferBackendId)
{
	if (!m_meshExportCombo || !m_host)
	{
		return;
	}
	const std::string prev = preferBackendId.empty() ? selectedMeshBackendId() : preferBackendId;
	m_meshExportCombo->clear();
	IPluginDocument* doc = activeDoc();
	if (!doc)
	{
		return;
	}
	for (const std::string& id : doc->backendIds())
	{
		if (doc->backendClassName(id) != "Model")
		{
			continue;
		}
		const QString label = QString::fromStdString(doc->backendDisplayName(id));
		m_meshExportCombo->addItem(label, QString::fromStdString(id));
	}
	for (int i = 0; i < m_meshExportCombo->count(); ++i)
	{
		if (m_meshExportCombo->itemData(i).toString().toStdString() == prev)
		{
			m_meshExportCombo->setCurrentIndex(i);
			break;
		}
	}
}

void PointCloudDockWidget::setBusy(const bool busy)
{
	m_busy = busy;
	m_scrollContent->setEnabled(!busy);
	if (m_progress)
	{
		m_progress->setValue(busy ? 0 : 0);
	}
	if (m_statusLabel)
	{
		m_statusLabel->setText(
			busy ? i18n(QStringLiteral("Running..."), QStringLiteral("运行中…"))
				 : i18n(QStringLiteral("Ready"), QStringLiteral("就绪")));
	}
}

void PointCloudDockWidget::runFinished(const bool ok, const QString& error, const PluginPointCloudJobResult& result)
{
	setBusy(false);
	if (!m_host)
	{
		return;
	}
	if (ok)
	{
		QString msg = i18n(QStringLiteral("Done. Points: %1"), QStringLiteral("完成。点数: %1"))
						  .arg(static_cast<qulonglong>(result.pointCountAfter));
		if (!result.newBackendId.empty())
		{
			msg += i18n(QStringLiteral("; new object: %1"), QStringLiteral("；新对象: %1"))
					   .arg(QString::fromStdString(result.newBackendId));
		}
		if (result.rmseMm > 0.0)
		{
			msg += i18n(QStringLiteral("; RMSE: %1 mm"), QStringLiteral("；RMSE: %1 mm"))
					   .arg(result.rmseMm, 0, 'f', 3);
		}
		m_host->logInfo(msg);
		if (m_statusLabel)
		{
			m_statusLabel->setText(msg);
		}
		refreshPointCloudList();
		refreshMeshExportList(result.newBackendId);
	}
	else
	{
		m_host->logError(error.isEmpty()
			? i18n(QStringLiteral("Point cloud operation failed"), QStringLiteral("点云操作失败"))
			: error);
		if (m_statusLabel)
		{
			m_statusLabel->setText(error);
		}
	}
	if (m_progress)
	{
		m_progress->setValue(ok ? 100 : 0);
	}
}

void PointCloudDockWidget::triggerImport()
{
	onImportClicked();
}

void PointCloudDockWidget::triggerVoxelDownsample()
{
	onVoxelDownsampleClicked();
}

void PointCloudDockWidget::triggerPoissonReconstruct()
{
	onReconstructPoissonAutoClicked();
}

void PointCloudDockWidget::triggerExportMesh()
{
	onExportMeshClicked();
}

void PointCloudDockWidget::onImportClicked()
{
	if (!m_host)
	{
		return;
	}
	const QString path = QFileDialog::getOpenFileName(
		this,
		i18n(QStringLiteral("Import point cloud"), QStringLiteral("导入点云")),
		QString(),
		i18n(QStringLiteral("Point clouds (*.ply *.xyz);;All files (*.*)"),
			QStringLiteral("点云 (*.ply *.xyz);;所有文件 (*.*)")));
	if (path.isEmpty())
	{
		return;
	}
	std::string err;
	const std::string id = m_host->importFileIntoActiveDocument(path.toUtf8().constData(), true, &err);
	if (id.empty())
	{
		m_host->logError(QString::fromStdString(err));
		return;
	}
	m_host->logInfo(i18n(QStringLiteral("Imported point cloud: %1"), QStringLiteral("已导入点云: %1"))
						.arg(QString::fromStdString(id)));
	refreshPointCloudList();
	for (int i = 0; i < m_list->count(); ++i)
	{
		if (m_list->item(i)->data(Qt::UserRole).toString().toStdString() == id)
		{
			m_list->setCurrentRow(i);
			break;
		}
	}
}

void PointCloudDockWidget::onRefreshListClicked()
{
	refreshDocumentLabel();
	refreshPointCloudList();
}

void PointCloudDockWidget::onSelectionChanged()
{
	refreshSelectionInfo();
}

void PointCloudDockWidget::onVoxelDownsampleClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudDownsampleVoxelParams params;
	params.voxelSizeMm = m_voxelSpin->value();
	pch->downsamplePointCloudVoxel(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onRandomDownsampleClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudDownsampleRandomParams params;
	params.retainedFraction = m_randomSpin->value();
	pch->downsamplePointCloudRandom(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onBoxCropClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	PluginPointCloudInfo info;
	if (!doc->queryPointCloudInfo(id, info) || !info.bounds.valid)
	{
		m_host->logWarn(i18n(QStringLiteral("No valid bounding box for crop"), QStringLiteral("无有效包围盒，无法裁剪")));
		return;
	}
	setBusy(true);
	PluginPointCloudCropBoxParams params;
	params.box = info.bounds;
	pch->cropPointCloudByBox(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onSphereCropClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	PluginPointCloudMeasure measure;
	if (!doc->measurePointCloud(id, measure))
	{
		return;
	}
	setBusy(true);
	PluginPointCloudCropSphereParams params;
	params.centerMm = measure.centroidMm;
	params.radiusMm = 50.0;
	pch->cropPointCloudBySphere(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onRemoveOutliersClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudOutlierParams params;
	pch->removePointCloudOutliers(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onSmoothClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	pch->smoothPointCloudBilateral(
		doc,
		id,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onNormalsPcaClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudNormalsParams params;
	pch->estimatePointCloudNormalsPca(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onNormalsOrientClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudNormalsParams params;
	pch->orientPointCloudNormalsMst(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onIcpClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string sourceId = selectedBackendId();
	if (!pch || !doc || sourceId.empty() || m_icpTargetCombo->currentIndex() < 0)
	{
		return;
	}
	const std::string targetId = m_icpTargetCombo->currentData().toString().toStdString();
	if (targetId.empty() || targetId == sourceId)
	{
		m_host->logWarn(
			i18n(QStringLiteral("Select a different target point cloud for ICP"),
				QStringLiteral("请为 ICP 选择不同的目标点云")));
		return;
	}
	setBusy(true);
	PluginPointCloudIcpParams params;
	params.targetBackendIdUtf8 = targetId;
	params.applyTransformToSource = true;
	pch->rigidRegisterPointCloudsIcp(
		doc,
		sourceId,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onReconstructPoissonAutoClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudReconstructPoissonAutoParams params;
	params.voxelPrefilterMm = m_prefilterSpin->value();
	params.meshOptions.displayName = i18n(QStringLiteral("PoissonMesh"), QStringLiteral("Poisson网格"));
	params.meshOptions.selectInTree = true;
	pch->reconstructMeshPoissonAuto(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onReconstructScaleSpaceClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginPointCloudReconstructScaleSpaceParams params;
	params.meshOptions.displayName = i18n(QStringLiteral("ScaleSpaceMesh"), QStringLiteral("ScaleSpace网格"));
	params.meshOptions.selectInTree = true;
	pch->reconstructMeshScaleSpace(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
		});
}

void PointCloudDockWidget::onExportMeshClicked()
{
	if (!m_host)
	{
		return;
	}
	IPluginDocument* doc = activeDoc();
	const std::string meshId = selectedMeshBackendId();
	if (!doc || meshId.empty())
	{
		m_host->logWarn(i18n(QStringLiteral("No mesh to export"), QStringLiteral("没有可导出的网格")));
		return;
	}
	const QString defaultName =
		QString::fromStdString(doc->backendDisplayName(meshId)).replace(QLatin1Char(' '), QLatin1Char('_'))
		+ QStringLiteral(".ply");
	const QString path = QFileDialog::getSaveFileName(
		this,
		i18n(QStringLiteral("Export mesh PLY"), QStringLiteral("导出网格 PLY")),
		defaultName,
		i18n(QStringLiteral("Mesh PLY (*.ply);;All files (*.*)"), QStringLiteral("网格 PLY (*.ply);;所有文件 (*.*)")));
	if (path.isEmpty())
	{
		return;
	}
	std::string err;
	if (!doc->exportMeshToPly(meshId, path.toUtf8().toStdString(), &err))
	{
		m_host->logError(err.empty()
			? i18n(QStringLiteral("Failed to export mesh PLY"), QStringLiteral("导出网格 PLY 失败"))
			: QString::fromStdString(err));
		return;
	}
	m_host->logInfo(i18n(QStringLiteral("Mesh exported: %1"), QStringLiteral("网格已导出: %1")).arg(path));
	if (m_statusLabel)
	{
		m_statusLabel->setText(i18n(QStringLiteral("Mesh exported"), QStringLiteral("网格已导出")));
	}
}
