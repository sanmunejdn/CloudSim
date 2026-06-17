#include "PointCloudDockWidget.h"

#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "IPluginPointCloudHost.h"
#include "PluginGeometryTypes.h"
#include "PluginPointCloudTypes.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QSpinBox>
#include <QSizePolicy>
#include <QVBoxLayout>

namespace
{
bool isInactiveFaceAction(const std::string& action)
{
	return action == "Unchanged" || action == "SkippedNoPoints";
}

void logBrepUpdateSkippedFaceDiagnostic(
	IPluginHostContext* host,
	const std::vector<PluginPointCloudFaceUpdateReport>& perFace)
{
	if (!host)
	{
		return;
	}
	constexpr int kMaxSkippedLines = 16;
	int logged = 0;
	for (const PluginPointCloudFaceUpdateReport& faceReport : perFace)
	{
		if (faceReport.action != "SkippedNoPoints")
		{
			continue;
		}
		host->logInfo(
			QStringLiteral("  F%1 SkippedNoPoints (%2)")
				.arg(faceReport.faceIndex)
				.arg(QString::fromStdString(faceReport.surfaceTypeName)));
		if (++logged >= kMaxSkippedLines)
		{
			break;
		}
	}
}

void logBrepUpdateFaceSummary(
	IPluginHostContext* host,
	const std::vector<PluginPointCloudFaceUpdateReport>& perFace)
{
	if (!host)
	{
		return;
	}
	constexpr int kMaxDetailLines = 25;
	int logged = 0;
	std::size_t omitted = 0U;
	for (const PluginPointCloudFaceUpdateReport& faceReport : perFace)
	{
		if (isInactiveFaceAction(faceReport.action))
		{
			++omitted;
			continue;
		}
		if (logged >= kMaxDetailLines)
		{
			break;
		}
		host->logInfo(QStringLiteral("  F%1 %2 %3 maxDev=%4mm")
						  .arg(faceReport.faceIndex)
						  .arg(QString::fromStdString(faceReport.surfaceTypeName))
						  .arg(QString::fromStdString(faceReport.action))
						  .arg(faceReport.maxDeviationMm, 0, 'f', 3));
		++logged;
	}
	if (omitted > 0U || logged < static_cast<int>(perFace.size()))
	{
		host->logInfo(QStringLiteral("  ... %1 face report(s) omitted from UI log")
						  .arg(static_cast<qulonglong>(perFace.size() - static_cast<std::size_t>(logged))));
	}
}

QString truncateStatusText(const QString& text, const int maxChars = 120)
{
	if (text.size() <= maxChars)
	{
		return text;
	}
	return text.left(maxChars - 3) + QStringLiteral("...");
}
} // namespace

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
	auto* cropLayout = new QVBoxLayout(m_cropGroup);
	auto* cropRow1 = new QHBoxLayout();
	m_boxCropBtn = new QPushButton(m_cropGroup);
	m_sphereCropBtn = new QPushButton(m_cropGroup);
	cropRow1->addWidget(m_boxCropBtn);
	cropRow1->addWidget(m_sphereCropBtn);
	cropLayout->addLayout(cropRow1);
	auto* cropRow2 = new QHBoxLayout();
	m_polylineCropModeCombo = new QComboBox(m_cropGroup);
	m_polylineCropBtn = new QPushButton(m_cropGroup);
	cropRow2->addWidget(m_polylineCropModeCombo, 1);
	cropRow2->addWidget(m_polylineCropBtn);
	cropLayout->addLayout(cropRow2);
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

	m_reGroup = new QGroupBox(m_scrollContent);
	auto* reLayout = new QVBoxLayout(m_reGroup);
	auto* templateRow = new QHBoxLayout;
	m_templateBrepLabel = new QLabel(m_reGroup);
	m_templateBrepCombo = new QComboBox(m_reGroup);
	templateRow->addWidget(m_templateBrepLabel);
	templateRow->addWidget(m_templateBrepCombo, 1);
	reLayout->addLayout(templateRow);
	auto* faceBandRow = new QHBoxLayout;
	m_faceBandLabel = new QLabel(m_reGroup);
	m_faceBandSpin = new QDoubleSpinBox(m_reGroup);
	m_faceBandSpin->setRange(0.1, 50.0);
	m_faceBandSpin->setValue(2.0);
	m_faceBandSpin->setDecimals(2);
	faceBandRow->addWidget(m_faceBandLabel);
	faceBandRow->addWidget(m_faceBandSpin);
	reLayout->addLayout(faceBandRow);
	auto* reNormalRow = new QHBoxLayout;
	m_reNormalLabel = new QLabel(m_reGroup);
	m_reNormalSpin = new QDoubleSpinBox(m_reGroup);
	m_reNormalSpin->setRange(1.0, 90.0);
	m_reNormalSpin->setValue(35.0);
	m_reNormalSpin->setDecimals(1);
	reNormalRow->addWidget(m_reNormalLabel);
	reNormalRow->addWidget(m_reNormalSpin);
	reLayout->addLayout(reNormalRow);
	auto* reMinPtsRow = new QHBoxLayout;
	m_reMinPtsLabel = new QLabel(m_reGroup);
	m_reMinPointsSpin = new QDoubleSpinBox(m_reGroup);
	m_reMinPointsSpin->setRange(10, 100000);
	m_reMinPointsSpin->setValue(30);
	m_reMinPointsSpin->setDecimals(0);
	reMinPtsRow->addWidget(m_reMinPtsLabel);
	reMinPtsRow->addWidget(m_reMinPointsSpin);
	reLayout->addLayout(reMinPtsRow);
	auto* reMaxDevRow = new QHBoxLayout;
	m_reMaxDevLabel = new QLabel(m_reGroup);
	m_reMaxDevSpin = new QDoubleSpinBox(m_reGroup);
	m_reMaxDevSpin->setRange(0.0, 100.0);
	m_reMaxDevSpin->setValue(0.5);
	m_reMaxDevSpin->setDecimals(3);
	reMaxDevRow->addWidget(m_reMaxDevLabel);
	reMaxDevRow->addWidget(m_reMaxDevSpin);
	reLayout->addLayout(reMaxDevRow);
	auto* bsplineGridRow = new QHBoxLayout;
	m_bsplineUvGridLabel = new QLabel(m_reGroup);
	m_bsplineUvGridUSpin = new QSpinBox(m_reGroup);
	m_bsplineUvGridUSpin->setRange(4, 128);
	m_bsplineUvGridUSpin->setValue(24);
	m_bsplineUvGridVSpin = new QSpinBox(m_reGroup);
	m_bsplineUvGridVSpin->setRange(4, 128);
	m_bsplineUvGridVSpin->setValue(12);
	bsplineGridRow->addWidget(m_bsplineUvGridLabel);
	bsplineGridRow->addWidget(m_bsplineUvGridUSpin);
	bsplineGridRow->addWidget(m_bsplineUvGridVSpin);
	reLayout->addLayout(bsplineGridRow);
	auto* maxAssignRow = new QHBoxLayout;
	m_maxAssignPointsLabel = new QLabel(m_reGroup);
	m_maxAssignPointsSpin = new QSpinBox(m_reGroup);
	m_maxAssignPointsSpin->setRange(100, 50000);
	m_maxAssignPointsSpin->setSingleStep(100);
	m_maxAssignPointsSpin->setValue(800);
	maxAssignRow->addWidget(m_maxAssignPointsLabel);
	maxAssignRow->addWidget(m_maxAssignPointsSpin);
	reLayout->addLayout(maxAssignRow);
	auto* poleSmoothRow = new QHBoxLayout;
	m_bsplinePoleSmoothLabel = new QLabel(m_reGroup);
	m_bsplinePoleSmoothSpin = new QSpinBox(m_reGroup);
	m_bsplinePoleSmoothSpin->setRange(0, 10);
	m_bsplinePoleSmoothSpin->setValue(2);
	poleSmoothRow->addWidget(m_bsplinePoleSmoothLabel);
	poleSmoothRow->addWidget(m_bsplinePoleSmoothSpin);
	reLayout->addLayout(poleSmoothRow);
	auto* facePickRow = new QHBoxLayout;
	m_pickFaceBtn = new QPushButton(m_reGroup);
	m_clearFacesBtn = new QPushButton(m_reGroup);
	facePickRow->addWidget(m_pickFaceBtn);
	facePickRow->addWidget(m_clearFacesBtn);
	reLayout->addLayout(facePickRow);
	m_selectedFacesList = new QListWidget(m_reGroup);
	m_selectedFacesList->setMaximumHeight(72);
	reLayout->addWidget(m_selectedFacesList);
	m_matchStatusLabel = new QLabel(m_reGroup);
	reLayout->addWidget(m_matchStatusLabel);
	auto* reBtnRow = new QHBoxLayout;
	m_coarseMatchBtn = new QPushButton(m_reGroup);
	m_fineMatchBtn = new QPushButton(m_reGroup);
	m_refactorBtn = new QPushButton(m_reGroup);
	reBtnRow->addWidget(m_coarseMatchBtn);
	reBtnRow->addWidget(m_fineMatchBtn);
	reBtnRow->addWidget(m_refactorBtn);
	reLayout->addLayout(reBtnRow);
	layout->addWidget(m_reGroup);

	// === 网格后处理组 ===
	m_meshPostGroup = new QGroupBox(m_scrollContent);
	auto* meshPostLayout = new QVBoxLayout(m_meshPostGroup);

	// 网格对象选择
	auto* meshTargetRow = new QHBoxLayout;
	m_meshTargetLabel = new QLabel(m_meshPostGroup);
	m_meshTargetCombo = new QComboBox(m_meshPostGroup);
	meshTargetRow->addWidget(m_meshTargetLabel);
	meshTargetRow->addWidget(m_meshTargetCombo, 1);
	meshPostLayout->addLayout(meshTargetRow);

	// 网格信息
	m_meshInfoLabel = new QLabel(m_meshPostGroup);
	m_meshInfoLabel->setWordWrap(true);
	meshPostLayout->addWidget(m_meshInfoLabel);

	// 简化行
	auto* simplifyRow = new QHBoxLayout;
	m_simplifyTargetLabel = new QLabel(m_meshPostGroup);
	m_simplifyTargetSpin = new QSpinBox(m_meshPostGroup);
	m_simplifyTargetSpin->setRange(100, 10000000);
	m_simplifyTargetSpin->setValue(10000);
	m_simplifyTargetSpin->setSingleStep(1000);
	m_simplifyQualityLabel = new QLabel(m_meshPostGroup);
	m_simplifyQualitySpin = new QDoubleSpinBox(m_meshPostGroup);
	m_simplifyQualitySpin->setRange(0.01, 1.0);
	m_simplifyQualitySpin->setValue(0.3);
	m_simplifyQualitySpin->setDecimals(2);
	m_simplifyBtn = new QPushButton(m_meshPostGroup);
	simplifyRow->addWidget(m_simplifyTargetLabel);
	simplifyRow->addWidget(m_simplifyTargetSpin);
	simplifyRow->addWidget(m_simplifyQualityLabel);
	simplifyRow->addWidget(m_simplifyQualitySpin);
	simplifyRow->addWidget(m_simplifyBtn);
	meshPostLayout->addLayout(simplifyRow);

	// 平滑行
	auto* smoothRow = new QHBoxLayout;
	m_smoothIterLabel = new QLabel(m_meshPostGroup);
	m_smoothIterSpin = new QSpinBox(m_meshPostGroup);
	m_smoothIterSpin->setRange(1, 100);
	m_smoothIterSpin->setValue(3);
	m_smoothLaplacianBtn = new QPushButton(m_meshPostGroup);
	m_smoothImplicitBtn = new QPushButton(m_meshPostGroup);
	smoothRow->addWidget(m_smoothIterLabel);
	smoothRow->addWidget(m_smoothIterSpin);
	smoothRow->addWidget(m_smoothLaplacianBtn);
	smoothRow->addWidget(m_smoothImplicitBtn);
	meshPostLayout->addLayout(smoothRow);

	// 修复 + 重网格行
	auto* repairRemeshRow = new QHBoxLayout;
	m_repairBtn = new QPushButton(m_meshPostGroup);
	m_remeshEdgeLabel = new QLabel(m_meshPostGroup);
	m_remeshEdgeSpin = new QDoubleSpinBox(m_meshPostGroup);
	m_remeshEdgeSpin->setRange(0.01, 1000.0);
	m_remeshEdgeSpin->setValue(2.0);
	m_remeshEdgeSpin->setDecimals(2);
	m_remeshBtn = new QPushButton(m_meshPostGroup);
	repairRemeshRow->addWidget(m_repairBtn);
	repairRemeshRow->addWidget(m_remeshEdgeLabel);
	repairRemeshRow->addWidget(m_remeshEdgeSpin);
	repairRemeshRow->addWidget(m_remeshBtn);
	meshPostLayout->addLayout(repairRemeshRow);

	layout->addWidget(m_meshPostGroup);

	// === 曲面重构组 ===
	m_surfaceReconGroup = new QGroupBox(m_scrollContent);
	auto* surfaceReconLayout = new QVBoxLayout(m_surfaceReconGroup);

	m_surfaceReconPreprocessSectionLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconPreprocessSectionLabel->setStyleSheet(QStringLiteral("font-weight: bold; margin-top: 2px;"));
	surfaceReconLayout->addWidget(m_surfaceReconPreprocessSectionLabel);

	auto* normalSmoothRow = new QHBoxLayout;
	m_normalSmoothIterLabel = new QLabel(m_surfaceReconGroup);
	m_normalSmoothIterSpin = new QSpinBox(m_surfaceReconGroup);
	m_normalSmoothIterSpin->setRange(0, 50);
	m_normalSmoothIterSpin->setValue(6);
	m_featureThresholdLabel = new QLabel(m_surfaceReconGroup);
	m_featureThresholdSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_featureThresholdSpin->setRange(0.1, 2.0);
	m_featureThresholdSpin->setDecimals(2);
	m_featureThresholdSpin->setValue(0.8);
	m_featureThresholdSpin->setToolTip(
		QStringLiteral("预处理重网格特征边与分块特征棱共用；越小切分越多"));
	normalSmoothRow->addWidget(m_normalSmoothIterLabel);
	normalSmoothRow->addWidget(m_normalSmoothIterSpin);
	normalSmoothRow->addWidget(m_featureThresholdLabel);
	normalSmoothRow->addWidget(m_featureThresholdSpin);
	surfaceReconLayout->addLayout(normalSmoothRow);

	m_runVcgRepairCheck = new QCheckBox(m_surfaceReconGroup);
	m_runVcgRepairCheck->setChecked(true);
	surfaceReconLayout->addWidget(m_runVcgRepairCheck);

	m_runIsotropicRemeshCheck = new QCheckBox(m_surfaceReconGroup);
	m_runIsotropicRemeshCheck->setChecked(true);
	surfaceReconLayout->addWidget(m_runIsotropicRemeshCheck);

	auto* remeshRow = new QHBoxLayout;
	m_remeshTargetEdgeLabel = new QLabel(m_surfaceReconGroup);
	m_remeshTargetEdgeSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_remeshTargetEdgeSpin->setRange(0.0, 1000.0);
	m_remeshTargetEdgeSpin->setDecimals(3);
	m_remeshTargetEdgeSpin->setValue(0.0);
	m_remeshTargetEdgeSpin->setSpecialValueText(QStringLiteral("Auto"));
	m_remeshTargetEdgeSpin->setToolTip(
		QStringLiteral("0 = use median edge length after repair"));
	m_remeshIterLabel = new QLabel(m_surfaceReconGroup);
	m_remeshIterSpin = new QSpinBox(m_surfaceReconGroup);
	m_remeshIterSpin->setRange(1, 20);
	m_remeshIterSpin->setValue(3);
	remeshRow->addWidget(m_remeshTargetEdgeLabel);
	remeshRow->addWidget(m_remeshTargetEdgeSpin);
	remeshRow->addWidget(m_remeshIterLabel);
	remeshRow->addWidget(m_remeshIterSpin);
	surfaceReconLayout->addLayout(remeshRow);

	m_exportPreprocessedMeshCheck = new QCheckBox(m_surfaceReconGroup);
	m_exportPreprocessedMeshCheck->setChecked(true);
	surfaceReconLayout->addWidget(m_exportPreprocessedMeshCheck);

	m_surfaceReconPartitionSectionLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconPartitionSectionLabel->setStyleSheet(QStringLiteral("font-weight: bold; margin-top: 6px;"));
	surfaceReconLayout->addWidget(m_surfaceReconPartitionSectionLabel);

	auto* partitionModeRow = new QHBoxLayout;
	m_partitionModeLabel = new QLabel(m_surfaceReconGroup);
	m_partitionModeCombo = new QComboBox(m_surfaceReconGroup);
	m_partitionModeCombo->addItem(
		QStringLiteral("Geodesic Voronoi (v3)"),
		static_cast<int>(PluginMeshSurfacePartitionMode::GeodesicVoronoiV3));
	m_partitionModeCombo->addItem(
		QStringLiteral("Hybrid (normal+CVT)"),
		static_cast<int>(PluginMeshSurfacePartitionMode::HybridNormalCvt));
	partitionModeRow->addWidget(m_partitionModeLabel);
	partitionModeRow->addWidget(m_partitionModeCombo);
	partitionModeRow->addStretch();
	surfaceReconLayout->addLayout(partitionModeRow);

	auto* patchCountRow = new QHBoxLayout;
	m_patchCountLabel = new QLabel(m_surfaceReconGroup);
	m_patchCountSpin = new QSpinBox(m_surfaceReconGroup);
	m_patchCountSpin->setRange(0, 9999);
	m_patchCountSpin->setValue(0);
	m_patchCountSpin->setSpecialValueText(QStringLiteral("Auto"));
	m_patchCountSpin->setToolTip(QStringLiteral("0=自动 sqrt(面数/80)；增大可让凹坑等区域单独成块"));
	patchCountRow->addWidget(m_patchCountLabel);
	patchCountRow->addWidget(m_patchCountSpin);
	patchCountRow->addStretch();
	surfaceReconLayout->addLayout(patchCountRow);

	m_v3PartitionParamsWidget = new QWidget(m_surfaceReconGroup);
	auto* v3PartitionLayout = new QVBoxLayout(m_v3PartitionParamsWidget);
	v3PartitionLayout->setContentsMargins(0, 0, 0, 0);
	auto* partitionRow = new QHBoxLayout;
	m_partitionNormalSmoothLabel = new QLabel(m_v3PartitionParamsWidget);
	m_partitionNormalSmoothSpin = new QSpinBox(m_v3PartitionParamsWidget);
	m_partitionNormalSmoothSpin->setRange(0, 10);
	m_partitionNormalSmoothSpin->setValue(2);
	m_partitionNormalSmoothSpin->setToolTip(
		QStringLiteral("分块前法向平滑；0=保留锐角，2=默认抑制 confetti"));
	partitionRow->addWidget(m_partitionNormalSmoothLabel);
	partitionRow->addWidget(m_partitionNormalSmoothSpin);
	auto* partitionRow2 = new QHBoxLayout;
	m_featureAnglePercentileLabel = new QLabel(m_v3PartitionParamsWidget);
	m_featureAnglePercentileSpin = new QDoubleSpinBox(m_v3PartitionParamsWidget);
	m_featureAnglePercentileSpin->setRange(0.50, 0.99);
	m_featureAnglePercentileSpin->setDecimals(2);
	m_featureAnglePercentileSpin->setSingleStep(0.01);
	m_featureAnglePercentileSpin->setValue(0.88);
	m_featureAnglePercentileSpin->setToolTip(
		QStringLiteral("特征棱角度百分位；越低切分越多（如 0.75），越高越合并"));
	partitionRow2->addWidget(m_featureAnglePercentileLabel);
	partitionRow2->addWidget(m_featureAnglePercentileSpin);
	partitionRow2->addStretch();
	v3PartitionLayout->addLayout(partitionRow);
	v3PartitionLayout->addLayout(partitionRow2);
	surfaceReconLayout->addWidget(m_v3PartitionParamsWidget);

	m_hybridPartitionParamsWidget = new QWidget(m_surfaceReconGroup);
	auto* hybridPartitionLayout = new QVBoxLayout(m_hybridPartitionParamsWidget);
	hybridPartitionLayout->setContentsMargins(0, 0, 0, 0);

	auto* hybridRow1 = new QHBoxLayout;
	m_hybridFeatureAngleLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridFeatureAngleSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridFeatureAngleSpin->setRange(15.0, 90.0);
	m_hybridFeatureAngleSpin->setDecimals(1);
	m_hybridFeatureAngleSpin->setValue(60.0);
	m_hybridClusterItersLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridClusterItersSpin = new QSpinBox(m_hybridPartitionParamsWidget);
	m_hybridClusterItersSpin->setRange(1, 100);
	m_hybridClusterItersSpin->setValue(30);
	hybridRow1->addWidget(m_hybridFeatureAngleLabel);
	hybridRow1->addWidget(m_hybridFeatureAngleSpin);
	hybridRow1->addWidget(m_hybridClusterItersLabel);
	hybridRow1->addWidget(m_hybridClusterItersSpin);
	hybridPartitionLayout->addLayout(hybridRow1);

	auto* hybridRow2 = new QHBoxLayout;
	m_hybridSampleScaleLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridSampleScaleSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridSampleScaleSpin->setRange(1.0, 50.0);
	m_hybridSampleScaleSpin->setDecimals(1);
	m_hybridSampleScaleSpin->setValue(10.0);
	m_hybridRegionAdjustCheck = new QCheckBox(m_hybridPartitionParamsWidget);
	m_hybridRegionAdjustCheck->setChecked(true);
	hybridRow2->addWidget(m_hybridSampleScaleLabel);
	hybridRow2->addWidget(m_hybridSampleScaleSpin);
	hybridRow2->addWidget(m_hybridRegionAdjustCheck);
	hybridPartitionLayout->addLayout(hybridRow2);

	auto* hybridRow3 = new QHBoxLayout;
	m_hybridMergeCosHighLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridMergeCosHighSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridMergeCosHighSpin->setRange(0.50, 0.99);
	m_hybridMergeCosHighSpin->setDecimals(2);
	m_hybridMergeCosHighSpin->setValue(0.70);
	m_hybridMergeCosLowBaseLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridMergeCosLowBaseSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridMergeCosLowBaseSpin->setRange(0.00, 0.50);
	m_hybridMergeCosLowBaseSpin->setDecimals(2);
	m_hybridMergeCosLowBaseSpin->setValue(0.20);
	m_hybridMergeCosLowScaleLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridMergeCosLowScaleSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridMergeCosLowScaleSpin->setRange(0.00, 0.50);
	m_hybridMergeCosLowScaleSpin->setDecimals(2);
	m_hybridMergeCosLowScaleSpin->setValue(0.30);
	hybridRow3->addWidget(m_hybridMergeCosHighLabel);
	hybridRow3->addWidget(m_hybridMergeCosHighSpin);
	hybridRow3->addWidget(m_hybridMergeCosLowBaseLabel);
	hybridRow3->addWidget(m_hybridMergeCosLowBaseSpin);
	hybridRow3->addWidget(m_hybridMergeCosLowScaleLabel);
	hybridRow3->addWidget(m_hybridMergeCosLowScaleSpin);
	hybridPartitionLayout->addLayout(hybridRow3);

	auto* hybridRow4 = new QHBoxLayout;
	m_hybridSmallRegionRatioLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionRatioSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionRatioSpin->setRange(0.001, 0.05);
	m_hybridSmallRegionRatioSpin->setDecimals(3);
	m_hybridSmallRegionRatioSpin->setValue(0.01);
	m_hybridSmallRegionMinLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionMinSpin = new QSpinBox(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionMinSpin->setRange(1, 500);
	m_hybridSmallRegionMinSpin->setValue(10);
	m_hybridSmallRegionMaxLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionMaxSpin = new QSpinBox(m_hybridPartitionParamsWidget);
	m_hybridSmallRegionMaxSpin->setRange(10, 10000);
	m_hybridSmallRegionMaxSpin->setValue(100);
	hybridRow4->addWidget(m_hybridSmallRegionRatioLabel);
	hybridRow4->addWidget(m_hybridSmallRegionRatioSpin);
	hybridRow4->addWidget(m_hybridSmallRegionMinLabel);
	hybridRow4->addWidget(m_hybridSmallRegionMinSpin);
	hybridRow4->addWidget(m_hybridSmallRegionMaxLabel);
	hybridRow4->addWidget(m_hybridSmallRegionMaxSpin);
	hybridPartitionLayout->addLayout(hybridRow4);

	auto* hybridRow5 = new QHBoxLayout;
	m_hybridCollapseValenceLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridCollapseValenceSpin = new QSpinBox(m_hybridPartitionParamsWidget);
	m_hybridCollapseValenceSpin->setRange(4, 12);
	m_hybridCollapseValenceSpin->setValue(6);
	m_hybridCollapseLengthRatioLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridCollapseLengthRatioSpin = new QDoubleSpinBox(m_hybridPartitionParamsWidget);
	m_hybridCollapseLengthRatioSpin->setRange(0.30, 1.00);
	m_hybridCollapseLengthRatioSpin->setDecimals(2);
	m_hybridCollapseLengthRatioSpin->setValue(0.60);
	m_hybridAdjustPassesLabel = new QLabel(m_hybridPartitionParamsWidget);
	m_hybridAdjustPassesSpin = new QSpinBox(m_hybridPartitionParamsWidget);
	m_hybridAdjustPassesSpin->setRange(1, 50);
	m_hybridAdjustPassesSpin->setValue(10);
	hybridRow5->addWidget(m_hybridCollapseValenceLabel);
	hybridRow5->addWidget(m_hybridCollapseValenceSpin);
	hybridRow5->addWidget(m_hybridCollapseLengthRatioLabel);
	hybridRow5->addWidget(m_hybridCollapseLengthRatioSpin);
	hybridRow5->addWidget(m_hybridAdjustPassesLabel);
	hybridRow5->addWidget(m_hybridAdjustPassesSpin);
	hybridPartitionLayout->addLayout(hybridRow5);

	surfaceReconLayout->addWidget(m_hybridPartitionParamsWidget);
	m_hybridPartitionParamsWidget->hide();

	connect(
		m_partitionModeCombo,
		QOverload<int>::of(&QComboBox::currentIndexChanged),
		this,
		&PointCloudDockWidget::onPartitionModeChanged);
	updatePartitionModeUi();

	m_surfaceReconSampleSectionLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconSampleSectionLabel->setStyleSheet(QStringLiteral("font-weight: bold; margin-top: 6px;"));
	surfaceReconLayout->addWidget(m_surfaceReconSampleSectionLabel);

	auto* patchSampleRow = new QHBoxLayout;
	m_samplesPerEdgeLabel = new QLabel(m_surfaceReconGroup);
	m_samplesPerEdgeSpin = new QSpinBox(m_surfaceReconGroup);
	m_samplesPerEdgeSpin->setRange(4, 32);
	m_samplesPerEdgeSpin->setValue(16);
	patchSampleRow->addWidget(m_samplesPerEdgeLabel);
	patchSampleRow->addWidget(m_samplesPerEdgeSpin);
	patchSampleRow->addStretch();
	surfaceReconLayout->addLayout(patchSampleRow);

	auto* uvAdaptiveRow = new QHBoxLayout;
	m_uvSpacingLabel = new QLabel(m_surfaceReconGroup);
	m_uvSpacingSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_uvSpacingSpin->setRange(0.0, 500.0);
	m_uvSpacingSpin->setDecimals(1);
	m_uvSpacingSpin->setValue(30.0);
	m_uvSpacingSpin->setSpecialValueText(QStringLiteral("Off"));
	m_minSamplesLabel = new QLabel(m_surfaceReconGroup);
	m_minSamplesSpin = new QSpinBox(m_surfaceReconGroup);
	m_minSamplesSpin->setRange(4, 32);
	m_minSamplesSpin->setValue(4);
	m_maxSamplesLabel = new QLabel(m_surfaceReconGroup);
	m_maxSamplesSpin = new QSpinBox(m_surfaceReconGroup);
	m_maxSamplesSpin->setRange(0, 9999);
	m_maxSamplesSpin->setValue(0);
	m_maxSamplesSpin->setSpecialValueText(i18n(QStringLiteral("No limit"), QStringLiteral("无上限")));
	uvAdaptiveRow->addWidget(m_uvSpacingLabel);
	uvAdaptiveRow->addWidget(m_uvSpacingSpin);
	uvAdaptiveRow->addWidget(m_minSamplesLabel);
	uvAdaptiveRow->addWidget(m_minSamplesSpin);
	uvAdaptiveRow->addWidget(m_maxSamplesLabel);
	uvAdaptiveRow->addWidget(m_maxSamplesSpin);
	surfaceReconLayout->addLayout(uvAdaptiveRow);

	m_surfaceReconFitSectionLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconFitSectionLabel->setStyleSheet(QStringLiteral("font-weight: bold; margin-top: 6px;"));
	surfaceReconLayout->addWidget(m_surfaceReconFitSectionLabel);

	auto* fitGridRow = new QHBoxLayout;
	m_maxFitGridLabel = new QLabel(m_surfaceReconGroup);
	m_maxFitGridSpin = new QSpinBox(m_surfaceReconGroup);
	m_maxFitGridSpin->setRange(0, 9999);
	m_maxFitGridSpin->setValue(9);
	m_maxFitGridSpin->setSpecialValueText(i18n(QStringLiteral("No limit"), QStringLiteral("无上限")));
	m_fitUvSpacingLabel = new QLabel(m_surfaceReconGroup);
	m_fitUvSpacingSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_fitUvSpacingSpin->setRange(0.0, 500.0);
	m_fitUvSpacingSpin->setDecimals(1);
	m_fitUvSpacingSpin->setValue(0.0);
	m_fitUvSpacingSpin->setSpecialValueText(QStringLiteral("Off"));
	fitGridRow->addWidget(m_maxFitGridLabel);
	fitGridRow->addWidget(m_maxFitGridSpin);
	fitGridRow->addWidget(m_fitUvSpacingLabel);
	fitGridRow->addWidget(m_fitUvSpacingSpin);
	surfaceReconLayout->addLayout(fitGridRow);

	auto* amrtoRow = new QHBoxLayout();
	m_sampleRateLabel = new QLabel(m_surfaceReconGroup);
	m_sampleRateSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_sampleRateSpin->setRange(0.1, 20.0);
	m_sampleRateSpin->setDecimals(1);
	m_sampleRateSpin->setValue(2.0);
	m_ctrlPtDensityLabel = new QLabel(m_surfaceReconGroup);
	m_ctrlPtDensitySpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_ctrlPtDensitySpin->setRange(0.1, 2.0);
	m_ctrlPtDensitySpin->setDecimals(2);
	m_ctrlPtDensitySpin->setValue(0.5);
	m_nurbsFitModeLabel = new QLabel(m_surfaceReconGroup);
	m_nurbsFitModeCombo = new QComboBox(m_surfaceReconGroup);
	m_nurbsFitModeCombo->addItem(QStringLiteral("LSQ+ctrlpts"), static_cast<int>(PluginMeshSurfaceNurbsFitMode::ApproxFixedCtrlpts));
	m_nurbsFitModeCombo->addItem(QStringLiteral("Centripetal"), static_cast<int>(PluginMeshSurfaceNurbsFitMode::ApproxCentripetal));
	m_nurbsFitModeCombo->addItem(QStringLiteral("Centripetal+ctrlpts"), static_cast<int>(PluginMeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts));
	m_nurbsFitModeCombo->addItem(QStringLiteral("Interpolate"), static_cast<int>(PluginMeshSurfaceNurbsFitMode::Interpolate));
	amrtoRow->addWidget(m_sampleRateLabel);
	amrtoRow->addWidget(m_sampleRateSpin);
	amrtoRow->addWidget(m_ctrlPtDensityLabel);
	amrtoRow->addWidget(m_ctrlPtDensitySpin);
	amrtoRow->addWidget(m_nurbsFitModeLabel);
	amrtoRow->addWidget(m_nurbsFitModeCombo);
	surfaceReconLayout->addLayout(amrtoRow);

	m_surfaceReconBlendSectionLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconBlendSectionLabel->setStyleSheet(QStringLiteral("font-weight: bold; margin-top: 6px;"));
	surfaceReconLayout->addWidget(m_surfaceReconBlendSectionLabel);

	auto* fairingRow = new QHBoxLayout;
	m_fairingEpsilonLabel = new QLabel(m_surfaceReconGroup);
	m_fairingEpsilonSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_fairingEpsilonSpin->setRange(1e-6, 1.0);
	m_fairingEpsilonSpin->setDecimals(6);
	m_fairingEpsilonSpin->setValue(1e-3);
	m_fairingMaxIterLabel = new QLabel(m_surfaceReconGroup);
	m_fairingMaxIterSpin = new QSpinBox(m_surfaceReconGroup);
	m_fairingMaxIterSpin->setRange(1, 200);
	m_fairingMaxIterSpin->setValue(50);
	fairingRow->addWidget(m_fairingEpsilonLabel);
	fairingRow->addWidget(m_fairingEpsilonSpin);
	fairingRow->addWidget(m_fairingMaxIterLabel);
	fairingRow->addWidget(m_fairingMaxIterSpin);
	surfaceReconLayout->addLayout(fairingRow);

	auto* blendRow = new QHBoxLayout;
	m_blendStripWidthLabel = new QLabel(m_surfaceReconGroup);
	m_blendStripWidthSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_blendStripWidthSpin->setRange(0.0, 10.0);
	m_blendStripWidthSpin->setDecimals(3);
	m_blendStripWidthSpin->setValue(0.0);
	m_blendStripWidthSpin->setSpecialValueText(QStringLiteral("Auto"));
	blendRow->addWidget(m_blendStripWidthLabel);
	blendRow->addWidget(m_blendStripWidthSpin);
	surfaceReconLayout->addLayout(blendRow);

	auto* tessRow = new QHBoxLayout;
	m_tessellateDeflectionLabel = new QLabel(m_surfaceReconGroup);
	m_tessellateDeflectionSpin = new QDoubleSpinBox(m_surfaceReconGroup);
	m_tessellateDeflectionSpin->setRange(0.001, 5.0);
	m_tessellateDeflectionSpin->setDecimals(3);
	m_tessellateDeflectionSpin->setValue(0.1);
	tessRow->addWidget(m_tessellateDeflectionLabel);
	tessRow->addWidget(m_tessellateDeflectionSpin);
	surfaceReconLayout->addLayout(tessRow);

	auto* stageRow1 = new QHBoxLayout;
	m_surfaceReconPreprocessBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconPartitionBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconSampleBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconFitBtn = new QPushButton(m_surfaceReconGroup);
	stageRow1->addWidget(m_surfaceReconPreprocessBtn);
	stageRow1->addWidget(m_surfaceReconPartitionBtn);
	stageRow1->addWidget(m_surfaceReconSampleBtn);
	stageRow1->addWidget(m_surfaceReconFitBtn);
	surfaceReconLayout->addLayout(stageRow1);

	auto* stageRow2 = new QHBoxLayout;
	m_surfaceReconBoundaryBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconJunctionBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconFairBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconAssembleBtn = new QPushButton(m_surfaceReconGroup);
	stageRow2->addWidget(m_surfaceReconBoundaryBtn);
	stageRow2->addWidget(m_surfaceReconJunctionBtn);
	stageRow2->addWidget(m_surfaceReconFairBtn);
	stageRow2->addWidget(m_surfaceReconAssembleBtn);
	surfaceReconLayout->addLayout(stageRow2);

	auto* stageRow3 = new QHBoxLayout;
	m_surfaceReconBtn = new QPushButton(m_surfaceReconGroup);
	m_surfaceReconResetBtn = new QPushButton(m_surfaceReconGroup);
	stageRow3->addWidget(m_surfaceReconBtn);
	stageRow3->addWidget(m_surfaceReconResetBtn);
	surfaceReconLayout->addLayout(stageRow3);

	m_surfaceReconLog = new QTextEdit(m_surfaceReconGroup);
	m_surfaceReconLog->setReadOnly(true);
	m_surfaceReconLog->setMaximumHeight(120);
	surfaceReconLayout->addWidget(m_surfaceReconLog);

	m_surfaceReconSummaryLabel = new QLabel(m_surfaceReconGroup);
	m_surfaceReconSummaryLabel->setWordWrap(true);
	surfaceReconLayout->addWidget(m_surfaceReconSummaryLabel);

	layout->addWidget(m_surfaceReconGroup);

	m_statusLabel = new QLabel(m_scrollContent);
	m_statusLabel->setWordWrap(true);
	m_statusLabel->setMaximumHeight(48);
	m_statusLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
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
	connect(m_polylineCropBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onPolylineCropClicked);
	connect(m_outlierBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onRemoveOutliersClicked);
	connect(m_smoothBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onSmoothClicked);
	connect(m_pcaBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onNormalsPcaClicked);
	connect(m_orientBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onNormalsOrientClicked);
	connect(m_icpBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onIcpClicked);
	connect(m_poissonBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onReconstructPoissonAutoClicked);
	connect(m_scaleBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onReconstructScaleSpaceClicked);
	connect(m_exportMeshBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onExportMeshClicked);
	connect(m_pickFaceBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onPickTemplateFaceClicked);
	connect(m_clearFacesBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onClearSelectedFacesClicked);
	connect(m_coarseMatchBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onCoarseRegisterScanToTemplateClicked);
	connect(m_fineMatchBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onFineRegisterScanToTemplateClicked);
	connect(m_refactorBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onUpdateTemplateBrepClicked);
	connect(m_simplifyBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onMeshSimplifyClicked);
	connect(m_smoothLaplacianBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onMeshSmoothLaplacianClicked);
	connect(m_smoothImplicitBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onMeshSmoothImplicitClicked);
	connect(m_repairBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onMeshRepairClicked);
	connect(m_remeshBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onMeshRemeshClicked);
	connect(m_surfaceReconBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onSurfaceReconstructClicked);
	connect(m_surfaceReconResetBtn, &QPushButton::clicked, this, &PointCloudDockWidget::onSurfaceReconstructResetSessionClicked);
	connect(m_surfaceReconPreprocessBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Preprocess);
	});
	connect(m_surfaceReconPartitionBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Partition);
	});
	connect(m_surfaceReconSampleBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Sample);
	});
	connect(m_surfaceReconFitBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Fit);
	});
	connect(m_surfaceReconBoundaryBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::BoundaryBlend);
	});
	connect(m_surfaceReconJunctionBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::JunctionBlend);
	});
	connect(m_surfaceReconFairBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Fair);
	});
	connect(m_surfaceReconAssembleBtn, &QPushButton::clicked, this, [this]() {
		runSurfaceReconStage(PluginMeshSurfaceReconstructStage::Assemble);
	});
	connect(m_meshTargetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](const int index) {
		if (index < 0)
		{
			return;
		}
		const std::string meshId = selectedMeshTargetId();
		// 刷新列表时重选同一网格不应清空分阶段会话
		if (m_surfaceReconSessionId.valid() && !meshId.empty() && meshId == m_surfaceReconMeshBackendId)
		{
			refreshMeshInfo();
			return;
		}
		resetSurfaceReconSessionUi();
		refreshMeshInfo();
	});

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
	m_reGroup->setTitle(i18n(QStringLiteral("CAD template B-rep update"), QStringLiteral("CAD 模板 B-rep 更新")));
	m_meshPostGroup->setTitle(i18n(QStringLiteral("Mesh post-process"), QStringLiteral("网格后处理")));
	if (m_surfaceReconGroup)
	{
		const bool surfaceReconAvailable = m_host && m_host->hostVersion() >= 0x00010C00U;
		const bool stagedApi = m_host && m_host->hostVersion() >= 0x00010D00U;
		m_surfaceReconGroup->setVisible(surfaceReconAvailable);
		m_surfaceReconGroup->setTitle(i18n(QStringLiteral("Surface reconstruct"), QStringLiteral("曲面重构")));
		if (m_surfaceReconPreprocessSectionLabel)
		{
			m_surfaceReconPreprocessSectionLabel->setText(
				i18n(QStringLiteral("Preprocess"), QStringLiteral("预处理")));
		}
		if (m_surfaceReconPartitionSectionLabel)
		{
			m_surfaceReconPartitionSectionLabel->setText(
				i18n(QStringLiteral("Partition"), QStringLiteral("分块")));
		}
		if (m_surfaceReconSampleSectionLabel)
		{
			m_surfaceReconSampleSectionLabel->setText(
				i18n(QStringLiteral("Grid sample"), QStringLiteral("栅格采样")));
		}
		if (m_surfaceReconFitSectionLabel)
		{
			m_surfaceReconFitSectionLabel->setText(
				i18n(QStringLiteral("NURBS fit"), QStringLiteral("NURBS 拟合")));
		}
		if (m_surfaceReconBlendSectionLabel)
		{
			m_surfaceReconBlendSectionLabel->setText(
				i18n(QStringLiteral("Blend / fair / assemble"), QStringLiteral("混合 / 光顺 / 装配")));
		}
		m_normalSmoothIterLabel->setText(
			i18n(QStringLiteral("Normal smooth iter:"), QStringLiteral("法矢光顺迭代:")));
		m_featureThresholdLabel->setText(i18n(QStringLiteral("Feature c0:"), QStringLiteral("特征阈值 c0:")));
		if (m_featureThresholdSpin)
		{
			m_featureThresholdSpin->setToolTip(
				i18n(QStringLiteral("Shared by remesh feature edges and partition; lower = more splits"),
					QStringLiteral("预处理重网格特征边与分块特征棱共用；越小切分越多")));
		}
		m_patchCountLabel->setText(i18n(QStringLiteral("Patches (0=auto):"), QStringLiteral("分块数(0=自动):")));
		if (m_patchCountSpin)
		{
			m_patchCountSpin->setToolTip(
				i18n(QStringLiteral("0=auto sqrt(faces/80); increase to split pits/corners"),
					QStringLiteral("0=自动 sqrt(面数/80)；增大可让凹坑等区域单独成块")));
		}
		if (m_partitionNormalSmoothLabel)
		{
			m_partitionNormalSmoothLabel->setText(
				i18n(QStringLiteral("Part. norm smooth:"), QStringLiteral("分块法向平滑:")));
		}
		if (m_partitionNormalSmoothSpin)
		{
			m_partitionNormalSmoothSpin->setToolTip(
				i18n(QStringLiteral("Pre-partition normal smooth; 0=sharp corners, 2=default"),
					QStringLiteral("分块前法向平滑；0=保留锐角，2=默认抑制 confetti")));
		}
		if (m_featureAnglePercentileLabel)
		{
			m_featureAnglePercentileLabel->setText(
				i18n(QStringLiteral("Feature P%:"), QStringLiteral("特征百分位:")));
		}
		if (m_featureAnglePercentileSpin)
		{
			m_featureAnglePercentileSpin->setToolTip(
				i18n(QStringLiteral("Dihedral percentile; lower (e.g. 0.75) = more feature edges"),
					QStringLiteral("特征棱角度百分位；越低切分越多（如 0.75），越高越合并")));
		}
		if (m_partitionModeLabel)
		{
			m_partitionModeLabel->setText(i18n(QStringLiteral("Partition algo:"), QStringLiteral("分块算法:")));
		}
		if (m_partitionModeCombo)
		{
			const int idx = m_partitionModeCombo->currentIndex();
			m_partitionModeCombo->blockSignals(true);
			m_partitionModeCombo->clear();
			m_partitionModeCombo->addItem(
				i18n(QStringLiteral("Geodesic Voronoi (v3)"), QStringLiteral("测地 Voronoi (v3)")),
				static_cast<int>(PluginMeshSurfacePartitionMode::GeodesicVoronoiV3));
			m_partitionModeCombo->addItem(
				i18n(QStringLiteral("Hybrid (normal+CVT)"), QStringLiteral("混合策略 (法向+CVT)")),
				static_cast<int>(PluginMeshSurfacePartitionMode::HybridNormalCvt));
			m_partitionModeCombo->setCurrentIndex(idx >= 0 ? idx : 0);
			m_partitionModeCombo->blockSignals(false);
		}
		if (m_hybridFeatureAngleLabel)
		{
			m_hybridFeatureAngleLabel->setText(i18n(QStringLiteral("Feat. angle°:"), QStringLiteral("特征广义角°:")));
		}
		if (m_hybridClusterItersLabel)
		{
			m_hybridClusterItersLabel->setText(i18n(QStringLiteral("Cluster iters:"), QStringLiteral("聚类迭代:")));
		}
		if (m_hybridSampleScaleLabel)
		{
			m_hybridSampleScaleLabel->setText(i18n(QStringLiteral("CVT scale:"), QStringLiteral("二次采样系数:")));
		}
		if (m_hybridRegionAdjustCheck)
		{
			m_hybridRegionAdjustCheck->setText(i18n(QStringLiteral("Quad adjust"), QStringLiteral("四边区域调整")));
		}
		if (m_hybridMergeCosHighLabel)
		{
			m_hybridMergeCosHighLabel->setText(i18n(QStringLiteral("Merge cos high:"), QStringLiteral("合并阈值高:")));
		}
		if (m_hybridMergeCosLowBaseLabel)
		{
			m_hybridMergeCosLowBaseLabel->setText(i18n(QStringLiteral("Merge base:"), QStringLiteral("合并基线:")));
		}
		if (m_hybridMergeCosLowScaleLabel)
		{
			m_hybridMergeCosLowScaleLabel->setText(i18n(QStringLiteral("Merge slope:"), QStringLiteral("合并斜率:")));
		}
		if (m_hybridSmallRegionRatioLabel)
		{
			m_hybridSmallRegionRatioLabel->setText(i18n(QStringLiteral("Small ratio:"), QStringLiteral("小片比例:")));
		}
		if (m_hybridSmallRegionMinLabel)
		{
			m_hybridSmallRegionMinLabel->setText(i18n(QStringLiteral("Small min:"), QStringLiteral("小片下限:")));
		}
		if (m_hybridSmallRegionMaxLabel)
		{
			m_hybridSmallRegionMaxLabel->setText(i18n(QStringLiteral("Small max:"), QStringLiteral("小片上限:")));
		}
		if (m_hybridCollapseValenceLabel)
		{
			m_hybridCollapseValenceLabel->setText(i18n(QStringLiteral("Valence≤:"), QStringLiteral("收缩度数和≤:")));
		}
		if (m_hybridCollapseLengthRatioLabel)
		{
			m_hybridCollapseLengthRatioLabel->setText(
				i18n(QStringLiteral("Neigh len ratio:"), QStringLiteral("邻边长度比:")));
		}
		if (m_hybridAdjustPassesLabel)
		{
			m_hybridAdjustPassesLabel->setText(i18n(QStringLiteral("Adjust passes:"), QStringLiteral("调整轮数:")));
		}
		updatePartitionModeUi();
		m_samplesPerEdgeLabel->setText(
			i18n(QStringLiteral("Samples/edge (spacing=0):"), QStringLiteral("每边 n(间距0):")));
		m_uvSpacingLabel->setText(
			i18n(QStringLiteral("UV spacing mm (0=fixed):"), QStringLiteral("UV间距mm(0=固定):")));
		m_minSamplesLabel->setText(i18n(QStringLiteral("Min/edge:"), QStringLiteral("最少/边:")));
		m_maxSamplesLabel->setText(i18n(QStringLiteral("Max/edge:"), QStringLiteral("最多/边:")));
		if (m_maxSamplesSpin)
		{
			m_maxSamplesSpin->setSpecialValueText(i18n(QStringLiteral("No limit"), QStringLiteral("无上限")));
		}
		if (m_maxFitGridLabel)
		{
			m_maxFitGridLabel->setText(
				i18n(QStringLiteral("Fit max/edge (0=none):"), QStringLiteral("拟合最多/边(0=无):")));
		}
		if (m_maxFitGridSpin)
		{
			m_maxFitGridSpin->setSpecialValueText(i18n(QStringLiteral("No limit"), QStringLiteral("无上限")));
		}
		if (m_fitUvSpacingLabel)
		{
			m_fitUvSpacingLabel->setText(
				i18n(QStringLiteral("Fit UV spacing mm (0=off):"), QStringLiteral("拟合UV间距mm(0=关):")));
		}
		if (m_sampleRateLabel)
		{
			m_sampleRateLabel->setText(
				i18n(QStringLiteral("Sample rate k:"), QStringLiteral("采样率 k:")));
		}
		if (m_ctrlPtDensityLabel)
		{
			m_ctrlPtDensityLabel->setText(
				i18n(QStringLiteral("Ctrl pt density:"), QStringLiteral("控制点密度:")));
		}
		if (m_nurbsFitModeLabel)
		{
			m_nurbsFitModeLabel->setText(
				i18n(QStringLiteral("NURBS fit mode:"), QStringLiteral("NURBS 拟合模式:")));
		}
		if (m_nurbsFitModeCombo)
		{
			const int idx = m_nurbsFitModeCombo->currentIndex();
			m_nurbsFitModeCombo->setItemText(0, i18n(QStringLiteral("LSQ+ctrlpts"), QStringLiteral("最小二乘+控制点")));
			m_nurbsFitModeCombo->setItemText(1, i18n(QStringLiteral("Centripetal"), QStringLiteral("Centripetal")));
			m_nurbsFitModeCombo->setItemText(2, i18n(QStringLiteral("Centripetal+ctrlpts"), QStringLiteral("Centripetal+控制点")));
			m_nurbsFitModeCombo->setItemText(3, i18n(QStringLiteral("Interpolate"), QStringLiteral("插值")));
			m_nurbsFitModeCombo->setCurrentIndex(idx);
		}
		m_fairingEpsilonLabel->setText(i18n(QStringLiteral("Fairing eps:"), QStringLiteral("光顺 ε:")));
		m_fairingMaxIterLabel->setText(
			i18n(QStringLiteral("Fairing max iter:"), QStringLiteral("光顺最大迭代:")));
		m_runVcgRepairCheck->setText(
			i18n(QStringLiteral("Vcg repair before partition"), QStringLiteral("分块前 Vcg 修复")));
		if (m_runIsotropicRemeshCheck)
		{
			m_runIsotropicRemeshCheck->setText(
				i18n(QStringLiteral("Isotropic remesh in preprocess"), QStringLiteral("预处理均匀化重网格")));
		}
		if (m_remeshTargetEdgeLabel)
		{
			m_remeshTargetEdgeLabel->setText(
				i18n(QStringLiteral("Remesh edge mm (0=auto):"), QStringLiteral("目标边长mm(0=自动):")));
		}
		if (m_remeshIterLabel)
		{
			m_remeshIterLabel->setText(
				i18n(QStringLiteral("Remesh iter:"), QStringLiteral("重网格迭代:")));
		}
		if (m_remeshTargetEdgeSpin)
		{
			m_remeshTargetEdgeSpin->setSpecialValueText(
				i18n(QStringLiteral("Auto"), QStringLiteral("自动")));
			m_remeshTargetEdgeSpin->setToolTip(
				i18n(QStringLiteral("0 = median edge length after repair"),
					QStringLiteral("0 = 修复后网格边长中位数")));
		}
		m_blendStripWidthLabel->setText(
			i18n(QStringLiteral("Blend strip width (0=auto):"), QStringLiteral("混合带宽度(0=自动):")));
		m_tessellateDeflectionLabel->setText(
			i18n(QStringLiteral("Tessellate deflection (mm):"), QStringLiteral("装配离散精度(mm):")));
		m_exportPreprocessedMeshCheck->setText(
			i18n(QStringLiteral("Write preprocessed mesh to scene"), QStringLiteral("预处理后写入场景网格")));
		m_surfaceReconPreprocessBtn->setText(i18n(QStringLiteral("Preprocess"), QStringLiteral("预处理")));
		m_surfaceReconPartitionBtn->setText(i18n(QStringLiteral("Partition"), QStringLiteral("分块")));
		m_surfaceReconSampleBtn->setText(i18n(QStringLiteral("Sample"), QStringLiteral("栅格采样")));
		m_surfaceReconFitBtn->setText(i18n(QStringLiteral("NURBS fit"), QStringLiteral("NURBS拟合")));
		m_surfaceReconBoundaryBtn->setText(i18n(QStringLiteral("Boundary blend"), QStringLiteral("边界混合")));
		m_surfaceReconJunctionBtn->setText(i18n(QStringLiteral("Junction blend"), QStringLiteral("交汇混合")));
		m_surfaceReconFairBtn->setText(i18n(QStringLiteral("Fair"), QStringLiteral("光顺")));
		m_surfaceReconAssembleBtn->setText(i18n(QStringLiteral("Assemble"), QStringLiteral("装配输出")));
		m_surfaceReconBtn->setText(
			i18n(QStringLiteral("Full pipeline"), QStringLiteral("全流程")));
		m_surfaceReconResetBtn->setText(i18n(QStringLiteral("Reset session"), QStringLiteral("重置会话")));
		m_surfaceReconSummaryLabel->setText(
			i18n(QStringLiteral("No reconstruction yet"), QStringLiteral("尚未执行曲面重构")));
		m_surfaceReconPreprocessBtn->setVisible(stagedApi);
		m_surfaceReconPartitionBtn->setVisible(stagedApi);
		m_surfaceReconSampleBtn->setVisible(stagedApi);
		m_surfaceReconFitBtn->setVisible(stagedApi);
		m_surfaceReconBoundaryBtn->setVisible(stagedApi);
		m_surfaceReconJunctionBtn->setVisible(stagedApi);
		m_surfaceReconFairBtn->setVisible(stagedApi);
		m_surfaceReconAssembleBtn->setVisible(stagedApi);
		m_surfaceReconResetBtn->setVisible(stagedApi);
		m_exportPreprocessedMeshCheck->setVisible(stagedApi);
		m_surfaceReconLog->setVisible(stagedApi);
		updateSurfaceReconButtonStates();
	}
	m_meshTargetLabel->setText(i18n(QStringLiteral("Mesh:"), QStringLiteral("网格对象:")));
	m_meshInfoLabel->setText(i18n(QStringLiteral("Select a mesh"), QStringLiteral("请选择网格")));
	m_simplifyTargetLabel->setText(i18n(QStringLiteral("Target faces:"), QStringLiteral("目标面数:")));
	m_simplifyQualityLabel->setText(i18n(QStringLiteral("Quality:"), QStringLiteral("质量阈值:")));
	m_simplifyBtn->setText(i18n(QStringLiteral("Simplify"), QStringLiteral("网格简化")));
	m_smoothIterLabel->setText(i18n(QStringLiteral("Iterations:"), QStringLiteral("迭代次数:")));
	m_smoothLaplacianBtn->setText(i18n(QStringLiteral("Laplacian smooth"), QStringLiteral("Laplacian 平滑")));
	m_smoothImplicitBtn->setText(i18n(QStringLiteral("Implicit fairing"), QStringLiteral("隐式平滑")));
	m_repairBtn->setText(i18n(QStringLiteral("Repair mesh"), QStringLiteral("网格修复")));
	m_remeshEdgeLabel->setText(i18n(QStringLiteral("Edge len (mm):"), QStringLiteral("目标边长(mm):")));
	m_remeshBtn->setText(i18n(QStringLiteral("Isotropic remesh"), QStringLiteral("各向同性重网格")));
	m_templateBrepLabel->setText(i18n(QStringLiteral("Template B-rep:"), QStringLiteral("CAD 模板:")));
	m_faceBandLabel->setText(i18n(QStringLiteral("Face band (mm):"), QStringLiteral("面归属带(mm):")));
	m_reMinPtsLabel->setText(i18n(QStringLiteral("Min pts per face:"), QStringLiteral("每面最少点:")));
	m_reNormalLabel->setText(i18n(QStringLiteral("Normal tol (deg):"), QStringLiteral("法向容差(°):")));
	m_reMaxDevLabel->setText(i18n(QStringLiteral("Max deviation (mm):"), QStringLiteral("最大偏差(mm):")));
	m_bsplineUvGridLabel->setText(
		i18n(QStringLiteral("BSpline UV grid U/V:"), QStringLiteral("BSpline UV 网格 U/V:")));
	m_maxAssignPointsLabel->setText(
		i18n(QStringLiteral("Max assign pts/face:"), QStringLiteral("每面归属点数上限:")));
	m_bsplinePoleSmoothLabel->setText(
		i18n(QStringLiteral("BSpline pole smooth:"), QStringLiteral("BSpline 极点平滑:")));
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
	m_polylineCropBtn->setText(i18n(QStringLiteral("Draw polygon crop..."), QStringLiteral("多边形裁剪…")));
	if (m_polylineCropModeCombo->count() == 0)
	{
		m_polylineCropModeCombo->addItem(
			i18n(QStringLiteral("Keep inside"), QStringLiteral("保留内部")),
			true);
		m_polylineCropModeCombo->addItem(
			i18n(QStringLiteral("Delete inside"), QStringLiteral("删除内部")),
			false);
	}
	else
	{
		m_polylineCropModeCombo->setItemText(
			0, i18n(QStringLiteral("Keep inside"), QStringLiteral("保留内部")));
		m_polylineCropModeCombo->setItemText(
			1, i18n(QStringLiteral("Delete inside"), QStringLiteral("删除内部")));
	}
	m_outlierBtn->setText(i18n(QStringLiteral("Remove outliers"), QStringLiteral("离群移除")));
	m_smoothBtn->setText(i18n(QStringLiteral("Bilateral smooth"), QStringLiteral("双边平滑")));
	m_pcaBtn->setText(i18n(QStringLiteral("Normals PCA"), QStringLiteral("法线 PCA")));
	m_orientBtn->setText(i18n(QStringLiteral("Orient MST"), QStringLiteral("MST 定向")));
	m_icpBtn->setText(i18n(QStringLiteral("ICP register"), QStringLiteral("ICP 配准")));
	m_poissonBtn->setText(i18n(QStringLiteral("Poisson Auto"), QStringLiteral("Poisson Auto")));
	m_scaleBtn->setText(i18n(QStringLiteral("Scale-space"), QStringLiteral("Scale-space")));
	m_meshExportLabel->setText(i18n(QStringLiteral("Mesh:"), QStringLiteral("网格:")));
	m_exportMeshBtn->setText(i18n(QStringLiteral("Export PLY..."), QStringLiteral("导出 PLY…")));
	m_pickFaceBtn->setText(i18n(QStringLiteral("Pick face..."), QStringLiteral("选择面…")));
	m_clearFacesBtn->setText(i18n(QStringLiteral("Clear faces"), QStringLiteral("清空面")));
	m_coarseMatchBtn->setText(i18n(QStringLiteral("Coarse match"), QStringLiteral("粗匹配")));
	m_fineMatchBtn->setText(i18n(QStringLiteral("Fine match"), QStringLiteral("精匹配")));
	m_refactorBtn->setText(i18n(QStringLiteral("Refactor faces"), QStringLiteral("面重构")));
	m_matchStatusLabel->setText(i18n(
		QStringLiteral("No registration cached"),
		QStringLiteral("尚未执行匹配")));
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

void PointCloudDockWidget::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	refreshPointCloudList();
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
	const std::string prevTemplate = m_templateBrepCombo && m_templateBrepCombo->currentIndex() >= 0
		? m_templateBrepCombo->currentData().toString().toStdString()
		: std::string();
	m_list->clear();
	m_icpTargetCombo->clear();
	if (m_templateBrepCombo)
	{
		m_templateBrepCombo->clear();
	}
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
	if (m_templateBrepCombo)
	{
		if (IPluginGeometryHost* geoHost = m_host ? m_host->geometryHost() : nullptr)
		{
			std::vector<PluginGeometryBackendEntry> backends;
			if (geoHost->listComputableBackends(doc, backends, nullptr))
			{
				for (const PluginGeometryBackendEntry& entry : backends)
				{
					if (entry.className != "BrepModel")
					{
						continue;
					}
					m_templateBrepCombo->addItem(
						QString::fromStdString(entry.displayName),
						QString::fromStdString(entry.backendId));
				}
			}
		}
		for (int i = 0; i < m_templateBrepCombo->count(); ++i)
		{
			if (m_templateBrepCombo->itemData(i).toString().toStdString() == prevTemplate)
			{
				m_templateBrepCombo->setCurrentIndex(i);
				break;
			}
		}
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
	const std::string prevMeshTargetId =
		m_meshTargetCombo ? selectedMeshTargetId() : std::string();
	const std::string prevExportId = preferBackendId.empty() ? selectedMeshBackendId() : preferBackendId;
	const std::string restoreMeshTargetId =
		!prevMeshTargetId.empty() ? prevMeshTargetId : prevExportId;

	const QSignalBlocker exportBlocker(m_meshExportCombo);
	const QSignalBlocker targetBlocker(m_meshTargetCombo);

	m_meshExportCombo->clear();
	if (m_meshTargetCombo)
	{
		m_meshTargetCombo->clear();
	}
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
		if (m_meshTargetCombo)
		{
			m_meshTargetCombo->addItem(label, QString::fromStdString(id));
		}
	}
	auto selectComboById = [](QComboBox* combo, const std::string& backendId) {
		if (!combo || backendId.empty())
		{
			return;
		}
		for (int i = 0; i < combo->count(); ++i)
		{
			if (combo->itemData(i).toString().toStdString() == backendId)
			{
				combo->setCurrentIndex(i);
				break;
			}
		}
	};
	selectComboById(m_meshExportCombo, prevExportId);
	selectComboById(m_meshTargetCombo, restoreMeshTargetId);
	refreshMeshInfo();
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
	updateSurfaceReconButtonStates();
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

void PointCloudDockWidget::triggerMeshSimplify()
{
	onMeshSimplifyClicked();
}

void PointCloudDockWidget::triggerMeshSmoothLaplacian()
{
	onMeshSmoothLaplacianClicked();
}

void PointCloudDockWidget::triggerSurfaceReconstruct()
{
	onSurfaceReconstructClicked();
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

void PointCloudDockWidget::onPolylineCropClicked()
{
	if (!m_host || m_host->hostVersion() < 0x00010B00U)
	{
		if (m_host)
		{
			m_host->logWarn(i18n(
				QStringLiteral("Polyline crop requires host 1.11.0+"),
				QStringLiteral("多边形裁剪需要宿主 1.11.0+")));
		}
		return;
	}
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedBackendId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	const bool keepInside = m_polylineCropModeCombo
		? m_polylineCropModeCombo->currentData().toBool()
		: true;
	m_host->logInfo(i18n(
		QStringLiteral("Draw polygon in 3D view: left-click vertices, right-click/double-click close, Esc cancel"),
		QStringLiteral("请在 3D 视图绘制多边形：左键加点，右键/双击闭合，Esc 取消")));
	pch->pickPolylineFromViewport(
		doc,
		[this, pch, doc, id, keepInside](
			const bool ok, const QString& error, const PluginPointCloudPolylinePickResult& pick) {
			if (!ok)
			{
				if (!error.isEmpty() && m_host)
				{
					m_host->logWarn(error);
				}
				return;
			}
			setBusy(true);
			PluginPointCloudCropPolylineParams params;
			params.polylineScreenXy = pick.polylineScreenXy;
			for (int i = 0; i < 16; ++i)
			{
				params.mvpMatrix[i] = pick.mvpMatrix[i];
			}
			params.viewportWidth = pick.viewportWidth;
			params.viewportHeight = pick.viewportHeight;
			params.keepInside = keepInside;
			pch->cropPointCloudByPolyline(
				doc,
				id,
				params,
				[this](const bool cropOk, const QString& cropError, const PluginPointCloudJobResult& result) {
					runFinished(cropOk, cropError, result);
				});
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

PluginPointCloudTemplateBrepUpdateParams PointCloudDockWidget::buildTemplateBrepParams() const
{
	PluginPointCloudTemplateBrepUpdateParams params;
	if (m_templateBrepCombo && m_templateBrepCombo->currentIndex() >= 0)
	{
		params.templateBrepBackendIdUtf8 = m_templateBrepCombo->currentData().toString().toStdString();
	}
	params.voxelPrefilterMm = m_prefilterSpin->value();
	params.faceBandMm = m_faceBandSpin->value();
	params.normalThresholdDeg = m_reNormalSpin->value();
	params.minPointsPerFace = static_cast<std::size_t>(m_reMinPointsSpin->value());
	params.maxAllowedDeviationMm = m_reMaxDevSpin->value();
	params.maxAssignPointsPerFace = static_cast<std::size_t>(m_maxAssignPointsSpin->value());
	params.bsplineUvGridCellsU = m_bsplineUvGridUSpin->value();
	params.bsplineUvGridCellsV = m_bsplineUvGridVSpin->value();
	params.bsplinePoleSmoothPasses = m_bsplinePoleSmoothSpin->value();
	params.selectedFaceIndices = selectedFaceIndices();
	return params;
}

std::vector<int> PointCloudDockWidget::selectedFaceIndices() const
{
	std::vector<int> indices;
	if (!m_selectedFacesList)
	{
		return indices;
	}
	indices.reserve(static_cast<std::size_t>(m_selectedFacesList->count()));
	for (int row = 0; row < m_selectedFacesList->count(); ++row)
	{
		const QListWidgetItem* item = m_selectedFacesList->item(row);
		if (!item)
		{
			continue;
		}
		bool ok = false;
		const int faceIndex = item->data(Qt::UserRole).toInt(&ok);
		if (ok)
		{
			indices.push_back(faceIndex);
		}
	}
	return indices;
}

void PointCloudDockWidget::addSelectedFaceIndex(const int faceIndex)
{
	if (!m_selectedFacesList || faceIndex < 0)
	{
		return;
	}
	for (int row = 0; row < m_selectedFacesList->count(); ++row)
	{
		const QListWidgetItem* item = m_selectedFacesList->item(row);
		if (item && item->data(Qt::UserRole).toInt() == faceIndex)
		{
			return;
		}
	}
	auto* item = new QListWidgetItem(
		i18n(QStringLiteral("Face %1"), QStringLiteral("面 %1")).arg(faceIndex),
		m_selectedFacesList);
	item->setData(Qt::UserRole, faceIndex);
}

void PointCloudDockWidget::onPickTemplateFaceClicked()
{
	if (!m_host || !m_host->geometryHost())
	{
		return;
	}
	IPluginDocument* doc = activeDoc();
	if (!doc || !m_templateBrepCombo || m_templateBrepCombo->currentIndex() < 0)
	{
		m_host->logWarn(i18n(
			QStringLiteral("Select CAD template B-rep first"),
			QStringLiteral("请先选择 CAD 模板 B-rep")));
		return;
	}
	const std::string templateId = m_templateBrepCombo->currentData().toString().toStdString();
	if (templateId.empty())
	{
		return;
	}
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = templateId;
	if (m_statusLabel)
	{
		m_statusLabel->setText(i18n(
			QStringLiteral("Pick a face in 3D view..."),
			QStringLiteral("请在 3D 视图点选面…")));
	}
	m_host->geometryHost()->pickStepElementFromViewport(
		doc,
		req,
		[this](const bool ok, const QString& err, const PluginGeometryStepRef& ref) {
			if (!m_host)
			{
				return;
			}
			if (!ok)
			{
				m_host->logWarn(err);
				if (m_statusLabel)
				{
					m_statusLabel->setText(err);
				}
				return;
			}
			addSelectedFaceIndex(ref.faceIndex);
			const QString msg = i18n(
				QStringLiteral("Selected face %1 (empty list = all faces)"),
				QStringLiteral("已选面 %1（列表为空则处理全部面）"))
									.arg(ref.faceIndex);
			m_host->logInfo(msg);
			if (m_statusLabel)
			{
				m_statusLabel->setText(msg);
			}
		});
}

void PointCloudDockWidget::onClearSelectedFacesClicked()
{
	if (m_selectedFacesList)
	{
		m_selectedFacesList->clear();
	}
	if (m_statusLabel)
	{
		m_statusLabel->setText(i18n(
			QStringLiteral("Face list cleared (all faces will be refactored)"),
			QStringLiteral("已清空面列表（将重构全部面）")));
	}
}

void PointCloudDockWidget::onCoarseRegisterScanToTemplateClicked()
{
	runTemplateBrepRegistration(PluginPointCloudTemplateBrepRegistrationStage::CoarseOnly);
}

void PointCloudDockWidget::onFineRegisterScanToTemplateClicked()
{
	runTemplateBrepRegistration(PluginPointCloudTemplateBrepRegistrationStage::FineOnly);
}

void PointCloudDockWidget::runTemplateBrepRegistration(
	const PluginPointCloudTemplateBrepRegistrationStage stage)
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string scanId = selectedBackendId();
	if (!pch || !doc || scanId.empty() || !m_templateBrepCombo || m_templateBrepCombo->currentIndex() < 0)
	{
		if (m_host)
		{
			m_host->logWarn(i18n(
				QStringLiteral("Select scan point cloud and CAD template B-rep"),
				QStringLiteral("请选择扫描点云与 CAD 模板 B-rep")));
		}
		return;
	}
	PluginPointCloudTemplateBrepUpdateParams params = buildTemplateBrepParams();
	params.registrationStage = stage;
	if (params.templateBrepBackendIdUtf8.empty())
	{
		return;
	}
	setBusy(true);
	pch->registerScanToCadTemplate(
		doc,
		scanId,
		params,
		[this, stage](const bool ok, const QString& error, const PluginPointCloudTemplateBrepRegisterResult& result) {
			setBusy(false);
			if (!m_host)
			{
				return;
			}
			if (ok)
			{
				const QString stageName =
					stage == PluginPointCloudTemplateBrepRegistrationStage::CoarseOnly
						? i18n(QStringLiteral("Coarse match"), QStringLiteral("粗匹配"))
						: (stage == PluginPointCloudTemplateBrepRegistrationStage::FineOnly
							  ? i18n(QStringLiteral("Fine match"), QStringLiteral("精匹配"))
							  : i18n(QStringLiteral("Match"), QStringLiteral("匹配")));
				const QString msg = error.isEmpty()
					? i18n(
						  QStringLiteral("%1 OK, ICP RMSE %2 mm"),
						  QStringLiteral("%1 完成，ICP RMSE %2 mm"))
						  .arg(stageName)
						  .arg(result.icpRmseMm, 0, 'f', 3)
					: error;
				if (error.isEmpty())
				{
					m_host->logInfo(msg);
				}
				else
				{
					m_host->logWarn(msg);
				}
				if (m_statusLabel)
				{
					m_statusLabel->setText(msg);
				}
				if (m_matchStatusLabel)
				{
					m_matchStatusLabel->setText(msg);
				}
				refreshPointCloudList();
			}
			else
			{
				m_host->logError(error.isEmpty()
					? i18n(QStringLiteral("Registration failed"), QStringLiteral("匹配失败"))
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
		});
}

void PointCloudDockWidget::onUpdateTemplateBrepClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string scanId = selectedBackendId();
	if (!pch || !doc || scanId.empty() || !m_templateBrepCombo || m_templateBrepCombo->currentIndex() < 0)
	{
		if (m_host)
		{
			m_host->logWarn(i18n(
				QStringLiteral("Select scan point cloud and CAD template B-rep"),
				QStringLiteral("请选择扫描点云与 CAD 模板 B-rep")));
		}
		return;
	}
	const PluginPointCloudTemplateBrepUpdateParams params = buildTemplateBrepParams();
	if (params.templateBrepBackendIdUtf8.empty())
	{
		return;
	}
	setBusy(true);
	pch->updateTemplateBrepFromAlignedScan(
		doc,
		scanId,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudTemplateBrepUpdateResult& result) {
			setBusy(false);
			if (!m_host)
			{
				return;
			}
			if (ok)
			{
				QString msg = i18n(
					QStringLiteral("Updated B-rep: %1 faces, max dev %2mm"),
					QStringLiteral("已更新 B-rep: %1 面，最大偏差 %2mm"))
								  .arg(result.updatedFaceCount)
								  .arg(result.globalMaxDeviationMm, 0, 'f', 3);
				if (!result.qualityGatePassed)
				{
					msg += i18n(QStringLiteral(" [QUALITY GATE FAILED]"), QStringLiteral(" 【质量门控未通过】"));
				}
				m_host->logInfo(msg);
				logBrepUpdateFaceSummary(m_host, result.perFace);
				if (m_statusLabel)
				{
					m_statusLabel->setText(truncateStatusText(msg));
					m_statusLabel->setToolTip(msg);
				}
				refreshPointCloudList();
			}
			else
			{
				const QString failMsg = error.isEmpty()
					? i18n(QStringLiteral("Template B-rep refactor failed"),
						   QStringLiteral("模板 B-rep 重构失败"))
					: error;
				m_host->logError(failMsg);
				if (result.skippedBadBboxFaceCount > 0U)
				{
					m_host->logWarn(
						i18n(
							QStringLiteral("%1 face(s) skipped by bounds guard"),
							QStringLiteral("%1 面因包围盒守卫被跳过"))
							.arg(result.skippedBadBboxFaceCount));
				}
				logBrepUpdateSkippedFaceDiagnostic(m_host, result.perFace);
				logBrepUpdateFaceSummary(m_host, result.perFace);
				if (m_statusLabel)
				{
					m_statusLabel->setText(truncateStatusText(failMsg));
					m_statusLabel->setToolTip(failMsg);
				}
			}
			if (m_progress)
			{
				m_progress->setValue(ok ? 100 : 0);
			}
		});
}

// === 网格后处理 ===

std::string PointCloudDockWidget::selectedMeshTargetId() const
{
	if (!m_meshTargetCombo || m_meshTargetCombo->currentIndex() < 0)
	{
		return std::string();
	}
	return m_meshTargetCombo->currentData().toString().toStdString();
}

void PointCloudDockWidget::refreshMeshInfo()
{
	if (!m_meshInfoLabel || !m_host)
	{
		return;
	}
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!doc || id.empty())
	{
		m_meshInfoLabel->setText(i18n(QStringLiteral("Select a mesh"), QStringLiteral("请选择网格")));
		return;
	}
	IPluginPointCloudHost* pch = pointCloudHost();
	if (!pch)
	{
		return;
	}
	PluginMeshInfo info;
	if (pch->queryMeshInfo(doc, id, info))
	{
		m_meshInfoLabel->setText(
			i18n(QStringLiteral("Faces: %1, Vertices: %2"), QStringLiteral("面数: %1，顶点数: %2"))
				.arg(static_cast<qulonglong>(info.faceCount))
				.arg(static_cast<qulonglong>(info.vertexCount)));
	}
	else
	{
		m_meshInfoLabel->setText(i18n(QStringLiteral("Not a mesh"), QStringLiteral("非网格对象")));
	}
}

void PointCloudDockWidget::onMeshSimplifyClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginMeshSimplifyParams params;
	params.targetFaceCount = m_simplifyTargetSpin->value();
	params.qualityThreshold = m_simplifyQualitySpin->value();
	params.resultOptions.displayName = i18n(QStringLiteral("Simplified"), QStringLiteral("简化网格"));
	params.resultOptions.selectInTree = true;
	pch->simplifyMesh(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
			refreshMeshInfo();
		});
}

void PointCloudDockWidget::onMeshSmoothLaplacianClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginMeshSmoothParams params;
	params.iterations = m_smoothIterSpin->value();
	params.useImplicitFairing = false;
	params.resultOptions.displayName = i18n(QStringLiteral("Smoothed"), QStringLiteral("平滑网格"));
	params.resultOptions.selectInTree = true;
	pch->smoothMesh(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
			refreshMeshInfo();
		});
}

void PointCloudDockWidget::onMeshSmoothImplicitClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginMeshSmoothParams params;
	params.iterations = m_smoothIterSpin->value();
	params.useImplicitFairing = true;
	params.resultOptions.displayName = i18n(QStringLiteral("Fairing"), QStringLiteral("隐式平滑"));
	params.resultOptions.selectInTree = true;
	pch->smoothMesh(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
			refreshMeshInfo();
		});
}

void PointCloudDockWidget::onMeshRepairClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginMeshRepairParams params;
	params.resultOptions.displayName = i18n(QStringLiteral("Repaired"), QStringLiteral("修复网格"));
	params.resultOptions.selectInTree = true;
	pch->repairMesh(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
			refreshMeshInfo();
		});
}

void PointCloudDockWidget::onMeshRemeshClicked()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	PluginMeshRemeshParams params;
	params.targetEdgeLengthMm = m_remeshEdgeSpin->value();
	params.iterations = 3;
	params.resultOptions.displayName = i18n(QStringLiteral("Remeshed"), QStringLiteral("重网格"));
	params.resultOptions.selectInTree = true;
	pch->remeshMeshIsotropic(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginPointCloudJobResult& result) {
			runFinished(ok, error, result);
			refreshMeshInfo();
		});
}

void PointCloudDockWidget::refreshSurfaceReconstructSummary(const PluginMeshSurfaceReconstructReport& report)
{
	if (!m_surfaceReconSummaryLabel)
	{
		return;
	}
	m_surfaceReconSummaryLabel->setText(
		i18n(QStringLiteral("Patches: %1 | Junctions: %2 | Max dev: %3 mm | Fairing: %4 | C2: %5"),
			QStringLiteral("分块: %1 | 交汇: %2 | 最大偏差: %3 mm | 光顺指标: %4 | C2: %5"))
			.arg(report.patchCount)
			.arg(report.junctionCount)
			.arg(report.maxDeviationMm, 0, 'f', 4)
			.arg(report.globalFairingMetric, 0, 'f', 6)
			.arg(report.c2BlendSucceeded
					 ? i18n(QStringLiteral("yes"), QStringLiteral("是"))
					 : i18n(QStringLiteral("no"), QStringLiteral("否"))));
}

void PointCloudDockWidget::onSurfaceReconstructClicked()
{
	if (!m_host || m_host->hostVersion() < 0x00010C00U)
	{
		if (m_host)
		{
			m_host->logWarn(i18n(
				QStringLiteral("Surface reconstruct requires host 1.12.0+"),
				QStringLiteral("曲面重构需要宿主 1.12.0+")));
		}
		return;
	}
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	setBusy(true);
	const PluginMeshSurfaceReconstructParams params = buildSurfaceReconParams();
	pch->reconstructSurfaceFromMesh(
		doc,
		id,
		params,
		[this](const bool ok, const QString& error, const PluginMeshSurfaceReconstructReport& report) {
			setBusy(false);
			if (!m_host)
			{
				return;
			}
			if (ok)
			{
				refreshSurfaceReconstructSummary(report);
				const QString msg = i18n(
					QStringLiteral("Surface reconstruct done: %1 patches, max dev %2 mm"),
					QStringLiteral("曲面重构完成: %1 片, 最大偏差 %2 mm"))
										.arg(report.patchCount)
										.arg(report.maxDeviationMm, 0, 'f', 4);
				m_host->logInfo(msg);
				if (m_statusLabel)
				{
					m_statusLabel->setText(msg);
				}
			}
			else
			{
				m_host->logError(error);
				if (m_statusLabel)
				{
					m_statusLabel->setText(error);
				}
			}
		});
}

namespace
{
QString surfaceReconStageTitleZh(const PluginMeshSurfaceReconstructStage stage)
{
	switch (stage)
	{
	case PluginMeshSurfaceReconstructStage::Preprocess:
		return QStringLiteral("预处理");
	case PluginMeshSurfaceReconstructStage::Partition:
		return QStringLiteral("分块");
	case PluginMeshSurfaceReconstructStage::Sample:
		return QStringLiteral("栅格采样");
	case PluginMeshSurfaceReconstructStage::Fit:
		return QStringLiteral("NURBS拟合");
	case PluginMeshSurfaceReconstructStage::BoundaryBlend:
		return QStringLiteral("边界混合");
	case PluginMeshSurfaceReconstructStage::JunctionBlend:
		return QStringLiteral("交汇混合");
	case PluginMeshSurfaceReconstructStage::Fair:
		return QStringLiteral("光顺");
	case PluginMeshSurfaceReconstructStage::Assemble:
		return QStringLiteral("装配输出");
	default:
		return QStringLiteral("未知阶段");
	}
}
} // namespace

void PointCloudDockWidget::onPartitionModeChanged()
{
	updatePartitionModeUi();
}

void PointCloudDockWidget::updatePartitionModeUi()
{
	if (!m_partitionModeCombo || !m_v3PartitionParamsWidget || !m_hybridPartitionParamsWidget)
	{
		return;
	}
	const auto mode = static_cast<PluginMeshSurfacePartitionMode>(m_partitionModeCombo->currentData().toInt());
	const bool hybrid = mode == PluginMeshSurfacePartitionMode::HybridNormalCvt;
	m_v3PartitionParamsWidget->setVisible(!hybrid);
	m_hybridPartitionParamsWidget->setVisible(hybrid);
	if (m_patchCountSpin)
	{
		m_patchCountSpin->setToolTip(
			hybrid
				? i18n(QStringLiteral("0=paper adaptive CVT; >0 scales secondary sample density"),
					QStringLiteral("0=论文自适应二次 CVT；>0 缩放采样密度"))
				: i18n(QStringLiteral("0=auto sqrt(faces/80); increase to split pits/corners"),
					QStringLiteral("0=自动 sqrt(面数/80)；增大可让凹坑等区域单独成块")));
	}
}

PluginMeshSurfaceReconstructParams PointCloudDockWidget::buildSurfaceReconParams() const
{
	PluginMeshSurfaceReconstructParams params;
	params.normalSmoothIterations = m_normalSmoothIterSpin->value();
	params.featureThresholdC0 = m_featureThresholdSpin->value();
	params.runVcgRepairFirst = m_runVcgRepairCheck->isChecked();
	if (m_runIsotropicRemeshCheck)
	{
		params.runIsotropicRemesh = m_runIsotropicRemeshCheck->isChecked();
	}
	if (m_remeshTargetEdgeSpin)
	{
		params.remeshTargetEdgeLengthMm = m_remeshTargetEdgeSpin->value();
	}
	if (m_remeshIterSpin)
	{
		params.remeshIterations = m_remeshIterSpin->value();
	}
	params.patchCountHint = m_patchCountSpin->value();
	if (m_partitionModeCombo)
	{
		params.partitionMode = static_cast<PluginMeshSurfacePartitionMode>(m_partitionModeCombo->currentData().toInt());
	}
	if (m_partitionNormalSmoothSpin)
	{
		params.partitionNormalSmoothIters = m_partitionNormalSmoothSpin->value();
	}
	if (m_featureAnglePercentileSpin)
	{
		params.featureAnglePercentile = m_featureAnglePercentileSpin->value();
	}
	if (m_hybridFeatureAngleSpin)
	{
		params.hybridFeatureAngleDeg = m_hybridFeatureAngleSpin->value();
	}
	if (m_hybridClusterItersSpin)
	{
		params.hybridClusterMaxIters = m_hybridClusterItersSpin->value();
	}
	if (m_hybridSampleScaleSpin)
	{
		params.hybridSecondarySampleScale = m_hybridSampleScaleSpin->value();
	}
	if (m_hybridRegionAdjustCheck)
	{
		params.hybridEnableRegionAdjust = m_hybridRegionAdjustCheck->isChecked();
	}
	if (m_hybridMergeCosHighSpin)
	{
		params.hybridMergeCosHigh = m_hybridMergeCosHighSpin->value();
	}
	if (m_hybridMergeCosLowBaseSpin)
	{
		params.hybridMergeCosLowBase = m_hybridMergeCosLowBaseSpin->value();
	}
	if (m_hybridMergeCosLowScaleSpin)
	{
		params.hybridMergeCosLowScale = m_hybridMergeCosLowScaleSpin->value();
	}
	if (m_hybridSmallRegionRatioSpin)
	{
		params.hybridSmallRegionRatio = m_hybridSmallRegionRatioSpin->value();
	}
	if (m_hybridSmallRegionMinSpin)
	{
		params.hybridSmallRegionMin = m_hybridSmallRegionMinSpin->value();
	}
	if (m_hybridSmallRegionMaxSpin)
	{
		params.hybridSmallRegionMax = m_hybridSmallRegionMaxSpin->value();
	}
	if (m_hybridCollapseValenceSpin)
	{
		params.hybridCollapseValenceSumMax = m_hybridCollapseValenceSpin->value();
	}
	if (m_hybridCollapseLengthRatioSpin)
	{
		params.hybridCollapseLengthRatio = m_hybridCollapseLengthRatioSpin->value();
	}
	if (m_hybridAdjustPassesSpin)
	{
		params.hybridRegionAdjustMaxPasses = m_hybridAdjustPassesSpin->value();
	}
	params.samplesPerPatchEdge = m_samplesPerEdgeSpin->value();
	params.targetUvSpacingMm = m_uvSpacingSpin->value();
	params.minSamplesPerEdge = m_minSamplesSpin->value();
	params.maxSamplesPerEdge = m_maxSamplesSpin->value();
	params.maxFitGridPerEdge = m_maxFitGridSpin->value();
	params.fitUvSpacingMm = m_fitUvSpacingSpin->value();
	if (m_sampleRateSpin)
	{
		params.sampleRateFactor = m_sampleRateSpin->value();
	}
	if (m_ctrlPtDensitySpin)
	{
		params.controlPointDensityFactor = m_ctrlPtDensitySpin->value();
	}
	if (m_nurbsFitModeCombo)
	{
		params.fitMode = static_cast<PluginMeshSurfaceNurbsFitMode>(m_nurbsFitModeCombo->currentData().toInt());
	}
	params.blendStripWidth = m_blendStripWidthSpin->value();
	params.fairingEpsilon = m_fairingEpsilonSpin->value();
	params.fairingMaxIterations = m_fairingMaxIterSpin->value();
	params.tessellateLinearDeflectionMm = m_tessellateDeflectionSpin->value();
	params.displayName = i18n(QStringLiteral("Reconstructed B-rep"), QStringLiteral("重构曲面"));
	params.selectInTree = true;
	if (m_exportPreprocessedMeshCheck)
	{
		params.exportPreprocessedMeshToScene = m_exportPreprocessedMeshCheck->isChecked();
	}
	return params;
}

void PointCloudDockWidget::appendSurfaceReconLog(const QString& line)
{
	if (!m_surfaceReconLog)
	{
		return;
	}
	m_surfaceReconLog->append(line);
}

void PointCloudDockWidget::resetSurfaceReconSessionUi()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	if (pch && doc && m_surfaceReconSessionId.valid())
	{
		pch->clearMeshSurfaceReconstructSession(doc, m_surfaceReconSessionId);
	}
	m_surfaceReconSessionId = {};
	m_surfaceReconLastStage = PluginMeshSurfaceReconstructStage::None;
	m_surfaceReconMeshBackendId.clear();
	if (m_surfaceReconLog)
	{
		m_surfaceReconLog->clear();
	}
	if (m_surfaceReconSummaryLabel)
	{
		m_surfaceReconSummaryLabel->setText(
			i18n(QStringLiteral("No reconstruction yet"), QStringLiteral("尚未执行曲面重构")));
	}
	updateSurfaceReconButtonStates();
}

void PointCloudDockWidget::updateSurfaceReconButtonStates()
{
	if (!m_surfaceReconGroup || !m_host || m_host->hostVersion() < 0x00010D00U)
	{
		return;
	}
	const bool hasMesh = !selectedMeshTargetId().empty();
	const bool busy = m_busy;
	const auto last = m_surfaceReconLastStage;

	auto canRunStage = [&](const PluginMeshSurfaceReconstructStage stage) {
		if (busy || !hasMesh)
		{
			return false;
		}
		if (stage == PluginMeshSurfaceReconstructStage::Preprocess)
		{
			return last == PluginMeshSurfaceReconstructStage::None;
		}
		return static_cast<int>(stage) == static_cast<int>(last) + 1;
	};

	m_surfaceReconPreprocessBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Preprocess));
	m_surfaceReconPartitionBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Partition));
	m_surfaceReconSampleBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Sample));
	m_surfaceReconFitBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Fit));
	m_surfaceReconBoundaryBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::BoundaryBlend));
	m_surfaceReconJunctionBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::JunctionBlend));
	m_surfaceReconFairBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Fair));
	m_surfaceReconAssembleBtn->setEnabled(canRunStage(PluginMeshSurfaceReconstructStage::Assemble));
	m_surfaceReconResetBtn->setEnabled(!busy && m_surfaceReconSessionId.valid());
	if (m_surfaceReconBtn)
	{
		m_surfaceReconBtn->setEnabled(!busy && hasMesh);
	}
}

void PointCloudDockWidget::ensureSurfaceReconSession()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string meshId = selectedMeshTargetId();
	if (!pch || !doc || meshId.empty())
	{
		return;
	}
	if (m_surfaceReconSessionId.valid() && m_surfaceReconMeshBackendId == meshId)
	{
		return;
	}
	if (m_surfaceReconSessionId.valid())
	{
		pch->clearMeshSurfaceReconstructSession(doc, m_surfaceReconSessionId);
		m_surfaceReconSessionId = {};
		m_surfaceReconLastStage = PluginMeshSurfaceReconstructStage::None;
	}
	m_surfaceReconSessionId = pch->beginMeshSurfaceReconstructSession(doc, meshId);
	m_surfaceReconMeshBackendId = meshId;
	m_surfaceReconLastStage = PluginMeshSurfaceReconstructStage::None;
}

void PointCloudDockWidget::runSurfaceReconStage(const PluginMeshSurfaceReconstructStage stage)
{
	if (!m_host || m_host->hostVersion() < 0x00010D00U)
	{
		if (m_host)
		{
			m_host->logWarn(i18n(
				QStringLiteral("Staged surface reconstruct requires host 1.13.0+"),
				QStringLiteral("分阶段曲面重构需要宿主 1.13.0+")));
		}
		return;
	}
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshTargetId();
	if (!pch || !doc || id.empty())
	{
		return;
	}
	ensureSurfaceReconSession();
	if (!m_surfaceReconSessionId.valid())
	{
		if (m_host)
		{
			m_host->logError(i18n(
				QStringLiteral("Failed to begin surface reconstruct session"),
				QStringLiteral("无法创建曲面重构会话")));
		}
		return;
	}

	setBusy(true);
	appendSurfaceReconLog(
		i18n(QStringLiteral("Running %1..."), QStringLiteral("正在执行 %1…"))
			.arg(surfaceReconStageTitleZh(stage)));

	const PluginMeshSurfaceReconstructParams params = buildSurfaceReconParams();
	pch->runMeshSurfaceReconstructStage(
		doc,
		m_surfaceReconSessionId,
		stage,
		params,
		[this, stage](const bool ok, const QString& error, const PluginMeshSurfaceReconstructReport& report) {
			setBusy(false);
			if (!m_host)
			{
				return;
			}
			if (ok)
			{
				m_surfaceReconLastStage = report.lastCompletedStage;
				const QString summary = report.stageSummaryZh.isEmpty()
					? surfaceReconStageTitleZh(stage)
					: report.stageSummaryZh;
				appendSurfaceReconLog(summary);
				refreshSurfaceReconstructSummary(report);
				m_host->logInfo(
					i18n(QStringLiteral("[Surface reconstruct] %1"), QStringLiteral("[曲面重构] %1"))
						.arg(summary));
				if (m_statusLabel)
				{
					m_statusLabel->setText(summary);
				}
				if ((stage == PluginMeshSurfaceReconstructStage::Preprocess
						&& !report.preprocessedMeshBackendId.empty())
					|| (stage == PluginMeshSurfaceReconstructStage::Partition
						&& !report.partitionColoredMeshBackendId.empty())
					|| (stage == PluginMeshSurfaceReconstructStage::Fit
						&& !report.fitPreviewBrepBackendId.empty())
					|| (stage == PluginMeshSurfaceReconstructStage::BoundaryBlend
						&& !report.boundaryBlendPreviewBrepBackendId.empty())
					|| (stage == PluginMeshSurfaceReconstructStage::JunctionBlend
						&& !report.junctionBlendPreviewBrepBackendId.empty()))
				{
					// 仅刷新列表；会话仍绑定源网格
					refreshMeshExportList(m_surfaceReconMeshBackendId);
				}
			}
			else
			{
				appendSurfaceReconLog(
					i18n(QStringLiteral("Failed: %1"), QStringLiteral("失败: %1")).arg(error));
				m_host->logError(error);
				if (m_statusLabel)
				{
					m_statusLabel->setText(error);
				}
			}
			updateSurfaceReconButtonStates();
		});
}

void PointCloudDockWidget::onSurfaceReconstructResetSessionClicked()
{
	resetSurfaceReconSessionUi();
	appendSurfaceReconLog(i18n(QStringLiteral("Session reset"), QStringLiteral("会话已重置")));
	if (m_host)
	{
		m_host->logInfo(i18n(
			QStringLiteral("[Surface reconstruct] Session reset"),
			QStringLiteral("[曲面重构] 会话已重置")));
	}
}
