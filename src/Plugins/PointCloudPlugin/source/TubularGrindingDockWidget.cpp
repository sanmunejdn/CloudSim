#include "TubularGrindingDockWidget.h"
#include "IPluginDocument.h"
#include "IPluginHostContext.h"
#include "IPluginPointCloudHost.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QShowEvent>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QVBoxLayout>

namespace
{
QString templateKindLabel(const PluginTubularGrindingTemplateKind kind, const bool zh)
{
	switch (kind)
	{
	case PluginTubularGrindingTemplateKind::Helical:
		return zh ? QStringLiteral("螺旋") : QStringLiteral("Helical");
	case PluginTubularGrindingTemplateKind::Circumferential:
		return zh ? QStringLiteral("环形") : QStringLiteral("Circumferential");
	case PluginTubularGrindingTemplateKind::AxialParallel:
		return zh ? QStringLiteral("轴向母线") : QStringLiteral("Axial parallel");
	case PluginTubularGrindingTemplateKind::Zigzag:
		return zh ? QStringLiteral("锯齿") : QStringLiteral("Zigzag");
	default:
		return zh ? QStringLiteral("自动") : QStringLiteral("Auto");
	}
}

void styleParamHint(QLabel* hint)
{
	if (!hint)
	{
		return;
	}
	hint->setWordWrap(true);
	hint->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 11px;"));
}
} // namespace

TubularGrindingDockWidget::TubularGrindingDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
{
	auto* outer = new QVBoxLayout(this);
	outer->setContentsMargins(4, 4, 4, 4);
	auto* scroll = new QScrollArea(this);
	scroll->setWidgetResizable(true);
	auto* content = new QWidget(scroll);
	auto* layout = new QVBoxLayout(content);
	m_rootGroup = new QGroupBox(content);
	auto* groupLayout = new QVBoxLayout(m_rootGroup);
	auto* meshRow = new QHBoxLayout();
	m_meshLabel = new QLabel(m_rootGroup);
	m_meshCombo = new QComboBox(m_rootGroup);
	meshRow->addWidget(m_meshLabel);
	meshRow->addWidget(m_meshCombo, 1);
	groupLayout->addLayout(meshRow);
	m_paramTabs = new QTabWidget(m_rootGroup);
	// --- 中心线提取 ---
	auto* centerlinePage = new QWidget(m_paramTabs);
	auto* centerlineForm = new QFormLayout(centerlinePage);
	centerlineForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
	centerlineForm->setLabelAlignment(Qt::AlignTop);
	m_centerlineMethodCombo = new QComboBox(centerlinePage);
	m_centerlineMethodCombo->addItem(
		QStringLiteral("Laplacian"),
		static_cast<int>(PluginTubularGrindingCenterlineMethod::Laplacian));
	m_centerlineMethodCombo->addItem(
		QStringLiteral("OTLC"),
		static_cast<int>(PluginTubularGrindingCenterlineMethod::OtLc));
	addParamRow(
		centerlineForm,
		m_centerlineMethodLabel,
		m_centerlineMethodCombo,
		m_centerlineMethodHint,
		QString(),
		QString(),
		&m_centerlineMethodRow);
	m_sectionSpacingSpin = new QDoubleSpinBox(centerlinePage);
	m_sectionSpacingSpin->setRange(0.5, 50.0);
	m_sectionSpacingSpin->setSingleStep(0.5);
	m_sectionSpacingSpin->setDecimals(2);
	m_sectionSpacingSpin->setValue(2.0);
	m_sectionSpacingSpin->setSuffix(QStringLiteral(" mm"));
	addParamRow(
		centerlineForm,
		m_sectionSpacingLabel,
		m_sectionSpacingSpin,
		m_sectionSpacingHint,
		QString(),
		QString(),
		&m_sectionSpacingRow);
	m_centerlineIterSpin = new QSpinBox(centerlinePage);
	m_centerlineIterSpin->setRange(10, 300);
	m_centerlineIterSpin->setSingleStep(5);
	m_centerlineIterSpin->setValue(80);
	addParamRow(
		centerlineForm,
		m_centerlineIterLabel,
		m_centerlineIterSpin,
		m_centerlineIterHint,
		QString(),
		QString(),
		&m_centerlineIterRow);
	m_laplacianLambdaSpin = new QDoubleSpinBox(centerlinePage);
	m_laplacianLambdaSpin->setRange(0.01, 2.0);
	m_laplacianLambdaSpin->setSingleStep(0.02);
	m_laplacianLambdaSpin->setDecimals(2);
	m_laplacianLambdaSpin->setValue(0.1);
	addParamRow(
		centerlineForm,
		m_laplacianLambdaLabel,
		m_laplacianLambdaSpin,
		m_laplacianLambdaHint,
		QString(),
		QString(),
		&m_laplacianLambdaRow);
	m_laplacianAttractionSpin = new QDoubleSpinBox(centerlinePage);
	m_laplacianAttractionSpin->setRange(0.05, 2.0);
	m_laplacianAttractionSpin->setSingleStep(0.05);
	m_laplacianAttractionSpin->setDecimals(2);
	m_laplacianAttractionSpin->setValue(0.2);
	addParamRow(
		centerlineForm,
		m_laplacianAttractionLabel,
		m_laplacianAttractionSpin,
		m_laplacianAttractionHint,
		QString(),
		QString(),
		&m_laplacianAttractionRow);
	m_otSampleRateSpin = new QDoubleSpinBox(centerlinePage);
	m_otSampleRateSpin->setRange(0.01, 0.50);
	m_otSampleRateSpin->setSingleStep(0.01);
	m_otSampleRateSpin->setDecimals(2);
	m_otSampleRateSpin->setValue(0.10);
	addParamRow(
		centerlineForm,
		m_otSampleRateLabel,
		m_otSampleRateSpin,
		m_otSampleRateHint,
		QString(),
		QString(),
		&m_otSampleRateRow);
	m_otCostBetaSpin = new QDoubleSpinBox(centerlinePage);
	m_otCostBetaSpin->setRange(0.5, 10.0);
	m_otCostBetaSpin->setSingleStep(0.1);
	m_otCostBetaSpin->setDecimals(1);
	m_otCostBetaSpin->setValue(3.0);
	addParamRow(
		centerlineForm,
		m_otCostBetaLabel,
		m_otCostBetaSpin,
		m_otCostBetaHint,
		QString(),
		QString(),
		&m_otCostBetaRow);
	m_otcPreStepsSpin = new QSpinBox(centerlinePage);
	m_otcPreStepsSpin->setRange(1, 20);
	m_otcPreStepsSpin->setValue(3);
	addParamRow(
		centerlineForm,
		m_otcPreStepsLabel,
		m_otcPreStepsSpin,
		m_otcPreStepsHint,
		QString(),
		QString(),
		&m_otcPreStepsRow);
	m_otcOuterLoopsSpin = new QSpinBox(centerlinePage);
	m_otcOuterLoopsSpin->setRange(1, 20);
	m_otcOuterLoopsSpin->setValue(3);
	addParamRow(
		centerlineForm,
		m_otcOuterLoopsLabel,
		m_otcOuterLoopsSpin,
		m_otcOuterLoopsHint,
		QString(),
		QString(),
		&m_otcOuterLoopsRow);
	m_otLcOuterMaxItersSpin = new QSpinBox(centerlinePage);
	m_otLcOuterMaxItersSpin->setRange(5, 200);
	m_otLcOuterMaxItersSpin->setSingleStep(5);
	m_otLcOuterMaxItersSpin->setValue(40);
	addParamRow(
		centerlineForm,
		m_otLcOuterMaxItersLabel,
		m_otLcOuterMaxItersSpin,
		m_otLcOuterMaxItersHint,
		QString(),
		QString(),
		&m_otLcOuterMaxItersRow);
	m_minRootsSpin = new QSpinBox(centerlinePage);
	m_minRootsSpin->setRange(0, 200);
	m_minRootsSpin->setSingleStep(5);
	m_minRootsSpin->setValue(0);
	m_minRootsSpin->setSpecialValueText(QStringLiteral("Auto"));
	addParamRow(
		centerlineForm,
		m_minRootsLabel,
		m_minRootsSpin,
		m_minRootsHint,
		QString(),
		QString(),
		&m_minRootsRow);
	m_paramTabs->addTab(centerlinePage, QStringLiteral("Centerline"));
	// --- 轨迹与投影 ---
	auto* trajectoryPage = new QWidget(m_paramTabs);
	auto* trajectoryForm = new QFormLayout(trajectoryPage);
	trajectoryForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
	trajectoryForm->setLabelAlignment(Qt::AlignTop);
	m_templateCombo = new QComboBox(trajectoryPage);
	m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Auto, true),
		static_cast<int>(PluginTubularGrindingTemplateKind::Auto));
	m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Helical, true),
		static_cast<int>(PluginTubularGrindingTemplateKind::Helical));
	m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Circumferential, true),
		static_cast<int>(PluginTubularGrindingTemplateKind::Circumferential));
	m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::AxialParallel, true),
		static_cast<int>(PluginTubularGrindingTemplateKind::AxialParallel));
	m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Zigzag, true),
		static_cast<int>(PluginTubularGrindingTemplateKind::Zigzag));
	addParamRow(
		trajectoryForm,
		m_templateLabel,
		m_templateCombo,
		m_templateHint,
		QString(),
		QString());
	m_helicalCoilsSpin = new QSpinBox(trajectoryPage);
	m_helicalCoilsSpin->setRange(1, 64);
	m_helicalCoilsSpin->setValue(8);
	addParamRow(
		trajectoryForm,
		m_helicalCoilsLabel,
		m_helicalCoilsSpin,
		m_helicalCoilsHint,
		QString(),
		QString());
	m_projectionDistSpin = new QDoubleSpinBox(trajectoryPage);
	m_projectionDistSpin->setRange(1.0, 100.0);
	m_projectionDistSpin->setSingleStep(1.0);
	m_projectionDistSpin->setDecimals(1);
	m_projectionDistSpin->setValue(10.0);
	m_projectionDistSpin->setSuffix(QStringLiteral(" mm"));
	addParamRow(
		trajectoryForm,
		m_projectionDistLabel,
		m_projectionDistSpin,
		m_projectionDistHint,
		QString(),
		QString());
	m_paramTabs->addTab(trajectoryPage, QStringLiteral("Trajectory"));
	groupLayout->addWidget(m_paramTabs);
	auto* stageRow = new QHBoxLayout();
	m_centerlineBtn = new QPushButton(m_rootGroup);
	m_templateBtn = new QPushButton(m_rootGroup);
	m_projectBtn = new QPushButton(m_rootGroup);
	stageRow->addWidget(m_centerlineBtn);
	stageRow->addWidget(m_templateBtn);
	stageRow->addWidget(m_projectBtn);
	groupLayout->addLayout(stageRow);
	m_resetBtn = new QPushButton(m_rootGroup);
	groupLayout->addWidget(m_resetBtn);
	m_log = new QTextEdit(m_rootGroup);
	m_log->setReadOnly(true);
	m_log->setMaximumHeight(120);
	groupLayout->addWidget(m_log);
	m_summaryLabel = new QLabel(m_rootGroup);
	m_summaryLabel->setWordWrap(true);
	groupLayout->addWidget(m_summaryLabel);
	layout->addWidget(m_rootGroup);
	scroll->setWidget(content);
	outer->addWidget(scroll);
	connect(m_meshCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, &TubularGrindingDockWidget::onMeshSelectionChanged);
	connect(m_centerlineMethodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, &TubularGrindingDockWidget::onCenterlineMethodChanged);
	connect(m_resetBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onResetSessionClicked);
	connect(m_centerlineBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onCenterlineClicked);
	connect(m_templateBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onTemplateClicked);
	connect(m_projectBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onProjectClicked);
	applyLanguage();
	syncCenterlineMethodForSource();
	updateCenterlineParamVisibility();
	updateButtonStates();
}

void TubularGrindingDockWidget::addParamRow(
	QFormLayout* form,
	QLabel*& labelOut,
	QWidget* editor,
	QLabel*& hintOut,
	const QString& labelText,
	const QString& hintText,
	QWidget** outFieldWrap)
{
	if (!form || !editor)
	{
		return;
	}
	labelOut = new QLabel(labelText, form->parentWidget());
	labelOut->setAlignment(Qt::AlignTop | Qt::AlignRight);
	hintOut = new QLabel(hintText, form->parentWidget());
	styleParamHint(hintOut);
	auto* fieldWrap = new QWidget(form->parentWidget());
	auto* fieldLayout = new QVBoxLayout(fieldWrap);
	fieldLayout->setContentsMargins(0, 0, 0, 0);
	fieldLayout->setSpacing(4);
	fieldLayout->addWidget(editor);
	fieldLayout->addWidget(hintOut);
	form->addRow(labelOut, fieldWrap);
	if (outFieldWrap)
	{
		*outFieldWrap = fieldWrap;
	}
}

void TubularGrindingDockWidget::setParamRowVisible(QLabel* label, QWidget* fieldWrap, bool visible)
{
	if (label)
	{
		label->setVisible(visible);
	}
	if (fieldWrap)
	{
		fieldWrap->setVisible(visible);
	}
}

void TubularGrindingDockWidget::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	refreshMeshList();
}

QString TubularGrindingDockWidget::i18n(const QString& en, const QString& zh) const
{
	return m_host && m_host->useChinese() ? zh : en;
}

IPluginDocument* TubularGrindingDockWidget::activeDoc() const
{
	return m_host ? m_host->activeDocument() : nullptr;
}

IPluginPointCloudHost* TubularGrindingDockWidget::pointCloudHost() const
{
	return m_host ? m_host->pointCloudHost() : nullptr;
}

std::string TubularGrindingDockWidget::selectedMeshBackendId() const
{
	if (!m_meshCombo || m_meshCombo->currentIndex() < 0)
	{
		return std::string();
	}
	return m_meshCombo->currentData().toString().toStdString();
}

bool TubularGrindingDockWidget::selectedSourceIsPointCloud() const
{
	IPluginDocument* doc = activeDoc();
	const std::string id = selectedMeshBackendId();
	if (!doc || id.empty())
	{
		return false;
	}
	return doc->backendClassName(id) == "PointCloudBackendData";
}

PluginTubularGrindingCenterlineMethod TubularGrindingDockWidget::selectedCenterlineMethod() const
{
	if (!m_centerlineMethodCombo || m_centerlineMethodCombo->currentIndex() < 0)
	{
		return PluginTubularGrindingCenterlineMethod::Laplacian;
	}
	return static_cast<PluginTubularGrindingCenterlineMethod>(
		m_centerlineMethodCombo->currentData().toInt());
}

void TubularGrindingDockWidget::syncCenterlineMethodForSource()
{
	if (!m_centerlineMethodCombo)
	{
		return;
	}
	if (selectedSourceIsPointCloud()
		&& selectedCenterlineMethod() == PluginTubularGrindingCenterlineMethod::Laplacian)
	{
		const QSignalBlocker blocker(m_centerlineMethodCombo);
		const int otIdx = m_centerlineMethodCombo->findData(
			static_cast<int>(PluginTubularGrindingCenterlineMethod::OtLc));
		if (otIdx >= 0)
		{
			m_centerlineMethodCombo->setCurrentIndex(otIdx);
		}
	}
}

void TubularGrindingDockWidget::updateCenterlineParamVisibility()
{
	const bool isOtLc =
		selectedCenterlineMethod() == PluginTubularGrindingCenterlineMethod::OtLc;
	setParamRowVisible(m_sectionSpacingLabel, m_sectionSpacingRow, true);
	setParamRowVisible(m_centerlineIterLabel, m_centerlineIterRow, !isOtLc);
	setParamRowVisible(m_laplacianLambdaLabel, m_laplacianLambdaRow, !isOtLc);
	setParamRowVisible(m_laplacianAttractionLabel, m_laplacianAttractionRow, !isOtLc);
	setParamRowVisible(m_otSampleRateLabel, m_otSampleRateRow, isOtLc);
	setParamRowVisible(m_otCostBetaLabel, m_otCostBetaRow, isOtLc);
	setParamRowVisible(m_otcPreStepsLabel, m_otcPreStepsRow, isOtLc);
	setParamRowVisible(m_otcOuterLoopsLabel, m_otcOuterLoopsRow, isOtLc);
	setParamRowVisible(m_otLcOuterMaxItersLabel, m_otLcOuterMaxItersRow, isOtLc);
	setParamRowVisible(m_minRootsLabel, m_minRootsRow, isOtLc);
}

void TubularGrindingDockWidget::onCenterlineMethodChanged()
{
	updateCenterlineParamVisibility();
	updateButtonStates();
}

PluginTubularGrindingParams TubularGrindingDockWidget::buildParams() const
{
	PluginTubularGrindingParams params;
	params.sectionSpacingMm = m_sectionSpacingSpin ? m_sectionSpacingSpin->value() : 2.0;
	params.centerlineIterations = m_centerlineIterSpin ? m_centerlineIterSpin->value() : 80;
	params.laplacianLambda = m_laplacianLambdaSpin ? m_laplacianLambdaSpin->value() : 0.1;
	params.laplacianAttraction = m_laplacianAttractionSpin ? m_laplacianAttractionSpin->value() : 0.2;
	params.centerlineMethod = selectedCenterlineMethod();
	params.otSampleRate = m_otSampleRateSpin ? m_otSampleRateSpin->value() : 0.10;
	params.otCostBeta = m_otCostBetaSpin ? m_otCostBetaSpin->value() : 3.0;
	params.otcPreSteps = m_otcPreStepsSpin ? m_otcPreStepsSpin->value() : 3;
	params.otcOuterLoops = m_otcOuterLoopsSpin ? m_otcOuterLoopsSpin->value() : 3;
	params.otLcOuterMaxIters = m_otLcOuterMaxItersSpin ? m_otLcOuterMaxItersSpin->value() : 40;
	params.minRootsBySamples = m_minRootsSpin ? m_minRootsSpin->value() : 0;
	params.projectionMaxDistMm = m_projectionDistSpin ? m_projectionDistSpin->value() : 10.0;
	params.helicalCoils = m_helicalCoilsSpin ? m_helicalCoilsSpin->value() : 8;
	if (m_templateCombo)
	{
		params.templateKind = static_cast<PluginTubularGrindingTemplateKind>(
			m_templateCombo->currentData().toInt());
	}
	return params;
}

void TubularGrindingDockWidget::appendLog(const QString& line)
{
	if (m_log)
	{
		m_log->append(line);
	}
}

void TubularGrindingDockWidget::applyLanguage()
{
	const bool zh = m_host && m_host->useChinese();
	if (m_rootGroup)
	{
		m_rootGroup->setTitle(zh ? QStringLiteral("管状铸件特征构建") : QStringLiteral("Tubular feature build"));
	}
	if (m_meshLabel)
	{
		m_meshLabel->setText(zh ? QStringLiteral("数据源") : QStringLiteral("Source"));
	}
	if (m_paramTabs)
	{
		m_paramTabs->setTabText(0, zh ? QStringLiteral("中心线提取") : QStringLiteral("Centerline"));
		m_paramTabs->setTabText(1, zh ? QStringLiteral("轨迹与投影") : QStringLiteral("Trajectory"));
	}
	if (m_sectionSpacingLabel)
	{
		m_sectionSpacingLabel->setText(zh ? QStringLiteral("截面间距") : QStringLiteral("Section spacing"));
	}
	if (m_sectionSpacingHint)
	{
		m_sectionSpacingHint->setText(zh
			? QStringLiteral("PCA 质心分箱宽度与输出中心线采样间距。过大易漏点，过小采样密、计算慢。")
			: QStringLiteral("PCA bin width and output centerline spacing. Too large skips samples; too small is dense and slower."));
	}
	if (m_sectionSpacingSpin)
	{
		m_sectionSpacingSpin->setToolTip(m_sectionSpacingHint->text());
	}
	if (m_centerlineIterLabel)
	{
		m_centerlineIterLabel->setText(zh ? QStringLiteral("收缩迭代次数") : QStringLiteral("Contraction iterations"));
	}
	if (m_centerlineIterHint)
	{
		m_centerlineIterHint->setText(zh
			? QStringLiteral("Laplacian 收缩 + 边塌缩循环次数。增大更贴近骨架，过大可能塌没或断图。")
			: QStringLiteral("Laplacian contraction and edge-collapse loops. Higher pulls toward skeleton; too high may collapse or disconnect the graph."));
	}
	if (m_centerlineIterSpin)
	{
		m_centerlineIterSpin->setToolTip(m_centerlineIterHint->text());
	}
	if (m_laplacianLambdaLabel)
	{
		m_laplacianLambdaLabel->setText(zh ? QStringLiteral("收缩强度 λ") : QStringLiteral("Contraction strength λ"));
	}
	if (m_laplacianLambdaHint)
	{
		m_laplacianLambdaHint->setText(zh
			? QStringLiteral("映射为中期锚定权重峰值（约 10–200）。越大收缩越快，中心线易贴壳或过度收缩。")
			: QStringLiteral("Maps to peak anchor weight (~10–200). Higher contracts faster; may stick to shell or over-shrink."));
	}
	if (m_laplacianLambdaSpin)
	{
		m_laplacianLambdaSpin->setToolTip(m_laplacianLambdaHint->text());
	}
	if (m_laplacianAttractionLabel)
	{
		m_laplacianAttractionLabel->setText(zh ? QStringLiteral("初始锚定强度") : QStringLiteral("Initial anchoring"));
	}
	if (m_laplacianAttractionHint)
	{
		m_laplacianAttractionHint->setText(zh
			? QStringLiteral("前期把顶点拉回原始表面的力度。增大更稳、不易飞点；过大则难向管腔中心收缩。")
			: QStringLiteral("Early pull toward original surface. Higher stabilizes; too high hinders inward shrink toward the lumen."));
	}
	if (m_laplacianAttractionSpin)
	{
		m_laplacianAttractionSpin->setToolTip(m_laplacianAttractionHint->text());
	}
	if (m_centerlineMethodLabel)
	{
		m_centerlineMethodLabel->setText(zh ? QStringLiteral("提取算法") : QStringLiteral("Algorithm"));
	}
	if (m_centerlineMethodCombo)
	{
		const int lapIdx = m_centerlineMethodCombo->findData(
			static_cast<int>(PluginTubularGrindingCenterlineMethod::Laplacian));
		const int otIdx = m_centerlineMethodCombo->findData(
			static_cast<int>(PluginTubularGrindingCenterlineMethod::OtLc));
		if (lapIdx >= 0)
		{
			m_centerlineMethodCombo->setItemText(
				lapIdx,
				zh ? QStringLiteral("Laplacian 骨架") : QStringLiteral("Laplacian skeleton"));
		}
		if (otIdx >= 0)
		{
			m_centerlineMethodCombo->setItemText(
				otIdx,
				zh ? QStringLiteral("OTLC 骨架") : QStringLiteral("OTLC skeleton"));
		}
	}
	if (m_centerlineMethodHint)
	{
		m_centerlineMethodHint->setText(zh
			? QStringLiteral("Laplacian 适用于网格；点云请选 OTLC。切换后下方参数同步更新。")
			: QStringLiteral("Laplacian for mesh; use OTLC for point clouds. Parameters below follow the selected method."));
	}
	if (m_otSampleRateLabel)
	{
		m_otSampleRateLabel->setText(zh ? QStringLiteral("OT 采样率") : QStringLiteral("OT sample rate"));
	}
	if (m_otSampleRateHint)
	{
		m_otSampleRateHint->setText(zh
			? QStringLiteral("最优传输下采样比例（相对输入点数）。越小越快但骨架可能稀疏。")
			: QStringLiteral("OT downsample fraction of input points. Lower is faster but may sparsify the skeleton."));
	}
	if (m_otCostBetaLabel)
	{
		m_otCostBetaLabel->setText(zh ? QStringLiteral("OT 代价 β") : QStringLiteral("OT cost β"));
	}
	if (m_otCostBetaHint)
	{
		m_otCostBetaHint->setText(zh
			? QStringLiteral("质心更新时距离权重的幂指数。越大越强调近邻匹配。")
			: QStringLiteral("Power on distance weights during barycentric OT update. Higher favors nearer matches."));
	}
	if (m_otcPreStepsLabel)
	{
		m_otcPreStepsLabel->setText(zh ? QStringLiteral("OTC 预迭代") : QStringLiteral("OTC pre-steps"));
	}
	if (m_otcPreStepsHint)
	{
		m_otcPreStepsHint->setText(zh
			? QStringLiteral("外层 Laplacian 前的 OT 聚类预处理步数。")
			: QStringLiteral("OT cluster-merge steps before the outer Laplacian loop."));
	}
	if (m_otcOuterLoopsLabel)
	{
		m_otcOuterLoopsLabel->setText(zh ? QStringLiteral("OTC 内层循环") : QStringLiteral("OTC inner loops"));
	}
	if (m_otcOuterLoopsHint)
	{
		m_otcOuterLoopsHint->setText(zh
			? QStringLiteral("每轮外层迭代内的 OT 更新 + 聚类合并次数。")
			: QStringLiteral("OT update + cluster merge passes per outer iteration."));
	}
	if (m_otLcOuterMaxItersLabel)
	{
		m_otLcOuterMaxItersLabel->setText(zh ? QStringLiteral("OTLC 外层迭代") : QStringLiteral("OTLC outer iters"));
	}
	if (m_otLcOuterMaxItersHint)
	{
		m_otLcOuterMaxItersHint->setText(zh
			? QStringLiteral("约束 Laplacian + OTC 交替的最大轮数。")
			: QStringLiteral("Max alternating constrained-Laplacian and OTC rounds."));
	}
	if (m_templateLabel)
	{
		m_templateLabel->setText(zh ? QStringLiteral("模板类型") : QStringLiteral("Template kind"));
	}
	if (m_templateHint)
	{
		m_templateHint->setText(zh
			? QStringLiteral("沿中心线生成打磨/涂胶理想点位的轨迹模式。Auto 按管段几何自动选择。")
			: QStringLiteral("Ideal tool-point pattern along the centerline. Auto picks from segment geometry."));
	}
	if (m_templateCombo)
	{
		const int prev = m_templateCombo->currentIndex();
		const QSignalBlocker blocker(m_templateCombo);
		m_templateCombo->clear();
		m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Auto, zh),
			static_cast<int>(PluginTubularGrindingTemplateKind::Auto));
		m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Helical, zh),
			static_cast<int>(PluginTubularGrindingTemplateKind::Helical));
		m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Circumferential, zh),
			static_cast<int>(PluginTubularGrindingTemplateKind::Circumferential));
		m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::AxialParallel, zh),
			static_cast<int>(PluginTubularGrindingTemplateKind::AxialParallel));
		m_templateCombo->addItem(templateKindLabel(PluginTubularGrindingTemplateKind::Zigzag, zh),
			static_cast<int>(PluginTubularGrindingTemplateKind::Zigzag));
		if (prev >= 0 && prev < m_templateCombo->count())
		{
			m_templateCombo->setCurrentIndex(prev);
		}
	}
	if (m_helicalCoilsLabel)
	{
		m_helicalCoilsLabel->setText(zh ? QStringLiteral("螺旋圈数") : QStringLiteral("Helical coils"));
	}
	if (m_helicalCoilsHint)
	{
		m_helicalCoilsHint->setText(zh
			? QStringLiteral("螺旋模板沿中心线绕行的圈数，仅在选择螺旋模板时生效。")
			: QStringLiteral("Number of helical turns; used only when the helical template is selected."));
	}
	if (m_projectionDistLabel)
	{
		m_projectionDistLabel->setText(zh ? QStringLiteral("投影最大距离") : QStringLiteral("Projection max distance"));
	}
	if (m_projectionDistHint)
	{
		m_projectionDistHint->setText(zh
			? QStringLiteral("模板点沿法向射线搜索管壁的最大长度。过小易漏投，过大可能投到对面壁面。")
			: QStringLiteral("Max ray length from template points to hit the mesh. Too short misses; too long may hit the far wall."));
	}
	if (m_projectionDistSpin)
	{
		m_projectionDistSpin->setToolTip(m_projectionDistHint->text());
	}
	if (m_centerlineBtn)
	{
		m_centerlineBtn->setText(zh ? QStringLiteral("运行中心线") : QStringLiteral("Run centerline"));
	}
	if (m_templateBtn)
	{
		m_templateBtn->setText(zh ? QStringLiteral("模板点位") : QStringLiteral("Templates"));
	}
	if (m_projectBtn)
	{
		m_projectBtn->setText(zh ? QStringLiteral("表面投影") : QStringLiteral("Project"));
	}
	if (m_resetBtn)
	{
		m_resetBtn->setText(zh ? QStringLiteral("重置会话") : QStringLiteral("Reset session"));
	}
	if (m_summaryLabel)
	{
		m_summaryLabel->setText(zh ? QStringLiteral("尚未执行特征构建") : QStringLiteral("No feature build yet"));
	}
	updateCenterlineParamVisibility();
}

void TubularGrindingDockWidget::refreshMeshList()
{
	if (!m_meshCombo || !m_host)
	{
		return;
	}
	const std::string prevId = selectedMeshBackendId();
	const QSignalBlocker blocker(m_meshCombo);
	m_meshCombo->clear();
	IPluginDocument* doc = activeDoc();
	if (!doc)
	{
		updateButtonStates();
		return;
	}
	for (const std::string& id : doc->backendIds())
	{
		const std::string className = doc->backendClassName(id);
		QString suffix;
		if (className == "Model")
		{
			suffix = i18n(QStringLiteral(" [Mesh]"), QStringLiteral(" [网格]"));
		}
		else if (className == "PointCloudBackendData")
		{
			suffix = i18n(QStringLiteral(" [PC]"), QStringLiteral(" [点云]"));
		}
		else
		{
			continue;
		}
		m_meshCombo->addItem(
			QString::fromStdString(doc->backendDisplayName(id)) + suffix,
			QString::fromStdString(id));
	}
	if (!prevId.empty())
	{
		for (int i = 0; i < m_meshCombo->count(); ++i)
		{
			if (m_meshCombo->itemData(i).toString().toStdString() == prevId)
			{
				m_meshCombo->setCurrentIndex(i);
				break;
			}
		}
	}
	updateButtonStates();
	syncCenterlineMethodForSource();
	updateCenterlineParamVisibility();
}

void TubularGrindingDockWidget::resetSessionUi()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	if (pch && doc && m_sessionId.valid())
	{
		pch->clearTubularGrindingSession(doc, m_sessionId);
	}
	m_sessionId = {};
	m_lastStage = PluginTubularGrindingStage::None;
	m_meshBackendId.clear();
	if (m_log)
	{
		m_log->clear();
	}
	if (m_summaryLabel)
	{
		m_summaryLabel->setText(i18n(QStringLiteral("No feature build yet"), QStringLiteral("尚未执行特征构建")));
	}
	updateButtonStates();
}

void TubularGrindingDockWidget::updateButtonStates()
{
	if (!m_host || m_host->hostVersion() < 0x00010F00U)
	{
		if (m_rootGroup)
		{
			m_rootGroup->setEnabled(false);
		}
		return;
	}
	const bool hasMesh = !selectedMeshBackendId().empty();
	const bool laplacianOnPointCloud =
		hasMesh
		&& selectedSourceIsPointCloud()
		&& selectedCenterlineMethod() == PluginTubularGrindingCenterlineMethod::Laplacian;
	const auto last = m_lastStage;
	auto canRun = [&](const PluginTubularGrindingStage stage) {
		if (m_busy || !hasMesh || laplacianOnPointCloud)
		{
			return false;
		}
		if (stage == PluginTubularGrindingStage::Centerline)
		{
			return last == PluginTubularGrindingStage::None;
		}
		return static_cast<int>(stage) == static_cast<int>(last) + 1;
	};
	if (m_centerlineBtn)
	{
		m_centerlineBtn->setEnabled(canRun(PluginTubularGrindingStage::Centerline));
		if (laplacianOnPointCloud)
		{
			m_centerlineBtn->setToolTip(i18n(
				QStringLiteral("Laplacian requires mesh input; switch to OTLC or select a mesh."),
				QStringLiteral("Laplacian 仅支持网格，请改用 OTLC 或选择网格对象。")));
		}
		else
		{
			m_centerlineBtn->setToolTip(QString());
		}
	}
	if (m_templateBtn)
	{
		m_templateBtn->setEnabled(canRun(PluginTubularGrindingStage::TemplatePoints));
	}
	if (m_projectBtn)
	{
		m_projectBtn->setEnabled(canRun(PluginTubularGrindingStage::Project));
	}
	if (m_resetBtn)
	{
		m_resetBtn->setEnabled(!m_busy && m_sessionId.valid());
	}
}

void TubularGrindingDockWidget::ensureSession()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string meshId = selectedMeshBackendId();
	if (!pch || !doc || meshId.empty())
	{
		return;
	}
	if (m_sessionId.valid() && m_meshBackendId == meshId)
	{
		return;
	}
	if (m_sessionId.valid())
	{
		pch->clearTubularGrindingSession(doc, m_sessionId);
		m_sessionId = {};
		m_lastStage = PluginTubularGrindingStage::None;
	}
	m_sessionId = pch->beginTubularGrindingSession(doc, meshId);
	m_meshBackendId = meshId;
	m_lastStage = PluginTubularGrindingStage::None;
}

void TubularGrindingDockWidget::runStage(const PluginTubularGrindingStage stage)
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	if (!pch || !doc || m_busy)
	{
		return;
	}
	ensureSession();
	if (!m_sessionId.valid())
	{
		appendLog(i18n(QStringLiteral("Failed to create session"), QStringLiteral("无法创建会话")));
		return;
	}
	m_busy = true;
	updateButtonStates();
	const PluginTubularGrindingParams params = buildParams();
	pch->runTubularGrindingStage(
		doc,
		m_sessionId,
		stage,
		params,
		[this](const bool ok, const QString& error, const PluginTubularGrindingReport& report) {
			m_busy = false;
			if (!ok)
			{
				appendLog(error);
				if (m_host)
				{
					m_host->logError(error);
				}
			}
			else
			{
				m_lastStage = report.lastCompletedStage;
				refreshSummary(report);
				if (!report.stageSummaryZh.isEmpty())
				{
					appendLog(report.stageSummaryZh);
				}
			}
			updateButtonStates();
		});
}

void TubularGrindingDockWidget::refreshSummary(const PluginTubularGrindingReport& report)
{
	if (!m_summaryLabel)
	{
		return;
	}
	m_summaryLabel->setText(report.stageSummaryZh);
}

void TubularGrindingDockWidget::onMeshSelectionChanged()
{
	IPluginPointCloudHost* pch = pointCloudHost();
	IPluginDocument* doc = activeDoc();
	const std::string meshId = selectedMeshBackendId();
	if (m_sessionId.valid() && meshId != m_meshBackendId && pch && doc)
	{
		pch->clearTubularGrindingSession(doc, m_sessionId);
		m_sessionId = {};
		m_lastStage = PluginTubularGrindingStage::None;
		m_meshBackendId.clear();
	}
	syncCenterlineMethodForSource();
	updateCenterlineParamVisibility();
	updateButtonStates();
}

void TubularGrindingDockWidget::onResetSessionClicked()
{
	resetSessionUi();
}

void TubularGrindingDockWidget::onCenterlineClicked()
{
	runStage(PluginTubularGrindingStage::Centerline);
}

void TubularGrindingDockWidget::onTemplateClicked()
{
	runStage(PluginTubularGrindingStage::TemplatePoints);
}

void TubularGrindingDockWidget::onProjectClicked()
{
	runStage(PluginTubularGrindingStage::Project);
}
