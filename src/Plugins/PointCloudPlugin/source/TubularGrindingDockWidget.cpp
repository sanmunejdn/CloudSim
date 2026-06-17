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

	auto* segmentForm = new QFormLayout();
	m_regionGrowAngleSpin = new QDoubleSpinBox(m_rootGroup);
	m_regionGrowAngleSpin->setRange(0.0, 200.0);
	m_regionGrowAngleSpin->setValue(0.0);
	m_regionGrowAngleSpin->setSuffix(QStringLiteral(" mm"));
	m_regionGrowAngleLabel = new QLabel(m_rootGroup);
	segmentForm->addRow(m_regionGrowAngleLabel, m_regionGrowAngleSpin);

	m_rayConvergenceSpin = new QDoubleSpinBox(m_rootGroup);
	m_rayConvergenceSpin->setRange(0.0, 100.0);
	m_rayConvergenceSpin->setValue(0.0);
	m_rayConvergenceSpin->setSingleStep(1.0);
	m_rayConvergenceSpin->setSuffix(QStringLiteral(" mm"));
	m_rayConvergenceLabel = new QLabel(m_rootGroup);
	segmentForm->addRow(m_rayConvergenceLabel, m_rayConvergenceSpin);

	m_axisMergeAngleSpin = new QDoubleSpinBox(m_rootGroup);
	m_axisMergeAngleSpin->setRange(5.0, 90.0);
	m_axisMergeAngleSpin->setValue(28.0);
	m_axisMergeAngleSpin->setSuffix(QStringLiteral(" °"));
	m_axisMergeAngleLabel = new QLabel(m_rootGroup);
	segmentForm->addRow(m_axisMergeAngleLabel, m_axisMergeAngleSpin);

	m_junctionSpreadSpin = new QDoubleSpinBox(m_rootGroup);
	m_junctionSpreadSpin->setRange(15.0, 90.0);
	m_junctionSpreadSpin->setValue(38.0);
	m_junctionSpreadSpin->setSuffix(QStringLiteral(" °"));
	m_junctionSpreadLabel = new QLabel(m_rootGroup);
	segmentForm->addRow(m_junctionSpreadLabel, m_junctionSpreadSpin);

	m_minSegmentFacesSpin = new QSpinBox(m_rootGroup);
	m_minSegmentFacesSpin->setRange(10, 50000);
	m_minSegmentFacesSpin->setValue(40);
	m_minSegmentFacesLabel = new QLabel(m_rootGroup);
	segmentForm->addRow(m_minSegmentFacesLabel, m_minSegmentFacesSpin);
	groupLayout->addLayout(segmentForm);

	auto* paramForm = new QFormLayout();
	m_templateLabel = new QLabel(m_rootGroup);
	m_templateCombo = new QComboBox(m_rootGroup);
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
	paramForm->addRow(m_templateLabel, m_templateCombo);

	m_sectionSpacingSpin = new QDoubleSpinBox(m_rootGroup);
	m_sectionSpacingSpin->setRange(0.5, 50.0);
	m_sectionSpacingSpin->setValue(2.0);
	m_sectionSpacingSpin->setSuffix(QStringLiteral(" mm"));
	m_sectionSpacingLabel = new QLabel(m_rootGroup);
	paramForm->addRow(m_sectionSpacingLabel, m_sectionSpacingSpin);

	m_helicalCoilsSpin = new QSpinBox(m_rootGroup);
	m_helicalCoilsSpin->setRange(1, 64);
	m_helicalCoilsSpin->setValue(8);
	m_helicalCoilsLabel = new QLabel(m_rootGroup);
	paramForm->addRow(m_helicalCoilsLabel, m_helicalCoilsSpin);

	m_projectionDistSpin = new QDoubleSpinBox(m_rootGroup);
	m_projectionDistSpin->setRange(1.0, 100.0);
	m_projectionDistSpin->setValue(10.0);
	m_projectionDistSpin->setSuffix(QStringLiteral(" mm"));
	m_projectionDistLabel = new QLabel(m_rootGroup);
	paramForm->addRow(m_projectionDistLabel, m_projectionDistSpin);
	groupLayout->addLayout(paramForm);

	auto* stageRow = new QHBoxLayout();
	m_segmentBtn = new QPushButton(m_rootGroup);
	m_centerlineBtn = new QPushButton(m_rootGroup);
	m_templateBtn = new QPushButton(m_rootGroup);
	m_projectBtn = new QPushButton(m_rootGroup);
	stageRow->addWidget(m_segmentBtn);
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
	connect(m_resetBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onResetSessionClicked);
	connect(m_segmentBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onSegmentClicked);
	connect(m_centerlineBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onCenterlineClicked);
	connect(m_templateBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onTemplateClicked);
	connect(m_projectBtn, &QPushButton::clicked, this, &TubularGrindingDockWidget::onProjectClicked);

	applyLanguage();
	updateButtonStates();
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

PluginTubularGrindingParams TubularGrindingDockWidget::buildParams() const
{
	PluginTubularGrindingParams params;
	params.ringCenterClusterEpsMm = m_regionGrowAngleSpin ? m_regionGrowAngleSpin->value() : 0.0;
	params.ringRayConvergenceEpsMm = m_rayConvergenceSpin ? m_rayConvergenceSpin->value() : 0.0;
	params.minRingFaces = 4.0;
	params.regionGrowAxisAngleDeg = m_axisMergeAngleSpin ? m_axisMergeAngleSpin->value() : 28.0;
	params.axisMergeAngleDeg = m_axisMergeAngleSpin ? m_axisMergeAngleSpin->value() : 28.0;
	params.junctionAxisSpreadDeg = m_junctionSpreadSpin ? m_junctionSpreadSpin->value() : 38.0;
	params.minSegmentFaces = m_minSegmentFacesSpin ? m_minSegmentFacesSpin->value() : 40.0;
	params.sectionSpacingMm = m_sectionSpacingSpin ? m_sectionSpacingSpin->value() : 2.0;
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
		m_meshLabel->setText(zh ? QStringLiteral("网格对象") : QStringLiteral("Mesh"));
	}
	if (m_regionGrowAngleLabel)
	{
		m_regionGrowAngleLabel->setText(zh ? QStringLiteral("环心簇半径(0自动)") : QStringLiteral("Ring cluster eps"));
	}
	if (m_rayConvergenceLabel)
	{
		m_rayConvergenceLabel->setText(zh ? QStringLiteral("法向汇聚容差(0自动)") : QStringLiteral("Normal convergence tol"));
	}
	if (m_axisMergeAngleLabel)
	{
		m_axisMergeAngleLabel->setText(zh ? QStringLiteral("环链合并角") : QStringLiteral("Ring chain angle"));
	}
	if (m_junctionSpreadLabel)
	{
		m_junctionSpreadLabel->setText(zh ? QStringLiteral("三通判定角") : QStringLiteral("Junction spread"));
	}
	if (m_minSegmentFacesLabel)
	{
		m_minSegmentFacesLabel->setText(zh ? QStringLiteral("最小面数/管段") : QStringLiteral("Min faces/segment"));
	}
	if (m_templateLabel)
	{
		m_templateLabel->setText(zh ? QStringLiteral("模板类型") : QStringLiteral("Template"));
	}
	if (m_sectionSpacingLabel)
	{
		m_sectionSpacingLabel->setText(zh ? QStringLiteral("截面间距") : QStringLiteral("Section spacing"));
	}
	if (m_helicalCoilsLabel)
	{
		m_helicalCoilsLabel->setText(zh ? QStringLiteral("螺旋圈数") : QStringLiteral("Helical coils"));
	}
	if (m_projectionDistLabel)
	{
		m_projectionDistLabel->setText(zh ? QStringLiteral("投影最大距离") : QStringLiteral("Projection max dist"));
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
	if (m_segmentBtn)
	{
		m_segmentBtn->setText(zh ? QStringLiteral("管段分割") : QStringLiteral("Segment"));
	}
	if (m_centerlineBtn)
	{
		m_centerlineBtn->setText(zh ? QStringLiteral("中心线") : QStringLiteral("Centerline"));
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
		if (doc->backendClassName(id) != "Model")
		{
			continue;
		}
		m_meshCombo->addItem(QString::fromStdString(doc->backendDisplayName(id)),
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
	const auto last = m_lastStage;
	auto canRun = [&](const PluginTubularGrindingStage stage) {
		if (m_busy || !hasMesh)
		{
			return false;
		}
		if (stage == PluginTubularGrindingStage::Segment)
		{
			return last == PluginTubularGrindingStage::None;
		}
		return static_cast<int>(stage) == static_cast<int>(last) + 1;
	};
	if (m_segmentBtn)
	{
		m_segmentBtn->setEnabled(canRun(PluginTubularGrindingStage::Segment));
	}
	if (m_centerlineBtn)
	{
		m_centerlineBtn->setEnabled(canRun(PluginTubularGrindingStage::Centerline));
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
	updateButtonStates();
}

void TubularGrindingDockWidget::onResetSessionClicked()
{
	resetSessionUi();
}

void TubularGrindingDockWidget::onSegmentClicked()
{
	runStage(PluginTubularGrindingStage::Segment);
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
