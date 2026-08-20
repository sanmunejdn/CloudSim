/// @file AiAssistantDockWidget.cpp
/// @brief AiAssistantDockWidget 实现

#include "AiAssistantDockWidget.h"

#include "AiConfirmPanel.h"
#include "AiDomainTypes.h"
#include "AiLlmSettingsDialog.h"
#include "IAiAssistantHost.h"
#include "UiIconDecorators.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QStyle>
#include <QTextBrowser>
#include <QVBoxLayout>

#include <json.hpp>

AiAssistantDockWidget::AiAssistantDockWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	m_domainCombo = new QComboBox(this);
	m_domainCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_domainCombo->setMaxVisibleItems(12);
	m_domainCombo->addItem(QStringLiteral("Auto"), AiDomainIds::autoDomain());
	m_domainCombo->addItem(QStringLiteral("Create mesh"), AiDomainIds::meshCreate());
	m_domainCombo->addItem(QStringLiteral("Compose (boolean)"), AiDomainIds::meshCompose());
	m_domainCombo->addItem(QStringLiteral("Feature compose"), AiDomainIds::featureCompose());
	m_domainCombo->addItem(QStringLiteral("Design parts"), AiDomainIds::designParts());
	m_domainCombo->addItem(QStringLiteral("Geometry recognize"), AiDomainIds::geometryRecognize());
	m_domainCombo->addItem(QStringLiteral("Trajectory feature"), AiDomainIds::trajectoryFeature());
	m_domainCombo->addItem(QStringLiteral("Point cloud"), AiDomainIds::pointCloudOps());
	m_domainCombo->addItem(QStringLiteral("Document import"), AiDomainIds::documentImport());
	m_domainCombo->addItem(QStringLiteral("Geometry ops"), AiDomainIds::geometryOps());
	m_domainCombo->addItem(QStringLiteral("Feature build"), AiDomainIds::featureBuild());
	m_domainCombo->addItem(QStringLiteral("Labeling"), AiDomainIds::labelingAnnot());
	m_domainCombo->addItem(QStringLiteral("Scene ops"), AiDomainIds::sceneOps());
	m_domainCombo->addItem(QStringLiteral("Process flow"), AiDomainIds::processFlow());
	root->addWidget(m_domainCombo, 0);

	m_viewportHint = new QLabel(this);
	m_viewportHint->setWordWrap(true);
	m_viewportHint->hide();
	root->addWidget(m_viewportHint, 0);

	m_history = new QTextBrowser(this);
	m_history->setOpenExternalLinks(false);
	// 可压到较小高度，优先保证底部输入/设置可见
	m_history->setMinimumHeight(48);
	m_history->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	root->addWidget(m_history, 1);

	m_confirmPanel = new AiConfirmPanel(this);
	m_confirmPanel->hide();
	connect(m_confirmPanel, &AiConfirmPanel::accepted, this, &AiAssistantDockWidget::agentConfirmAccepted);
	connect(m_confirmPanel, &AiConfirmPanel::rejected, this, &AiAssistantDockWidget::agentConfirmRejected);
	connect(m_confirmPanel, &AiConfirmPanel::secondaryClicked, this, &AiAssistantDockWidget::agentConfirmSecondary);
	root->addWidget(m_confirmPanel, 0);

	auto* inputBar = new QWidget(this);
	inputBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	auto* row = new QHBoxLayout(inputBar);
	row->setContentsMargins(0, 0, 0, 0);
	row->setSpacing(6);
	m_input = new QLineEdit(inputBar);
	m_input->setMinimumHeight(28);
	m_input->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_settingsBtn = new QPushButton(inputBar);
	m_settingsBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_sendBtn = new QPushButton(inputBar);
	m_sendBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	connect(m_settingsBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSettingsClicked);
	connect(m_sendBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSendClicked);
	connect(m_input, &QLineEdit::returnPressed, this, &AiAssistantDockWidget::onSendClicked);
	connect(m_domainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&AiAssistantDockWidget::onDomainChanged);
	row->addWidget(m_input, 1);
	row->addWidget(m_settingsBtn, 0);
	row->addWidget(m_sendBtn, 0);
	root->addWidget(inputBar, 0);

	UiIconDecorators::apply(m_settingsBtn, UiIconId::Settings, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Medium);
	UiIconDecorators::apply(m_sendBtn, UiIconId::Send, UiIconDecorators::IconPlacement::Leading, UiIcons::Size::Medium);
	m_sendBtn->setProperty("btnRole", QLatin1String("primary"));
	if (m_sendBtn->style())
	{
		m_sendBtn->style()->unpolish(m_sendBtn);
		m_sendBtn->style()->polish(m_sendBtn);
	}
	m_settingsBtn->setProperty("btnRole", QLatin1String("secondary"));
	if (m_settingsBtn->style())
	{
		m_settingsBtn->style()->unpolish(m_settingsBtn);
		m_settingsBtn->style()->polish(m_settingsBtn);
	}

	setUseChinese(m_useChinese);
	onDomainChanged(m_domainCombo->currentIndex());
	appendSystemMessage(m_useChinese ? QStringLiteral("AI 助手：默认本地模型 + 规则。单位 mm。")
									 : QStringLiteral("AI assistant: local models + rules. Units: mm."));
}

void AiAssistantDockWidget::setAiHost(IAiAssistantHost* host)
{
	m_aiHost = host;
}

QString AiAssistantDockWidget::selectedDomainId() const
{
	if (!m_domainCombo)
		return AiDomainIds::autoDomain();
	return m_domainCombo->currentData().toString();
}

void AiAssistantDockWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	if (m_confirmPanel)
		m_confirmPanel->setUseChinese(chinese);
	m_settingsBtn->setText(chinese ? QStringLiteral("设置") : QStringLiteral("Settings"));
	m_sendBtn->setText(chinese ? QStringLiteral("发送") : QStringLiteral("Send"));
	auto setById = [this, chinese](const QString& id, const QString& zh, const QString& en)
	{
		const int i = m_domainCombo->findData(id);
		if (i >= 0)
			m_domainCombo->setItemText(i, chinese ? zh : en);
	};
	setById(AiDomainIds::autoDomain(), QStringLiteral("自动"), QStringLiteral("Auto"));
	setById(AiDomainIds::meshCreate(), QStringLiteral("创建网格"), QStringLiteral("Create mesh"));
	setById(AiDomainIds::meshCompose(), QStringLiteral("布尔组合"), QStringLiteral("Compose (boolean)"));
	setById(AiDomainIds::featureCompose(), QStringLiteral("参数化特征"), QStringLiteral("Feature compose"));
	setById(AiDomainIds::designParts(), QStringLiteral("标准件"), QStringLiteral("Design parts"));
	setById(AiDomainIds::geometryRecognize(), QStringLiteral("几何识别"), QStringLiteral("Geometry recognize"));
	setById(AiDomainIds::trajectoryFeature(), QStringLiteral("轨迹特征"), QStringLiteral("Trajectory feature"));
	setById(AiDomainIds::pointCloudOps(), QStringLiteral("点云操作"), QStringLiteral("Point cloud"));
	setById(AiDomainIds::documentImport(), QStringLiteral("文档导入"), QStringLiteral("Document import"));
	setById(AiDomainIds::geometryOps(), QStringLiteral("几何操作"), QStringLiteral("Geometry ops"));
	setById(AiDomainIds::featureBuild(), QStringLiteral("特征构建"), QStringLiteral("Feature build"));
	setById(AiDomainIds::labelingAnnot(), QStringLiteral("标注"), QStringLiteral("Labeling"));
	setById(AiDomainIds::sceneOps(), QStringLiteral("场景操作"), QStringLiteral("Scene ops"));
	setById(AiDomainIds::processFlow(), QStringLiteral("工艺流程"), QStringLiteral("Process flow"));
	onDomainChanged(m_domainCombo->currentIndex());
}

void AiAssistantDockWidget::onDomainChanged(int)
{
	if (!m_viewportHint || !m_domainCombo)
		return;
	const QString domain = selectedDomainId();
	const bool geom = domain == AiDomainIds::geometryRecognize();
	const bool traj = domain == AiDomainIds::trajectoryFeature();
	m_viewportHint->setVisible(geom || traj);
	if (geom)
	{
		m_viewportHint->setText(m_useChinese ? QStringLiteral("将使用当前 3D 视口截图进行识别。")
											 : QStringLiteral("Will capture the active 3D viewport for recognition."));
	}
	else if (traj)
	{
		m_viewportHint->setText(
			m_useChinese ? QStringLiteral("请先在「轨迹生成」页选择 STEP 工件；识别结果将编号高亮，确认后离散。")
						 : QStringLiteral("Select a STEP workpiece on Trajectory Generation tab first."));
	}
	else
	{
		m_viewportHint->clear();
	}
}

void AiAssistantDockWidget::showTrajectoryFeatureResult(const QByteArray& planJsonUtf8,
														const QByteArray& catalogSliceUtf8, const QString& parserVia)
{
	const bool selectionOnly = (parserVia == QStringLiteral("Selection"));
	QString body = selectionOnly ? (m_useChinese ? QStringLiteral("已选中特征（3D 视口已高亮）：\n")
												 : QStringLiteral("Selected features (highlighted in 3D):\n"))
								 : (m_useChinese ? QStringLiteral("特征候选（3D 视口已编号高亮）：\n")
												 : QStringLiteral("Feature candidates (numbered in 3D view):\n"));
	try
	{
		const nlohmann::json slice = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		if (slice.contains("candidates") && slice["candidates"].is_array())
		{
			for (const auto& c : slice["candidates"])
			{
				body += QStringLiteral("%1. %2 — %3\n")
							.arg(c.value("displayIndex", 0))
							.arg(QString::fromStdString(c.value("candidateId", std::string())),
								 QString::fromStdString(c.value("summary", std::string())));
			}
		}
		body += m_useChinese
					? QStringLiteral("\n可输入「选 1 和 3」或「选 face_63」；选好后输入「确认」或「确认并离散」。")
					: QStringLiteral("\nType selection (e.g. face_63), then say confirm.");
		if (!selectionOnly && slice.value("truncated", false))
		{
			const int matched = slice.value("matchedTotal", 0);
			const int shown = slice.value("shownCount", 0);
			body += m_useChinese
						? QStringLiteral("\n（列表已截断：显示 %1 / 共 %2；可用「选 face_N」点名未列出的面）")
							  .arg(shown)
							  .arg(matched)
						: QStringLiteral("\n(Truncated: showing %1 / %2; use face_N to pick omitted ones.)")
							  .arg(shown)
							  .arg(matched);
		}
	}
	catch (...)
	{
		body += m_useChinese
					? QStringLiteral("\n可输入「选 1 和 3」或「选 face_63」；选好后输入「确认」或「确认并离散」。")
					: QStringLiteral("\nType selection (e.g. face_63), then say confirm.");
	}
	appendAssistantMessage(prefixWithParser(parserVia, body));
	(void)planJsonUtf8;
}

void AiAssistantDockWidget::hideTrajectoryFeatureConfirmButtons()
{
	// 确认已并入 AiConfirmPanel
}

void AiAssistantDockWidget::showRecognitionResult(const QByteArray& jsonUtf8, const QString& parserVia)
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		appendAssistantMessage(prefixWithParser(parserVia, QStringLiteral("识别结果 JSON 无效。")));
		return;
	}

	const std::string prim = j.value("primitive", std::string());
	const std::string label = j.value("label", std::string());
	const double confidence = j.value("confidence", 0.0);
	const nlohmann::json dims = j.value("dimensions_mm", nlohmann::json::object());

	QString body;
	if (m_useChinese)
	{
		body = QStringLiteral("识别结果：\n类型：%1\n标签：%2\n置信度：%3")
				   .arg(QString::fromStdString(prim), QString::fromStdString(label))
				   .arg(confidence, 0, 'f', 2);
		if (dims.is_object() && !dims.empty())
		{
			body += QStringLiteral("\n尺寸 (mm)：");
			for (auto it = dims.begin(); it != dims.end(); ++it)
			{
				if (it.value().is_number())
					body += QStringLiteral("\n  %1 = %2")
								.arg(QString::fromStdString(it.key()))
								.arg(it.value().get<double>(), 0, 'g', 4);
			}
		}
		if (prim == "unknown")
			body += QStringLiteral("\n无法确定类型，无法创建基本体。");
		else
			body += QStringLiteral("\n场景未变化；请在下方面板确认后创建对应基本体。");
	}
	else
	{
		body = QStringLiteral("Recognition:\nprimitive: %1\nlabel: %2\nconfidence: %3")
				   .arg(QString::fromStdString(prim), QString::fromStdString(label))
				   .arg(confidence, 0, 'f', 2);
		if (dims.is_object() && !dims.empty())
		{
			body += QStringLiteral("\ndimensions_mm:");
			for (auto it = dims.begin(); it != dims.end(); ++it)
			{
				if (it.value().is_number())
					body += QStringLiteral("\n  %1 = %2")
								.arg(QString::fromStdString(it.key()))
								.arg(it.value().get<double>(), 0, 'g', 4);
			}
		}
		if (prim == "unknown")
			body += QStringLiteral("\nType unknown; cannot create primitive.");
		else
			body += QStringLiteral("\nScene unchanged; confirm in the panel below to create.");
	}

	appendAssistantMessage(prefixWithParser(parserVia, body));
}

void AiAssistantDockWidget::hideCreateFromRecognitionButton()
{
	// 确认已并入 AiConfirmPanel
}

void AiAssistantDockWidget::showAgentConfirmPanel(const QString& pendingId, const QString& title, const QString& risk,
												  const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
												  const QByteArray& sceneSnapshotJson, const QString& confirmLabel,
												  const QString& secondaryLabel)
{
	if (m_confirmPanel)
		m_confirmPanel->showToolConfirm(pendingId, title, risk, argsSchemaJson, proposedArgsJson, sceneSnapshotJson,
										confirmLabel, secondaryLabel);
}

void AiAssistantDockWidget::hideAgentConfirmPanel()
{
	if (m_confirmPanel)
		m_confirmPanel->hidePanel();
}

QString AiAssistantDockWidget::prefixWithParser(const QString& parserVia, const QString& text)
{
	if (parserVia.isEmpty())
		return text;
	return QStringLiteral("[%1] %2").arg(parserVia, text);
}

void AiAssistantDockWidget::appendUserMessage(const QString& text)
{
	const QString who = m_useChinese ? QStringLiteral("你") : QStringLiteral("You");
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped().replace(QLatin1Char('\n'), QStringLiteral("<br/>"))));
}

void AiAssistantDockWidget::appendAssistantMessage(const QString& text)
{
	const QString who = m_useChinese ? QStringLiteral("助手") : QStringLiteral("Assistant");
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped().replace(QLatin1Char('\n'), QStringLiteral("<br/>"))));
}

void AiAssistantDockWidget::appendSystemMessage(const QString& text)
{
	m_history->append(QStringLiteral("<i>%1</i>").arg(text.toHtmlEscaped().replace(QLatin1Char('\n'), QStringLiteral("<br/>"))));
}

void AiAssistantDockWidget::setBusy(bool busy)
{
	m_sendBtn->setEnabled(!busy);
	m_settingsBtn->setEnabled(!busy);
	m_input->setEnabled(!busy);
	m_domainCombo->setEnabled(!busy);
	if (m_confirmPanel)
		m_confirmPanel->setEnabled(true);
}

void AiAssistantDockWidget::onSettingsClicked()
{
	AiLlmSettingsDialog dlg(window());
	dlg.setUseChinese(m_useChinese);
	dlg.setAiHost(m_aiHost);
	if (dlg.exec() == QDialog::Accepted)
		appendSystemMessage(m_useChinese ? QStringLiteral("设置已保存。") : QStringLiteral("Settings saved."));
}

void AiAssistantDockWidget::onSendClicked()
{
	const QString t = m_input->text().trimmed();
	if (t.isEmpty())
		return;
	m_input->clear();
	appendUserMessage(t);
	emit messageSubmitted(t);
}
