#include "AiAssistantDockWidget.h"
#include "UiIconDecorators.h"

#include "AiDomainTypes.h"
#include "AiLlmSettingsDialog.h"
#include "IAiAssistantHost.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QVBoxLayout>

#include <json.hpp>

AiAssistantDockWidget::AiAssistantDockWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);

	m_domainCombo = new QComboBox(this);
	m_domainCombo->addItem(QStringLiteral("Auto"), AiDomainIds::autoDomain());
	m_domainCombo->addItem(QStringLiteral("Create mesh"), AiDomainIds::meshCreate());
	m_domainCombo->addItem(QStringLiteral("Compose (boolean)"), AiDomainIds::meshCompose());
	m_domainCombo->addItem(QStringLiteral("Geometry recognize"), AiDomainIds::geometryRecognize());
	m_domainCombo->addItem(QStringLiteral("Trajectory feature"), AiDomainIds::trajectoryFeature());
	root->addWidget(m_domainCombo);

	m_viewportHint = new QLabel(this);
	m_viewportHint->setWordWrap(true);
	m_viewportHint->hide();
	root->addWidget(m_viewportHint);

	m_history = new QTextBrowser(this);
	m_history->setOpenExternalLinks(false);
	m_history->setMinimumHeight(160);
	root->addWidget(m_history, 1);

	m_createFromRecognitionBtn = new QPushButton(this);
	m_createFromRecognitionBtn->hide();
	connect(m_createFromRecognitionBtn, &QPushButton::clicked, this,
		&AiAssistantDockWidget::createFromRecognitionClicked);
	root->addWidget(m_createFromRecognitionBtn);

	m_confirmTrajectoryBtn = new QPushButton(this);
	m_retryTrajectoryBtn = new QPushButton(this);
	m_confirmTrajectoryBtn->hide();
	m_retryTrajectoryBtn->hide();
	connect(m_confirmTrajectoryBtn, &QPushButton::clicked, this,
		&AiAssistantDockWidget::confirmTrajectoryFeaturesClicked);
	connect(m_retryTrajectoryBtn, &QPushButton::clicked, this,
		&AiAssistantDockWidget::retryTrajectoryFeaturesClicked);
	root->addWidget(m_confirmTrajectoryBtn);
	root->addWidget(m_retryTrajectoryBtn);

	auto* row = new QHBoxLayout;
	m_input = new QLineEdit(this);
	m_settingsBtn = new QPushButton(this);
	connect(m_settingsBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSettingsClicked);
	m_sendBtn = new QPushButton(this);
	connect(m_sendBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSendClicked);
	connect(m_input, &QLineEdit::returnPressed, this, &AiAssistantDockWidget::onSendClicked);
	connect(m_domainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
		&AiAssistantDockWidget::onDomainChanged);
	row->addWidget(m_input, 1);
	row->addWidget(m_settingsBtn);
	row->addWidget(m_sendBtn);
	root->addLayout(row);

	UiIconDecorators::apply(m_settingsBtn, UiIconId::Settings, UiIconDecorators::IconPlacement::IconOnly, UiIcons::Size::Medium);
	UiIconDecorators::apply(m_sendBtn, UiIconId::Send, UiIconDecorators::IconPlacement::Leading, UiIcons::Size::Medium);

	setUseChinese(m_useChinese);
	onDomainChanged(m_domainCombo->currentIndex());
	appendSystemMessage(m_useChinese
		? QStringLiteral("AI 助手：默认本地模型 + 规则。单位 mm。")
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
	m_settingsBtn->setText(chinese ? QStringLiteral("设置") : QStringLiteral("Settings"));
	m_sendBtn->setText(chinese ? QStringLiteral("发送") : QStringLiteral("Send"));
	m_domainCombo->setItemText(0, chinese ? QStringLiteral("自动") : QStringLiteral("Auto"));
	m_domainCombo->setItemText(1, chinese ? QStringLiteral("创建网格") : QStringLiteral("Create mesh"));
	m_domainCombo->setItemText(2, chinese ? QStringLiteral("布尔组合") : QStringLiteral("Compose (boolean)"));
	m_domainCombo->setItemText(3, chinese ? QStringLiteral("几何识别") : QStringLiteral("Geometry recognize"));
	m_domainCombo->setItemText(4, chinese ? QStringLiteral("轨迹特征") : QStringLiteral("Trajectory feature"));
	if (m_createFromRecognitionBtn)
		m_createFromRecognitionBtn->setText(chinese ? QStringLiteral("创建基本体") : QStringLiteral("Create primitive"));
	if (m_confirmTrajectoryBtn)
		m_confirmTrajectoryBtn->setText(chinese ? QStringLiteral("确认并离散") : QStringLiteral("Confirm & discretize"));
	if (m_retryTrajectoryBtn)
		m_retryTrajectoryBtn->setText(chinese ? QStringLiteral("重新识别") : QStringLiteral("Retry"));
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
		m_viewportHint->setText(m_useChinese
			? QStringLiteral("请先在「轨迹生成」页选择 STEP 工件；识别结果将编号高亮，确认后离散。")
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
	QString body = selectionOnly
		? (m_useChinese ? QStringLiteral("已选中特征（3D 视口已高亮）：\n")
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
	}
	catch (...)
	{
	}
	body += m_useChinese ? QStringLiteral("\n可输入「选 1 和 3」调整，或点「确认并离散」。")
		: QStringLiteral("\nType selection or click Confirm.");
	appendAssistantMessage(prefixWithParser(parserVia, body));
	if (m_confirmTrajectoryBtn)
		m_confirmTrajectoryBtn->show();
	if (m_retryTrajectoryBtn)
		m_retryTrajectoryBtn->show();
	(void)planJsonUtf8;
}

void AiAssistantDockWidget::hideTrajectoryFeatureConfirmButtons()
{
	if (m_confirmTrajectoryBtn)
		m_confirmTrajectoryBtn->hide();
	if (m_retryTrajectoryBtn)
		m_retryTrajectoryBtn->hide();
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
			body += QStringLiteral("\n场景未变化；确认后可创建对应基本体。");
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
			body += QStringLiteral("\nScene unchanged; confirm to create primitive.");
	}

	appendAssistantMessage(prefixWithParser(parserVia, body));
	if (prim != "unknown" && !prim.empty())
		m_createFromRecognitionBtn->show();
	else
		m_createFromRecognitionBtn->hide();
}

void AiAssistantDockWidget::hideCreateFromRecognitionButton()
{
	if (m_createFromRecognitionBtn)
		m_createFromRecognitionBtn->hide();
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
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped()));
}

void AiAssistantDockWidget::appendAssistantMessage(const QString& text)
{
	const QString who = m_useChinese ? QStringLiteral("助手") : QStringLiteral("Assistant");
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped()));
}

void AiAssistantDockWidget::appendSystemMessage(const QString& text)
{
	m_history->append(QStringLiteral("<i>%1</i>").arg(text.toHtmlEscaped()));
}

void AiAssistantDockWidget::setBusy(bool busy)
{
	m_sendBtn->setEnabled(!busy);
	m_settingsBtn->setEnabled(!busy);
	m_input->setEnabled(!busy);
	m_domainCombo->setEnabled(!busy);
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
