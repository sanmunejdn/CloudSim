/// @file RobotCommPageWidget.cpp
/// @brief 机器人通讯页 UI

#include "RobotCommPageWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

RobotCommPageWidget::RobotCommPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);

	auto* form = new QFormLayout();
	m_brandCombo = new QComboBox(this);
	m_brandCombo->addItem(QStringLiteral("FANUC"), QStringLiteral("fanuc"));
	m_brandCombo->addItem(QStringLiteral("ABB"), QStringLiteral("abb"));
	m_brandCombo->addItem(QStringLiteral("KUKA"), QStringLiteral("kuka"));
	m_bridgeHostEdit = new QLineEdit(QStringLiteral("127.0.0.1"), this);
	m_bridgePortSpin = new QSpinBox(this);
	m_bridgePortSpin->setRange(1, 65535);
	m_bridgePortSpin->setValue(19610);
	m_robotHostEdit = new QLineEdit(QStringLiteral("127.0.0.1"), this);
	m_robotPortSpin = new QSpinBox(this);
	m_robotPortSpin->setRange(0, 65535);
	m_robotPortSpin->setSpecialValueText(QStringLiteral("auto"));
	m_robotPortSpin->setValue(0);
	m_userEdit = new QLineEdit(QStringLiteral("Default User"), this);
	m_passEdit = new QLineEdit(QStringLiteral("robotics"), this);
	m_passEdit->setEchoMode(QLineEdit::Password);
	m_mirrorCheck = new QCheckBox(this);
	m_pollSpin = new QSpinBox(this);
	m_pollSpin->setRange(50, 5000);
	m_pollSpin->setValue(200);
	m_pollSpin->setSuffix(QStringLiteral(" ms"));

	m_titleBrand = new QLabel(this);
	m_titleBridge = new QLabel(this);
	m_titleRobot = new QLabel(this);
	m_titleUser = new QLabel(this);
	m_titlePass = new QLabel(this);
	m_titlePoll = new QLabel(this);

	form->addRow(m_titleBrand, m_brandCombo);
	auto* bridgeRow = new QHBoxLayout();
	bridgeRow->addWidget(m_bridgeHostEdit, 1);
	bridgeRow->addWidget(m_bridgePortSpin);
	form->addRow(m_titleBridge, bridgeRow);
	auto* robotRow = new QHBoxLayout();
	robotRow->addWidget(m_robotHostEdit, 1);
	robotRow->addWidget(m_robotPortSpin);
	form->addRow(m_titleRobot, robotRow);
	form->addRow(m_titleUser, m_userEdit);
	form->addRow(m_titlePass, m_passEdit);
	form->addRow(QString(), m_mirrorCheck);
	form->addRow(m_titlePoll, m_pollSpin);
	root->addLayout(form);

	auto* btnRow = new QHBoxLayout();
	m_connectBtn = new QPushButton(this);
	m_disconnectBtn = new QPushButton(this);
	m_disconnectBtn->setEnabled(false);
	btnRow->addWidget(m_connectBtn);
	btnRow->addWidget(m_disconnectBtn);
	btnRow->addStretch(1);
	root->addLayout(btnRow);

	m_statusLabel = new QLabel(this);
	m_jointsLabel = new QLabel(QStringLiteral("-"), this);
	m_poseLabel = new QLabel(QStringLiteral("-"), this);
	m_jointsLabel->setWordWrap(true);
	m_poseLabel->setWordWrap(true);
	root->addWidget(m_statusLabel);
	root->addWidget(m_jointsLabel);
	root->addWidget(m_poseLabel);
	root->addStretch(1);

	connect(m_connectBtn, &QPushButton::clicked, this, &RobotCommPageWidget::connectRequested);
	connect(m_disconnectBtn, &QPushButton::clicked, this, &RobotCommPageWidget::disconnectRequested);
	connect(m_mirrorCheck, &QCheckBox::toggled, this, &RobotCommPageWidget::mirrorToggled);
	connect(m_pollSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &RobotCommPageWidget::pollIntervalChanged);

	retranslate();
}

void RobotCommPageWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	retranslate();
}

void RobotCommPageWidget::retranslate()
{
	if (m_useChinese)
	{
		m_titleBrand->setText(QStringLiteral("品牌"));
		m_titleBridge->setText(QStringLiteral("Bridge"));
		m_titleRobot->setText(QStringLiteral("机器人 IP"));
		m_titleUser->setText(QStringLiteral("用户"));
		m_titlePass->setText(QStringLiteral("密码"));
		m_titlePoll->setText(QStringLiteral("采样周期"));
		m_mirrorCheck->setText(QStringLiteral("镜像到场景"));
		m_connectBtn->setText(QStringLiteral("连接"));
		m_disconnectBtn->setText(QStringLiteral("断开"));
	}
	else
	{
		m_titleBrand->setText(QStringLiteral("Brand"));
		m_titleBridge->setText(QStringLiteral("Bridge"));
		m_titleRobot->setText(QStringLiteral("Robot IP"));
		m_titleUser->setText(QStringLiteral("User"));
		m_titlePass->setText(QStringLiteral("Password"));
		m_titlePoll->setText(QStringLiteral("Poll interval"));
		m_mirrorCheck->setText(QStringLiteral("Mirror to scene"));
		m_connectBtn->setText(QStringLiteral("Connect"));
		m_disconnectBtn->setText(QStringLiteral("Disconnect"));
	}
}

void RobotCommPageWidget::setStatusText(const QString& text)
{
	m_statusLabel->setText(text);
}

void RobotCommPageWidget::setFeedbackText(const QString& jointsLine, const QString& poseLine)
{
	m_jointsLabel->setText(jointsLine);
	m_poseLabel->setText(poseLine);
}

void RobotCommPageWidget::setConnectedUi(bool bridgeOk, bool robotOk)
{
	m_connectBtn->setEnabled(!(bridgeOk && robotOk));
	m_disconnectBtn->setEnabled(bridgeOk);
	Q_UNUSED(robotOk);
}

QString RobotCommPageWidget::bridgeHost() const
{
	return m_bridgeHostEdit->text().trimmed();
}

int RobotCommPageWidget::bridgePort() const
{
	return m_bridgePortSpin->value();
}

QString RobotCommPageWidget::brand() const
{
	return m_brandCombo->currentData().toString();
}

QString RobotCommPageWidget::robotHost() const
{
	return m_robotHostEdit->text().trimmed();
}

int RobotCommPageWidget::robotPort() const
{
	return m_robotPortSpin->value();
}

QString RobotCommPageWidget::user() const
{
	return m_userEdit->text();
}

QString RobotCommPageWidget::password() const
{
	return m_passEdit->text();
}

bool RobotCommPageWidget::mirrorEnabled() const
{
	return m_mirrorCheck->isChecked();
}

int RobotCommPageWidget::pollIntervalMs() const
{
	return m_pollSpin->value();
}
