/// @file RobotCollisionSettingsWidget.cpp
/// @brief RobotCollisionSettingsWidget 实现

#include "RobotCollisionSettingsWidget.h"

#include "CollisionWorld.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QVBoxLayout>

RobotCollisionSettingsWidget::RobotCollisionSettingsWidget(QWidget* parent) : QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	auto* group = new QGroupBox(this);
	auto* form = new QFormLayout(group);
	m_enabledCheck = new QCheckBox(group);
	m_marginSpin = new QDoubleSpinBox(group);
	m_marginSpin->setRange(0.0, 500.0);
	m_marginSpin->setDecimals(2);
	m_marginSpin->setSingleStep(0.5);
	m_marginSpin->setValue(1.0);
	m_marginSpin->setSuffix(QStringLiteral(" mm"));
	form->addRow(QStringLiteral("Enable"), m_enabledCheck);
	form->addRow(QStringLiteral("Margin"), m_marginSpin);
	layout->addWidget(group);
	layout->addStretch(1);

	connect(m_enabledCheck, &QCheckBox::toggled, this, &RobotCollisionSettingsWidget::onFieldChanged);
	connect(m_marginSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotCollisionSettingsWidget::onFieldChanged);

	if (!collision::CollisionWorld::hasCoalBackend())
	{
		// 内置实现可用；仅提示可选 coal
		m_enabledCheck->setToolTip(QStringLiteral("Built-in mesh collision (AABB + triangles)"));
	}
	setUseChinese(true);
	refreshEnabledState();
}

void RobotCollisionSettingsWidget::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
	m_enabledCheck->setText(chinese ? QStringLiteral("启用碰撞检测") : QStringLiteral("Enable collision check"));
	// form labels via buddy text on checkbox/spin already
	refreshEnabledState();
}

void RobotCollisionSettingsWidget::setSettings(const RobotCollision::Settings& s)
{
	m_block = true;
	m_enabledCheck->setChecked(s.enabled);
	m_marginSpin->setValue(s.securityMarginMm);
	m_block = false;
	refreshEnabledState();
}

RobotCollision::Settings RobotCollisionSettingsWidget::settings() const
{
	RobotCollision::Settings s;
	s.enabled = m_enabledCheck->isChecked();
	s.securityMarginMm = m_marginSpin->value();
	return s;
}

void RobotCollisionSettingsWidget::refreshEnabledState()
{
	m_marginSpin->setEnabled(m_enabledCheck->isChecked());
}

void RobotCollisionSettingsWidget::onFieldChanged()
{
	if (m_block)
		return;
	refreshEnabledState();
	emit settingsChanged();
}
