/// @file RobotAxisControlWidget.cpp
/// @brief RobotAxisControlWidget 实现

#include "RobotAxisControlWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QDoubleValidator>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QTimer>
#include <algorithm>
#include <cmath>

static constexpr double kPi = 3.14159265358979323846;

RobotAxisControlWidget::RobotAxisControlWidget(QWidget* parent) : QWidget(parent)
{
	createUI();
}

RobotAxisControlWidget::~RobotAxisControlWidget() {}

void RobotAxisControlWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	if (m_targetLabel)
	{
		m_targetLabel->setText(chinese ? QStringLiteral("目标") : QStringLiteral("Target"));
	}
	if (m_resetAllButton)
	{
		m_resetAllButton->setText(chinese ? QStringLiteral("重置所有关节") : QStringLiteral("Reset all joints"));
		m_resetAllButton->setToolTip(chinese ? QStringLiteral("将所有关节重置到零位；外部轴重置到回零位")
											 : QStringLiteral("Reset arm joints to zero; external axes to home"));
	}
	if (m_reachableWorkspaceCheck)
	{
		setReachableWorkspaceBusy(m_reachableWorkspaceBusy);
		m_reachableWorkspaceCheck->setToolTip(
			chinese ? QStringLiteral("采样臂关节；启用的 RobotBase 外轴按行程分层扫满，半透明点显示 TCP 可达域")
					: QStringLiteral("Sample arm joints; sweep enabled RobotBase axes by travel; show TCP workspace"));
	}
	if (m_reachableWorkspaceDensityLabel)
	{
		const int pct = reachableWorkspaceDensityPercent();
		m_reachableWorkspaceDensityLabel->setText(
			chinese ? QStringLiteral("密度 %1%").arg(pct) : QStringLiteral("Density %1%").arg(pct));
	}
	if (m_reachableWorkspaceDensitySlider)
	{
		m_reachableWorkspaceDensitySlider->setToolTip(
			chinese ? QStringLiteral("控制可达点采样密度（默认 50%）")
					: QStringLiteral("Reachable-point sample density (default 50%)"));
	}
	rebuildExternalAxisControls();
}

void RobotAxisControlWidget::setInteractionEnabled(bool enabled)
{
	setEnabled(enabled);
}

void RobotAxisControlWidget::setControlTargets(const QVector<AxisControlTargetItem>& targets)
{
	m_controlTargets = targets;
	if (!m_targetCombo)
	{
		return;
	}
	const AxisControlTargetItem prev = currentControlTarget();
	{
		const QSignalBlocker blocker(m_targetCombo);
		m_targetCombo->clear();
		for (const AxisControlTargetItem& t : m_controlTargets)
		{
			m_targetCombo->addItem(t.displayLabel);
		}
		int select = 0;
		for (int i = 0; i < m_controlTargets.size(); ++i)
		{
			if (m_controlTargets[i].kind == prev.kind && m_controlTargets[i].id == prev.id)
			{
				select = i;
				break;
			}
		}
		if (!m_controlTargets.isEmpty())
		{
			m_targetCombo->setCurrentIndex(select);
		}
	}
	// blocker 下改索引不会发信号；自定义设备轴依赖本信号重建滑条
	if (!m_controlTargets.isEmpty())
	{
		notifyCurrentControlTargetApplied();
	}
}

AxisControlTargetItem RobotAxisControlWidget::currentControlTarget() const
{
	if (!m_targetCombo || m_controlTargets.isEmpty())
	{
		return {};
	}
	const int idx = m_targetCombo->currentIndex();
	if (idx < 0 || idx >= m_controlTargets.size())
	{
		return m_controlTargets.front();
	}
	return m_controlTargets[idx];
}

void RobotAxisControlWidget::selectControlTarget(const AxisControlTargetKind kind, const QString& id)
{
	if (!m_targetCombo)
	{
		return;
	}
	for (int i = 0; i < m_controlTargets.size(); ++i)
	{
		if (m_controlTargets[i].kind == kind && m_controlTargets[i].id == id)
		{
			if (m_targetCombo->currentIndex() == i)
			{
				notifyCurrentControlTargetApplied();
			}
			else
			{
				m_targetCombo->setCurrentIndex(i);
			}
			return;
		}
	}
}

void RobotAxisControlWidget::updateTargetDependentChrome(const AxisControlTargetItem&)
{
	updateReachableWorkspaceChrome();
}

void RobotAxisControlWidget::updateReachableWorkspaceChrome()
{
	const bool isRobot = currentControlTarget().kind == AxisControlTargetKind::RobotInstance;
	const bool show = m_reachableWorkspaceFeatureEnabled && isRobot;
	if (m_reachableWorkspaceCheck)
	{
		m_reachableWorkspaceCheck->setVisible(show);
	}
	if (m_reachableWorkspaceDensityLabel)
	{
		m_reachableWorkspaceDensityLabel->setVisible(show);
	}
	if (m_reachableWorkspaceDensitySlider)
	{
		m_reachableWorkspaceDensitySlider->setVisible(show);
	}
	if (!show && isReachableWorkspaceChecked())
	{
		setReachableWorkspaceChecked(false);
		emit reachableWorkspaceToggled(false);
	}
}

void RobotAxisControlWidget::setReachableWorkspaceFeatureEnabled(const bool enabled)
{
	if (m_reachableWorkspaceFeatureEnabled == enabled)
	{
		updateReachableWorkspaceChrome();
		return;
	}
	m_reachableWorkspaceFeatureEnabled = enabled;
	updateReachableWorkspaceChrome();
}

void RobotAxisControlWidget::notifyCurrentControlTargetApplied()
{
	if (m_controlTargets.isEmpty())
	{
		return;
	}
	const AxisControlTargetItem t = currentControlTarget();
	updateTargetDependentChrome(t);
	emit controlTargetChanged(t.kind, t.id);
}

void RobotAxisControlWidget::onControlTargetComboChanged(const int index)
{
	if (index < 0 || index >= m_controlTargets.size())
	{
		return;
	}
	notifyCurrentControlTargetApplied();
}

void RobotAxisControlWidget::setJoints(const QStringList& jointNames, const QVector<double>& lowerLimits,
									   const QVector<double>& upperLimits)
{
	const QHash<QString, osg::MatrixTransform*> emptyTransforms;
	setupJointControls(jointNames, lowerLimits, upperLimits, emptyTransforms);
}

void RobotAxisControlWidget::clearJoints()
{
	setJoints(QStringList(), QVector<double>(), QVector<double>());
}

int RobotAxisControlWidget::jointCount() const
{
	return m_jointOrder.size();
}

QVector<double> RobotAxisControlWidget::jointAnglesRad() const
{
	QVector<double> out;
	out.reserve(m_jointOrder.size());
	for (const QString& name : m_jointOrder)
	{
		auto it = m_jointControls.constFind(name);
		out.push_back(it != m_jointControls.cend() ? it.value().currentAngle : 0.0);
	}
	return out;
}

void RobotAxisControlWidget::setJointAnglesRad(const QVector<double>& jointAnglesRad)
{
	setJointAnglesRadSilent(jointAnglesRad);
	emit allJointAnglesChanged(jointAnglesRad);
}

void RobotAxisControlWidget::setJointAnglesRadSilent(const QVector<double>& jointAnglesRad)
{
	if (jointAnglesRad.size() != m_jointOrder.size())
	{
		return;
	}
	for (int i = 0; i < m_jointOrder.size(); ++i)
	{
		setJointAngle(m_jointOrder[i], jointAnglesRad[i]);
	}
}

void RobotAxisControlWidget::emitAllJointAnglesNow()
{
	emit allJointAnglesChanged(jointAnglesRad());
}

void RobotAxisControlWidget::clearExternalAxes()
{
	setExternalAxes({});
}

int RobotAxisControlWidget::externalAxisCount() const
{
	return m_externalControls.size();
}

QVector<double> RobotAxisControlWidget::externalAxisValues() const
{
	QVector<double> out;
	out.reserve(m_externalControls.size());
	for (const ExternalAxisControl& ec : m_externalControls)
	{
		out.push_back(ec.currentValue);
	}
	return out;
}

void RobotAxisControlWidget::setExternalAxisValues(const QVector<double>& values)
{
	setExternalAxisValuesSilent(values);
	emitExternalAxisValuesNow();
}

void RobotAxisControlWidget::setExternalAxisValuesSilent(const QVector<double>& values)
{
	if (values.size() != m_externalControls.size())
	{
		return;
	}
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		setExternalAxisValueAt(i, values[i]);
	}
}

void RobotAxisControlWidget::emitExternalAxisValuesNow()
{
	emit externalAxisValuesChanged(externalAxisValues());
}

void RobotAxisControlWidget::setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes)
{
	const QVector<double> prev = externalAxisValues();
	for (ExternalAxisControl& ec : m_externalControls)
	{
		if (ec.groupBox)
		{
			m_contentLayout->removeWidget(ec.groupBox);
			delete ec.groupBox;
		}
	}
	m_externalControls.clear();

	for (const RobotExternal::RobotExternalAxisConfig& cfgIn : axes.axes)
	{
		if (!cfgIn.enabled)
		{
			continue;
		}
		RobotExternal::RobotExternalAxisConfig cfg = cfgIn;
		RobotExternal::normalizeExternalAxisConfig(cfg);
		ExternalAxisControl ec;
		ec.config = cfg;
		ec.currentValue = std::clamp(cfg.home, cfg.lower, cfg.upper);
		m_externalControls.push_back(std::move(ec));
	}
	rebuildExternalAxisControls();
	if (prev.size() == m_externalControls.size())
	{
		setExternalAxisValuesSilent(prev);
	}
}

void RobotAxisControlWidget::createUI()
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setContentsMargins(4, 4, 4, 4);
	mainLayout->setSpacing(4);

	auto* targetRow = new QHBoxLayout();
	m_targetLabel = new QLabel(QStringLiteral("目标"), this);
	m_targetCombo = new QComboBox(this);
	m_targetCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	targetRow->addWidget(m_targetLabel);
	targetRow->addWidget(m_targetCombo, 1);
	mainLayout->addLayout(targetRow);
	connect(m_targetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&RobotAxisControlWidget::onControlTargetComboChanged);

	m_reachableWorkspaceCheck = new QCheckBox(this);
	m_reachableWorkspaceCheck->setText(QStringLiteral("显示可达域"));
	m_reachableWorkspaceCheck->setToolTip(
		QStringLiteral("采样臂关节；启用的 RobotBase 外轴按行程分层扫满，半透明点显示 TCP 可达域"));
	connect(m_reachableWorkspaceCheck, &QCheckBox::toggled, this, &RobotAxisControlWidget::reachableWorkspaceToggled);
	mainLayout->addWidget(m_reachableWorkspaceCheck);

	auto* densityRow = new QHBoxLayout();
	m_reachableWorkspaceDensityLabel = new QLabel(QStringLiteral("密度 50%"), this);
	m_reachableWorkspaceDensitySlider = new QSlider(Qt::Horizontal, this);
	m_reachableWorkspaceDensitySlider->setRange(1, 100);
	m_reachableWorkspaceDensitySlider->setValue(50);
	m_reachableWorkspaceDensitySlider->setToolTip(QStringLiteral("控制可达点采样密度（默认 50%）"));
	densityRow->addWidget(m_reachableWorkspaceDensityLabel);
	densityRow->addWidget(m_reachableWorkspaceDensitySlider, 1);
	mainLayout->addLayout(densityRow);

	m_reachableWorkspaceDensityDebounce = new QTimer(this);
	m_reachableWorkspaceDensityDebounce->setSingleShot(true);
	m_reachableWorkspaceDensityDebounce->setInterval(280);
	connect(m_reachableWorkspaceDensitySlider, &QSlider::valueChanged, this,
			&RobotAxisControlWidget::onReachableWorkspaceDensitySliderChanged);
	connect(m_reachableWorkspaceDensityDebounce, &QTimer::timeout, this,
			&RobotAxisControlWidget::emitReachableWorkspaceDensityDebounced);

	m_scrollArea = new QScrollArea(this);
	m_scrollArea->setWidgetResizable(true);
	m_scrollArea->setFrameShape(QFrame::NoFrame);
	m_scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	m_contentWidget = new QWidget();
	m_contentLayout = new QVBoxLayout(m_contentWidget);
	m_contentLayout->setContentsMargins(0, 0, 0, 0);
	m_contentLayout->setSpacing(8);
	m_contentLayout->addStretch();

	m_scrollArea->setWidget(m_contentWidget);
	mainLayout->addWidget(m_scrollArea);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch();

	m_resetAllButton = new QPushButton(tr("重置所有关节"), this);
	m_resetAllButton->setToolTip(tr("将所有关节重置到零位；外部轴重置到回零位"));
	connect(m_resetAllButton, &QPushButton::clicked, this, &RobotAxisControlWidget::onResetAllButtonClicked);
	buttonLayout->addWidget(m_resetAllButton);

	mainLayout->addLayout(buttonLayout);
}

void RobotAxisControlWidget::setReachableWorkspaceChecked(bool checked)
{
	if (!m_reachableWorkspaceCheck)
	{
		return;
	}
	const QSignalBlocker blocker(m_reachableWorkspaceCheck);
	m_reachableWorkspaceCheck->setChecked(checked);
}

bool RobotAxisControlWidget::isReachableWorkspaceChecked() const
{
	return m_reachableWorkspaceCheck && m_reachableWorkspaceCheck->isChecked();
}

void RobotAxisControlWidget::setReachableWorkspaceBusy(bool busy)
{
	m_reachableWorkspaceBusy = busy;
	if (!m_reachableWorkspaceCheck)
	{
		return;
	}
	if (busy)
	{
		m_reachableWorkspaceCheck->setText(m_useChinese ? QStringLiteral("可达域计算中…")
														: QStringLiteral("Computing workspace…"));
	}
	else
	{
		m_reachableWorkspaceCheck->setText(m_useChinese ? QStringLiteral("显示可达域")
														: QStringLiteral("Show reachable workspace"));
	}
}

int RobotAxisControlWidget::reachableWorkspaceDensityPercent() const
{
	return m_reachableWorkspaceDensitySlider ? m_reachableWorkspaceDensitySlider->value() : 50;
}

void RobotAxisControlWidget::onReachableWorkspaceDensitySliderChanged(int value)
{
	if (m_reachableWorkspaceDensityLabel)
	{
		m_reachableWorkspaceDensityLabel->setText(m_useChinese ? QStringLiteral("密度 %1%").arg(value)
															   : QStringLiteral("Density %1%").arg(value));
	}
	if (m_reachableWorkspaceDensityDebounce)
	{
		m_reachableWorkspaceDensityDebounce->start();
	}
}

void RobotAxisControlWidget::emitReachableWorkspaceDensityDebounced()
{
	emit reachableWorkspaceDensityChanged(reachableWorkspaceDensityPercent());
}

void RobotAxisControlWidget::clearContentExceptStretch()
{
	QLayoutItem* child;
	while ((child = m_contentLayout->takeAt(0)) != nullptr)
	{
		if (child->spacerItem())
		{
			m_contentLayout->addItem(child);
			break;
		}
		if (QWidget* w = child->widget())
		{
			delete w;
		}
		delete child;
	}
}

void RobotAxisControlWidget::setupJointControls(const QStringList& jointNames, const QVector<double>& lowerLimits,
												const QVector<double>& upperLimits,
												const QHash<QString, osg::MatrixTransform*>& jointTransforms)
{
	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it)
	{
		JointControl& jc = it.value();
		if (jc.slider)
			delete jc.slider;
		if (jc.spinBox)
			delete jc.spinBox;
		if (jc.inputEdit)
			delete jc.inputEdit;
		if (jc.resetButton)
			delete jc.resetButton;
	}
	m_jointControls.clear();
	m_jointOrder.clear();

	// 重建关节时会清掉外轴控件指针，配置暂存后重建
	RobotExternal::RobotExternalAxisConfigSet extSet;
	QVector<double> extValues = externalAxisValues();
	for (const ExternalAxisControl& ec : m_externalControls)
	{
		extSet.axes.push_back(ec.config);
	}
	m_externalControls.clear();

	clearContentExceptStretch();

	int count = jointNames.size();
	if (count == 0 || lowerLimits.size() != count || upperLimits.size() != count)
	{
		if (!extSet.axes.empty())
		{
			setExternalAxes(extSet);
			if (extValues.size() == externalAxisCount())
			{
				setExternalAxisValuesSilent(extValues);
			}
		}
		return;
	}

	for (int i = 0; i < count; ++i)
	{
		const QString& name = jointNames[i];
		double lower = lowerLimits[i];
		double upper = upperLimits[i];

		m_jointOrder.append(name);

		JointControl& jc = m_jointControls[name];
		jc.name = name;
		jc.lowerLimit = lower;
		jc.upperLimit = upper;
		jc.currentAngle = 0.0;
		jc.transformNode = jointTransforms.value(name, nullptr);

		QGroupBox* groupBox = new QGroupBox(name, m_contentWidget);
		groupBox->setObjectName("jointGroup_" + name);

		QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
		groupLayout->setContentsMargins(4, 8, 4, 4);
		groupLayout->setSpacing(4);

		QHBoxLayout* sliderLayout = new QHBoxLayout();
		QLabel* minLabel = new QLabel(QString::number(lower * 180.0 / kPi, 'f', 1) + "°", groupBox);
		minLabel->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: 500;"));
		jc.slider = new QSlider(Qt::Horizontal, groupBox);
		jc.slider->setMinimum(angleToSliderValue(lower));
		jc.slider->setMaximum(angleToSliderValue(upper));
		jc.slider->setValue(0);
		jc.slider->setTracking(true);
		QLabel* maxLabel = new QLabel(QString::number(upper * 180.0 / kPi, 'f', 1) + "°", groupBox);
		maxLabel->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: 500;"));
		sliderLayout->addWidget(minLabel);
		sliderLayout->addWidget(jc.slider, 1);
		sliderLayout->addWidget(maxLabel);
		groupLayout->addLayout(sliderLayout);

		QHBoxLayout* inputLayout = new QHBoxLayout();
		inputLayout->setSpacing(2);
		QLabel* valueLabel = new QLabel(tr("角度:"), groupBox);
		jc.spinBox = new QDoubleSpinBox(groupBox);
		jc.spinBox->setDecimals(3);
		jc.spinBox->setRange(lower * 180.0 / kPi, upper * 180.0 / kPi);
		jc.spinBox->setValue(0.0);
		jc.spinBox->setSuffix("°");
		jc.spinBox->setSingleStep(1.0);
		QLabel* radLabel = new QLabel(tr("弧度:"), groupBox);
		jc.inputEdit = new QLineEdit(groupBox);
		jc.inputEdit->setText("0.000");
		jc.inputEdit->setFixedWidth(70);
		jc.inputEdit->setValidator(new QDoubleValidator(lower, upper, 6, jc.inputEdit));
		QLabel* radUnitLabel = new QLabel("rad", groupBox);

		jc.resetButton = new QPushButton(tr("重置"), groupBox);
		jc.resetButton->setFixedWidth(40);
		jc.resetButton->setToolTip(tr("将关节重置到零位"));

		inputLayout->addWidget(valueLabel);
		inputLayout->addWidget(jc.spinBox);
		inputLayout->addSpacing(4);
		inputLayout->addWidget(radLabel);
		inputLayout->addWidget(jc.inputEdit);
		inputLayout->addWidget(radUnitLabel);
		inputLayout->addStretch();
		inputLayout->addWidget(jc.resetButton);
		groupLayout->addLayout(inputLayout);

		connect(jc.slider, &QSlider::valueChanged, this, &RobotAxisControlWidget::onSliderValueChanged);
		connect(jc.spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&RobotAxisControlWidget::onSpinBoxValueChanged);
		connect(jc.inputEdit, &QLineEdit::returnPressed, this, &RobotAxisControlWidget::onLineEditReturnPressed);
		connect(jc.resetButton, &QPushButton::clicked, this, &RobotAxisControlWidget::onResetButtonClicked);

		m_contentLayout->insertWidget(m_contentLayout->count() - 1, groupBox);
	}

	if (!extSet.axes.empty())
	{
		setExternalAxes(extSet);
		if (extValues.size() == externalAxisCount())
		{
			setExternalAxisValuesSilent(extValues);
		}
	}
}

void RobotAxisControlWidget::rebuildExternalAxisControls()
{
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		ExternalAxisControl& ec = m_externalControls[i];
		if (ec.groupBox)
		{
			m_contentLayout->removeWidget(ec.groupBox);
			delete ec.groupBox;
			ec.groupBox = nullptr;
			ec.slider = nullptr;
			ec.spinBox = nullptr;
			ec.resetButton = nullptr;
		}

		const RobotExternal::RobotExternalAxisConfig& cfg = ec.config;
		const QString title = QString::fromStdString(cfg.displayName.empty() ? cfg.jointName : cfg.displayName);
		ec.groupBox = new QGroupBox(title, m_contentWidget);
		ec.groupBox->setObjectName(QStringLiteral("externalAxisGroup_%1").arg(i));

		QVBoxLayout* groupLayout = new QVBoxLayout(ec.groupBox);
		groupLayout->setContentsMargins(4, 8, 4, 4);
		groupLayout->setSpacing(4);

		const bool translate = isTranslateAxis(cfg);
		const double loUi = uiValueFromInternal(cfg, cfg.lower);
		const double hiUi = uiValueFromInternal(cfg, cfg.upper);
		const double curUi = uiValueFromInternal(cfg, ec.currentValue);
		const QString unit = translate ? QStringLiteral("mm") : QStringLiteral("°");

		QHBoxLayout* sliderLayout = new QHBoxLayout();
		QLabel* minLabel = new QLabel(QString::number(loUi, 'f', 1) + unit, ec.groupBox);
		minLabel->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: 500;"));
		ec.slider = new QSlider(Qt::Horizontal, ec.groupBox);
		if (translate)
		{
			ec.slider->setMinimum(mmToSliderValue(loUi));
			ec.slider->setMaximum(mmToSliderValue(hiUi));
			ec.slider->setValue(mmToSliderValue(curUi));
		}
		else
		{
			ec.slider->setMinimum(degToSliderValue(loUi));
			ec.slider->setMaximum(degToSliderValue(hiUi));
			ec.slider->setValue(degToSliderValue(curUi));
		}
		ec.slider->setTracking(true);
		QLabel* maxLabel = new QLabel(QString::number(hiUi, 'f', 1) + unit, ec.groupBox);
		maxLabel->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: 500;"));
		sliderLayout->addWidget(minLabel);
		sliderLayout->addWidget(ec.slider, 1);
		sliderLayout->addWidget(maxLabel);
		groupLayout->addLayout(sliderLayout);

		QHBoxLayout* inputLayout = new QHBoxLayout();
		const QString valueText = m_useChinese ? (translate ? QStringLiteral("行程:") : QStringLiteral("角度:"))
											   : (translate ? QStringLiteral("Stroke:") : QStringLiteral("Angle:"));
		QLabel* valueLabel = new QLabel(valueText, ec.groupBox);
		ec.spinBox = new QDoubleSpinBox(ec.groupBox);
		ec.spinBox->setDecimals(2);
		ec.spinBox->setRange(std::min(loUi, hiUi), std::max(loUi, hiUi));
		ec.spinBox->setValue(curUi);
		ec.spinBox->setSuffix(QStringLiteral(" ") + unit);
		ec.spinBox->setSingleStep(translate ? 1.0 : 1.0);
		ec.resetButton = new QPushButton(m_useChinese ? QStringLiteral("回零") : QStringLiteral("Home"), ec.groupBox);
		ec.resetButton->setFixedWidth(40);
		ec.resetButton->setToolTip(m_useChinese ? QStringLiteral("重置到回零位") : QStringLiteral("Reset to home"));

		inputLayout->addWidget(valueLabel);
		inputLayout->addWidget(ec.spinBox);
		inputLayout->addStretch();
		inputLayout->addWidget(ec.resetButton);
		groupLayout->addLayout(inputLayout);

		connect(ec.slider, &QSlider::valueChanged, this, &RobotAxisControlWidget::onExternalSliderValueChanged);
		connect(ec.spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&RobotAxisControlWidget::onExternalSpinBoxValueChanged);
		connect(ec.resetButton, &QPushButton::clicked, this, &RobotAxisControlWidget::onExternalResetButtonClicked);

		m_contentLayout->insertWidget(m_contentLayout->count() - 1, ec.groupBox);
	}
}

void RobotAxisControlWidget::setExternalAxisValueAt(int index, double value)
{
	if (index < 0 || index >= m_externalControls.size())
	{
		return;
	}
	ExternalAxisControl& ec = m_externalControls[index];
	value = qBound(ec.config.lower, value, ec.config.upper);
	ec.currentValue = value;
	if (!ec.slider || !ec.spinBox)
	{
		return;
	}
	const bool translate = isTranslateAxis(ec.config);
	const double ui = uiValueFromInternal(ec.config, value);
	bool blocked = ec.slider->blockSignals(true);
	ec.slider->setValue(translate ? mmToSliderValue(ui) : degToSliderValue(ui));
	ec.slider->blockSignals(blocked);
	blocked = ec.spinBox->blockSignals(true);
	ec.spinBox->setValue(ui);
	ec.spinBox->blockSignals(blocked);
}

void RobotAxisControlWidget::setJointAngle(const QString& jointName, double angleRad)
{
	auto it = m_jointControls.find(jointName);
	if (it == m_jointControls.end())
	{
		qDebug() << "[RobotAxisControlWidget] Joint not found:" << jointName;
		return;
	}

	JointControl& jc = it.value();

	angleRad = qBound(jc.lowerLimit, angleRad, jc.upperLimit);
	jc.currentAngle = angleRad;

	bool blocked;

	blocked = jc.slider->blockSignals(true);
	jc.slider->setValue(angleToSliderValue(angleRad));
	jc.slider->blockSignals(blocked);

	blocked = jc.spinBox->blockSignals(true);
	jc.spinBox->setValue(angleRad * 180.0 / kPi);
	jc.spinBox->blockSignals(blocked);

	jc.inputEdit->setText(QString::number(angleRad, 'f', 6));

	updateJointTransform(jointName, angleRad);
}

double RobotAxisControlWidget::getJointAngle(const QString& jointName) const
{
	auto it = m_jointControls.find(jointName);
	if (it != m_jointControls.end())
	{
		return it.value().currentAngle;
	}
	return 0.0;
}

void RobotAxisControlWidget::resetAllJoints()
{
	for (const QString& name : m_jointOrder)
	{
		setJointAngle(name, 0.0);
	}
	emitAllJointAnglesNow();
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		setExternalAxisValueAt(i, m_externalControls[i].config.home);
	}
	if (!m_externalControls.isEmpty())
	{
		emitExternalAxisValuesNow();
	}
}

void RobotAxisControlWidget::onSliderValueChanged(int value)
{
	QSlider* slider = qobject_cast<QSlider*>(sender());
	if (!slider)
		return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it)
	{
		if (it.value().slider == slider)
		{
			double angleRad = sliderValueToAngle(value);
			setJointAngle(it.key(), angleRad);
			emit jointAngleChanged(it.key(), angleRad);
			emitAllJointAnglesNow();
			break;
		}
	}
}

void RobotAxisControlWidget::onSpinBoxValueChanged(double value)
{
	QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
	if (!spinBox)
		return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it)
	{
		if (it.value().spinBox == spinBox)
		{
			double angleRad = value * kPi / 180.0;
			setJointAngle(it.key(), angleRad);
			emit jointAngleChanged(it.key(), angleRad);
			emitAllJointAnglesNow();
			break;
		}
	}
}

void RobotAxisControlWidget::onLineEditReturnPressed()
{
	QLineEdit* lineEdit = qobject_cast<QLineEdit*>(sender());
	if (!lineEdit)
		return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it)
	{
		if (it.value().inputEdit == lineEdit)
		{
			bool ok;
			double angleRad = lineEdit->text().toDouble(&ok);
			if (ok)
			{
				setJointAngle(it.key(), angleRad);
				emit jointAngleChanged(it.key(), angleRad);
				emitAllJointAnglesNow();
			}
			break;
		}
	}
}

void RobotAxisControlWidget::onResetButtonClicked()
{
	QPushButton* button = qobject_cast<QPushButton*>(sender());
	if (!button)
		return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it)
	{
		if (it.value().resetButton == button)
		{
			setJointAngle(it.key(), 0.0);
			emit jointAngleChanged(it.key(), 0.0);
			emitAllJointAnglesNow();
			break;
		}
	}
}

void RobotAxisControlWidget::onResetAllButtonClicked()
{
	resetAllJoints();
}

void RobotAxisControlWidget::onExternalSliderValueChanged(int value)
{
	QSlider* slider = qobject_cast<QSlider*>(sender());
	if (!slider)
	{
		return;
	}
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		if (m_externalControls[i].slider != slider)
		{
			continue;
		}
		const RobotExternal::RobotExternalAxisConfig& cfg = m_externalControls[i].config;
		const double ui = isTranslateAxis(cfg) ? sliderValueToMm(value) : sliderValueToDeg(value);
		setExternalAxisValueAt(i, internalFromUiValue(cfg, ui));
		emitExternalAxisValuesNow();
		break;
	}
}

void RobotAxisControlWidget::onExternalSpinBoxValueChanged(double value)
{
	QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
	if (!spinBox)
	{
		return;
	}
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		if (m_externalControls[i].spinBox != spinBox)
		{
			continue;
		}
		setExternalAxisValueAt(i, internalFromUiValue(m_externalControls[i].config, value));
		emitExternalAxisValuesNow();
		break;
	}
}

void RobotAxisControlWidget::onExternalResetButtonClicked()
{
	QPushButton* button = qobject_cast<QPushButton*>(sender());
	if (!button)
	{
		return;
	}
	for (int i = 0; i < m_externalControls.size(); ++i)
	{
		if (m_externalControls[i].resetButton != button)
		{
			continue;
		}
		setExternalAxisValueAt(i, m_externalControls[i].config.home);
		emitExternalAxisValuesNow();
		break;
	}
}

void RobotAxisControlWidget::updateJointTransform(const QString& jointName, double angleRad)
{
	auto it = m_jointControls.find(jointName);
	if (it == m_jointControls.end())
		return;

	JointControl& jc = it.value();
	if (!jc.transformNode)
	{
		return;
	}
	(void)angleRad;
}

int RobotAxisControlWidget::angleToSliderValue(double angleRad) const
{
	return static_cast<int>(angleRad * SLIDER_SCALE);
}

double RobotAxisControlWidget::sliderValueToAngle(int value) const
{
	return value / SLIDER_SCALE;
}

int RobotAxisControlWidget::mmToSliderValue(double mm) const
{
	return static_cast<int>(std::lround(mm * SLIDER_SCALE_MM));
}

double RobotAxisControlWidget::sliderValueToMm(int value) const
{
	return static_cast<double>(value) / SLIDER_SCALE_MM;
}

int RobotAxisControlWidget::degToSliderValue(double deg) const
{
	return static_cast<int>(std::lround(deg * 10.0));
}

double RobotAxisControlWidget::sliderValueToDeg(int value) const
{
	return static_cast<double>(value) / 10.0;
}

bool RobotAxisControlWidget::isTranslateAxis(const RobotExternal::RobotExternalAxisConfig& cfg)
{
	return cfg.motionType == RobotExternal::RobotExternalMotionType::Translate;
}

double RobotAxisControlWidget::uiValueFromInternal(const RobotExternal::RobotExternalAxisConfig& cfg, double q)
{
	return isTranslateAxis(cfg) ? q : (q * 180.0 / kPi);
}

double RobotAxisControlWidget::internalFromUiValue(const RobotExternal::RobotExternalAxisConfig& cfg, double ui)
{
	return isTranslateAxis(cfg) ? ui : (ui * kPi / 180.0);
}
