#include "RobotAxisControlWidget.h"

#include <QGridLayout>
#include <QDebug>
#include <cmath>

static constexpr double kPi = 3.14159265358979323846;

RobotAxisControlWidget::RobotAxisControlWidget(QWidget* parent)
	: QWidget(parent)
{
	createUI();
}

RobotAxisControlWidget::~RobotAxisControlWidget()
{
}

void RobotAxisControlWidget::setUseChinese(bool chinese)
{
	(void)chinese;
}

void RobotAxisControlWidget::setInteractionEnabled(bool enabled)
{
	setEnabled(enabled);
}

void RobotAxisControlWidget::setJoints(
	const QStringList& jointNames, const QVector<double>& lowerLimits, const QVector<double>& upperLimits)
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

void RobotAxisControlWidget::createUI()
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setContentsMargins(8, 6, 8, 8);
	mainLayout->setSpacing(6);

	QLabel* descLabel = new QLabel(tr("使用滑块或输入框调整各关节角度"), this);
	descLabel->setStyleSheet("color: gray; font-size: 11px;");
	mainLayout->addWidget(descLabel);

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
	m_resetAllButton->setToolTip(tr("将所有关节重置到零位"));
	connect(m_resetAllButton, &QPushButton::clicked, this, &RobotAxisControlWidget::onResetAllButtonClicked);
	buttonLayout->addWidget(m_resetAllButton);

	mainLayout->addLayout(buttonLayout);
}

void RobotAxisControlWidget::setupJointControls(
	const QStringList& jointNames,
	const QVector<double>& lowerLimits,
	const QVector<double>& upperLimits,
	const QHash<QString, osg::MatrixTransform*>& jointTransforms)
{
	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it) {
		JointControl& jc = it.value();
		if (jc.nameLabel) delete jc.nameLabel;
		if (jc.limitLabel) delete jc.limitLabel;
		if (jc.slider) delete jc.slider;
		if (jc.spinBox) delete jc.spinBox;
		if (jc.inputEdit) delete jc.inputEdit;
		if (jc.resetButton) delete jc.resetButton;
	}
	m_jointControls.clear();
	m_jointOrder.clear();

	QLayoutItem* child;
	while ((child = m_contentLayout->takeAt(0)) != nullptr) {
		if (child->spacerItem()) {
			m_contentLayout->addItem(child);
			break;
		}
		delete child;
	}

	int count = jointNames.size();
	if (count == 0 || lowerLimits.size() != count || upperLimits.size() != count) {
		qDebug() << "[RobotAxisControlWidget] Invalid joint configuration";
		return;
	}

	for (int i = 0; i < count; ++i) {
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
		groupLayout->setContentsMargins(8, 12, 8, 8);
		groupLayout->setSpacing(6);

		QHBoxLayout* limitLayout = new QHBoxLayout();
		jc.limitLabel = new QLabel(groupBox);
		updateLimitLabel(jc); // 更新显示上下限
		jc.limitLabel->setStyleSheet("color: gray; font-size: 10px;");
		limitLayout->addWidget(jc.limitLabel);
		limitLayout->addStretch();
		groupLayout->addLayout(limitLayout);

		QHBoxLayout* sliderLayout = new QHBoxLayout();
		QLabel* minLabel = new QLabel(QString::number(lower * 180.0 / kPi, 'f', 1) + "°", groupBox);
		minLabel->setStyleSheet("font-size: 10px;");
		jc.slider = new QSlider(Qt::Horizontal, groupBox);
		jc.slider->setMinimum(angleToSliderValue(lower));
		jc.slider->setMaximum(angleToSliderValue(upper));
		jc.slider->setValue(0);
		jc.slider->setTracking(true); // 实时跟踪
		QLabel* maxLabel = new QLabel(QString::number(upper * 180.0 / kPi, 'f', 1) + "°", groupBox);
		maxLabel->setStyleSheet("font-size: 10px;");
		sliderLayout->addWidget(minLabel);
		sliderLayout->addWidget(jc.slider, 1);
		sliderLayout->addWidget(maxLabel);
		groupLayout->addLayout(sliderLayout);

		QHBoxLayout* inputLayout = new QHBoxLayout();
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
		jc.inputEdit->setFixedWidth(80);
		jc.inputEdit->setValidator(new QDoubleValidator(lower, upper, 6, jc.inputEdit));
		QLabel* radUnitLabel = new QLabel("rad", groupBox);
		
		jc.resetButton = new QPushButton(tr("重置"), groupBox);
		jc.resetButton->setFixedWidth(50);
		jc.resetButton->setToolTip(tr("将关节重置到零位"));

		inputLayout->addWidget(valueLabel);
		inputLayout->addWidget(jc.spinBox);
		inputLayout->addSpacing(10);
		inputLayout->addWidget(radLabel);
		inputLayout->addWidget(jc.inputEdit);
		inputLayout->addWidget(radUnitLabel);
		inputLayout->addStretch();
		inputLayout->addWidget(jc.resetButton);
		groupLayout->addLayout(inputLayout);

		connect(jc.slider, &QSlider::valueChanged, this, &RobotAxisControlWidget::onSliderValueChanged);
		connect(jc.spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
				this, &RobotAxisControlWidget::onSpinBoxValueChanged);
		connect(jc.inputEdit, &QLineEdit::returnPressed, this, &RobotAxisControlWidget::onLineEditReturnPressed);
		connect(jc.resetButton, &QPushButton::clicked, this, &RobotAxisControlWidget::onResetButtonClicked);

		m_contentLayout->insertWidget(m_contentLayout->count() - 1, groupBox);
	}
}

void RobotAxisControlWidget::updateLimitLabel(JointControl& jc)
{
	if (jc.limitLabel) {
		QString limitText = QString(tr("范围: %1° ~ %2° (%3 ~ %4 rad)"))
			.arg(jc.lowerLimit * 180.0 / kPi, 0, 'f', 1)
			.arg(jc.upperLimit * 180.0 / kPi, 0, 'f', 1)
			.arg(jc.lowerLimit, 0, 'f', 4)
			.arg(jc.upperLimit, 0, 'f', 4);
		jc.limitLabel->setText(limitText);
	}
}

void RobotAxisControlWidget::setJointAngle(const QString& jointName, double angleRad)
{
	auto it = m_jointControls.find(jointName);
	if (it == m_jointControls.end()) {
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
	if (it != m_jointControls.end()) {
		return it.value().currentAngle;
	}
	return 0.0;
}

void RobotAxisControlWidget::resetAllJoints()
{
	for (const QString& name : m_jointOrder) {
		setJointAngle(name, 0.0);
	}
	emitAllJointAnglesNow();
}

void RobotAxisControlWidget::onSliderValueChanged(int value)
{
	QSlider* slider = qobject_cast<QSlider*>(sender());
	if (!slider) return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it) {
		if (it.value().slider == slider) {
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
	if (!spinBox) return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it) {
		if (it.value().spinBox == spinBox) {
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
	if (!lineEdit) return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it) {
		if (it.value().inputEdit == lineEdit) {
			bool ok;
			double angleRad = lineEdit->text().toDouble(&ok);
			if (ok) {
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
	if (!button) return;

	for (auto it = m_jointControls.begin(); it != m_jointControls.end(); ++it) {
		if (it.value().resetButton == button) {
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

void RobotAxisControlWidget::updateJointTransform(const QString& jointName, double angleRad)
{
	auto it = m_jointControls.find(jointName);
	if (it == m_jointControls.end()) return;

	JointControl& jc = it.value();
	if (!jc.transformNode) {
		qDebug() << "[RobotAxisControlWidget] No transform node for joint:" << jointName;
		return;
	}

}

int RobotAxisControlWidget::angleToSliderValue(double angleRad) const
{
	return static_cast<int>(angleRad * SLIDER_SCALE);
}

double RobotAxisControlWidget::sliderValueToAngle(int value) const
{
	return value / SLIDER_SCALE;
}