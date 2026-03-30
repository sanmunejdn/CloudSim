#include "RobotAxisControlWidget.h"

#include <QLabel>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>

#include <cmath>

namespace
{
static constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
static constexpr int kSliderMax = 10000;
} // namespace

RobotAxisControlWidget::RobotAxisControlWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);
	m_hintLabel = new QLabel(QStringLiteral("Drag sliders to move joints (uses URDF limits)."));
	m_hintLabel->setWordWrap(true);
	root->addWidget(m_hintLabel);

	m_scroll = new QScrollArea(this);
	m_scroll->setWidgetResizable(true);
	m_scroll->setFrameShape(QFrame::NoFrame);
	auto* inner = new QWidget;
	m_rowsLayout = new QVBoxLayout(inner);
	m_rowsLayout->setContentsMargins(0, 0, 0, 0);
	m_rowsLayout->addStretch(1);
	m_scroll->setWidget(inner);
	root->addWidget(m_scroll, 1);
}

void RobotAxisControlWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	if (m_hintLabel)
	{
		m_hintLabel->setText(chinese ? QStringLiteral("\u62D6\u52A8\u6ED1\u5757\u63A7\u5236\u5404\u8F74\uFF08\u8303\u56F4\u6765\u81EA URDF limit\uFF09\u3002")
									 : QStringLiteral("Drag sliders to move joints (uses URDF limits)."));
	}
}

void RobotAxisControlWidget::clearJoints()
{
	m_sliders.clear();
	m_valueLabels.clear();
	m_lowerRad.clear();
	m_upperRad.clear();
	m_anglesRad.clear();
	if (!m_rowsLayout)
	{
		return;
	}
	while (m_rowsLayout->count() > 0)
	{
		QLayoutItem* top = m_rowsLayout->takeAt(0);
		if (QLayout* row = top->layout())
		{
			while (row->count() > 0)
			{
				delete row->takeAt(0);
			}
		}
		else if (QWidget* w = top->widget())
		{
			delete w;
		}
		delete top;
	}
	m_rowsLayout->addStretch(1);
}

void RobotAxisControlWidget::setJoints(const QStringList& jointNames, const QVector<double>& lowerRad, const QVector<double>& upperRad)
{
	clearJoints();
	const int n = jointNames.size();
	if (n <= 0 || lowerRad.size() != n || upperRad.size() != n)
	{
		return;
	}
	m_lowerRad = lowerRad;
	m_upperRad = upperRad;
	m_anglesRad.resize(n);
	m_anglesRad.fill(0.0);

	QLayoutItem* stretch = m_rowsLayout->takeAt(m_rowsLayout->count() - 1);
	delete stretch;

	for (int i = 0; i < n; ++i)
	{
		auto* row = new QHBoxLayout;
		auto* nameLab = new QLabel(jointNames[i]);
		nameLab->setMinimumWidth(100);
		auto* slider = new QSlider(Qt::Horizontal);
		slider->setRange(0, kSliderMax);
		const int idx = i;
		connect(slider, &QSlider::valueChanged, this, [this, idx]() { onSliderValueChanged(idx); });
		{
			const QSignalBlocker blocker(slider);
			slider->setValue(radToSliderValue(i, 0.0));
		}
		auto* valLab = new QLabel(QStringLiteral("0.0\u00B0"));
		valLab->setMinimumWidth(72);
		valLab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
		m_sliders.append(slider);
		m_valueLabels.append(valLab);
		row->addWidget(nameLab);
		row->addWidget(slider, 1);
		row->addWidget(valLab);
		m_rowsLayout->addLayout(row);
	}
	m_rowsLayout->addStretch(1);
}

double RobotAxisControlWidget::sliderToRad(int index, int sliderValue) const
{
	if (index < 0 || index >= m_lowerRad.size())
	{
		return 0.0;
	}
	const double t = static_cast<double>(sliderValue) / static_cast<double>(kSliderMax);
	const double lo = m_lowerRad[index];
	const double hi = m_upperRad[index];
	return lo + t * (hi - lo);
}

int RobotAxisControlWidget::radToSliderValue(int index, double rad) const
{
	if (index < 0 || index >= m_lowerRad.size())
	{
		return 0;
	}
	const double lo = m_lowerRad[index];
	const double hi = m_upperRad[index];
	const double span = hi - lo;
	if (std::abs(span) < 1e-12)
	{
		return 0;
	}
	const double t = (rad - lo) / span;
	const double clamped = std::max(0.0, std::min(1.0, t));
	return static_cast<int>(std::lround(clamped * static_cast<double>(kSliderMax)));
}

void RobotAxisControlWidget::onSliderValueChanged(int which)
{
	if (which < 0 || which >= m_sliders.size())
	{
		return;
	}
	QSlider* s = m_sliders[which];
	const double rad = sliderToRad(which, s->value());
	m_anglesRad[which] = rad;
	if (which < m_valueLabels.size())
	{
		const double deg = rad * kRadToDeg;
		m_valueLabels[which]->setText(QString::number(deg, 'f', 1) + QStringLiteral("\u00B0"));
	}
	emitAnglesFromSliders();
}

void RobotAxisControlWidget::emitAnglesFromSliders()
{
	emit jointAnglesChanged(m_anglesRad);
}

void RobotAxisControlWidget::setJointAnglesRad(const QVector<double>& rad)
{
	if (rad.size() != m_sliders.size())
	{
		return;
	}
	for (int i = 0; i < m_sliders.size(); ++i)
	{
		QSlider* s = m_sliders[i];
		s->blockSignals(true);
		s->setValue(radToSliderValue(i, rad[i]));
		s->blockSignals(false);
		m_anglesRad[i] = rad[i];
		if (i < m_valueLabels.size())
		{
			m_valueLabels[i]->setText(QString::number(rad[i] * kRadToDeg, 'f', 1) + QStringLiteral("\u00B0"));
		}
	}
}

QVector<double> RobotAxisControlWidget::jointAnglesRad() const
{
	return m_anglesRad;
}

void RobotAxisControlWidget::setInteractionEnabled(bool enabled)
{
	for (QSlider* s : m_sliders)
	{
		s->setEnabled(enabled);
	}
}
