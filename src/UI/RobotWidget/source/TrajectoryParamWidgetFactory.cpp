/// @file TrajectoryParamWidgetFactory.cpp
/// @brief TrajectoryParamWidget 工厂

#include "TrajectoryParamWidgetFactory.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QSpinBox>
#include <QWidget>

namespace trajectory_algo
{
namespace
{
class ParamHostWidget : public QWidget
{
public:
	TrajectoryOpParamField field{};
	std::function<bool(const TrajectoryParamValue&)> write;
	std::function<bool(TrajectoryParamValue&)> read;
};

constexpr int kParamControlHeight = 26;

QHBoxLayout* makeCompactRowLayout(QWidget* host)
{
	auto* layout = new QHBoxLayout(host);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(2);
	host->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	return layout;
}

void applyCompactControlHeight(QWidget* widget)
{
	if (widget)
	{
		widget->setFixedHeight(kParamControlHeight);
		widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	}
}

} // namespace

TrajectoryParamBinding TrajectoryParamWidgetFactory::create(const TrajectoryOpParamField& field, const bool useChinese)
{
	TrajectoryParamBinding binding{};
	binding.field = field;
	binding.label = new QLabel(useChinese && !field.labelZh.empty() ? QString::fromStdString(field.labelZh)
																	: QString::fromStdString(field.labelEn));

	if (field.type == TrajectoryParamType::Message)
	{
		auto* label = new QLabel(useChinese && !field.messageZh.empty() ? QString::fromStdString(field.messageZh)
																		: QString::fromStdString(field.messageEn));
		label->setWordWrap(true);
		label->setStyleSheet(QStringLiteral("color: gray;"));
		binding.widget = label;
		return binding;
	}

	auto* host = new ParamHostWidget();
	host->field = field;
	binding.widget = host;

	if (field.type == TrajectoryParamType::Double)
	{
		auto* spin = new QDoubleSpinBox(host);
		spin->setRange(field.minValue, field.maxValue);
		spin->setSingleStep(field.step);
		if (!field.unit.empty())
		{
			spin->setSuffix(QStringLiteral(" ") + QString::fromStdString(field.unit));
		}
		spin->setDecimals(field.step < 0.1 ? 3 : 2);
		applyCompactControlHeight(spin);
		makeCompactRowLayout(host)->addWidget(spin, 1);
		binding.read = [spin](TrajectoryParamValue& out)
		{
			out.kind = TrajectoryParamValue::Kind::Double;
			out.asDouble = spin->value();
			return true;
		};
		binding.write = [spin](const TrajectoryParamValue& in)
		{
			if (in.kind != TrajectoryParamValue::Kind::Double)
			{
				return false;
			}
			spin->setValue(in.asDouble);
			return true;
		};
	}
	else if (field.type == TrajectoryParamType::Int)
	{
		auto* spin = new QSpinBox(host);
		spin->setRange(field.minInt, field.maxInt);
		applyCompactControlHeight(spin);
		makeCompactRowLayout(host)->addWidget(spin, 1);
		binding.read = [spin](TrajectoryParamValue& out)
		{
			out.kind = TrajectoryParamValue::Kind::Int;
			out.asInt = spin->value();
			return true;
		};
		binding.write = [spin](const TrajectoryParamValue& in)
		{
			if (in.kind != TrajectoryParamValue::Kind::Int)
			{
				return false;
			}
			spin->setValue(in.asInt);
			return true;
		};
	}
	else if (field.type == TrajectoryParamType::Bool)
	{
		auto* box = new QCheckBox(host);
		auto* rowLayout = makeCompactRowLayout(host);
		rowLayout->addWidget(box);
		rowLayout->addStretch(1);
		binding.read = [box](TrajectoryParamValue& out)
		{
			out.kind = TrajectoryParamValue::Kind::Bool;
			out.asBool = box->isChecked();
			return true;
		};
		binding.write = [box](const TrajectoryParamValue& in)
		{
			if (in.kind != TrajectoryParamValue::Kind::Bool)
			{
				return false;
			}
			box->setChecked(in.asBool);
			return true;
		};
	}
	else if (field.type == TrajectoryParamType::Enum)
	{
		auto* combo = new QComboBox(host);
		applyCompactControlHeight(combo);
		makeCompactRowLayout(host)->addWidget(combo, 1);
		for (size_t i = 0; i < field.enumValues.size(); ++i)
		{
			const QString label = (useChinese && i < field.enumLabelsZh.size())
									  ? QString::fromStdString(field.enumLabelsZh[i])
									  : (i < field.enumLabelsEn.size() ? QString::fromStdString(field.enumLabelsEn[i])
																	   : QString::fromStdString(field.enumValues[i]));
			combo->addItem(label, QString::fromStdString(field.enumValues[i]));
		}
		binding.read = [combo](TrajectoryParamValue& out)
		{
			out.kind = TrajectoryParamValue::Kind::Int;
			bool ok = false;
			out.asInt = combo->currentData().toInt(&ok);
			if (!ok)
			{
				out.asInt = combo->currentIndex();
			}
			return true;
		};
		binding.write = [combo, field](const TrajectoryParamValue& in)
		{
			if (in.kind != TrajectoryParamValue::Kind::Int)
			{
				return false;
			}
			const int idx = combo->findData(QString::number(in.asInt));
			if (idx >= 0)
			{
				combo->setCurrentIndex(idx);
				return true;
			}
			if (in.asInt >= 0 && in.asInt < combo->count())
			{
				combo->setCurrentIndex(in.asInt);
				return true;
			}
			(void)field;
			return false;
		};
	}
	else if (field.type == TrajectoryParamType::Vec3)
	{
		auto* layout = makeCompactRowLayout(host);
		auto* spinX = new QDoubleSpinBox(host);
		auto* spinY = new QDoubleSpinBox(host);
		auto* spinZ = new QDoubleSpinBox(host);
		for (QDoubleSpinBox* spin : {spinX, spinY, spinZ})
		{
			spin->setRange(field.minValue, field.maxValue);
			spin->setSingleStep(field.step);
			spin->setDecimals(field.step < 0.1 ? 3 : 2);
			applyCompactControlHeight(spin);
		}
		layout->addWidget(spinX, 1);
		layout->addWidget(spinY, 1);
		layout->addWidget(spinZ, 1);
		binding.readVec3 = [spinX, spinY, spinZ](double& x, double& y, double& z)
		{
			x = spinX->value();
			y = spinY->value();
			z = spinZ->value();
			return true;
		};
		binding.writeVec3 = [spinX, spinY, spinZ](const double x, const double y, const double z)
		{
			spinX->setValue(x);
			spinY->setValue(y);
			spinZ->setValue(z);
			return true;
		};
	}

	return binding;
}

} // namespace trajectory_algo
