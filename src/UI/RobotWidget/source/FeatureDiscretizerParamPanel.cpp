/// @file FeatureDiscretizerParamPanel.cpp
/// @brief FeatureDiscretizerParam 面板

#include "FeatureDiscretizerParamPanel.h"

#include "GeometryRef.h"
#include "TrajectoryParamWidgetFactory.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include <TrajectoryOpParamSchema.h>

namespace
{
trajectory_algo::TrajectoryParamType mapParamType(const geoalgo::FeatureParamType type)
{
	switch (type)
	{
	case geoalgo::FeatureParamType::Int:
		return trajectory_algo::TrajectoryParamType::Int;
	case geoalgo::FeatureParamType::Bool:
		return trajectory_algo::TrajectoryParamType::Bool;
	case geoalgo::FeatureParamType::Enum:
		return trajectory_algo::TrajectoryParamType::Enum;
	case geoalgo::FeatureParamType::Vec3:
		return trajectory_algo::TrajectoryParamType::Vec3;
	case geoalgo::FeatureParamType::Message:
		return trajectory_algo::TrajectoryParamType::Message;
	case geoalgo::FeatureParamType::Double:
	default:
		return trajectory_algo::TrajectoryParamType::Double;
	}
}

} // namespace

trajectory_algo::TrajectoryOpParamField
FeatureDiscretizerParamPanel::toTrajectoryField(const geoalgo::FeatureDiscretizerParamField& field)
{
	trajectory_algo::TrajectoryOpParamField out{};
	out.key = field.key;
	out.type = mapParamType(field.type);
	out.labelEn = field.labelEn;
	out.labelZh = field.labelZh;
	out.unit = field.unit;
	out.group = field.group;
	out.order = field.order;
	out.minValue = field.minValue;
	out.maxValue = field.maxValue;
	out.step = field.step;
	out.minInt = field.minInt;
	out.maxInt = field.maxInt;
	out.defaultDouble = field.defaultDouble;
	out.defaultInt = field.defaultInt;
	out.defaultBool = field.defaultBool;
	out.enumValues = field.enumValues;
	out.enumLabelsZh = field.enumLabelsZh;
	out.enumLabelsEn = field.enumLabelsEn;
	out.messageEn = field.messageEn;
	out.messageZh = field.messageZh;
	return out;
}

nlohmann::json
FeatureDiscretizerParamPanel::defaultParamsFromFields(const std::vector<geoalgo::FeatureDiscretizerParamField>& fields)
{
	nlohmann::json params = nlohmann::json::object();
	for (const geoalgo::FeatureDiscretizerParamField& field : fields)
	{
		switch (field.type)
		{
		case geoalgo::FeatureParamType::Int:
			params[field.key] = field.defaultInt;
			break;
		case geoalgo::FeatureParamType::Bool:
			params[field.key] = field.defaultBool;
			break;
		case geoalgo::FeatureParamType::Enum:
			if (!field.enumValues.empty())
			{
				const int idx = std::clamp(field.defaultInt, 0, static_cast<int>(field.enumValues.size()) - 1);
				params[field.key] = field.enumValues[static_cast<std::size_t>(idx)];
			}
			break;
		case geoalgo::FeatureParamType::Message:
			break;
		case geoalgo::FeatureParamType::Double:
		default:
			params[field.key] = field.defaultDouble;
			break;
		}
	}
	return params;
}

void FeatureDiscretizerParamPanel::writeJsonValue(nlohmann::json& params,
												  const geoalgo::FeatureDiscretizerParamField& field,
												  const trajectory_algo::TrajectoryParamValue& value)
{
	switch (field.type)
	{
	case geoalgo::FeatureParamType::Int:
		params[field.key] = value.asInt;
		break;
	case geoalgo::FeatureParamType::Bool:
		params[field.key] = value.asBool;
		break;
	case geoalgo::FeatureParamType::Enum:
		if (value.kind == trajectory_algo::TrajectoryParamValue::Kind::String)
		{
			params[field.key] = value.asString;
		}
		else if (!field.enumValues.empty() && value.asInt >= 0 &&
				 value.asInt < static_cast<int>(field.enumValues.size()))
		{
			params[field.key] = field.enumValues[static_cast<std::size_t>(value.asInt)];
		}
		break;
	case geoalgo::FeatureParamType::Double:
	default:
		params[field.key] = value.asDouble;
		break;
	}
}

bool FeatureDiscretizerParamPanel::readJsonValue(const nlohmann::json& params,
												 const geoalgo::FeatureDiscretizerParamField& field,
												 trajectory_algo::TrajectoryParamValue& out)
{
	if (!params.contains(field.key))
	{
		return false;
	}
	const nlohmann::json& v = params.at(field.key);
	switch (field.type)
	{
	case geoalgo::FeatureParamType::Int:
		if (!v.is_number_integer())
		{
			return false;
		}
		out.kind = trajectory_algo::TrajectoryParamValue::Kind::Int;
		out.asInt = v.get<int>();
		return true;
	case geoalgo::FeatureParamType::Bool:
		if (!v.is_boolean())
		{
			return false;
		}
		out.kind = trajectory_algo::TrajectoryParamValue::Kind::Bool;
		out.asBool = v.get<bool>();
		return true;
	case geoalgo::FeatureParamType::Enum:
		if (v.is_string())
		{
			out.kind = trajectory_algo::TrajectoryParamValue::Kind::String;
			out.asString = v.get<std::string>();
			return true;
		}
		return false;
	case geoalgo::FeatureParamType::Double:
	default:
		if (!v.is_number())
		{
			return false;
		}
		out.kind = trajectory_algo::TrajectoryParamValue::Kind::Double;
		out.asDouble = v.get<double>();
		return true;
	}
}

FeatureDiscretizerParamPanel::FeatureDiscretizerParamPanel(QWidget* parent) : QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	m_form = new QFormLayout();
	m_form->setContentsMargins(0, 0, 0, 0);
	m_form->setSpacing(1);
	m_form->setVerticalSpacing(1);
	m_form->setHorizontalSpacing(8);
	m_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
	m_form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	layout->addLayout(m_form, 1);
}

void FeatureDiscretizerParamPanel::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
}

void FeatureDiscretizerParamPanel::setLoading(const bool loading)
{
	m_loading = loading;
}

void FeatureDiscretizerParamPanel::clear()
{
	clearRows();
}

void FeatureDiscretizerParamPanel::clearRows()
{
	if (m_clearingRows)
	{
		return;
	}
	m_clearingRows = true;
	for (ParamBinding& row : m_rows)
	{
		if (row.label && m_form->indexOf(row.label) >= 0)
		{
			m_form->removeWidget(row.label);
		}
		if (row.widget && m_form->indexOf(row.widget) >= 0)
		{
			m_form->removeWidget(row.widget);
		}
		delete row.label;
		delete row.widget;
		row.label = nullptr;
		row.widget = nullptr;
	}
	m_rows.clear();
	m_clearingRows = false;
}

void FeatureDiscretizerParamPanel::rebuildForStrategy(const std::string& strategyId)
{
	if (m_rebuilding || m_clearingRows)
	{
		return;
	}
	m_rebuilding = true;
	const bool wasLoading = m_loading;
	m_loading = true;
	clearRows();
	m_strategyId = strategyId;
	const std::vector<geoalgo::FeatureDiscretizerParamField> fields =
		geometry_backend_ops::featureDiscretizerAllParamFields(strategyId);
	for (const geoalgo::FeatureDiscretizerParamField& field : fields)
	{
		const trajectory_algo::TrajectoryOpParamField trajField = toTrajectoryField(field);
		trajectory_algo::TrajectoryParamBinding binding =
			trajectory_algo::TrajectoryParamWidgetFactory::create(trajField, m_useChinese);
		ParamBinding row{};
		row.label = binding.label;
		row.widget = binding.widget;
		row.field = field;
		row.read = std::move(binding.read);
		row.write = std::move(binding.write);
		row.readVec3 = std::move(binding.readVec3);
		row.writeVec3 = std::move(binding.writeVec3);
		if (row.label && row.widget)
		{
			m_form->addRow(row.label, row.widget);
		}
		if (field.type != geoalgo::FeatureParamType::Message && row.widget)
		{
			if (auto* spin = row.widget->findChild<QDoubleSpinBox*>())
			{
				spin->setKeyboardTracking(false);
				connect(spin, &QAbstractSpinBox::editingFinished, this,
						[this]()
						{
							if (!m_loading)
							{
								emit paramsChanged();
							}
						});
			}
			if (auto* spin = row.widget->findChild<QSpinBox*>())
			{
				spin->setKeyboardTracking(false);
				connect(spin, &QAbstractSpinBox::editingFinished, this,
						[this]()
						{
							if (!m_loading)
							{
								emit paramsChanged();
							}
						});
			}
			if (auto* combo = row.widget->findChild<QComboBox*>())
			{
				connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
						[this]()
						{
							if (!m_loading)
							{
								emit paramsChanged();
							}
						});
			}
			if (auto* box = row.widget->findChild<QCheckBox*>())
			{
				connect(box, &QCheckBox::toggled, this,
						[this]()
						{
							if (!m_loading)
							{
								emit paramsChanged();
							}
						});
			}
		}
		m_rows.push_back(std::move(row));
	}
	m_loading = wasLoading;
	m_rebuilding = false;
}

void FeatureDiscretizerParamPanel::loadParams(const nlohmann::json& params)
{
	const bool wasLoading = m_loading;
	m_loading = true;
	for (ParamBinding& row : m_rows)
	{
		trajectory_algo::TrajectoryParamValue value{};
		if (!readJsonValue(params, row.field, value))
		{
			continue;
		}
		if (row.write)
		{
			row.write(value);
		}
	}
	m_loading = wasLoading;
}

bool FeatureDiscretizerParamPanel::applyParams(nlohmann::json& outParams) const
{
	if (m_rebuilding)
	{
		return false;
	}
	if (!outParams.is_object())
	{
		outParams = nlohmann::json::object();
	}
	for (const ParamBinding& row : m_rows)
	{
		if (!row.read)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamValue value{};
		if (!row.read(value))
		{
			continue;
		}
		writeJsonValue(outParams, row.field, value);
	}
	return true;
}
