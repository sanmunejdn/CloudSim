#include "RobotInstructionModel.h"

#include "RobotInstructionAttribute.h"

#include <atomic>

namespace
{
std::string makeInstructionId()
{
	static std::atomic<unsigned long long> sCounter{1ULL};
	const unsigned long long v = sCounter.fetch_add(1ULL);
	return std::string("INS_") + std::to_string(v);
}
} // namespace

namespace RobotInstruction
{
Base::Base()
	: m_id(makeInstructionId())
{
}

nlohmann::json Base::snapshotPropertyRows() const
{
	nlohmann::json rows = nlohmann::json::array();
	for (const auto& attr : m_attributes)
	{
		if (attr)
		{
			attr->appendRows(*this, rows);
		}
	}
	for (const auto& kv : m_extensionProperties)
	{
		nlohmann::json row;
		row["key"] = kv.first;
		row["label"] = kv.first;
		row["editable"] = true;
		row["value"] = kv.second;
		rows.push_back(std::move(row));
	}
	return rows;
}

bool Base::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg)
{
	for (const auto& attr : m_attributes)
	{
		if (!attr || !attr->handlesKey(*this, key))
		{
			continue;
		}
		if (attr->apply(*this, key, value, errMsg))
		{
			return true;
		}
		return false;
	}
	m_extensionProperties[key] = value;
	return true;
}

void Base::addAttribute(const std::shared_ptr<AttributeBase>& attr)
{
	if (attr)
	{
		m_attributes.push_back(attr);
	}
}

PtpInstruction::PtpInstruction()
{
	setType(Type::PTP);
	setName("PTP");
	addAttribute(std::make_shared<PoseAttribute>());
	addAttribute(std::make_shared<EulerAttribute>());
	addAttribute(std::make_shared<SpeedAttribute>());
	addAttribute(std::make_shared<AccelAttribute>());
	addAttribute(std::make_shared<AxisConfigAttribute>());
}

LineInstruction::LineInstruction()
{
	setType(Type::LINE);
	setName("LINE");
	addAttribute(std::make_shared<PoseAttribute>());
	addAttribute(std::make_shared<EulerAttribute>());
	addAttribute(std::make_shared<SpeedAttribute>());
	addAttribute(std::make_shared<AccelAttribute>());
	addAttribute(std::make_shared<BlendRadiusAttribute>());
}

} // namespace RobotInstruction
