#include "RobotInstructionAttribute.h"

#include "RobotInstructionModel.h"

#include <sstream>

namespace
{
std::string fmt(double v)
{
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(3);
	oss << v;
	return oss.str();
}

bool parseDouble(const std::string& text, double& out, std::string* errMsg)
{
	try
	{
		size_t idx = 0;
		out = std::stod(text, &idx);
		while (idx < text.size() && (text[idx] == ' ' || text[idx] == '\t'))
		{
			++idx;
		}
		if (idx != text.size())
		{
			if (errMsg)
			{
				*errMsg = "Invalid number.";
			}
			return false;
		}
		return true;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "Invalid number.";
		}
		return false;
	}
}

void appendRow(nlohmann::json& rows, const char* key, const char* label, bool editable, const std::string& value)
{
	nlohmann::json row;
	row["key"] = key;
	row["label"] = label;
	row["editable"] = editable;
	row["value"] = value;
	rows.push_back(std::move(row));
}
} // namespace

namespace RobotInstruction
{
void PoseAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasPoseProperty())
	{
		return;
	}
	const Vec3 p = cmd.pose();
	appendRow(rows, "motion.target.pose.x", "Target X (mm)", true, fmt(p.x));
	appendRow(rows, "motion.target.pose.y", "Target Y (mm)", true, fmt(p.y));
	appendRow(rows, "motion.target.pose.z", "Target Z (mm)", true, fmt(p.z));
}

bool PoseAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	if (!cmd.hasPoseProperty())
	{
		return false;
	}
	return key == "motion.target.pose.x" || key == "motion.target.pose.y" || key == "motion.target.pose.z";
}

bool PoseAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	Vec3 p = cmd.pose();
	if (key == "motion.target.pose.x")
	{
		p.x = v;
	}
	else if (key == "motion.target.pose.y")
	{
		p.y = v;
	}
	else
	{
		p.z = v;
	}
	cmd.setPose(p);
	return true;
}

void EulerAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasEulerProperty())
	{
		return;
	}
	const Vec3 r = cmd.eulerDeg();
	appendRow(rows, "motion.target.euler.rx", "Euler RX (deg)", true, fmt(r.x));
	appendRow(rows, "motion.target.euler.ry", "Euler RY (deg)", true, fmt(r.y));
	appendRow(rows, "motion.target.euler.rz", "Euler RZ (deg)", true, fmt(r.z));
}

bool EulerAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	if (!cmd.hasEulerProperty())
	{
		return false;
	}
	return key == "motion.target.euler.rx" || key == "motion.target.euler.ry" || key == "motion.target.euler.rz";
}

bool EulerAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	Vec3 r = cmd.eulerDeg();
	if (key == "motion.target.euler.rx")
	{
		r.x = v;
	}
	else if (key == "motion.target.euler.ry")
	{
		r.y = v;
	}
	else
	{
		r.z = v;
	}
	cmd.setEulerDeg(r);
	return true;
}

void SpeedAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasSpeedProperty())
	{
		return;
	}
	appendRow(rows, "motion.speed", "Speed", true, fmt(cmd.speed()));
}

bool SpeedAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	return cmd.hasSpeedProperty() && key == "motion.speed";
}

bool SpeedAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	cmd.setSpeed(v);
	return true;
}

void AccelAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasAccelProperty())
	{
		return;
	}
	appendRow(rows, "motion.acc", "Acceleration", true, fmt(cmd.accel()));
}

bool AccelAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	return cmd.hasAccelProperty() && key == "motion.acc";
}

bool AccelAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	cmd.setAccel(v);
	return true;
}

void AxisConfigAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasAxisConfigProperty())
	{
		return;
	}
	appendRow(rows, "motion.axisConfig", "Axis Configuration", true, cmd.axisConfig());
}

bool AxisConfigAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	return cmd.hasAxisConfigProperty() && key == "motion.axisConfig";
}

bool AxisConfigAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	(void)errMsg;
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	cmd.setAxisConfig(value);
	return true;
}

void BlendRadiusAttribute::appendRows(const Base& cmd, nlohmann::json& rows) const
{
	if (!cmd.hasBlendRadiusProperty())
	{
		return;
	}
	appendRow(rows, "motion.blendRadius", "Blend Radius (mm)", true, fmt(cmd.blendRadius()));
}

bool BlendRadiusAttribute::handlesKey(const Base& cmd, const std::string& key) const
{
	return cmd.hasBlendRadiusProperty() && key == "motion.blendRadius";
}

bool BlendRadiusAttribute::apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(cmd, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	cmd.setBlendRadius(v);
	return true;
}
} // namespace RobotInstruction
