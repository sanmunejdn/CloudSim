#include "BackendObjectAttribute.h"

#include "BackendDataBase.h"

#include "BackendPropertyRow.h"

#include <sstream>

namespace {

std::string formatDouble(double v)
{
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(3);
	oss << v;
	return oss.str();
}

bool parseDouble(const std::string& s, double& out, std::string* errMsg)
{
	try
	{
		size_t idx = 0;
		out = std::stod(s, &idx);
		while (idx < s.size() && (s[idx] == ' ' || s[idx] == '\t'))
		{
			++idx;
		}
		if (idx != s.size())
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

} // namespace

void BackendPoseAttribute::appendRows(const BackendDataBase& data, nlohmann::json& rows) const
{
	if (!data.hasPoseProperty())
	{
		return;
	}
	const auto p = data.pose();
	backend_property_json::appendRow(rows, "pose.x", "Pose X", true, formatDouble(p.x));
	backend_property_json::appendRow(rows, "pose.y", "Pose Y", true, formatDouble(p.y));
	backend_property_json::appendRow(rows, "pose.z", "Pose Z", true, formatDouble(p.z));
}

bool BackendPoseAttribute::handlesKey(const BackendDataBase& data, const std::string& key) const
{
	if (!data.hasPoseProperty())
	{
		return false;
	}
	return key == "pose.x" || key == "pose.y" || key == "pose.z";
}

bool BackendPoseAttribute::apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(data, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	BackendVec3 p = data.pose();
	if (key == "pose.x")
	{
		p.x = v;
	}
	else if (key == "pose.y")
	{
		p.y = v;
	}
	else if (key == "pose.z")
	{
		p.z = v;
	}
	else
	{
		return false;
	}
	data.setPose(p);
	return true;
}

void BackendRotationAttribute::appendRows(const BackendDataBase& data, nlohmann::json& rows) const
{
	if (!data.hasRotationProperty())
	{
		return;
	}
	const auto r = data.rotation();
	backend_property_json::appendRow(rows, "rotation.x", "Rotation X (deg)", true, formatDouble(r.x));
	backend_property_json::appendRow(rows, "rotation.y", "Rotation Y (deg)", true, formatDouble(r.y));
	backend_property_json::appendRow(rows, "rotation.z", "Rotation Z (deg)", true, formatDouble(r.z));
}

bool BackendRotationAttribute::handlesKey(const BackendDataBase& data, const std::string& key) const
{
	if (!data.hasRotationProperty())
	{
		return false;
	}
	return key == "rotation.x" || key == "rotation.y" || key == "rotation.z";
}

bool BackendRotationAttribute::apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(data, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	BackendVec3 r = data.rotation();
	if (key == "rotation.x")
	{
		r.x = v;
	}
	else if (key == "rotation.y")
	{
		r.y = v;
	}
	else if (key == "rotation.z")
	{
		r.z = v;
	}
	else
	{
		return false;
	}
	data.setRotation(r);
	return true;
}

void BackendDisplayColorAttribute::appendRows(const BackendDataBase& data, nlohmann::json& rows) const
{
	if (!data.hasColorProperty())
	{
		return;
	}
	const BackendColor c = data.color();
	backend_property_json::appendRow(rows, "color.r", "Color R", true, formatDouble(static_cast<double>(c.r)));
	backend_property_json::appendRow(rows, "color.g", "Color G", true, formatDouble(static_cast<double>(c.g)));
	backend_property_json::appendRow(rows, "color.b", "Color B", true, formatDouble(static_cast<double>(c.b)));
	backend_property_json::appendRow(rows, "color.a", "Color A", true, formatDouble(static_cast<double>(c.a)));
}

bool BackendDisplayColorAttribute::handlesKey(const BackendDataBase& data, const std::string& key) const
{
	if (!data.hasColorProperty())
	{
		return false;
	}
	return key == "color.r" || key == "color.g" || key == "color.b" || key == "color.a";
}

bool BackendDisplayColorAttribute::apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const
{
	if (!handlesKey(data, key))
	{
		return false;
	}
	double v = 0.0;
	if (!parseDouble(value, v, errMsg))
	{
		return false;
	}
	BackendColor c = data.color();
	if (key == "color.r")
	{
		c.r = static_cast<float>(v);
	}
	else if (key == "color.g")
	{
		c.g = static_cast<float>(v);
	}
	else if (key == "color.b")
	{
		c.b = static_cast<float>(v);
	}
	else if (key == "color.a")
	{
		c.a = static_cast<float>(v);
	}
	else
	{
		return false;
	}
	data.setColor(c);
	return true;
}

