/// @file PathPlanRawStore.cpp
/// @brief PathPlan Raw 存储

#include "RobotProgramCatalog.h"

namespace RobotInstruction
{
namespace
{
nlohmann::json vec3ToJson(const Vec3& v)
{
	return nlohmann::json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

Vec3 vec3FromJson(const nlohmann::json& j)
{
	Vec3 v;
	if (j.is_object())
	{
		v.x = j.value("x", 0.0);
		v.y = j.value("y", 0.0);
		v.z = j.value("z", 0.0);
	}
	return v;
}

nlohmann::json rawTrajectoryToJson(const RawTrajectory& raw)
{
	nlohmann::json j = nlohmann::json::object();
	nlohmann::json pts = nlohmann::json::array();
	for (const TrajectoryPoint& pt : raw.points)
	{
		nlohmann::json pj = nlohmann::json::object();
		pj["poseMm"] = vec3ToJson(pt.poseMm);
		pj["eulerDeg"] = vec3ToJson(pt.eulerDeg);
		pj["blendRadiusMm"] = pt.blendRadiusMm;
		pj["speedMmPerSec"] = pt.speedMmPerSec;
		pj["reachable"] = pt.reachable;
		pts.push_back(std::move(pj));
	}
	j["points"] = std::move(pts);
	if (!raw.segmentEndExclusive.empty())
	{
		j["segmentEndExclusive"] = raw.segmentEndExclusive;
	}
	nlohmann::json ctx = nlohmann::json::object();
	ctx["workpieceFrameId"] = raw.ctx.workpieceFrameId;
	ctx["toolFrameId"] = raw.ctx.toolFrameId;
	j["ctx"] = std::move(ctx);
	if (!raw.sourceFeatureJson.empty())
	{
		const nlohmann::json feat = nlohmann::json::parse(raw.sourceFeatureJson, nullptr, false);
		if (!feat.is_discarded())
		{
			j["sourceFeature"] = feat;
		}
		else
		{
			j["sourceFeature"] = raw.sourceFeatureJson;
		}
	}
	return j;
}

bool rawTrajectoryFromJson(const nlohmann::json& j, RawTrajectory& out, std::string* errMsg)
{
	(void)errMsg;
	out = RawTrajectory{};
	if (!j.is_object())
	{
		if (errMsg)
		{
			*errMsg = "RawTrajectory JSON must be an object";
		}
		return false;
	}
	if (j.contains("points") && j["points"].is_array())
	{
		for (const auto& pj : j["points"])
		{
			if (!pj.is_object())
			{
				continue;
			}
			TrajectoryPoint pt;
			pt.poseMm = vec3FromJson(pj.value("poseMm", nlohmann::json::object()));
			pt.eulerDeg = vec3FromJson(pj.value("eulerDeg", nlohmann::json::object()));
			pt.blendRadiusMm = pj.value("blendRadiusMm", 0.0);
			pt.speedMmPerSec = pj.value("speedMmPerSec", 0.0);
			pt.reachable = pj.value("reachable", true);
			out.points.push_back(pt);
		}
	}
	if (j.contains("segmentEndExclusive") && j["segmentEndExclusive"].is_array())
	{
		for (const auto& end : j["segmentEndExclusive"])
		{
			if (end.is_number_unsigned())
			{
				out.segmentEndExclusive.push_back(end.get<std::size_t>());
			}
		}
	}
	if (j.contains("ctx") && j["ctx"].is_object())
	{
		const auto& c = j["ctx"];
		out.ctx.workpieceFrameId = c.value("workpieceFrameId", out.ctx.workpieceFrameId);
		out.ctx.toolFrameId = c.value("toolFrameId", out.ctx.toolFrameId);
	}
	if (j.contains("sourceFeature"))
	{
		if (j["sourceFeature"].is_object())
		{
			out.sourceFeatureJson = j["sourceFeature"].dump();
		}
		else if (j["sourceFeature"].is_string())
		{
			out.sourceFeatureJson = j["sourceFeature"].get<std::string>();
		}
	}
	return true;
}
} // namespace

bool PathPlanRawStore::save(const std::string& pathPlanId, const RawTrajectory& raw)
{
	if (pathPlanId.empty())
	{
		return false;
	}
	m_entries[pathPlanId] = raw;
	return true;
}

bool PathPlanRawStore::load(const std::string& pathPlanId, RawTrajectory& out) const
{
	const auto it = m_entries.find(pathPlanId);
	if (it == m_entries.end())
	{
		return false;
	}
	out = it->second;
	return true;
}

bool PathPlanRawStore::remove(const std::string& pathPlanId)
{
	return m_entries.erase(pathPlanId) > 0;
}

bool PathPlanRawStore::contains(const std::string& pathPlanId) const
{
	return m_entries.find(pathPlanId) != m_entries.end();
}

void PathPlanRawStore::clear()
{
	m_entries.clear();
}

nlohmann::json PathPlanRawStore::toJson() const
{
	nlohmann::json arr = nlohmann::json::array();
	for (const auto& kv : m_entries)
	{
		nlohmann::json item = nlohmann::json::object();
		item["pathPlanId"] = kv.first;
		item["raw"] = rawTrajectoryToJson(kv.second);
		arr.push_back(std::move(item));
	}
	return arr;
}

bool PathPlanRawStore::fromJson(const nlohmann::json& j, PathPlanRawStore& out, std::string* errMsg)
{
	out.clear();
	if (!j.is_array())
	{
		return true;
	}
	for (const auto& item : j)
	{
		if (!item.is_object())
		{
			continue;
		}
		const std::string id = item.value("pathPlanId", std::string());
		if (id.empty() || !item.contains("raw"))
		{
			continue;
		}
		RawTrajectory raw;
		if (!rawTrajectoryFromJson(item["raw"], raw, errMsg))
		{
			return false;
		}
		out.save(id, raw);
	}
	return true;
}

} // namespace RobotInstruction
