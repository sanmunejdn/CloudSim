#include "RobotCoordinateFrames.h"

#include "BackendDataBase.h"
#include "ToolKinematics.h"

#include <Adapters.h>

#include <atomic>
#include <cmath>
#include <sstream>
#include <unordered_set>

namespace RobotCoordinate
{
const RobotToolFrame* findToolFrameById(const RobotCoordinateFrameSet& set, const std::string& id);

namespace
{
void readVec3Array(const nlohmann::json& j, double out[3])
{
	if (!j.is_array() || j.size() < 3)
	{
		return;
	}
	out[0] = j[0].get<double>();
	out[1] = j[1].get<double>();
	out[2] = j[2].get<double>();
}

void writeVec3Array(nlohmann::json& j, const double v[3])
{
	j = nlohmann::json::array({ v[0], v[1], v[2] });
}

bool readRigidFrameJson(const nlohmann::json& j, RobotRigidFrame& out)
{
	if (!j.is_object())
	{
		return false;
	}
	if (j.contains("positionMm"))
	{
		readVec3Array(j["positionMm"], out.positionMm);
	}
	if (j.contains("eulerDeg"))
	{
		readVec3Array(j["eulerDeg"], out.eulerDeg);
	}
	return true;
}

void writeRigidFrameJson(const RobotRigidFrame& frame, nlohmann::json& out)
{
	out = nlohmann::json::object();
	writeVec3Array(out["positionMm"], frame.positionMm);
	writeVec3Array(out["eulerDeg"], frame.eulerDeg);
}

std::atomic<unsigned long long>& toolFrameIdCounter()
{
	static std::atomic<unsigned long long> sCounter{ 1ULL };
	return sCounter;
}

unsigned long long maxToolFrameIdSuffix(const RobotCoordinateFrameSet& set)
{
	unsigned long long maxSuffix = 0;
	for (const RobotToolFrame& tf : set.toolFrames)
	{
		if (tf.id.size() <= 4 || tf.id.compare(0, 4, "TFR_") != 0)
		{
			continue;
		}
		try
		{
			maxSuffix = std::max(maxSuffix, std::stoull(tf.id.substr(4)));
		}
		catch (...)
		{
		}
	}
	return maxSuffix;
}

void bumpToolFrameIdCounterAtLeast(const unsigned long long nextId)
{
	unsigned long long current = toolFrameIdCounter().load();
	while (current < nextId && !toolFrameIdCounter().compare_exchange_weak(current, nextId))
	{
	}
}
} // namespace

std::string makeToolFrameId()
{
	return std::string("TFR_") + std::to_string(toolFrameIdCounter().fetch_add(1ULL));
}

std::string allocateUniqueToolFrameId(const RobotCoordinateFrameSet& set)
{
	bumpToolFrameIdCounterAtLeast(maxToolFrameIdSuffix(set) + 1ULL);
	for (int guard = 0; guard < 10000; ++guard)
	{
		const std::string id = makeToolFrameId();
		if (!findToolFrameById(set, id))
		{
			return id;
		}
	}
	return std::string("TFR_") + std::to_string(maxToolFrameIdSuffix(set) + 1000ULL);
}

void ensureUniqueToolFrameIds(RobotCoordinateFrameSet& set)
{
	std::unordered_set<std::string> seen;
	for (RobotToolFrame& tf : set.toolFrames)
	{
		while (tf.id.empty() || seen.count(tf.id) != 0)
		{
			tf.id = allocateUniqueToolFrameId(set);
		}
		seen.insert(tf.id);
	}
	bumpToolFrameIdCounterAtLeast(maxToolFrameIdSuffix(set) + 1ULL);
}

std::string makeUserFrameId()
{
	static std::atomic<unsigned long long> sCounter{ 1ULL };
	return std::string("UFR_") + std::to_string(sCounter.fetch_add(1ULL));
}

RobotRigidFrame identityRigidFrame()
{
	return RobotRigidFrame{};
}

engine::RigidTransform rigidTransformFromFrame(const RobotRigidFrame& frame)
{
	return engine::RigidTransform::fromTranslationEulerDeg(
		frame.positionMm[0],
		frame.positionMm[1],
		frame.positionMm[2],
		frame.eulerDeg[0],
		frame.eulerDeg[1],
		frame.eulerDeg[2]);
}

RobotRigidFrame frameFromRigidTransform(const engine::RigidTransform& t)
{
	RobotRigidFrame out{};
	t.translationMm(out.positionMm[0], out.positionMm[1], out.positionMm[2]);
	t.eulerDegForDisplay(out.eulerDeg[0], out.eulerDeg[1], out.eulerDeg[2]);
	return out;
}

engine::RigidTransform rigidTransformFromBackendMat4(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = m.v[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

BackendMat4 backendMat4FromRigidTransform(const engine::RigidTransform& t)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(t);
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}

engine::RigidTransform targetRigidTransformFromPose(
	double px,
	double py,
	double pz,
	double ex,
	double ey,
	double ez)
{
	return engine::RigidTransform::fromTranslationEulerDeg(px, py, pz, ex, ey, ez);
}

BackendMat4 frameToMat4(const RobotRigidFrame& frame)
{
	return backendMat4FromRigidTransform(rigidTransformFromFrame(frame));
}

RobotRigidFrame mat4ToFrame(const BackendMat4& m)
{
	return frameFromRigidTransform(rigidTransformFromBackendMat4(m));
}

BackendMat4 targetInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez)
{
	return backendMat4FromRigidTransform(targetRigidTransformFromPose(px, py, pz, ex, ey, ez));
}

BackendMat4 tcpInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez)
{
	return targetInBaseFromPose(px, py, pz, ex, ey, ez);
}

void poseEulerFromTargetInBase(const BackendMat4& T_base_target, double outPos[3], double outEulerDeg[3])
{
	const engine::RigidTransform t = rigidTransformFromBackendMat4(T_base_target);
	if (outPos)
	{
		t.translationMm(outPos[0], outPos[1], outPos[2]);
	}
	if (outEulerDeg)
	{
		t.eulerDegForDisplay(outEulerDeg[0], outEulerDeg[1], outEulerDeg[2]);
	}
}

void poseEulerFromTcpInBase(const BackendMat4& T_base_tcp, double outPos[3], double outEulerDeg[3])
{
	poseEulerFromTargetInBase(T_base_tcp, outPos, outEulerDeg);
}

BackendMat4 flangeTargetFromToolOriginInBase(const BackendMat4& T_base_target, const BackendMat4& T_flange_tool)
{
	const engine::RigidTransform flange = engine::flangeFromToolOrigin(
		rigidTransformFromBackendMat4(T_base_target),
		rigidTransformFromBackendMat4(T_flange_tool));
	return backendMat4FromRigidTransform(flange);
}

BackendMat4 flangeTargetFromBaseTcpAndTool(const BackendMat4& T_base_tcp, const BackendMat4& T_flange_tool)
{
	return flangeTargetFromToolOriginInBase(T_base_tcp, T_flange_tool);
}

BackendMat4 targetInBaseFromFlange(const BackendMat4& T_base_flange, const BackendMat4& T_flange_tool)
{
	const engine::RigidTransform target = engine::toolOriginFromFlange(
		rigidTransformFromBackendMat4(T_base_flange),
		rigidTransformFromBackendMat4(T_flange_tool));
	return backendMat4FromRigidTransform(target);
}

BackendMat4 tcpInBaseFromUserTcp(const BackendMat4& T_base_user, const BackendMat4& T_user_tcp)
{
	BackendMat4 out{};
	backend_mat4_multiply(T_base_user, T_user_tcp, out);
	return out;
}

BackendMat4 tcpInUserFromBaseTcp(const BackendMat4& T_base_user, const BackendMat4& T_base_tcp)
{
	BackendMat4 invUser{};
	if (!backend_mat4_invert_rigid(T_base_user, invUser))
	{
		return T_base_tcp;
	}
	BackendMat4 out{};
	backend_mat4_multiply(invUser, T_base_tcp, out);
	return out;
}

std::string encodeMat4Csv(const BackendMat4& m)
{
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	for (int i = 0; i < 16; ++i)
	{
		if (i > 0)
		{
			oss << ',';
		}
		oss << m.v[i];
	}
	return oss.str();
}

bool parseMat4Csv(const std::string& csv, BackendMat4& out)
{
	out = BackendMat4::identity();
	std::stringstream ss(csv);
	std::string token;
	int idx = 0;
	while (std::getline(ss, token, ',') && idx < 16)
	{
		if (token.empty())
		{
			continue;
		}
		try
		{
			out.v[idx++] = std::stod(token);
		}
		catch (...)
		{
			return false;
		}
	}
	return idx == 16;
}

const RobotToolFrame* resolveToolFrameForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext)
{
	const auto itId = ext.find(kExtMotionToolFrameId);
	if (itId != ext.end() && !itId->second.empty() && itId->second != "active")
	{
		if (const RobotToolFrame* tf = findToolFrameById(set, itId->second))
		{
			return tf;
		}
	}
	const auto itFrozenId = ext.find("context.activeToolFrameId");
	if (itFrozenId != ext.end() && !itFrozenId->second.empty())
	{
		if (const RobotToolFrame* tf = findToolFrameById(set, itFrozenId->second))
		{
			return tf;
		}
	}
	return activeToolFrame(set);
}

BackendMat4 toolMat4ForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext)
{
	if (const RobotToolFrame* tool = resolveToolFrameForExtension(set, ext))
	{
		return frameToMat4(tool->T_flange_tool);
	}
	const auto itTool = ext.find(kExtContextToolFrameMat4);
	if (itTool != ext.end() && !itTool->second.empty())
	{
		BackendMat4 fromIns{};
		if (parseMat4Csv(itTool->second, fromIns))
		{
			return fromIns;
		}
	}
	return BackendMat4::identity();
}

const RobotUserFrame* resolveUserFrameForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext)
{
	const auto itId = ext.find(kExtMotionUserFrameId);
	if (itId != ext.end() && !itId->second.empty() && itId->second != "active")
	{
		if (const RobotUserFrame* uf = findUserFrameById(set, itId->second))
		{
			return uf;
		}
	}
	return activeUserFrame(set);
}

bool instructionTargetDisplayUsesUserFrame(const std::unordered_map<std::string, std::string>& ext)
{
	const auto it = ext.find(kExtMotionTargetFrame);
	if (it == ext.end() || it->second.empty())
	{
		return false;
	}
	return it->second == "user" || it->second == "active_user";
}

RobotCoordinateFrameSet makeDefaultFrameSet(const std::string& defaultFlangeLinkName)
{
	RobotCoordinateFrameSet set;
	set.flangeLinkName = defaultFlangeLinkName;
	RobotToolFrame tf;
	tf.id = makeToolFrameId();
	tf.name = "TFrame1";
	tf.T_flange_tool = identityRigidFrame();
	set.toolFrames.push_back(std::move(tf));
	set.activeToolFrameId = set.toolFrames.front().id;
	RobotUserFrame uf;
	uf.id = makeUserFrameId();
	uf.name = "UFrame1";
	uf.T_base_user = identityRigidFrame();
	set.userFrames.push_back(std::move(uf));
	set.activeUserFrameId = set.userFrames.front().id;
	return set;
}

std::string effectiveFlangeLinkName(const RobotCoordinateFrameSet& set, const RobotToolFrame& tool)
{
	if (!tool.flangeLinkName.empty())
	{
		return tool.flangeLinkName;
	}
	return set.flangeLinkName;
}

const RobotToolFrame* findToolFrameById(const RobotCoordinateFrameSet& set, const std::string& id)
{
	for (const RobotToolFrame& tf : set.toolFrames)
	{
		if (tf.id == id)
		{
			return &tf;
		}
	}
	return nullptr;
}

const RobotToolFrame* activeToolFrame(const RobotCoordinateFrameSet& set)
{
	if (set.activeToolFrameId.empty())
	{
		return set.toolFrames.empty() ? nullptr : &set.toolFrames.front();
	}
	return findToolFrameById(set, set.activeToolFrameId);
}

const RobotUserFrame* findUserFrameById(const RobotCoordinateFrameSet& set, const std::string& id)
{
	for (const RobotUserFrame& uf : set.userFrames)
	{
		if (uf.id == id)
		{
			return &uf;
		}
	}
	return nullptr;
}

const RobotUserFrame* activeUserFrame(const RobotCoordinateFrameSet& set)
{
	if (set.activeUserFrameId.empty())
	{
		return set.userFrames.empty() ? nullptr : &set.userFrames.front();
	}
	return findUserFrameById(set, set.activeUserFrameId);
}

void writeCoordinateFrameSetToJson(const RobotCoordinateFrameSet& set, nlohmann::json& out)
{
	out = nlohmann::json::object();
	if (!set.flangeLinkName.empty())
	{
		out["flangeLinkName"] = set.flangeLinkName;
	}
	nlohmann::json toolArr = nlohmann::json::array();
	for (const RobotToolFrame& tf : set.toolFrames)
	{
		nlohmann::json item;
		item["id"] = tf.id;
		item["name"] = tf.name;
		writeRigidFrameJson(tf.T_flange_tool, item["T_flange_tool"]);
		if (!tf.flangeLinkName.empty())
		{
			item["flangeLinkName"] = tf.flangeLinkName;
		}
		item["showInScene"] = tf.showInScene;
		toolArr.push_back(std::move(item));
	}
	out["toolFrames"] = std::move(toolArr);
	if (!set.activeToolFrameId.empty())
	{
		out["activeToolFrameId"] = set.activeToolFrameId;
	}
	nlohmann::json arr = nlohmann::json::array();
	for (const RobotUserFrame& uf : set.userFrames)
	{
		nlohmann::json item;
		item["id"] = uf.id;
		item["name"] = uf.name;
		writeRigidFrameJson(uf.T_base_user, item["T_base_user"]);
		item["showInScene"] = uf.showInScene;
		arr.push_back(std::move(item));
	}
	out["userFrames"] = std::move(arr);
	if (!set.activeUserFrameId.empty())
	{
		out["activeUserFrameId"] = set.activeUserFrameId;
	}
	out["showToolFrame"] = set.showToolFrameInScene;
	out["showUserFrames"] = set.showUserFramesInScene;
}

bool readCoordinateFrameSetFromJson(const nlohmann::json& in, RobotCoordinateFrameSet& out)
{
	if (!in.is_object())
	{
		return false;
	}
	if (in.contains("flangeLinkName") && in["flangeLinkName"].is_string())
	{
		out.flangeLinkName = in["flangeLinkName"].get<std::string>();
	}
	out.toolFrames.clear();
	if (in.contains("toolFrames") && in["toolFrames"].is_array())
	{
		for (const auto& item : in["toolFrames"])
		{
			if (!item.is_object())
			{
				continue;
			}
			RobotToolFrame tf;
			if (item.contains("id") && item["id"].is_string())
			{
				tf.id = item["id"].get<std::string>();
			}
			if (item.contains("name") && item["name"].is_string())
			{
				tf.name = item["name"].get<std::string>();
			}
			if (item.contains("T_flange_tool"))
			{
				readRigidFrameJson(item["T_flange_tool"], tf.T_flange_tool);
			}
			else if (item.contains("toolInFlange"))
			{
				readRigidFrameJson(item["toolInFlange"], tf.T_flange_tool);
			}
			if (item.contains("flangeLinkName") && item["flangeLinkName"].is_string())
			{
				tf.flangeLinkName = item["flangeLinkName"].get<std::string>();
			}
			if (item.contains("showInScene") && item["showInScene"].is_boolean())
			{
				tf.showInScene = item["showInScene"].get<bool>();
			}
			if (tf.id.empty())
			{
				tf.id = makeToolFrameId();
			}
			if (tf.name.empty())
			{
				tf.name = "TFrame";
			}
			out.toolFrames.push_back(std::move(tf));
		}
	}
	else if (in.contains("toolInFlange"))
	{
		RobotToolFrame tf;
		tf.id = makeToolFrameId();
		tf.name = "TFrame1";
		readRigidFrameJson(in["toolInFlange"], tf.T_flange_tool);
		out.toolFrames.push_back(std::move(tf));
	}
	if (in.contains("activeToolFrameId") && in["activeToolFrameId"].is_string())
	{
		out.activeToolFrameId = in["activeToolFrameId"].get<std::string>();
	}
	else if (out.toolFrames.size() == 1)
	{
		out.activeToolFrameId = out.toolFrames.front().id;
	}
	out.userFrames.clear();
	if (in.contains("userFrames") && in["userFrames"].is_array())
	{
		for (const auto& item : in["userFrames"])
		{
			if (!item.is_object())
			{
				continue;
			}
			RobotUserFrame uf;
			if (item.contains("id") && item["id"].is_string())
			{
				uf.id = item["id"].get<std::string>();
			}
			if (item.contains("name") && item["name"].is_string())
			{
				uf.name = item["name"].get<std::string>();
			}
			if (item.contains("T_base_user"))
			{
				readRigidFrameJson(item["T_base_user"], uf.T_base_user);
			}
			if (item.contains("showInScene") && item["showInScene"].is_boolean())
			{
				uf.showInScene = item["showInScene"].get<bool>();
			}
			if (uf.id.empty())
			{
				uf.id = makeUserFrameId();
			}
			if (uf.name.empty())
			{
				uf.name = "UFrame";
			}
			out.userFrames.push_back(std::move(uf));
		}
	}
	if (in.contains("activeUserFrameId") && in["activeUserFrameId"].is_string())
	{
		out.activeUserFrameId = in["activeUserFrameId"].get<std::string>();
	}
	if (in.contains("showToolFrame"))
	{
		out.showToolFrameInScene = in["showToolFrame"].get<bool>();
	}
	if (in.contains("showUserFrames"))
	{
		out.showUserFramesInScene = in["showUserFrames"].get<bool>();
	}
	ensureUniqueToolFrameIds(out);
	return true;
}

} // namespace RobotCoordinate
