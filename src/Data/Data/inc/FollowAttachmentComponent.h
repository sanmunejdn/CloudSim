#pragma once

#include "BackendComponent.h"
#include "BackendDataBase.h"
#include "BackendFollowMath.h"
#include "data_global.h"

#include <json.hpp>
#include <mutex>
#include <string>

class BackendDataManager;

/// Optional rigid follow: follower outer-branch world = targetWorld * localTransform.
class DATA_EXPORT FollowAttachmentComponent : public IBackendComponent
{
public:
	static constexpr const char* typeKeyStatic() { return "FollowAttachment"; }

	std::string componentType() const override { return typeKeyStatic(); }

	bool enabled() const;
	void setEnabled(bool on);

	const std::string& targetBackendId() const;
	void setTargetBackendId(std::string id);

	BackendVec3 localPosition() const;
	void setLocalPosition(const BackendVec3& p);

	BackendVec3 localEulerDeg() const;
	void setLocalEulerDeg(const BackendVec3& e);

	/// When true, property solver skips updating this follower (e.g. user is dragging the gizmo on it).
	bool solverPaused() const;
	void setSolverPaused(bool on);

	/// True when follow target was set from backend parent/edge (cleared when user edits follow target).
	bool hierarchyDriven() const;
	void setHierarchyDriven(bool on);

	/// Single UI row: follow target by object \a name (resolved via \a mgr).
	void appendPropertyRows(nlohmann::json& rows, const BackendDataManager* mgr = nullptr) const;
	bool applyPropertyChange(BackendDataBase& owner, const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr);

	void writeJson(nlohmann::json& out) const;
	void readJson(const nlohmann::json& in);

	/// Recompute local offset from current world poses (target and follower must be valid in \a mgr).
	static bool recomputeLocalFromCurrentWorld(const BackendDataManager& mgr,
		const std::function<bool(const std::string&, BackendMat4& out)>& worldQuery, BackendDataBase& follower,
		std::string* errMsg);

private:
	mutable std::mutex m_mutex;
	bool m_enabled = false;
	std::string m_targetId;
	BackendVec3 m_localPos{};
	BackendVec3 m_localEuler{};
	bool m_solverPaused = false;
	bool m_hierarchyDriven = false;
};
