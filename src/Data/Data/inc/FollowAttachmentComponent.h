#ifndef DATA_FOLLOWATTACHMENTCOMPONENT_H
#define DATA_FOLLOWATTACHMENTCOMPONENT_H

/// @file FollowAttachmentComponent.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 可选刚体跟随：follower 外支路世界 = targetWorld * localTransform

#include "data_global.h"

#include "BackendComponent.h"
#include "BackendDataBase.h"
#include "BackendFollowMath.h"

#include <mutex>
#include <string>

#include <json.hpp>

class BackendDataManager;

/// 可选刚体跟随：follower 外支路世界 = targetWorld * localTransform
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

	/// true 时求解器跳过此 follower（如用户拖 gizmo）
	bool solverPaused() const;
	void setSolverPaused(bool on);

	/// 跟随目标来自层级边（用户改 target 后清除）
	bool hierarchyDriven() const;
	void setHierarchyDriven(bool on);

	/// 面板单行：按对象名选跟随目标
	void appendPropertyRows(nlohmann::json& rows, const BackendDataManager* mgr = nullptr) const;
	bool applyPropertyChange(BackendDataBase& owner, const std::string& key, const std::string& value,
							 std::string* errMsg, const BackendDataManager* mgr = nullptr);

	void writeJson(nlohmann::json& out) const;
	void readJson(const nlohmann::json& in);

	/// 从当前世界位姿重算 local 偏移
	static bool
	recomputeLocalFromCurrentWorld(const BackendDataManager& mgr,
								   const std::function<bool(const std::string&, BackendMat4& out)>& worldQuery,
								   BackendDataBase& follower, std::string* errMsg);

private:
	mutable std::mutex m_mutex;
	bool m_enabled = false;
	std::string m_targetId;
	BackendVec3 m_localPos{};
	BackendVec3 m_localEuler{};
	bool m_solverPaused = false;
	bool m_hierarchyDriven = false;
};

#endif // DATA_FOLLOWATTACHMENTCOMPONENT_H
