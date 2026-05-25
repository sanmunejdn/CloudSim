#pragma once

#include <array>
#include <string>
#include <vector>

#include "widget_global.h"

class BackendDataBase;
class BackendDataManager;
class BackendFollowReverseIndex;
class IBackendSceneBridge;
class IRobotBackendPoseSink;
class OsgWidget;

/// 后端 id 与场景/数据管理器轻量句柄
class OSG_WIDGET_API BackendSceneEntity
{
public:
	BackendSceneEntity() = default;
	BackendSceneEntity(std::string backendId, IBackendSceneBridge* bridge, BackendDataManager* mgr,
		BackendFollowReverseIndex* followIndex);

	const std::string& backendId() const { return m_id; }
	bool valid() const;

	void setVisible(bool visible);
	void show();
	void hide();

	bool setWorldMatrixColumnMajor(const std::array<double, 16>& columnMajor4x4);
	bool getWorldMatrixColumnMajor(std::array<double, 16>& outColumnMajor4x4) const;

	void removeVisual();
	bool hasSceneBranch() const;

	bool tryGetModelCenterMm(double& outCx, double& outCy, double& outCz) const;
	void syncOuterPatFromBackend(const BackendDataBase& data);
	void setParentBackend(const std::string& parentBackendId);

	std::vector<std::string> childBackendIds() const;
	std::vector<std::string> followerBackendIds() const;

private:
	std::string m_id;
	IBackendSceneBridge* m_bridge = nullptr;
	BackendDataManager* m_mgr = nullptr;
	BackendFollowReverseIndex* m_followIndex = nullptr;
};

/// 单文档门面：BackendDataManager + IBackendSceneBridge + 跟随反向索引
class OSG_WIDGET_API BackendSceneDocumentFacade
{
public:
	BackendSceneDocumentFacade() = default;
	BackendSceneDocumentFacade(BackendDataManager& mgr, IBackendSceneBridge& bridge,
		BackendFollowReverseIndex& followIndex, OsgWidget* osgWidget);

	BackendSceneEntity entity(const std::string& backendId) const;

	void setBackendsVisible(const std::vector<std::string>& backendIds, bool visible);

	IRobotBackendPoseSink* poseSink() const;
	/// 选中同步：加载缺失分支并 syncSelectionFromBackend
	void ensureSelectionVisualForBackend(const BackendDataBase& data, bool urdfLinkMesh = false);
	IBackendSceneBridge& bridge() const { return *m_bridge; }
	BackendDataManager& backendManager() const { return *m_mgr; }
	BackendFollowReverseIndex& followReverseIndex() const { return *m_followIndex; }

private:
	BackendDataManager* m_mgr = nullptr;
	IBackendSceneBridge* m_bridge = nullptr;
	BackendFollowReverseIndex* m_followIndex = nullptr;
	IRobotBackendPoseSink* m_poseSink = nullptr;
};
