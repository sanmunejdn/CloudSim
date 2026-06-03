#pragma once

#include "CoreTypes.h"
#include "cloudsim_core_global.h"

#include <functional>
#include <memory>

class QWidget;

namespace cloudsim::core {

using PickHandler = std::function<void(const ObjectId& backendId)>;

/// 文档渲染视口
class CLOUDSIM_CORE_EXPORT IRenderView
{
public:
	virtual ~IRenderView() = default;

	virtual QWidget* widget() = 0;
	virtual const QWidget* widget() const = 0;

	virtual void setWorldMatrix(const ObjectId& id, const Mat4& columnMajor) = 0;
	virtual bool getWorldMatrix(const ObjectId& id, Mat4& outColumnMajor) const = 0;

	virtual void setVisible(const ObjectId& id, bool visible) = 0;
	virtual void removeVisual(const ObjectId& id) = 0;
	virtual bool hasVisualBranch(const ObjectId& id) const = 0;

	virtual bool tryGetModelCenterMm(const ObjectId& id, double& outCx, double& outCy, double& outCz) const = 0;

	virtual void setPickHandler(PickHandler handler) = 0;
	virtual void clearPickHandler() = 0;

	virtual void requestRedraw() = 0;

	/// 聚焦后端子树
	virtual void focusCameraOnBackend(const ObjectId& id) = 0;
	/// 逻辑父链（不改 OSG）
	virtual void setBackendLogicalParent(const ObjectId& childId, const ObjectId& parentId) = 0;

	// 场景树快照（用于调试树视图）
	struct SceneNodeInfo
	{
		QString className;
		QString name;
		QString localMatrixSummary;
		std::vector<SceneNodeInfo> children;
	};
	virtual SceneNodeInfo sceneGraphSnapshot(int maxDepth = 8) const = 0;

	// 选中对象位姿查询
	virtual bool selectedPosition(float& outX, float& outY, float& outZ) const = 0;
	virtual bool selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const = 0;

	virtual void ensureSelectionVisualForBackend(const ObjectId& id, bool urdfLinkMesh = false) = 0;
	virtual bool syncOuterPatFromBackend(const ObjectId& id) = 0;
	virtual GeometryKind geometryKindForBackend(const ObjectId& id) const = 0;
	/// gizmo 松手：OSG 选中态写回后端位姿
	virtual bool commitGizmoPoseToBackend(const ObjectId& id) = 0;
};

/// 渲染视口工厂
class CLOUDSIM_CORE_EXPORT IRenderViewFactory
{
public:
	virtual ~IRenderViewFactory() = default;
	virtual std::unique_ptr<IRenderView> createView(QWidget* parent) = 0;
};

} // namespace cloudsim::core
