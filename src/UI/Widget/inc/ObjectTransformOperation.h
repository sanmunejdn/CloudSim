#ifndef WIDGET_OBJECTTRANSFORMOPERATION_H
#define WIDGET_OBJECTTRANSFORMOPERATION_H

/// @file ObjectTransformOperation.h
/// @brief 对象变换模式：在选中对象且显示坐标轴时：左键平移、右键旋转拖拽。

#include "SelectionOperation.h"

/// 对象变换模式：在选中对象且显示坐标轴时：左键平移、右键旋转拖拽。
class ObjectTransformOperation : public SelectionOperation
{
public:
	explicit ObjectTransformOperation(OsgWidget* owner);
	bool handleEvent(QObject* watched, QEvent* event) override;

private:
	void beginGizmoDragSession();
	void markGizmoSessionModified();

	/// 与 \ref OsgScene::kGizmoAxis* 一致；\c -1 表示尚未发出过悬停，避免重复 \c activeAxisChanged。
	int m_lastEmittedHoverAxis = -1;
	bool m_lastEmittedHoverRing = false;
	/// 本次按下到松开是否改过 outer 位姿（避免右键点按触发 commit/子树传播）
	bool m_gizmoSessionModified = false;
};

#endif // WIDGET_OBJECTTRANSFORMOPERATION_H
