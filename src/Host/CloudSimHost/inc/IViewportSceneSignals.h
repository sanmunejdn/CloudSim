#ifndef CLOUDSIMHOST_IVIEWPORTSCENESIGNALS_H
#define CLOUDSIMHOST_IVIEWPORTSCENESIGNALS_H

/// @file IViewportSceneSignals.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口场景 Qt 信号窄订阅（实现于 OsgWidget，不进入 IOsgWidgetView）

#include "PickTypes.h"

#include <QMetaObject>
#include <QObject>
#include <QString>
#include <QVector>
#include <functional>

/// Widget 层接线用：把 OsgWidget 的 Qt 信号与 IOsgWidgetView::observe* 分离
class IViewportSceneSignals
{
public:
	virtual ~IViewportSceneSignals() = default;

	/// 交互会话优先消费 mesh 拾取提交
	virtual bool dispatchMeshPickCommitToInteractionSession(const PickResult& pick, int pickKindInt) = 0;

	virtual QMetaObject::Connection observeSelectedObjectPoseChanged(QObject* ctx,
																	 std::function<void(float, float, float)> handler) = 0;
	virtual QMetaObject::Connection observeSelectedObjectRotationChanged(
		QObject* ctx, std::function<void(float, float, float)> handler) = 0;
	virtual QMetaObject::Connection observeSelectedObjectColorChanged(
		QObject* ctx, std::function<void(float, float, float, float)> handler) = 0;
	virtual QMetaObject::Connection observeTransformGizmoCommitted(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeTcpDragTeachPoseChanged(
		QObject* ctx, std::function<void(double, double, double, double, double, double)> handler) = 0;
	virtual QMetaObject::Connection observeTcpDragTeachEnded(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeActiveAxisChanged(QObject* ctx,
															 std::function<void(const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeSelectionCanceledByEsc(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeAnnotationCreated(
		QObject* ctx, std::function<void(const QString&, const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeAnnotationRemoved(QObject* ctx,
														   std::function<void(const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeAnnotationVisibilityChanged(
		QObject* ctx, std::function<void(const QString&, bool)> handler) = 0;
	virtual QMetaObject::Connection observePointPickFeedback(QObject* ctx,
														   std::function<void(const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeMeshPickFeedback(QObject* ctx,
														  std::function<void(const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeBackendObjectPicked(QObject* ctx,
															   std::function<void(const QString&)> handler) = 0;
	virtual QMetaObject::Connection observeInstructionWaypointPicked(
		QObject* ctx, std::function<void(const QString&, bool)> handler) = 0;
	virtual QMetaObject::Connection observeInstructionWaypointPickCanceled(QObject* ctx,
																		   std::function<void()> handler) = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTSCENESIGNALS_H
