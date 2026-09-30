#ifndef CLOUDSIMHOST_IVIEWPORTPICKOBSERVE_H
#define CLOUDSIMHOST_IVIEWPORTPICKOBSERVE_H

/// @file IViewportPickObserve.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口拾取模式与 observe*（IOsgWidgetView 切面；Qt 场景信号见 IViewportSceneSignals）

#include "PickTypes.h"

#include <QMetaObject>
#include <QObject>
#include <QString>
#include <QVector>
#include <functional>

#include <osg/Vec3d>

/// 拾取模式开关、平面求交与拾取提交观察
class IViewportPickObserve
{
public:
	virtual ~IViewportPickObserve() = default;

	virtual void setSelectionActive(bool active) = 0;
	virtual void setMeshLinePickMode(bool enabled) = 0;
	virtual void setMeshFacePickMode(bool enabled) = 0;
	virtual void setPolylinePickMode(bool enabled) = 0;
	virtual void setLabelingClickPickMode(bool enabled, bool meshFace) = 0;
	virtual void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx) = 0;

	virtual bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
											const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
											QString* outError = nullptr) const = 0;

	virtual QMetaObject::Connection observeMeshPickCommitted(QObject* ctx,
														   std::function<void(PickResult, int)> handler) = 0;
	virtual QMetaObject::Connection observePolylinePickCommitted(
		QObject* ctx, std::function<void(QVector<float>, QVector<double>, int, int)> handler) = 0;
	virtual QMetaObject::Connection observePolylinePickCanceled(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeLabelingClickCommitted(QObject* ctx,
																  std::function<void(PickResult)> handler) = 0;
	virtual QMetaObject::Connection observeLabelingBrushStroke(QObject* ctx,
															   std::function<void(QVector<int>)> handler) = 0;
	virtual QMetaObject::Connection observeLabelingBrushFinished(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeLabelingPickCanceled(QObject* ctx, std::function<void()> handler) = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTPICKOBSERVE_H
