#ifndef WIDGET_MESHEDGEFACEPICKOPERATION_H
#define WIDGET_MESHEDGEFACEPICKOPERATION_H

/// @file MeshEdgeFacePickOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 网格线面拾取模式：悬停高亮边或面，点击确认，支持中键辅助移动视图。

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "SelectionOperation.h"
#include "ViewportGestureRecognizer.h"

#include <QElapsedTimer>

/// 网格线面拾取模式：悬停高亮边或面，点击确认，支持中键辅助移动视图。
class MeshEdgeFacePickOperation final : public SelectionOperation
{
public:
	explicit MeshEdgeFacePickOperation(OsgWidget* owner);

private:
	ViewportGestureRecognizer m_gesture;
	QElapsedTimer m_clickHoldTimer;
	PickPreviewState m_preview;

	void applyPickResult(const PickResult& pick);
	void emitMeshFeedback(bool click, const PickResult& pick) const;
	PickQuery makePickQuery(const QPoint& pos) const;

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
};

#endif // WIDGET_MESHEDGEFACEPICKOPERATION_H
