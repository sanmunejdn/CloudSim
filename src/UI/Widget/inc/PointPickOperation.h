#pragma once

#include <QElapsedTimer>
#include <QPoint>
#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "SelectionOperation.h"
#include "ViewportGestureRecognizer.h"

/// 点云点选模式：悬停预览、点击拾取最近点并生成标注，左键拖动仍交给相机漫游。
class PointPickOperation : public SelectionOperation
{
public:
	explicit PointPickOperation(OsgWidget* owner);

private:
	ViewportGestureRecognizer m_gesture;
	QElapsedTimer m_clickHoldTimer;
	PickPreviewState m_preview;
	QPoint m_lastHoverPickPos{ -1000, -1000 };
	bool m_lastFeedbackHit = false;
	double m_lastFeedbackDistPx = -1.0;

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
	bool onMouseDoubleClick(QMouseEvent* e) override;
	bool onWheel(QWheelEvent* e) override;
};
