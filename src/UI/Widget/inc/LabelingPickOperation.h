#pragma once

#include <QElapsedTimer>
#include <QPoint>
#include <QSet>
#include <QVector>

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "SelectionOperation.h"
#include "ViewportGestureRecognizer.h"

/// 分割标注拾取：单击或刷选点云/网格面
class LabelingPickOperation : public SelectionOperation
{
public:
	explicit LabelingPickOperation(OsgWidget* owner);

private:
	ViewportGestureRecognizer m_gesture;
	QElapsedTimer m_clickHoldTimer;
	QSet<int> m_brushAccumulated;

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
	bool onMouseDoubleClick(QMouseEvent* e) override;
	bool onWheel(QWheelEvent* e) override;

	void emitClickPick(const QPoint& pos);
	void emitBrushStroke(const QPoint& pos);
};
