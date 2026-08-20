#ifndef WIDGET_LABELINGPICKOPERATION_H
#define WIDGET_LABELINGPICKOPERATION_H

/// @file LabelingPickOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 分割标注拾取：单击或刷选点云/网格面

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "SelectionOperation.h"
#include "ViewportGestureRecognizer.h"

#include <QElapsedTimer>
#include <QPoint>
#include <QSet>
#include <QVector>

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

#endif // WIDGET_LABELINGPICKOPERATION_H
