#ifndef CLOUDSIMHOST_POINTPICKOPERATION_H
#define CLOUDSIMHOST_POINTPICKOPERATION_H

/// @file PointPickOperation.h
/// @brief 点云点选模式：悬停预览、点击拾取最近点并生成标注，左键拖动仍交给相机漫游。

#include "SelectionOperation.h"

#include <QPoint>

/// 点云点选模式：悬停预览、点击拾取最近点并生成标注，左键拖动仍交给相机漫游。
class PointPickOperation : public SelectionOperation
{
public:
	explicit PointPickOperation(OsgWidget* owner);

private:
	bool m_leftPressed = false;
	QPoint m_pressPos;
	bool m_dragMoved = false;

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
	bool onMouseDoubleClick(QMouseEvent* e) override;
	bool onWheel(QWheelEvent* e) override;
};

#endif // CLOUDSIMHOST_POINTPICKOPERATION_H
