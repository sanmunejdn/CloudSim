#ifndef CLOUDSIMHOST_POLYLINEPICKOPERATION_H
#define CLOUDSIMHOST_POLYLINEPICKOPERATION_H

/// @file PolylinePickOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 多边形线框拾取：左键加点，右键/双击闭合，Esc 取消

#include "SelectionOperation.h"

#include <QPoint>
#include <vector>

/// 多边形线框拾取：左键加点，右键/双击闭合，Esc 取消
class PolylinePickOperation : public SelectionOperation
{
public:
	explicit PolylinePickOperation(IViewportInteractionHost* host);

private:
	std::vector<QPoint> m_vertices;
	QPoint m_cursorPos{-1, -1};
	bool m_hasCursor = false;

	void refreshOverlay() const;
	bool tryCommitPolygon();

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
	bool onMouseDoubleClick(QMouseEvent* e) override;
	bool onWheel(QWheelEvent* e) override;
};

#endif // CLOUDSIMHOST_POLYLINEPICKOPERATION_H
