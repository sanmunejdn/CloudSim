#pragma once

#include <QElapsedTimer>

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "SelectionOperation.h"
#include "ViewportGestureRecognizer.h"

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
