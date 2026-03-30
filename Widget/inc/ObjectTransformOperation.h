#pragma once

#include "SelectionOperation.h"

/// 对象变换模式：在选中对象且显示坐标轴时，处理左键平移、右键旋转等拖拽操作。
class ObjectTransformOperation : public SelectionOperation
{
public:
	explicit ObjectTransformOperation(OsgWidget* owner);
	bool handleEvent(QObject* watched, QEvent* event) override;
};

