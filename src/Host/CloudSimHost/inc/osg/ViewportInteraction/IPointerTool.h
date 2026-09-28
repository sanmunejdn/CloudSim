#ifndef CLOUDSIMHOST_IPOINTERTOOL_H
#define CLOUDSIMHOST_IPOINTERTOOL_H

/// @file IPointerTool.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 指针工具：手势与命中，不直接改业务树

#include "ViewportHit.h"

class QObject;
class QEvent;

class IPointerTool
{
public:
	virtual ~IPointerTool() = default;
	virtual const char* toolId() const = 0;
	virtual void onActivated() {}
	virtual void onDeactivated() {}
	virtual bool handleEvent(QObject* watched, QEvent* event) = 0;
};

#endif // CLOUDSIMHOST_IPOINTERTOOL_H
