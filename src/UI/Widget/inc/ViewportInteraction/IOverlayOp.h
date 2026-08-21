#ifndef WIDGET_IOVERLAYOP_H
#define WIDGET_IOVERLAYOP_H

/// @file IOverlayOp.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 叠加操作：罗盘/TCP/截面，不产业务 Hit

class QObject;
class QEvent;

class IOverlayOp
{
public:
	virtual ~IOverlayOp() = default;
	virtual const char* overlayId() const = 0;
	virtual bool handleEvent(QObject* watched, QEvent* event) = 0;
};

#endif // WIDGET_IOVERLAYOP_H
