#ifndef WIDGET_IINTERACTIONSESSION_H
#define WIDGET_IINTERACTIONSESSION_H

/// @file IInteractionSession.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 交互会话：业务消费 Hover/Commit/Cancel

#include "ViewportHit.h"

class IInteractionSession
{
public:
	virtual ~IInteractionSession() = default;
	virtual void onHover(const ViewportHit& hit) { (void)hit; }
	virtual void onCommit(const ViewportHit& hit) = 0;
	virtual void onCancel() {}
};

#endif // WIDGET_IINTERACTIONSESSION_H
