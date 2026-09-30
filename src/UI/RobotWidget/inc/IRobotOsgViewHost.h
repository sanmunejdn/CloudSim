#ifndef ROBOTWIDGET_IROBOTOSGVIEWHOST_H
#define ROBOTWIDGET_IROBOTOSGVIEWHOST_H

/// @file IRobotOsgViewHost.h

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com

/// @brief 机器人 UI 的 OSG/三维操作（四切面 MI 聚合，OsgWidget 实现）



#include "IRobotOsgOverlay.h"

#include "IRobotOsgPick.h"

#include "IRobotOsgSceneOps.h"

#include "IRobotOsgTeach.h"



struct PickResult;

enum class PickKind;



/// 机器人 UI 三维操作全量契约；新代码单切面依赖请用 IRobotOsgSceneOps 等

class ROBOTWIDGET_EXPORT IRobotOsgViewHost : public IRobotOsgSceneOps,

											 public IRobotOsgPick,

											 public IRobotOsgOverlay,

											 public IRobotOsgTeach

{

public:

	~IRobotOsgViewHost() override = default;

};

#endif // ROBOTWIDGET_IROBOTOSGVIEWHOST_H
