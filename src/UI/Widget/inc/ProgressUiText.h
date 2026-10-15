#ifndef WIDGET_PROGRESSUITEXT_H
#define WIDGET_PROGRESSUITEXT_H

/// @file ProgressUiText.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 进度条标题/状态英文键 → 中英显示（与软件语言同步）

#include "widget_global.h"

#include <QString>

namespace ProgressUiText
{
/// 将任务标题或状态文案转为当前 UI 语言；无法识别的原文原样返回
WIDGET_EXPORT QString localize(const QString& text, bool useChinese);
} // namespace ProgressUiText

#endif // WIDGET_PROGRESSUITEXT_H
