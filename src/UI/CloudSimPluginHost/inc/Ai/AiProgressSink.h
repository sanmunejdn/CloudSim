#ifndef CLOUDSIMPLUGINHOST_AIPROGRESSSINK_H
#define CLOUDSIMPLUGINHOST_AIPROGRESSSINK_H

/// @file AiProgressSink.h
/// @brief 长耗时 AI 任务进度回调

#include "aibackend_global.h"

#include <QString>
#include <functional>

/// 长耗时 AI 任务进度回调
using AiProgressSink = std::function<void(double fraction, const QString& message)>;

#endif // CLOUDSIMPLUGINHOST_AIPROGRESSSINK_H
