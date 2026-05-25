#pragma once

#include <functional>

#include <QString>

#include "aibackend_global.h"

/// 长耗时 AI 任务进度回调
using AiProgressSink = std::function<void(double fraction, const QString& message)>;
