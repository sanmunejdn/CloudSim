#pragma once

#include <functional>

#include <QString>

#include "aibackend_global.h"

/// Progress callback for long-running AI backend work (e.g. LLM HTTP).
using AiProgressSink = std::function<void(double fraction, const QString& message)>;
