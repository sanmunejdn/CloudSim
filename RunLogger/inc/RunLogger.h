#pragma once

#include "run_logger_global.h"

#include <functional>
#include <string>

namespace RunLogger
{
enum class LogLevel
{
	Trace = 0,
	Debug,
	Info,
	Warn,
	Error,
	Critical
};

using UiSink = std::function<void(LogLevel level, const std::string& message)>;

RUN_LOGGER_API bool initialize(const std::string& logDirectory, const std::string& fileNameBase = "PointCloudProcess");
RUN_LOGGER_API void shutdown();

RUN_LOGGER_API void setUiSink(UiSink sink);
RUN_LOGGER_API void clearUiSink();

RUN_LOGGER_API void log(LogLevel level, const std::string& message);
RUN_LOGGER_API void trace(const std::string& message);
RUN_LOGGER_API void debug(const std::string& message);
RUN_LOGGER_API void info(const std::string& message);
RUN_LOGGER_API void warn(const std::string& message);
RUN_LOGGER_API void error(const std::string& message);
RUN_LOGGER_API void critical(const std::string& message);

RUN_LOGGER_API const char* levelName(LogLevel level);
} // namespace RunLogger
