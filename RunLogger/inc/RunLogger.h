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

RUN_LOGGER_API bool initialize(const std::string& logDirectory, const std::string& fileNameBase = "CloudSim");
RUN_LOGGER_API void shutdown();

RUN_LOGGER_API void setUiSink(UiSink sink);
RUN_LOGGER_API void clearUiSink();

/// Default minimum level is \c Info (no \c debug/\c trace). Set \c POINTCLOUD_PROCESS_DEBUG=1 to enable debug logs.
RUN_LOGGER_API void setMinimumLogLevel(LogLevel level);
RUN_LOGGER_API LogLevel minimumLogLevel();

/// Robot/teach/IK UI diagnostics (\c [示教] / \c [IK残差] / \c [指令显示] / matrix self-test success). Same env as debug logs.
RUN_LOGGER_API bool isDiagnosticsEnabled();

RUN_LOGGER_API void log(LogLevel level, const std::string& message);
RUN_LOGGER_API void trace(const std::string& message);
RUN_LOGGER_API void debug(const std::string& message);
RUN_LOGGER_API void info(const std::string& message);
RUN_LOGGER_API void warn(const std::string& message);
RUN_LOGGER_API void error(const std::string& message);
RUN_LOGGER_API void critical(const std::string& message);

/// Flush file/console sinks (debug/info are not flushed automatically; see \c flush_on in RunLogger.cpp).
RUN_LOGGER_API void flush();

RUN_LOGGER_API const char* levelName(LogLevel level);
} // namespace RunLogger
