#ifndef RUNLOGGER_RUNLOGGER_H
#define RUNLOGGER_RUNLOGGER_H

/// @file RunLogger.h
/// @brief 默认 Info；POINTCLOUD_PROCESS_DEBUG=1 开 debug

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

/// 默认 Info；POINTCLOUD_PROCESS_DEBUG=1 开 debug
RUN_LOGGER_API void setMinimumLogLevel(LogLevel level);
RUN_LOGGER_API LogLevel minimumLogLevel();

/// 示教/IK/指令 UI 诊断；与 debug 同环境变量
RUN_LOGGER_API bool isDiagnosticsEnabled();

RUN_LOGGER_API void log(LogLevel level, const std::string& message);
RUN_LOGGER_API void trace(const std::string& message);
RUN_LOGGER_API void debug(const std::string& message);
RUN_LOGGER_API void info(const std::string& message);
RUN_LOGGER_API void warn(const std::string& message);
RUN_LOGGER_API void error(const std::string& message);
RUN_LOGGER_API void critical(const std::string& message);

/// 刷盘/控制台；debug/info 默认不自动 flush
RUN_LOGGER_API void flush();

RUN_LOGGER_API const char* levelName(LogLevel level);
} // namespace RunLogger

#endif // RUNLOGGER_RUNLOGGER_H
