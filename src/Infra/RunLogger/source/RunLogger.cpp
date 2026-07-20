/// @file RunLogger.cpp
/// @brief RunLogger 实现

#include "RunLogger.h"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <vector>

#include <spdlog/logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace
{
std::mutex gMutex;
std::shared_ptr<spdlog::logger> gLogger;
RunLogger::UiSink gUiSink;
std::atomic<RunLogger::LogLevel> gMinLevel{RunLogger::LogLevel::Info};

bool envFlagEnabled(const char* name)
{
	auto valueEnabled = [](const char* e) -> bool
	{
		if (!e || e[0] == '\0')
		{
			return false;
		}
		if (std::strcmp(e, "0") == 0)
		{
			return false;
		}
		if ((e[0] == 'f' || e[0] == 'F') && (e[1] == 'a' || e[1] == 'A') && (e[2] == 'l' || e[2] == 'L') &&
			(e[3] == 's' || e[3] == 'S') && (e[4] == 'e' || e[4] == 'E') && e[5] == '\0')
		{
			return false;
		}
		return true;
	};

#if defined(_MSC_VER)
	char* buf = nullptr;
	size_t len = 0;
	if (_dupenv_s(&buf, &len, name) != 0 || !buf)
	{
		return false;
	}
	const bool enabled = valueEnabled(buf);
	std::free(buf);
	return enabled;
#else
	return valueEnabled(std::getenv(name));
#endif
}

spdlog::level::level_enum toSpdlogLevel(RunLogger::LogLevel level)
{
	switch (level)
	{
	case RunLogger::LogLevel::Trace:
		return spdlog::level::trace;
	case RunLogger::LogLevel::Debug:
		return spdlog::level::debug;
	case RunLogger::LogLevel::Info:
		return spdlog::level::info;
	case RunLogger::LogLevel::Warn:
		return spdlog::level::warn;
	case RunLogger::LogLevel::Error:
		return spdlog::level::err;
	case RunLogger::LogLevel::Critical:
		return spdlog::level::critical;
	default:
		return spdlog::level::info;
	}
}

void dispatchToUiSink(RunLogger::LogLevel level, const std::string& message)
{
	RunLogger::UiSink sinkCopy;
	{
		std::lock_guard<std::mutex> lock(gMutex);
		sinkCopy = gUiSink;
	}
	if (sinkCopy)
	{
		sinkCopy(level, message);
	}
}
} // namespace

namespace RunLogger
{
bool initialize(const std::string& logDirectory, const std::string& fileNameBase)
{
	try
	{
		const std::string safeBaseName = fileNameBase.empty() ? "CloudSim" : fileNameBase;
		const std::filesystem::path directory =
			logDirectory.empty() ? std::filesystem::current_path() / "logs" : std::filesystem::path(logDirectory);
		std::filesystem::create_directories(directory);
		const std::filesystem::path logFilePath = directory / (safeBaseName + ".log");

		std::vector<spdlog::sink_ptr> sinks;
		sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
		sinks.push_back(
			std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logFilePath.string(), 5 * 1024 * 1024, 3));

		auto logger = std::make_shared<spdlog::logger>("RunLogger", sinks.begin(), sinks.end());
		logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
		const RunLogger::LogLevel minLevel =
			envFlagEnabled("POINTCLOUD_PROCESS_DEBUG") ? RunLogger::LogLevel::Debug : RunLogger::LogLevel::Info;
		gMinLevel.store(minLevel);
		logger->set_level(toSpdlogLevel(minLevel));
		logger->flush_on(spdlog::level::warn);

		{
			std::lock_guard<std::mutex> lock(gMutex);
			gLogger = logger;
		}
		return true;
	}
	catch (...)
	{
		return false;
	}
}

void shutdown()
{
	std::lock_guard<std::mutex> lock(gMutex);
	if (gLogger)
	{
		gLogger->flush();
	}
	gLogger.reset();
	gUiSink = nullptr;
}

void setUiSink(UiSink sink)
{
	std::lock_guard<std::mutex> lock(gMutex);
	gUiSink = std::move(sink);
}

void clearUiSink()
{
	std::lock_guard<std::mutex> lock(gMutex);
	gUiSink = nullptr;
}

void setMinimumLogLevel(LogLevel level)
{
	gMinLevel.store(level);
	std::lock_guard<std::mutex> lock(gMutex);
	if (gLogger)
	{
		gLogger->set_level(toSpdlogLevel(level));
	}
}

LogLevel minimumLogLevel()
{
	return gMinLevel.load();
}

bool isDiagnosticsEnabled()
{
	return envFlagEnabled("POINTCLOUD_PROCESS_DEBUG");
}

void log(LogLevel level, const std::string& message)
{
	if (level < gMinLevel.load())
	{
		return;
	}
	std::shared_ptr<spdlog::logger> loggerCopy;
	{
		std::lock_guard<std::mutex> lock(gMutex);
		loggerCopy = gLogger;
	}
	if (loggerCopy)
	{
		loggerCopy->log(toSpdlogLevel(level), message);
	}
	dispatchToUiSink(level, message);
}

void trace(const std::string& message)
{
	log(LogLevel::Trace, message);
}

void debug(const std::string& message)
{
	log(LogLevel::Debug, message);
}

void info(const std::string& message)
{
	log(LogLevel::Info, message);
}

void warn(const std::string& message)
{
	log(LogLevel::Warn, message);
}

void error(const std::string& message)
{
	log(LogLevel::Error, message);
}

void critical(const std::string& message)
{
	log(LogLevel::Critical, message);
}

void flush()
{
	std::shared_ptr<spdlog::logger> loggerCopy;
	{
		std::lock_guard<std::mutex> lock(gMutex);
		loggerCopy = gLogger;
	}
	if (loggerCopy)
	{
		loggerCopy->flush();
	}
}

const char* levelName(LogLevel level)
{
	switch (level)
	{
	case LogLevel::Trace:
		return "TRACE";
	case LogLevel::Debug:
		return "DEBUG";
	case LogLevel::Info:
		return "INFO";
	case LogLevel::Warn:
		return "WARN";
	case LogLevel::Error:
		return "ERROR";
	case LogLevel::Critical:
		return "CRITICAL";
	default:
		return "INFO";
	}
}
} // namespace RunLogger
