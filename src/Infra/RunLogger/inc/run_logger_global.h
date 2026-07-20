#ifndef RUNLOGGER_RUN_LOGGER_GLOBAL_H
#define RUNLOGGER_RUN_LOGGER_GLOBAL_H

/// @file run_logger_global.h
/// @brief RunLogger 导出宏

#if defined(RUN_LOGGER_STATIC) || defined(BUILD_STATIC)
#define RUN_LOGGER_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(RUN_LOGGER_LIB)
#define RUN_LOGGER_API __declspec(dllexport)
#else
#define RUN_LOGGER_API __declspec(dllimport)
#endif
#else
#define RUN_LOGGER_API
#endif

#endif // RUNLOGGER_RUN_LOGGER_GLOBAL_H
