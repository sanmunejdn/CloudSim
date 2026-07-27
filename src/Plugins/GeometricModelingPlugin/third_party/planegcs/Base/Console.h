#ifndef ONECAD_PLANEGCS_BASE_CONSOLE_H
#define ONECAD_PLANEGCS_BASE_CONSOLE_H

#include <chrono>
#include <cstdarg>
#include <cstdio>

namespace Base {

class ConsoleBackend {
public:
    void log(const char* format, ...) const {
        va_list args;
        va_start(args, format);
        std::vfprintf(stdout, format, args);
        va_end(args);
    }

    void warning(const char* format, ...) const {
        va_list args;
        va_start(args, format);
        std::vfprintf(stderr, format, args);
        va_end(args);
    }
};

inline ConsoleBackend& Console() {
    static ConsoleBackend instance;
    return instance;
}

class TimeElapsed {
public:
    TimeElapsed() : start_(std::chrono::steady_clock::now()) {}

    static double diffTimeF(const TimeElapsed& start, const TimeElapsed& end) {
        return std::chrono::duration<double>(end.start_ - start.start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

} // namespace Base

#endif  // ONECAD_PLANEGCS_BASE_CONSOLE_H
