#pragma once

#if defined(BACKENDVISUAL_STATIC)
#define BACKENDVISUAL_EXPORT
#else
#if defined(_WIN32)
#if defined(BACKENDVISUAL_LIBRARY)
#define BACKENDVISUAL_EXPORT __declspec(dllexport)
#else
#define BACKENDVISUAL_EXPORT __declspec(dllimport)
#endif
#else
#define BACKENDVISUAL_EXPORT
#endif
#endif
