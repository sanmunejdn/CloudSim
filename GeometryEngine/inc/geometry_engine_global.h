#pragma once

#if defined(GEOMETRY_ENGINE_STATIC) || !defined(GEOMETRY_ENGINE_EXPORTS)
#define GEOMETRY_ENGINE_API
#else
#if defined(_WIN32)
#ifdef GEOMETRY_ENGINE_EXPORTS
#define GEOMETRY_ENGINE_API __declspec(dllexport)
#else
#define GEOMETRY_ENGINE_API __declspec(dllimport)
#endif
#else
#define GEOMETRY_ENGINE_API __attribute__((visibility("default")))
#endif
#endif
