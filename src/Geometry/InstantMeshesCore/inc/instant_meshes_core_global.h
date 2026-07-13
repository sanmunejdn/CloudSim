#pragma once

#ifdef INSTANT_MESHES_CORE_STATIC
#define INSTANT_MESHES_CORE_API
#elif defined(INSTANT_MESHES_CORE_LIB)
#define INSTANT_MESHES_CORE_API __declspec(dllexport)
#else
#define INSTANT_MESHES_CORE_API __declspec(dllimport)
#endif
