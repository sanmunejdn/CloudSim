#ifndef INSTANTMESHESCORE_INSTANT_MESHES_CORE_GLOBAL_H
#define INSTANTMESHESCORE_INSTANT_MESHES_CORE_GLOBAL_H

/// @file instant_meshes_core_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief InstantMeshesCore 导出宏

#ifdef INSTANT_MESHES_CORE_STATIC
#define INSTANT_MESHES_CORE_API
#elif defined(INSTANT_MESHES_CORE_LIB)
#define INSTANT_MESHES_CORE_API __declspec(dllexport)
#else
#define INSTANT_MESHES_CORE_API __declspec(dllimport)
#endif

#endif // INSTANTMESHESCORE_INSTANT_MESHES_CORE_GLOBAL_H
