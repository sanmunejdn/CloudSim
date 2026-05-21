#pragma once

#if defined(OSGWIDGETCORE_STATIC) || !defined(OSGWIDGETCORE_LIBRARY)
#define OSGWIDGETCORE_EXPORT
#else
#if defined(OSGWIDGETCORE_LIBRARY)
#define OSGWIDGETCORE_EXPORT __declspec(dllexport)
#else
#define OSGWIDGETCORE_EXPORT __declspec(dllimport)
#endif
#endif
