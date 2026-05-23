#pragma once

#if defined(CLOUDSIM_BOOTSTRAP_LIB)
# define BOOTSTRAP_EXPORT __declspec(dllexport)
#else
# define BOOTSTRAP_EXPORT __declspec(dllimport)
#endif
