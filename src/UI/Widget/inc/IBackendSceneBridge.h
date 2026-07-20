#ifndef WIDGET_IBACKENDSCENEBRIDGE_H
#define WIDGET_IBACKENDSCENEBRIDGE_H

/// @file IBackendSceneBridge.h
/// @brief Scene-side operations for one backend id (OSG branch), without coupling \c Data to OSG headers.

#include "widget_global.h"

#include <array>
#include <string>

class BackendDataBase;

/// Scene-side operations for one backend id (OSG branch), without coupling \c Data to OSG headers.
/// World matrices use **column-major** 16 doubles compatible with \c osg::Matrixd(ptr) layout.
class WIDGET_EXPORT IBackendSceneBridge
{
public:
	virtual ~IBackendSceneBridge() = default;

	virtual void setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
													  const std::array<double, 16>& columnMajor4x4) = 0;
	virtual bool getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
													  std::array<double, 16>& outColumnMajor4x4) const = 0;

	virtual void setBackendObjectVisible(const std::string& backendId, bool visible) = 0;
	virtual void removeBackendObjectVisual(const std::string& backendId) = 0;
	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;

	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
											double& outCz) const = 0;

	virtual void syncOuterPatFromBackend(const BackendDataBase& data) = 0;
	virtual void setBackendParent(const std::string& backendId, const std::string& parentBackendId) = 0;
};

#endif // WIDGET_IBACKENDSCENEBRIDGE_H
