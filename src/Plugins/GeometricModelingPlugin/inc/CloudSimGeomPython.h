#ifndef GEOMETRICMODELINGPLUGIN_CLOUDSIMGEOMPYTHON_H
#define GEOMETRICMODELINGPLUGIN_CLOUDSIMGEOMPYTHON_H

/// @file CloudSimGeomPython.h
/// @brief 进程内 cloudsim_geom：转发 Host history / compose API

#include <string>
#include <vector>

class IPluginHostContext;
class QWidget;

namespace CloudSimGeomPython
{
void setHost(IPluginHostContext* host);
IPluginHostContext* host();

bool ensureReady(std::string* outError = nullptr);
bool registerModule(std::string* outError = nullptr);

std::string exportHistory(const std::string& bodyId, std::string* outError);
void importHistory(const std::string& jsonUtf8, const std::string& bodyId, std::string* outError);
std::string runCompose(const std::string& jsonUtf8, std::string* outError);
std::vector<std::string> listBodies(std::string* outError);

void openConsole(QWidget* parent);
} // namespace CloudSimGeomPython

#endif
