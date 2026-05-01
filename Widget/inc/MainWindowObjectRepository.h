#pragma once

#include <QString>
#include <memory>
#include <vector>

class BackendDataBase;
class MainWindow;

/// MainWindow 侧对象仓储门面，逐步替代对 activeBackend() 的散落调用。
class MainWindowObjectRepository
{
public:
	static std::shared_ptr<BackendDataBase> findById(MainWindow& mainWindow, const QString& backendId);
	static std::vector<std::shared_ptr<BackendDataBase>> listAll(MainWindow& mainWindow);
};
