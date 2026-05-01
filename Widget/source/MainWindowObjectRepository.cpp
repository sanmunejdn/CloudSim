#include "MainWindowObjectRepository.h"

#include "BackendDataManager.h"
#include "MainWindow.h"

std::shared_ptr<BackendDataBase> MainWindowObjectRepository::findById(
	MainWindow& mainWindow,
	const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return nullptr;
	}
	return mainWindow.activeBackend().getData(backendId.toStdString());
}

std::vector<std::shared_ptr<BackendDataBase>> MainWindowObjectRepository::listAll(MainWindow& mainWindow)
{
	return mainWindow.activeBackend().listData();
}
