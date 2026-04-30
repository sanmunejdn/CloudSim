#include "MainWindowObjectRepository.h"

#include "BackendDataManager.h"
#include "DocumentPage.h"
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

MainWindowObjectGraph MainWindowObjectRepository::buildGraph(MainWindow& mainWindow)
{
	const std::vector<std::shared_ptr<BackendDataBase>> objects = listAll(mainWindow);
	DocumentPage* const page = mainWindow.currentPage();
	const QMap<QString, QString> parentById = page ? page->backendParentId() : QMap<QString, QString>();
	return MainWindowObjectGraph::build(objects, parentById);
}
