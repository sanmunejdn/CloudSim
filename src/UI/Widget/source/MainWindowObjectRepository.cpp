#include "MainWindowObjectRepository.h"

#include "DocumentPage.h"
#include "IDataService.h"
#include "MainWindow.h"

std::optional<cloudsim::core::BackendObjectDto> MainWindowObjectRepository::findSnapshot(
	MainWindow& mainWindow, const QString& backendId)
{
	DocumentPage* page = mainWindow.currentPage();
	if (!page || backendId.isEmpty() || !page->data().isValid(backendId))
	{
		return std::nullopt;
	}
	const cloudsim::core::BackendObjectDto dto = page->data().objectSnapshot(backendId);
	if (dto.id.isEmpty())
	{
		return std::nullopt;
	}
	return dto;
}

QVector<cloudsim::core::BackendObjectDto> MainWindowObjectRepository::listSnapshots(MainWindow& mainWindow)
{
	DocumentPage* page = mainWindow.currentPage();
	if (!page)
	{
		return {};
	}
	return page->data().listObjectSnapshots();
}
