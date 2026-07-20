#ifndef WIDGET_MAINWINDOWOBJECTREPOSITORY_H
#define WIDGET_MAINWINDOWOBJECTREPOSITORY_H

/// @file MainWindowObjectRepository.h
/// @brief 基于 IDataService 的对象查询（Widget 不持有 BackendDataBase）

#include "CoreTypes.h"

#include <QVector>
#include <optional>

class MainWindow;

/// 基于 IDataService 的对象查询（Widget 不持有 BackendDataBase）
class MainWindowObjectRepository
{
public:
	static std::optional<cloudsim::core::BackendObjectDto> findSnapshot(MainWindow& mainWindow,
																		const QString& backendId);
	static QVector<cloudsim::core::BackendObjectDto> listSnapshots(MainWindow& mainWindow);
};

#endif // WIDGET_MAINWINDOWOBJECTREPOSITORY_H
