#pragma once

#include "CoreTypes.h"

#include <optional>
#include <QVector>

class MainWindow;

/// 基于 IDataService 的对象查询（Widget 不持有 BackendDataBase）
class MainWindowObjectRepository
{
public:
	static std::optional<cloudsim::core::BackendObjectDto> findSnapshot(MainWindow& mainWindow, const QString& backendId);
	static QVector<cloudsim::core::BackendObjectDto> listSnapshots(MainWindow& mainWindow);
};
