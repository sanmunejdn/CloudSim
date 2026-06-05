#pragma once

class DocumentPage;
class MainWindow;
class MainWindowRobotHost;

/// 文档页 OsgWidget Qt 信号 → MainWindow 槽（Widget 内唯一 OsgWidget 信号边界）
void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost);
