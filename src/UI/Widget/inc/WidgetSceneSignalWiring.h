#ifndef WIDGET_WIDGETSCENESIGNALWIRING_H
#define WIDGET_WIDGETSCENESIGNALWIRING_H

/// @file WidgetSceneSignalWiring.h
/// @brief 文档页 OsgWidget Qt 信号 → MainWindow 槽（Widget 内唯一 OsgWidget 信号边界）

class DocumentPage;
class MainWindow;
class MainWindowRobotHost;

/// 文档页 OsgWidget Qt 信号 → MainWindow 槽（Widget 内唯一 OsgWidget 信号边界）
void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost);

#endif // WIDGET_WIDGETSCENESIGNALWIRING_H
