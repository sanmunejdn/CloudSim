#ifndef WIDGET_WIDGETSCENESIGNALWIRING_H
#define WIDGET_WIDGETSCENESIGNALWIRING_H

/// @file WidgetSceneSignalWiring.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 文档页 OsgWidget Qt 信号 → MainWindow 槽（Widget 内唯一 OsgWidget 信号边界）

class DocumentPage;
class MainWindow;
class MainWindowRobotHost;

/// 文档页 OsgWidget Qt 信号 → MainWindow 槽（Widget 内唯一 OsgWidget 信号边界）
void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost);

#endif // WIDGET_WIDGETSCENESIGNALWIRING_H
