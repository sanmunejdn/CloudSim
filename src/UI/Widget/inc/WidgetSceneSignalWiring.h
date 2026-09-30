#ifndef WIDGET_WIDGETSCENESIGNALWIRING_H
#define WIDGET_WIDGETSCENESIGNALWIRING_H

/// @file WidgetSceneSignalWiring.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口场景信号经 IOsgWidgetView / IViewportSceneSignals 接 MainWindow

class DocumentPage;
class MainWindow;
class MainWindowRobotHost;

/// 视口场景信号经 IOsgWidgetView / IViewportSceneSignals 接 MainWindow
void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost);

#endif // WIDGET_WIDGETSCENESIGNALWIRING_H
