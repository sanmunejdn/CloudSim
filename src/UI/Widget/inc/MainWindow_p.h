#ifndef WIDGET_MAINWINDOW_P_H
#define WIDGET_MAINWINDOW_P_H

/// @file MainWindow_p.h
/// @brief 主窗口树形控件等用到的 Qt::ItemDataRole 常量与条目类型枚举（内部实现细节）。

#include <Qt>

/// 主窗口树形控件等用到的 Qt::ItemDataRole 常量与条目类型枚举（内部实现细节）。
namespace mainwindow_detail
{
constexpr int kRoleItemType = Qt::UserRole + 10;
constexpr int kRoleBackendId = Qt::UserRole + 11;
constexpr int kRoleAnnotationId = Qt::UserRole + 12;
constexpr int kItemTypeBackend = 1;
constexpr int kItemTypeAnnotation = 2;

} // namespace mainwindow_detail

#endif // WIDGET_MAINWINDOW_P_H
