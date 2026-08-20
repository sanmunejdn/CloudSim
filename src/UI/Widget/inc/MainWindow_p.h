#ifndef WIDGET_MAINWINDOW_P_H
#define WIDGET_MAINWINDOW_P_H

/// @file MainWindow_p.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 主窗口树形控件 ItemDataRole / 条目类型

#include <Qt>

namespace mainwindow_detail
{
constexpr int kRoleItemType = Qt::UserRole + 10;
constexpr int kRoleBackendId = Qt::UserRole + 11;
constexpr int kRoleAnnotationId = Qt::UserRole + 12;
constexpr int kRoleDocumentId = Qt::UserRole + 13;

constexpr int kItemTypeBackend = 1;
constexpr int kItemTypeAnnotation = 2;
constexpr int kItemTypeDocument = 3;
constexpr int kItemTypeAnnotationGroup = 4;

} // namespace mainwindow_detail

#endif // WIDGET_MAINWINDOW_P_H
