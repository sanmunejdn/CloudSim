#ifndef AIWIDGET_AICONFIRMPANEL_H
#define AIWIDGET_AICONFIRMPANEL_H

/// @file AiConfirmPanel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Dock 内嵌：按 args_schema 动态表单确认 Agent tool 参数

#include "aiwidget_global.h"

#include <QByteArray>
#include <QString>
#include <QVector>
#include <QWidget>

class QFormLayout;
class QLabel;
class QPushButton;
class QVBoxLayout;

class AIWIDGET_EXPORT AiConfirmPanel : public QWidget
{
	Q_OBJECT

public:
	explicit AiConfirmPanel(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void hidePanel();
	/// confirmLabel / secondaryLabel 空则用默认「确认执行」/隐藏次要按钮
	void showToolConfirm(const QString& pendingId, const QString& title, const QString& risk,
						 const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
						 const QByteArray& sceneSnapshotJson, const QString& confirmLabel = QString(),
						 const QString& secondaryLabel = QString());

signals:
	void accepted(const QString& pendingId, const QByteArray& argsJsonUtf8);
	void rejected(const QString& pendingId);
	void secondaryClicked(const QString& pendingId);

private:
	void clearForm();
	void rebuildForm(const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
					 const QByteArray& sceneSnapshotJson);
	QByteArray collectArgsJson() const;
	void updateConfirmEnabled();

	bool m_useChinese = true;
	QString m_pendingId;
	QLabel* m_titleLabel = nullptr;
	QLabel* m_riskLabel = nullptr;
	QWidget* m_formHost = nullptr;
	QFormLayout* m_form = nullptr;
	QPushButton* m_confirmBtn = nullptr;
	QPushButton* m_cancelBtn = nullptr;
	QPushButton* m_secondaryBtn = nullptr;
	QString m_defaultConfirmText;
	QString m_defaultCancelText;
	struct FieldBind
	{
		QString name;
		QString type;
		bool required = false;
		QWidget* editor = nullptr;
		QWidget* editor2 = nullptr; // backend_pair 目标
	};
	QVector<FieldBind> m_fields;
};

#endif
