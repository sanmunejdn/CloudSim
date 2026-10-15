/// @file ProgressDialogController.cpp
/// @brief 进度对话框控制器

#include "ProgressDialogController.h"

#include "JobSystem.h"
#include "MainWindow.h"
#include "ProgressDialog.h"
#include "ProgressUiText.h"
#include "RunInfoPage.h"

#include <QMetaObject>

ProgressDialogController::ProgressDialogController(MainWindow* mainWindow, QObject* parent)
	: QObject(parent), m_mainWindow(mainWindow)
{
}

ProgressDialogController::~ProgressDialogController()
{
	if (m_dialog)
	{
		m_dialog->close();
		m_dialog->deleteLater();
		m_dialog = nullptr;
	}
}

void ProgressDialogController::attachJobSystem(JobSystem* jobs)
{
	m_jobs = jobs;
}

void ProgressDialogController::setRunInfoPage(RunInfoPage* page)
{
	m_runInfo = page;
}

void ProgressDialogController::setChinese(bool useChinese)
{
	m_useChinese = useChinese;
	if (m_dialog)
	{
		m_dialog->setChinese(useChinese);
		if (!m_stack.isEmpty())
		{
			refreshDialog();
		}
	}
}

void ProgressDialogController::handleProgressEvent(const Progress::Event& ev)
{
	QMetaObject::invokeMethod(
		this, [this, ev]() { applyEventOnUi(ev); }, Qt::QueuedConnection);
}

void ProgressDialogController::applyEventOnUi(const Progress::Event& ev)
{
	const quint64 id = static_cast<quint64>(ev.id);
	switch (ev.kind)
	{
	case Progress::Kind::Started:
	{
		JobState st;
		st.title = QString::fromStdString(ev.title);
		st.cancellable = ev.cancellable;
		st.fraction = -1.0;
		m_jobsState.insert(id, st);
		m_stack.removeAll(id);
		m_stack.push_back(id);
		if (m_runInfo)
		{
			const QString tag = m_useChinese ? QStringLiteral("[进度]") : QStringLiteral("[Progress]");
			m_runInfo->appendInfo(QStringLiteral("%1 %2").arg(tag, ProgressUiText::localize(st.title, m_useChinese)));
		}
		refreshDialog();
		break;
	}
	case Progress::Kind::Update:
	{
		auto it = m_jobsState.find(id);
		if (it == m_jobsState.end())
		{
			break;
		}
		it->fraction = ev.fraction;
		it->message = QString::fromStdString(ev.message);
		refreshDialog();
		break;
	}
	case Progress::Kind::Finished:
	{
		const JobState st = m_jobsState.value(id);
		m_jobsState.remove(id);
		m_stack.removeAll(id);
		if (m_runInfo)
		{
			const QString tag = m_useChinese ? QStringLiteral("[进度]") : QStringLiteral("[Progress]");
			const QString title =
				ProgressUiText::localize(st.title.isEmpty() ? QString::number(id) : st.title, m_useChinese);
			if (!ev.success)
			{
				const QString err = QString::fromStdString(ev.error);
				const QString fail = ProgressUiText::localize(
					err.isEmpty() ? QStringLiteral("failed") : err, m_useChinese);
				m_runInfo->appendWarning(QStringLiteral("%1 %2 — %3").arg(tag, title, fail));
			}
			else
			{
				const QString done = m_useChinese ? QStringLiteral("完成") : QStringLiteral("done");
				m_runInfo->appendInfo(QStringLiteral("%1 %2 %3").arg(tag, title, done));
			}
		}
		refreshDialog();
		hideDialogIfIdle();
		break;
	}
	}
}

void ProgressDialogController::refreshDialog()
{
	if (m_stack.isEmpty() || !m_mainWindow)
	{
		return;
	}
	const quint64 id = m_stack.last();
	const JobState st = m_jobsState.value(id);
	if (!m_dialog)
	{
		m_dialog = new ProgressDialog(m_mainWindow);
		m_dialog->setChinese(m_useChinese);
		connect(m_dialog, &ProgressDialog::cancelRequested, this,
				[this]()
				{
					if (m_stack.isEmpty() || !m_jobs)
					{
						return;
					}
					const quint64 top = m_stack.last();
					if (m_jobsState.value(top).cancellable)
					{
						m_jobs->cancel(top);
					}
				});
	}
	m_dialog->setJobTitle(ProgressUiText::localize(st.title, m_useChinese));
	m_dialog->setStatusText(ProgressUiText::localize(st.message, m_useChinese));
	m_dialog->setFraction(st.fraction);
	m_dialog->setCancelEnabled(st.cancellable);
	m_dialog->setExtraJobsHint(qMax(0, m_stack.size() - 1));
	if (!m_dialog->isVisible())
	{
		m_dialog->show();
	}
}

void ProgressDialogController::hideDialogIfIdle()
{
	if (!m_stack.isEmpty() || !m_dialog)
	{
		return;
	}
	m_dialog->hide();
}
