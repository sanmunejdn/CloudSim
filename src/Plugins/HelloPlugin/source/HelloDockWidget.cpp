#include "HelloDockWidget.h"

#include "IPluginHostContext.h"
#include "IPluginDocument.h"

#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QVBoxLayout>

HelloDockWidget::HelloDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(12, 12, 12, 12);
	layout->setSpacing(10);

	m_label = new QLabel(this);
	m_label->setWordWrap(true);
	m_label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	m_label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
	layout->addWidget(m_label);

	auto* refreshBtn = new QPushButton(tr("Refresh count"), this);
	layout->addWidget(refreshBtn);
	connect(refreshBtn, &QPushButton::clicked, this, &HelloDockWidget::refreshBackendCount);

	layout->addStretch(1);

	setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
	refreshBackendCount();
}

void HelloDockWidget::refreshBackendCount()
{
	std::size_t count = 0;
	if (m_host)
	{
		if (const IPluginDocument* doc = m_host->activeDocument())
		{
			count = doc->backendObjectCount();
		}
	}
	m_label->setText(tr("Active document backend objects: %1").arg(static_cast<qulonglong>(count)));
}
