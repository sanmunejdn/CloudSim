#ifndef GEOMETRICMODELINGPLUGIN_BODYHISTORYCMD_H
#define GEOMETRICMODELINGPLUGIN_BODYHISTORYCMD_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CommandStack.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "PluginGeometryTypes.h"

#include <QByteArray>
#include <QString>
#include <functional>

class BodyHistoryCmd : public GeomodelingCommand
{
public:
	BodyHistoryCmd(IPluginHostContext* host, IPluginDocument* doc, QString bodyId, QByteArray before, QByteArray after,
				   std::function<void()> onApplied, bool alreadyApplied = false)
		: m_host(host)
		, m_doc(doc)
		, m_bodyId(std::move(bodyId))
		, m_before(std::move(before))
		, m_after(std::move(after))
		, m_onApplied(std::move(onApplied))
		, m_alreadyApplied(alreadyApplied)
	{
	}

	bool execute() override
	{
		if (m_alreadyApplied)
		{
			m_alreadyApplied = false;
			return true;
		}
		return apply(m_after);
	}
	bool undo() override { return apply(m_before); }
	std::string label() const override { return "ParametricBodyHistory"; }

private:
	bool apply(const QByteArray& hist)
	{
		if (!m_host || !m_doc || m_bodyId.isEmpty())
			return false;
		IPluginGeometryHost* geo = m_host->geometryHost();
		if (!geo)
			return false;
		bool ok = false;
		geo->setParametricBodyHistoryJson(
			m_doc, m_bodyId.toStdString(), hist,
			[&](bool success, const QString&, const PluginGeometryJobResult&)
			{
				ok = success;
				if (success && m_onApplied)
					m_onApplied();
			});
		return ok;
	}

	IPluginHostContext* m_host = nullptr;
	IPluginDocument* m_doc = nullptr;
	QString m_bodyId;
	QByteArray m_before;
	QByteArray m_after;
	std::function<void()> m_onApplied;
	bool m_alreadyApplied = false;
};

#endif
