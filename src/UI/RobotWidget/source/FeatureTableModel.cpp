#include "FeatureTableModel.h"

#include <QBrush>

#include <algorithm>

namespace
{

QString joinIndices(const std::vector<int>& indices, const QString& prefix)
{
	if (indices.empty())
	{
		return QString();
	}
	QStringList parts;
	parts.reserve(static_cast<int>(indices.size()));
	for (int idx : indices)
	{
		parts.push_back(QString::number(idx));
	}
	return prefix + parts.join(QStringLiteral(", "));
}

} // namespace

FeatureTableModel::FeatureTableModel(QObject* parent)
	: QAbstractTableModel(parent)
{
}

void FeatureTableModel::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
}

void FeatureTableModel::setStrategyDisplayNameResolver(
	std::function<QString(const std::string& strategyId)> resolver)
{
	m_strategyDisplayResolver = std::move(resolver);
}

void FeatureTableModel::setEntries(const std::vector<geoalgo::FeatureEntry>& entries)
{
	beginResetModel();
	m_entries = entries;
	m_statusByRow.assign(m_entries.size(), QString());
	m_selectedRow = m_entries.empty() ? -1 : 0;
	endResetModel();
	if (m_selectedRow >= 0)
	{
		emit selectionRowChanged(m_selectedRow);
	}
}

geoalgo::FeatureEntry FeatureTableModel::entryAt(const int row) const
{
	if (row < 0 || row >= static_cast<int>(m_entries.size()))
	{
		return {};
	}
	return m_entries[static_cast<std::size_t>(row)];
}

bool FeatureTableModel::updateEntry(const int row, const geoalgo::FeatureEntry& entry)
{
	if (row < 0 || row >= static_cast<int>(m_entries.size()))
	{
		return false;
	}
	m_entries[static_cast<std::size_t>(row)] = entry;
	const QModelIndex topLeft = index(row, 0);
	const QModelIndex bottomRight = index(row, ColCount - 1);
	emit dataChanged(topLeft, bottomRight);
	return true;
}

void FeatureTableModel::setRowStatus(const int row, const QString& status)
{
	if (row < 0 || row >= static_cast<int>(m_statusByRow.size()))
	{
		return;
	}
	m_statusByRow[static_cast<std::size_t>(row)] = status;
	const QModelIndex idx = index(row, ColStatus);
	emit dataChanged(idx, idx);
}

QString FeatureTableModel::rowStatus(const int row) const
{
	if (row < 0 || row >= static_cast<int>(m_statusByRow.size()))
	{
		return {};
	}
	return m_statusByRow[static_cast<std::size_t>(row)];
}

void FeatureTableModel::appendEntry(const geoalgo::FeatureEntry& entry)
{
	const int row = static_cast<int>(m_entries.size());
	beginInsertRows(QModelIndex(), row, row);
	m_entries.push_back(entry);
	m_statusByRow.push_back(QString());
	endInsertRows();
	setSelectedRow(row);
}

void FeatureTableModel::removeRows(const QList<int>& rows)
{
	if (rows.isEmpty())
	{
		return;
	}
	QList<int> sorted = rows;
	std::sort(sorted.begin(), sorted.end());
	sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
	for (int i = sorted.size() - 1; i >= 0; --i)
	{
		const int row = sorted[i];
		if (row < 0 || row >= static_cast<int>(m_entries.size()))
		{
			continue;
		}
		beginRemoveRows(QModelIndex(), row, row);
		m_entries.erase(m_entries.begin() + row);
		m_statusByRow.erase(m_statusByRow.begin() + row);
		endRemoveRows();
	}
	if (m_entries.empty())
	{
		m_selectedRow = -1;
		emit selectionRowChanged(-1);
	}
	else if (m_selectedRow >= static_cast<int>(m_entries.size()))
	{
		setSelectedRow(static_cast<int>(m_entries.size()) - 1);
	}
}

void FeatureTableModel::clearAll()
{
	beginResetModel();
	m_entries.clear();
	m_statusByRow.clear();
	m_selectedRow = -1;
	endResetModel();
	emit selectionRowChanged(-1);
}

void FeatureTableModel::setSelectedRow(const int row)
{
	if (row == m_selectedRow)
	{
		return;
	}
	m_selectedRow = row;
	emit selectionRowChanged(row);
}

int FeatureTableModel::rowCount(const QModelIndex& parent) const
{
	if (parent.isValid())
	{
		return 0;
	}
	return static_cast<int>(m_entries.size());
}

int FeatureTableModel::columnCount(const QModelIndex& parent) const
{
	if (parent.isValid())
	{
		return 0;
	}
	return ColCount;
}

QVariant FeatureTableModel::data(const QModelIndex& idx, const int role) const
{
	if (!idx.isValid() || idx.row() < 0 || idx.row() >= static_cast<int>(m_entries.size()))
	{
		return {};
	}
	const geoalgo::FeatureEntry& entry = m_entries[static_cast<std::size_t>(idx.row())];
	if (role == Qt::DisplayRole)
	{
		switch (idx.column())
		{
		case ColIndex:
			return idx.row() + 1;
		case ColFeatureId:
			return QString::fromStdString(entry.featureId);
		case ColStrategy:
			return strategyDisplayName(entry.strategyId);
		case ColGeometry:
			return geometrySummary(entry.geometry);
		case ColStatus:
			return m_statusByRow[static_cast<std::size_t>(idx.row())];
		default:
			break;
		}
	}
	if (role == Qt::ForegroundRole && idx.column() == ColStatus)
	{
		const QString status = m_statusByRow[static_cast<std::size_t>(idx.row())];
		if (status.contains(QStringLiteral("失败")) || status.contains(QStringLiteral("fail"), Qt::CaseInsensitive))
		{
			return QBrush(Qt::red);
		}
		if (status.contains(QStringLiteral("就绪")) || status.contains(QStringLiteral("ok"), Qt::CaseInsensitive))
		{
			return QBrush(Qt::darkGreen);
		}
	}
	return {};
}

QVariant FeatureTableModel::headerData(const int section, const Qt::Orientation orientation, const int role) const
{
	if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
	{
		return {};
	}
	if (m_chinese)
	{
		switch (section)
		{
		case ColIndex:
			return QStringLiteral("#");
		case ColFeatureId:
			return QStringLiteral("特征 ID");
		case ColStrategy:
			return QStringLiteral("离散策略");
		case ColGeometry:
			return QStringLiteral("几何摘要");
		case ColStatus:
			return QStringLiteral("状态");
		default:
			break;
		}
	}
	switch (section)
	{
	case ColIndex:
		return QStringLiteral("#");
	case ColFeatureId:
		return QStringLiteral("Feature ID");
	case ColStrategy:
		return QStringLiteral("Strategy");
	case ColGeometry:
		return QStringLiteral("Geometry");
	case ColStatus:
		return QStringLiteral("Status");
	default:
		break;
	}
	return {};
}

Qt::ItemFlags FeatureTableModel::flags(const QModelIndex& index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
}

QString FeatureTableModel::geometrySummary(const geoalgo::FeatureGeometry& geometry) const
{
	const QString edgeText = joinIndices(geometry.edgeIndices, m_chinese ? QStringLiteral("边 ") : QStringLiteral("edge "));
	const QString faceText = joinIndices(geometry.faceIndices, m_chinese ? QStringLiteral("面 ") : QStringLiteral("face "));
	if (!edgeText.isEmpty() && !faceText.isEmpty())
	{
		return edgeText + QStringLiteral("; ") + faceText;
	}
	if (!edgeText.isEmpty())
	{
		return edgeText;
	}
	if (!faceText.isEmpty())
	{
		return faceText;
	}
	if (!geometry.polylineXyz.empty())
	{
		const int ptCount = static_cast<int>(geometry.polylineXyz.size() / 3U);
		return m_chinese ? QStringLiteral("折线 %1 点").arg(ptCount)
						 : QStringLiteral("polyline %1 pts").arg(ptCount);
	}
	return m_chinese ? QStringLiteral("—") : QStringLiteral("—");
}

QString FeatureTableModel::strategyDisplayName(const std::string& strategyId) const
{
	if (m_strategyDisplayResolver)
	{
		return m_strategyDisplayResolver(strategyId);
	}
	return QString::fromStdString(strategyId);
}
