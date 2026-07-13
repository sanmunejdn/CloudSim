#pragma once

#include "robotwidget_global.h"

#include <FeatureListDocument.h>

#include <QAbstractTableModel>

#include <functional>
#include <vector>

class ROBOTWIDGET_EXPORT FeatureTableModel : public QAbstractTableModel
{
	Q_OBJECT

public:
	enum Column
	{
		ColIndex = 0,
		ColFeatureId,
		ColStrategy,
		ColGeometry,
		ColStatus,
		ColCount
	};

	explicit FeatureTableModel(QObject* parent = nullptr);

	void setUseChinese(bool chinese);
	void setEntries(const std::vector<geoalgo::FeatureEntry>& entries);
	const std::vector<geoalgo::FeatureEntry>& entries() const { return m_entries; }
	geoalgo::FeatureEntry entryAt(int row) const;
	bool updateEntry(int row, const geoalgo::FeatureEntry& entry);
	void setRowStatus(int row, const QString& status);
	QString rowStatus(int row) const;
	void appendEntry(const geoalgo::FeatureEntry& entry);
	void removeRows(const QList<int>& rows);
	void clearAll();
	int selectedRow() const { return m_selectedRow; }
	void setSelectedRow(int row);

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
	Qt::ItemFlags flags(const QModelIndex& index) const override;

	void setStrategyDisplayNameResolver(std::function<QString(const std::string& strategyId)> resolver);

signals:
	void selectionRowChanged(int row);

private:
	QString geometrySummary(const geoalgo::FeatureGeometry& geometry) const;
	QString strategyDisplayName(const std::string& strategyId) const;

	bool m_chinese = true;
	int m_selectedRow = -1;
	std::vector<geoalgo::FeatureEntry> m_entries;
	std::vector<QString> m_statusByRow;
	std::function<QString(const std::string& strategyId)> m_strategyDisplayResolver;
};
