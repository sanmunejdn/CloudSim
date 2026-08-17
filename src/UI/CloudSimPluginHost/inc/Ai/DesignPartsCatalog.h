#ifndef CLOUDSIMPLUGINHOST_DESIGNPARTSCATALOG_H
#define CLOUDSIMPLUGINHOST_DESIGNPARTSCATALOG_H

/// @file DesignPartsCatalog.h
/// @brief 标准件库：加载 part.json、填模 instantiate → feature.compose

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <QVector>

#include <optional>

struct DesignPartSpec
{
	QString thread;
	double dMm = 0;
	double sMm = 0;
	double kMm = 0;
	double mMm = 0;
	double dInnerMm = 0;
	double dOuterMm = 0;
	double thicknessMm = 0;
	int Z = 0;
	double bMm = 0;
	double boreMm = 0;
	QVector<double> lengthSeriesMm;
};

struct DesignPartInfo
{
	QString id;
	QString displayName;
	QStringList keywords;
	QString modelRef;
	QString modelFidelity;
	QString dirPath;
	QByteArray partJsonUtf8;
	QByteArray modelComposeUtf8;
	QVector<DesignPartSpec> specs;
	// defaults as JSON object utf8
	QByteArray defaultsJsonUtf8;
};

class DesignPartsCatalog
{
public:
	static QString resolvePartsRoot();
	bool load(const QString& partsRoot, QString* err = nullptr);
	const QVector<DesignPartInfo>& parts() const { return m_parts; }
	const DesignPartInfo* findById(const QString& partId) const;
	QVector<const DesignPartInfo*> search(const QString& query) const;

	/// 口语 → feature.compose；失败时 outErr/hint
	bool tryParseUserText(const QString& text, QByteArray* outPlanUtf8, QString* partId, QString* hint,
						  QString* err) const;

	bool instantiate(const QString& partId, const QByteArray& paramsJsonUtf8, QByteArray* outPlanUtf8,
					 QString* err) const;

private:
	QVector<DesignPartInfo> m_parts;
	QString m_root;
};

#endif
