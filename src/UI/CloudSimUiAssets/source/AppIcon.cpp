#include "AppIcon.h"

#include <QIcon>
#include <QSettings>
#include <QString>

static void initLogoResources()
{
	Q_INIT_RESOURCE(cloudsim_logo);
}

namespace {

bool g_logoResourcesInitialized = false;

QString themeFolder()
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	const QString value = settings.value(QStringLiteral("theme"), QStringLiteral("light")).toString();
	return value.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0
		? QStringLiteral("dark") : QStringLiteral("light");
}

} // namespace

namespace AppIcon {

QIcon logo()
{
	if (!g_logoResourcesInitialized)
	{
		initLogoResources();
		g_logoResourcesInitialized = true;
	}

	static QIcon cached;
	if (!cached.isNull())
	{
		return cached;
	}

	const QString theme = themeFolder();
	const int sizes[] = {16, 24, 32, 48, 64, 128, 256};
	for (int sz : sizes)
	{
		const QString path = QStringLiteral(":/cloudsim/logo/%1/cloudsim_logo_%2.png")
			.arg(theme).arg(sz);
		cached.addFile(path, QSize(sz, sz));
	}
	return cached;
}

} // namespace AppIcon
