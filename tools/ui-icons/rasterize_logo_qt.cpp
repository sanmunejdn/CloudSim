/// @file rasterize_logo_qt.cpp
/// @brief 将 cloudsim_logo.svg 栅格化为各尺寸 PNG（依赖 Qt5 Svg/Gui）

#include <QCoreApplication>
#include <QDir>
#include <QImage>
#include <QPainter>
#include <QSvgRenderer>
#include <cstdio>

int main(int argc, char* argv[])
{
	QCoreApplication app(argc, argv);
	if (argc < 3)
	{
		std::fprintf(stderr, "usage: %s <logo.svg> <out_dir>\n", argv[0]);
		return 2;
	}

	const QString svgPath = QString::fromLocal8Bit(argv[1]);
	const QString outRoot = QString::fromLocal8Bit(argv[2]);
	QSvgRenderer renderer(svgPath);
	if (!renderer.isValid())
	{
		std::fprintf(stderr, "invalid svg: %s\n", argv[1]);
		return 1;
	}

	const int sizes[] = {16, 24, 32, 48, 64, 128, 256};
	const char* themes[] = {"light", "dark"};
	for (const char* theme : themes)
	{
		const QString themeDir = outRoot + QLatin1Char('/') + QLatin1String(theme);
		QDir().mkpath(themeDir);
		for (int sz : sizes)
		{
			// 超采样再缩小，小尺寸抗锯齿更好
			const int hi = sz <= 48 ? sz * 4 : sz * 2;
			QImage hiImg(hi, hi, QImage::Format_ARGB32_Premultiplied);
			hiImg.fill(Qt::transparent);
			{
				QPainter p(&hiImg);
				p.setRenderHint(QPainter::Antialiasing, true);
				p.setRenderHint(QPainter::SmoothPixmapTransform, true);
				renderer.render(&p, QRectF(0, 0, hi, hi));
			}
			const QImage out = hiImg.scaled(sz, sz, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
			const QString path = themeDir + QStringLiteral("/cloudsim_logo_%1.png").arg(sz);
			if (!out.save(path, "PNG"))
			{
				std::fprintf(stderr, "save failed: %s\n", qPrintable(path));
				return 1;
			}
		}
	}
	std::printf("logo PNGs written under %s\n", argv[2]);
	return 0;
}
