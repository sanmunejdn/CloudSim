/// @file ProjectPackageZip.cpp
/// @brief 工程包 zip

#include "ProjectPackageZip.h"

#include <QByteArray>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QVector>

namespace project_package_zip
{
namespace
{
constexpr quint32 kLocalFileHeaderSig = 0x04034b50u;
constexpr quint32 kCentralFileHeaderSig = 0x02014b50u;
constexpr quint32 kEndOfCentralDirSig = 0x06054b50u;

quint32 crc32Update(quint32 crc, const QByteArray& data)
{
	static const quint32 table[256] = {
		0x00000000u, 0x77073096u, 0xee0e612cu, 0x990951bau, 0x076dc419u, 0x706af48fu, 0xe963a535u, 0x9e6495a3u,
		0x0edb8832u, 0x79dcb8a4u, 0xe0d5e91eu, 0x97d2d988u, 0x09b64c2bu, 0x7eb17cbdu, 0xe7b82d07u, 0x90bf1d91u,
		0x1db71064u, 0x6ab020f2u, 0xf3b97148u, 0x84be41deu, 0x1adad47du, 0x6ddde4ebu, 0xf4d4b551u, 0x83d385c7u,
		0x136c9856u, 0x646ba8c0u, 0xfd62f97au, 0x8a65c9ecu, 0x14015c4fu, 0x63066cd9u, 0xfa0f3d63u, 0x8d080df5u,
		0x3b6e20c8u, 0x4c69105eu, 0xd56041e4u, 0xa2677172u, 0x3c03e4d1u, 0x4b04d447u, 0xd20d85fdu, 0xa50ab56bu,
		0x35b5a8fau, 0x42b2986cu, 0xdbbbc9d6u, 0xacbcf940u, 0x32d86ce3u, 0x45df5c75u, 0xdcd60dcfu, 0xabd13d59u,
		0x26d930acu, 0x51de003au, 0xc8d75180u, 0xbfd06116u, 0x21b4f4b5u, 0x56b3c423u, 0xcfba9599u, 0xb8bda50fu,
		0x2802b89eu, 0x5f058808u, 0xc60cd9b2u, 0xb10be924u, 0x2f6f7c87u, 0x58684c11u, 0xc1611dabu, 0xb6662d3du,
		0x76dc4190u, 0x01db7106u, 0x98d220bcu, 0xefd5102au, 0x71b18589u, 0x06b6b51fu, 0x9fbfe4a5u, 0xe8b8d433u,
		0x7807c9a2u, 0x0f00f934u, 0x9609a88eu, 0xe10e9818u, 0x7f6a0dbbu, 0x086d3d2du, 0x91646c97u, 0xe6635c01u,
		0x6b6b51f4u, 0x1c6c6162u, 0x856530d8u, 0xf262004eu, 0x6c0695edu, 0x1b01a57bu, 0x8208f4c1u, 0xf50fc457u,
		0x65b0d9c6u, 0x12b7e950u, 0x8bbeb8eau, 0xfcb9887cu, 0x62dd1ddfu, 0x15da2d49u, 0x8cd37cf3u, 0xfbd44c65u,
		0x4db26158u, 0x3ab551ceu, 0xa3bc0074u, 0xd4bb30e2u, 0x4adfa541u, 0x3dd895d7u, 0xa4d1c46du, 0xd3d6f4fbu,
		0x4369e96au, 0x346ed9fcu, 0xad678846u, 0xda60b8d0u, 0x44042d73u, 0x33031de5u, 0xaa0a4c5fu, 0xdd0d7cc9u,
		0x5005713cu, 0x270241aau, 0xbe0b1010u, 0xc90c2086u, 0x5768b525u, 0x206f85b3u, 0xb966d409u, 0xce61e49fu,
		0x5edef90eu, 0x29d9c998u, 0xb0d09822u, 0xc7d7a8b4u, 0x59b33d17u, 0x2eb40d81u, 0xb7bd5c3bu, 0xc0ba6cadu,
		0xedb88320u, 0x9abfb3b6u, 0x03b6e20cu, 0x74b1d29au, 0xead54739u, 0x9dd277afu, 0x04db2615u, 0x73dc1683u,
		0xe3630b12u, 0x94643b84u, 0x0d6d6a3eu, 0x7a6a5aa8u, 0xe40ecf0bu, 0x9309ff9du, 0x0a00ae27u, 0x7d079eb1u,
		0xf00f9344u, 0x8708a3d2u, 0x1e01f268u, 0x6906c2feu, 0xf762575du, 0x806567cbu, 0x196c3671u, 0x6e6b06e7u,
		0xfed41b76u, 0x89d32be0u, 0x10da7a5au, 0x67dd4accu, 0xf9b9df6fu, 0x8ebeeff9u, 0x17b7be43u, 0x60b08ed5u,
		0xd6d6a3e8u, 0xa1d1937eu, 0x38d8c2c4u, 0x4fdff252u, 0xd1bb67f1u, 0xa6bc5767u, 0x3fb506ddu, 0x48b2364bu,
		0xd80d2bdau, 0xaf0a1b4cu, 0x36034af6u, 0x41047a60u, 0xdf60efc3u, 0xa867df55u, 0x316e8eefu, 0x4669be79u,
		0xcb61b38cu, 0xbc66831au, 0x256fd2a0u, 0x5268e236u, 0xcc0c7795u, 0xbb0b4703u, 0x220216b9u, 0x5505262fu,
		0xc5ba3bbeu, 0xb2bd0b28u, 0x2bb45a92u, 0x5cb36a04u, 0xc2d7ffa7u, 0xb5d0cf31u, 0x2cd99e8bu, 0x5bdeae1du,
		0x9b64c2b0u, 0xec63f226u, 0x756aa39cu, 0x026d930au, 0x9c0906a9u, 0xeb0e363fu, 0x72076785u, 0x05005713u,
		0x95bf4a82u, 0xe2b87a14u, 0x7bb12baeu, 0x0cb61b38u, 0x92d28e9bu, 0xe5d5be0du, 0x7cdcefb7u, 0x0bdbdf21u,
		0x86d3d2d4u, 0xf1d4e242u, 0x68ddb3f8u, 0x1fda836eu, 0x81be16cdu, 0xf6b9265bu, 0x6fb077e1u, 0x18b74777u,
		0x88085ae6u, 0xff0f6a70u, 0x66063bcau, 0x11010b5cu, 0x8f659effu, 0xf862ae69u, 0x616bffd3u, 0x166ccf45u,
		0xa00ae278u, 0xd70dd2eeu, 0x4e048354u, 0x3903b3c2u, 0xa7672661u, 0xd06016f7u, 0x4969474du, 0x3e6e77dbu,
		0xaed16a4au, 0xd9d65adcu, 0x40df0b66u, 0x37d83bf0u, 0xa9bcae53u, 0xdebb9ec5u, 0x47b2cf7fu, 0x30b5ffe9u,
		0xbdbdf21cu, 0xcabac28au, 0x53b39330u, 0x24b4a3a6u, 0xbad03605u, 0xcdd70693u, 0x54de5729u, 0x23d967bfu,
		0xb3667a2eu, 0xc4614ab8u, 0x5d681b02u, 0x2a6f2b94u, 0xb40bbe37u, 0xc30c8ea1u, 0x5a05df1bu, 0x2d02ef8du};
	crc = crc ^ 0xffffffffu;
	for (unsigned char b : data)
	{
		crc = table[(crc ^ static_cast<quint32>(b)) & 0xffu] ^ (crc >> 8);
	}
	return crc ^ 0xffffffffu;
}

void writeLe16(QIODevice* out, quint16 v)
{
	out->write(reinterpret_cast<const char*>(&v), 2);
}

void writeLe32(QIODevice* out, quint32 v)
{
	out->write(reinterpret_cast<const char*>(&v), 4);
}

bool readLe16(QIODevice* in, quint16* v)
{
	char buf[2];
	if (in->read(buf, 2) != 2)
	{
		return false;
	}
	*v = static_cast<quint16>(static_cast<uchar>(buf[0]) | (static_cast<quint16>(static_cast<uchar>(buf[1])) << 8));
	return true;
}

bool readLe32(QIODevice* in, quint32* v)
{
	char buf[4];
	if (in->read(buf, 4) != 4)
	{
		return false;
	}
	*v = static_cast<quint32>(static_cast<uchar>(buf[0])) | (static_cast<quint32>(static_cast<uchar>(buf[1])) << 8) |
		 (static_cast<quint32>(static_cast<uchar>(buf[2])) << 16) |
		 (static_cast<quint32>(static_cast<uchar>(buf[3])) << 24);
	return true;
}

struct ZipEntryOut
{
	QByteArray nameUtf8;
	QByteArray data;
	quint32 crc = 0;
	quint32 localHeaderOffset = 0;
};

} // namespace

bool isZipArchiveFile(const QString& filePath)
{
	QFile f(filePath);
	if (!f.open(QIODevice::ReadOnly))
	{
		return false;
	}
	const QByteArray sig = f.read(4);
	if (sig.size() != 4)
	{
		return false;
	}
	const auto* u = reinterpret_cast<const uchar*>(sig.constData());
	return u[0] == 0x50 && u[1] == 0x4b && u[2] == 0x03 && u[3] == 0x04;
}

bool zipDirectoryTree(const QString& zipFilePath, const QString& rootDir, QString* errorMessage)
{
	QDir root(rootDir);
	if (!root.exists())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Pack root does not exist.");
		}
		return false;
	}

	const QString rootAbs = QDir::cleanPath(QFileInfo(rootDir).absoluteFilePath()) + QLatin1Char('/');
	QVector<ZipEntryOut> entries;
	QDirIterator it(rootDir, QDir::Files, QDirIterator::Subdirectories);
	while (it.hasNext())
	{
		it.next();
		const QFileInfo fi = it.fileInfo();
		const QString abs = QDir::cleanPath(fi.absoluteFilePath());
		if (!abs.startsWith(rootAbs, Qt::CaseInsensitive))
		{
			continue;
		}
		QString rel = abs.mid(static_cast<int>(rootAbs.length()));
		rel.replace(QLatin1Char('\\'), QLatin1Char('/'));
		if (rel.isEmpty())
		{
			continue;
		}
		QFile rf(abs);
		if (!rf.open(QIODevice::ReadOnly))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Cannot read file: %1").arg(abs);
			}
			return false;
		}
		ZipEntryOut e;
		e.nameUtf8 = rel.toUtf8();
		e.data = rf.readAll();
		e.crc = crc32Update(0, e.data);
		entries.push_back(std::move(e));
	}

	QFile zf(zipFilePath);
	if (!zf.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Cannot create zip: %1").arg(zipFilePath);
		}
		return false;
	}

	const quint16 versionNeeded = 20; // 2.0 (UTF-8 name flag)
	const quint16 gpbfUtf8 = 0x0800;
	const quint16 methodStore = 0;
	const quint16 modTime = 0;
	const quint16 modDate = 0;

	for (ZipEntryOut& e : entries)
	{
		e.localHeaderOffset = static_cast<quint32>(zf.pos());
		writeLe32(&zf, kLocalFileHeaderSig);
		writeLe16(&zf, versionNeeded);
		writeLe16(&zf, gpbfUtf8);
		writeLe16(&zf, methodStore);
		writeLe16(&zf, modTime);
		writeLe16(&zf, modDate);
		writeLe32(&zf, e.crc);
		const quint32 sz = static_cast<quint32>(e.data.size());
		writeLe32(&zf, sz);
		writeLe32(&zf, sz);
		writeLe16(&zf, static_cast<quint16>(e.nameUtf8.size()));
		writeLe16(&zf, 0); // extra
		zf.write(e.nameUtf8.constData(), e.nameUtf8.size());
		zf.write(e.data.constData(), e.data.size());
	}

	const quint32 centralStart = static_cast<quint32>(zf.pos());
	for (const ZipEntryOut& e : entries)
	{
		writeLe32(&zf, kCentralFileHeaderSig);
		writeLe16(&zf, 0x0314); // made by: Unix, 2.0
		writeLe16(&zf, versionNeeded);
		writeLe16(&zf, gpbfUtf8);
		writeLe16(&zf, methodStore);
		writeLe16(&zf, modTime);
		writeLe16(&zf, modDate);
		writeLe32(&zf, e.crc);
		const quint32 sz = static_cast<quint32>(e.data.size());
		writeLe32(&zf, sz);
		writeLe32(&zf, sz);
		writeLe16(&zf, static_cast<quint16>(e.nameUtf8.size()));
		writeLe16(&zf, 0); // extra
		writeLe16(&zf, 0); // comment
		writeLe16(&zf, 0); // disk start
		writeLe16(&zf, 0); // int attrs
		writeLe32(&zf, 0); // ext attrs
		writeLe32(&zf, e.localHeaderOffset);
		zf.write(e.nameUtf8.constData(), e.nameUtf8.size());
	}
	const quint32 centralSize = static_cast<quint32>(zf.pos() - centralStart);

	writeLe32(&zf, kEndOfCentralDirSig);
	writeLe16(&zf, 0); // disk
	writeLe16(&zf, 0); // disk with CD
	writeLe16(&zf, static_cast<quint16>(entries.size()));
	writeLe16(&zf, static_cast<quint16>(entries.size()));
	writeLe32(&zf, centralSize);
	writeLe32(&zf, centralStart);
	writeLe16(&zf, 0); // comment len

	zf.close();
	return true;
}

bool extractZipArchive(const QString& zipFilePath, const QString& destDir, QString* errorMessage)
{
	QDir dest(destDir);
	if (!dest.exists() && !dest.mkpath(QStringLiteral(".")))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Cannot create extract directory.");
		}
		return false;
	}

	QFile zf(zipFilePath);
	if (!zf.open(QIODevice::ReadOnly))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Cannot open archive.");
		}
		return false;
	}
	const qint64 fileSize = zf.size();
	if (fileSize < 22)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Archive too small.");
		}
		return false;
	}

	qint64 eocdOff = -1;
	{
		const qint64 readSpan = (fileSize < 65557) ? fileSize : 65557;
		zf.seek(fileSize - readSpan);
		const QByteArray tail = zf.readAll();
		for (int i = tail.size() - 22; i >= 0; --i)
		{
			if (i + 4 > tail.size())
			{
				continue;
			}
			const uchar* p = reinterpret_cast<const uchar*>(tail.constData() + i);
			const quint32 sig = static_cast<quint32>(p[0]) | (static_cast<quint32>(p[1]) << 8) |
								(static_cast<quint32>(p[2]) << 16) | (static_cast<quint32>(p[3]) << 24);
			if (sig == kEndOfCentralDirSig)
			{
				eocdOff = fileSize - readSpan + i;
				break;
			}
		}
	}
	if (eocdOff < 0)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("End of central directory not found.");
		}
		return false;
	}

	zf.seek(eocdOff + 4);
	quint16 diskThis = 0;
	quint16 diskCd = 0;
	quint16 entriesThisDisk = 0;
	quint16 totalEntries = 0;
	quint32 centralSize = 0;
	quint32 centralOffset = 0;
	if (!readLe16(&zf, &diskThis) || !readLe16(&zf, &diskCd) || !readLe16(&zf, &entriesThisDisk) ||
		!readLe16(&zf, &totalEntries) || !readLe32(&zf, &centralSize) || !readLe32(&zf, &centralOffset))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Invalid end of central directory.");
		}
		return false;
	}
	Q_UNUSED(diskThis);
	Q_UNUSED(diskCd);
	Q_UNUSED(entriesThisDisk);

	zf.seek(centralOffset);
	for (int i = 0; i < totalEntries; ++i)
	{
		quint32 sig = 0;
		if (!readLe32(&zf, &sig) || sig != kCentralFileHeaderSig)
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Invalid central directory entry.");
			}
			return false;
		}
		quint16 verMade = 0;
		quint16 verExtract = 0;
		quint16 gpbf = 0;
		quint16 method = 0;
		quint16 modTime = 0;
		quint16 modDate = 0;
		if (!readLe16(&zf, &verMade) || !readLe16(&zf, &verExtract) || !readLe16(&zf, &gpbf) ||
			!readLe16(&zf, &method) || !readLe16(&zf, &modTime) || !readLe16(&zf, &modDate))
		{
			return false;
		}
		Q_UNUSED(verMade);
		Q_UNUSED(verExtract);
		Q_UNUSED(modTime);
		Q_UNUSED(modDate);
		quint32 crc = 0;
		quint32 csize = 0;
		quint32 usize = 0;
		if (!readLe32(&zf, &crc) || !readLe32(&zf, &csize) || !readLe32(&zf, &usize))
		{
			return false;
		}
		quint16 nameLen = 0;
		quint16 extraLen = 0;
		quint16 commentLen = 0;
		if (!readLe16(&zf, &nameLen) || !readLe16(&zf, &extraLen) || !readLe16(&zf, &commentLen))
		{
			return false;
		}
		quint16 diskStart = 0;
		quint16 intAttr = 0;
		quint32 extAttr = 0;
		quint32 localHdrOff = 0;
		if (!readLe16(&zf, &diskStart) || !readLe16(&zf, &intAttr) || !readLe32(&zf, &extAttr) ||
			!readLe32(&zf, &localHdrOff))
		{
			return false;
		}
		Q_UNUSED(diskStart);
		Q_UNUSED(intAttr);
		Q_UNUSED(extAttr);
		const QByteArray nameBytes = zf.read(nameLen);
		if (nameBytes.size() != nameLen)
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Truncated central file name.");
			}
			return false;
		}
		zf.seek(zf.pos() + extraLen + commentLen);

		if (method != 0)
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Unsupported compression (only STORE is supported).");
			}
			return false;
		}

		QString name = QString::fromUtf8(nameBytes);
		if (name.contains(QStringLiteral("..")))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Unsafe path in archive.");
			}
			return false;
		}

		const qint64 savePos = zf.pos();
		zf.seek(localHdrOff);
		quint32 lsig = 0;
		if (!readLe32(&zf, &lsig) || lsig != kLocalFileHeaderSig)
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Invalid local file header.");
			}
			return false;
		}
		zf.seek(localHdrOff + 26);
		quint16 lname = 0;
		quint16 lextra = 0;
		if (!readLe16(&zf, &lname) || !readLe16(&zf, &lextra))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Invalid local file header.");
			}
			return false;
		}
		zf.seek(localHdrOff + 30 + lname + lextra);
		const QByteArray payload = zf.read(static_cast<qint64>(csize));
		if (payload.size() != static_cast<int>(csize))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Truncated file data.");
			}
			return false;
		}
		if (crc32Update(0, payload) != crc)
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("CRC mismatch for %1.").arg(name);
			}
			return false;
		}
		zf.seek(savePos);

		const QString outPath = QDir(destDir).filePath(name);
		QFileInfo outFi(outPath);
		QDir().mkpath(outFi.absolutePath());
		QFile outF(outPath);
		if (!outF.open(QIODevice::WriteOnly | QIODevice::Truncate))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Cannot write: %1").arg(outPath);
			}
			return false;
		}
		outF.write(payload);
		outF.close();
		Q_UNUSED(gpbf);
		Q_UNUSED(usize);
	}

	return true;
}

} // namespace project_package_zip
