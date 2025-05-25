#include "taraparser.h"
#include "bytearraymap.h"
#include <QByteArray>
#include <QDebug>
#include "hash.h"
#include <QDebug>

namespace
{
	class FileInfo
	{
	public:
		QString name;
		int size;
	};


	class Stream
	{
		const uint8_t *data;
		const uint8_t *data_end;
		uint32_t length;

	public:

		Stream(const uint8_t *data, uint32_t length)
		{
			this->data = data;
			this->data_end = data + length;
			this->length = length;
		}

		uint16_t readUInt16()
		{
			if ((this->data + sizeof(uint16_t)) > this->data_end)
				throw 14131;

			uint16_t value = *(uint16_t*)this->data;
			value = _byteswap_ushort(value);
			this->data += sizeof(uint16_t);
			return value;
		}

		int32_t readInt32()
		{
			if ((this->data + sizeof(int32_t)) > this->data_end)
				throw 14132;

			int32_t value = *(int32_t*)this->data;
			value = _byteswap_ulong(value);
			this->data += sizeof(int32_t);
			return value;
		}

		QString readUtf8()
		{
			uint16_t length = this->readUInt16();
			if (length == 0)
				throw 14133;

			if ((this->data + length) > this->data_end)
				throw 14134;

			QString str = QString::fromUtf8((const char*)this->data, length);
			this->data += length;
			return str;
		}

		QByteArray readData(uint32_t length)
		{
			if ((this->data + length) > this->data_end)
				throw 14135;

			QByteArray _data((char*)this->data, length);
			this->data += length;
			return _data;
		}
	};
}

std::shared_ptr<ByteArrayMap> TARAParse(const QByteArray &data)
{
	try
	{
		Stream stream((const uint8_t*)data.constData(), data.size());

		int numFiles = stream.readInt32();

		QVector<FileInfo> files;
		files.resize(numFiles);

		for (int i = 0; i < numFiles; i++)
		{
			FileInfo &item = files[i];
			item.name = stream.readUtf8();
			item.size = stream.readInt32();
		}

		std::shared_ptr<ByteArrayMap> table = std::make_shared<ByteArrayMap>();

		for (int i = 0; i < numFiles; i++)
		{
			FileInfo &fileInfo = files[i];
			QByteArray fileData = stream.readData(fileInfo.size);
			table->putValue(fileInfo.name, fileData);
		}

		return table;
	}
	catch (int e)
	{
		qDebug("error TARAParser %d", e);
	}

	return nullptr;
}
