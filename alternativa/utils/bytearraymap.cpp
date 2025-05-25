#include "bytearraymap.h"

ByteArrayMap::ByteArrayMap()
{

}

QHash<QString, QByteArray> ByteArrayMap::getData()
{
	return this->_data;
}

void ByteArrayMap::setData(QHash<QString, QByteArray> &data)
{
	this->_data = data;
}

QByteArray ByteArrayMap::getValue(const QString &key)
{
	return this->_data[key];
}

void ByteArrayMap::putValue(const QString &key, const QByteArray &value)
{
	this->_data[key] = value;
}

void ByteArrayMap::destroy()
{
	_data.clear();
}
