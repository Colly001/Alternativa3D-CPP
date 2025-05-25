#include "shaderstreamwrite.h"
#include <QByteArray>
#include <QDebug>

ShaderStreamWrite::ShaderStreamWrite()
{
	_data = new QByteArray();
	_data->reserve(20);
	//qDebug("ShaderStreamWrite::ShaderStreamWrite()");
}

ShaderStreamWrite::~ShaderStreamWrite()
{
	delete _data;
}

void ShaderStreamWrite::writeUInt8(uint8_t v)
{
	//qDebug() << "writeUInt8" << QString::number(v, 16);
	_data->append((char*)&v, sizeof(uint8_t));
}

void ShaderStreamWrite::writeUInt16(uint16_t v)
{
	//qDebug() << "writeUInt16" << QString::number(v, 16);
	_data->append((char*)&v, sizeof(uint16_t));
}

void ShaderStreamWrite::writeUInt32(uint32_t v)
{
	//qDebug() << "writeUInt32" << QString::number(v, 16);
	_data->append((char*)&v, sizeof(uint32_t));
}

QByteArray ShaderStreamWrite::getData() const
{
	return *_data;
}
