#ifndef SHADERSTREAMWRITE_H
#define SHADERSTREAMWRITE_H

class QByteArray;

#include <stdint.h>

class ShaderStreamWrite
{
	QByteArray *_data;
public:
	ShaderStreamWrite();
	~ShaderStreamWrite();

	void writeUInt8(uint8_t v);
	void writeUInt16(uint16_t v);
	void writeUInt32(uint32_t v);

	QByteArray getData() const;
};

#endif // SHADERSTREAMWRITE_H
