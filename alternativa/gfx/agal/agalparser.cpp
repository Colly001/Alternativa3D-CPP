#include "agalparser.h"
#include <QByteArray>
#include <QDebug>

namespace
{
	class StreamRead
	{
		const uint8_t *data;
		const uint8_t *data_end;
		uint32_t length;

	public:

		StreamRead(const uint8_t *data, uint32_t length)
		{
			this->data = data;
			this->data_end = data + length;
			this->length = length;
		}

		uint8_t readUInt8()
		{
			if ((this->data + sizeof(uint8_t)) > this->data_end)
				throw 0;

			uint8_t value = *(uint8_t*)this->data;
			this->data += sizeof(uint8_t);
			return value;
		}

		uint16_t readUInt16()
		{
			if ((this->data + sizeof(uint16_t)) > this->data_end)
				throw 0;

			uint16_t value = *(uint16_t*)this->data;
			this->data += sizeof(uint16_t);
			return value;
		}

		uint32_t readUInt32()
		{
			if ((this->data + sizeof(uint32_t)) > this->data_end)
				throw 0;

			uint32_t value = *(uint32_t*)this->data;
			//value = _byteswap_ulong(value);
			this->data += sizeof(uint32_t);
			return value;
		}

		uint64_t readUInt64()
		{
			if ((this->data + sizeof(uint64_t)) > this->data_end)
				throw 0;

			uint64_t value = *(uint64_t*)this->data;
			this->data += sizeof(uint64_t);
			return value;
		}

		bool atEnd()
		{
			return this->data >= this->data_end;
		}
	};

	void SamplerReg_Parse(uint64_t v, AgalParser::ProgramType programType, AgalParser::SamplerReg *sr)
	{
		sr->programType = programType;
		sr->f = (int)((v >> 60) & 0xF); // filter
		sr->m = (int)((v >> 56) & 0xF); // mipmap
		sr->w = (int)((v >> 52) & 0xF); // wrap
		sr->s = (int)((v >> 48) & 0xF); // special
		sr->d = (int)((v >> 44) & 0xF); // dimension
		sr->type = (AgalParser::RegType)((v >> 32) & 0xF); // type
		sr->b = (int)((v >> 16) & 0xFF);
		sr->n = (int)(v & 0xFFFF);		// number
	}
}


AgalParser::AgalParser()
{

}

bool AgalParser::parser(const QByteArray &data)
{
	this->samplerReg = nullptr;

	StreamRead agal((const uint8_t*)data.constData(), data.size());
	try
	{
		int magic = agal.readUInt8();

		if (magic != 0xA0) {
			return false;
		}

		int version = agal.readUInt32();
		if (version != 1) {
			return false;
		}

		int shaderTypeId = agal.readUInt8();
		if (shaderTypeId != 0xA1) {
			return false;
		}

		SamplerReg *samplerReg_end = nullptr;

		ProgramType programType = (agal.readUInt8 () == 0) ? ProgramType::Vertex : ProgramType::Fragment;
		//qDebug("AgalParser::parser");

		while (!agal.atEnd())
		{
			int opcode = agal.readUInt32();
			uint32_t dest = agal.readUInt32();
			uint64_t source1 = agal.readUInt64();
			uint64_t source2 = agal.readUInt64();


			(void)dest;
			(void)source1;

			switch (opcode)
			{
				case 0x28: // tex
				{
					//qDebug("parser tex %016llX", source2);
					SamplerReg *sampler = new SamplerReg;
					SamplerReg_Parse(source2, programType, sampler);
					//qDebug() << (int)sampler->programType << sampler->f << sampler->m << sampler->w << sampler->s << sampler->d << (int)sampler->type << sampler->b << sampler->n;


					sampler->next = nullptr;

					if (this->samplerReg == nullptr)
						this->samplerReg = sampler;
					else
						samplerReg_end->next = sampler;

					samplerReg_end = sampler;
					break;
				}
			}
		}

		return true;
	}
	catch (int)
	{
		return false;
	}
}

