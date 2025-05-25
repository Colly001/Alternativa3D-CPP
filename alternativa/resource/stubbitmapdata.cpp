#include "stubbitmapdata.h"

static const int SIZE = 20;

StubBitmapData::StubBitmapData(uint32_t color) :
	BitmapData(SIZE, SIZE, false, 0)
{
	this->init(color);
}

void StubBitmapData::init(uint32_t color)
{
	int i = 0;
	while (i < SIZE)
	{
		int j = 0;
		while (j < SIZE)
		{
			setPixel((i % 2) ? j : (j + 1), i, color);
			j = j + 2;
		}
		i++;
	}
}
