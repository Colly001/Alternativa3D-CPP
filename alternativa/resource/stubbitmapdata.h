#ifndef STUBBITMAPDATA_H
#define STUBBITMAPDATA_H

#include "flash/display/bitmapdata.h"

class StubBitmapData : public BitmapData
{
public:
	StubBitmapData(uint32_t color);

private:
	void init(uint32_t color);
};

#endif // STUBBITMAPDATA_H
