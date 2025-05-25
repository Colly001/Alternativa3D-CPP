#include "bitmaputils.h"
#include "flash/display/bitmapdata.h"


BitmapData *BitmapUtils::mergeBitmapAlpha(BitmapData *bitmapData, BitmapData *alphaBitmapData, bool dispose)
{
	(void)bitmapData;
	(void)alphaBitmapData;
	(void)dispose;

	/*
	BitmapData *res = new BitmapData(bitmapData.width, bitmapData.height);
	res.copyPixels(bitmapData, bitmapData.rect, new Point());
	res.copyChannel(alphaBitmapData, alphaBitmapData.rect, new Point(), BitmapDataChannel.RED, BitmapDataChannel.ALPHA);
	if (dispose)
	{
		bitmapData.dispose();
		alphaBitmapData.dispose();
	}
	return res;
	*/
	return nullptr;
}

