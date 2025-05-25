#ifndef BITMAPUTILS_H
#define BITMAPUTILS_H

class BitmapData;

class BitmapUtils
{
public:
	static BitmapData* mergeBitmapAlpha(BitmapData *bitmapData, BitmapData *alphaBitmapData, bool dispose = false);
};

#endif // BITMAPUTILS_H
