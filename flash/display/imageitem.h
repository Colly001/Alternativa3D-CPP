#ifndef IMAGEITEM_H
#define IMAGEITEM_H

class BitmapData;
class IDirect3DTexture9;
class IDirect3DDevice9Ex;

class ImageItem
{
public:

	ImageItem *next;
	ImageItem *prev;

	BitmapData *data;

	IDirect3DTexture9 *texture;

	int width;
	int height;
	int x;
	int y;
	int level;

	ImageItem(BitmapData *data, int level);
	~ImageItem();

	void setX(int x)
	{
		this->x = x;
	}

	void setY(int y)
	{
		this->y = y;
	}

	void setPosition(int x, int y)
	{
		this->x = x;
		this->y = y;
	}

	bool createTexture(IDirect3DDevice9Ex *device);

	void upload();
};

#endif // IMAGEITEM_H
