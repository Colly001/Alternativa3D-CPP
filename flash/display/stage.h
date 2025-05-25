#ifndef STAGE_H
#define STAGE_H

#include <stdint.h>

class Stage3D;
class ImageItem;
class ImageNodes;
class BitmapData;

class Stage
{
	int _width;
	int _height;
	Stage3D *_stage3d;
	ImageNodes *imageNodes;

public:
	Stage();
	~Stage();

	void setStageSize(int width, int height);

	int stageWidth();
	int stageHeight();

	Stage3D* stage3D();
	void addImage(ImageItem *node);
	void removeImage(ImageItem *node);

	ImageItem* createImage(BitmapData *data, int level);
};

#endif // STAGE_H
