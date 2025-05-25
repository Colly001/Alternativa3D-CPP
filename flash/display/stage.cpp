#include "stage.h"
#include "stage3d.h"
#include "imageitem.h"
#include "imagenodes.h"
#include <QDebug>


Stage::Stage()
{
	this->_width = 100;
	this->_height = 100;
	this->imageNodes = new ImageNodes();
	this->_stage3d = new Stage3D(this, this->imageNodes);
}

Stage::~Stage()
{
	delete _stage3d;
	delete this->imageNodes;
}

void Stage::setStageSize(int width, int height)
{
	this->_width = width;
	this->_height = height;
	this->_stage3d->setSize(width, height);
}

int Stage::stageWidth()
{
	return this->_width;
}

int Stage::stageHeight()
{
	return this->_height;
}

Stage3D* Stage::stage3D()
{
	return _stage3d;
}

ImageItem* Stage::createImage(BitmapData *data, int level)
{
	ImageItem *image = new ImageItem(data, level);
	image->setPosition(0, 0);
	IDirect3DDevice9Ex *device = this->_stage3d->getIDirect3DDevice();
	if (!image->createTexture(device))
	{
		qDebug("error createTexture");
	}
	image->upload();
	return image;
}


void Stage::addImage(ImageItem *node)
{
	imageNodes->add(node);
}

void Stage::removeImage(ImageItem *node)
{
	imageNodes->remove(node);
}
