#include "texturebytedata.h"

TextureByteData::TextureByteData(const  QByteArray &diffuseData, const QByteArray &opacityData)
{
	this->diffuseData = diffuseData;
	this->opacityData = opacityData;
}
