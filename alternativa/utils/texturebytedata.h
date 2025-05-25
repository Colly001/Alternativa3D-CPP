#ifndef TEXTUREBYTEDATA_H
#define TEXTUREBYTEDATA_H

#include <QByteArray>

class TextureByteData
{
public:

	QByteArray diffuseData;
	QByteArray opacityData;

	TextureByteData(const  QByteArray &diffuseData, const QByteArray &opacityData);
};

#endif // TEXTUREBYTEDATA_H
