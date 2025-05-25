#ifndef TEXTUREMATERIAL_H
#define TEXTUREMATERIAL_H

#include "material.h"
#include "alternativa/engine3d/core/MipMapping.h"
#include <QVector>

class BitmapData;
class BitmapTextureResource;
class ProgramResource;
class CompressedTextureResource;


class TextureMaterial : public Material
{
protected:
	 double correctionConst[8];
	 double uvCorrection[4];
	 double fragmentConst[4];
	 double uvTransformConst[8];

public:
	int _id;

	static const T *TYPE;

	TextureMaterial(const T *type);
	virtual ~TextureMaterial();
	void init(bool repeat = false, bool smooth = true, MipMapping mipMapping = MipMapping::NONE, double resolution = 1);

	QString diffuseMapURL;
	QString opacityMapURL;
	std::shared_ptr<BitmapTextureResource> _textureResource;
	BitmapTextureResource *textureResource;
	CompressedTextureResource *textureATFResource;
	QByteArray *_textureATFAlpha;
	CompressedTextureResource *textureATFAlphaResource;
	std::shared_ptr<BitmapData> getTexture();
	void setTexture(std::shared_ptr<BitmapData>);

	BitmapData* getTexturePtr();

	double resolution;
	double threshold;

	int numMaps;
	MipMapping _mipMapping;

	bool smooth;
	bool repeat;
	bool _hardwareMipMaps;
	bool correctUV;

	virtual void disposeResource();
	bool getHardwareMipMaps();
	void setHardwareMipMaps(bool);
	void clonePropertiesFromTextureMaterial(TextureMaterial*);
	bool transparent() override;
	void drawOpaque(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*) override;
	void drawTransparent(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*, bool = false) override;
	void dispose() override;

	void setTextureResource(std::shared_ptr<BitmapTextureResource> &textureResource)
	{
		this->_textureResource = textureResource;
		this->textureResource = textureResource.get();
	}

	BitmapTextureResource* getTextureResource()
	{
		return this->textureResource;
	}

protected:
	virtual ProgramResource* getProgram(bool param1, bool param2, bool param3, bool param4,
								bool param5, bool param6, bool param7, bool param8,
								bool param9, bool param10, bool param11,
								bool param12, bool param13, bool param14,
								bool param15, bool param16, bool param17,
								bool param18, bool param19, bool param20, bool param21, bool param22);

protected:
	std::shared_ptr<BitmapData> bitmap;
};


std::shared_ptr<TextureMaterial> new_TextureMaterial();

std::shared_ptr<TextureMaterial> TextureMaterial_clone(std::shared_ptr<TextureMaterial> src);

#endif // TEXTUREMATERIAL_H
