#ifndef SHADOWATLAS_H
#define SHADOWATLAS_H

#include <QVector>
#include <QHash>

class Shadow;
class ProgramResource;
class RenderTargetTextureResource;
class Camera3D;
class VertexBufferResource;
class IndexBufferResource;


class ShadowAtlas
{
	double blurConst[8];
public:


	QVector<Shadow*> shadows;
	int shadowsCount;

	int mapSize;
	int blur;

	QHash<int, RenderTargetTextureResource*> maps;


	RenderTargetTextureResource *map1;
	RenderTargetTextureResource *map2;

	VertexBufferResource *blurVertexBuffer;
	IndexBufferResource *blurIndexBuffer;

	ShadowAtlas(int mapSize, int blur);
	void renderCasters(Camera3D *param1);
	void renderBlur(Camera3D *param1);
	void clear();
};

#endif // SHADOWATLAS_H
