#ifndef SHADOW_H
#define SHADOW_H

#include "alternativa/physics/collision/types/boundbox.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include <memory>
#include <QVector>

class ProgramResource;
class Camera3D;
class TextureResource;
class Mesh;
class DirectionalLight;
class VertexBufferResource;
class IndexBufferResource;
class Vector3;
class Vertex;


class Shadow
{
	/*
	public var offset:Number = 0;
	public var backFadeRange:Number = 0;
	*/

	Vector3 *direction;


	QVector<Mesh*> casters;
	int castersCount;






	std::shared_ptr<DirectionalLight> light;

	VertexBufferResource *volumeVertexBuffer;
	IndexBufferResource *volumeIndexBuffer;


	std::shared_ptr<Vertex> boundVertexList;

	double planeX;
	double planeY;

	double planeSize;

	double minZ;




	//alternativa3d var texture:TextureResource;


	double transformConst[12];
	double colorConst[12];
	double uvConst[8];
	double clampConst[4];
	double casterConst[20];

public:

	QVector<int> receiversBuffers;
	QVector<int> receiversFirstIndexes;
	QVector<int> receiversNumsTriangles;



	bool cameraInside;
	TextureResource *texture;

	double offset;
	double backFadeRange;
	double attenuation;
	double nearDistance;
	double farDistance;
	double alpha;
	double volumeTransformConst[20];
	double volumeFragmentConst[4];

	double textureScaleU;
	double textureScaleV;
	double textureOffsetU;
	double textureOffsetV;

	BoundBox bound;


	int mapSize;
	int blur;
	int color;
	int receiversCount;


	Shadow(int mapSize, int blur, double attenuation, double nearDistance, double farDistance, int color = 0, double alpha = 1);

	static ProgramResource* getCasterProgram();

	void renderVolume(Camera3D *param1);
	void renderReceivers(Camera3D *param1);
	void addCaster(Mesh *mesh);
	void removeCaster(Mesh *mesh);
	void removeAllCasters();
	bool checkVisibility(Camera3D *param1);
	void renderCasters(Camera3D *param1);
	bool cullingInCamera(Camera3D *param1);
};

#endif // SHADOW_H
