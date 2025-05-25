#ifndef CAMERA3D_H
#define CAMERA3D_H

#include "object3d.h"
#include <QVector>
#include <QHash>
#include <QSet>


class Vertex;
class Face;
class Wrapper;
class View;
class GameDraw;
class OmniLight;
class SpotLight;
class DirectionalLight;
class ShadowMap;
class TubeLight;
class Device;
class DepthRenderer;
class Decal;
class VertexBufferResource;
class IndexBufferResource;
class Material;
class Shadow;
class TextureResource;
class Light3D;
class IRender3D;
class ShadowAtlas;


class Camera3D : public Object3D
{
public:
	std::shared_ptr<Vertex> firstVertex;
	std::shared_ptr<Face> firstFace;
	std::shared_ptr<Wrapper> firstWrapper;
	std::shared_ptr<Wrapper> lastWrapper;

	DepthRenderer *depthRenderer;
	QVector<Decal*> decals;
	QVector<Shadow*> shadows;
	QHash<int, ShadowAtlas*> shadowAtlases;



	QVector<VertexBufferResource*> opaqueVertexBuffers;
	QVector<IndexBufferResource*> opaqueIndexBuffers;

	QVector<int> opaqueFirstIndexes;
	QVector<int> opaqueNumsTriangles;
	QVector<Object3D*> opaqueObjects;

	QVector<Material*> skyMaterials;

	QVector<VertexBufferResource*> skyVertexBuffers;
	QVector<IndexBufferResource*> skyIndexBuffers;


	QVector<int> skyFirstIndexes;
	QVector<int> skyNumsTriangles;
	QVector<Object3D*> skyObjects;
	QVector<Shadow*> shadowList;
	QVector<Object3D*> transparentBatchObjects;
	QVector<Material*> opaqueMaterials;
	QVector<Face*> transparentOpaqueFaceLists;
	QVector<Object3D*> transparentOpaqueObjects;
	QVector<Object3D*> depthObjects;
	QVector<Object3D*> casterObjects;
	QVector<Light3D*> lights;


	void composeCameraMatrix();

	int opaqueCount;
	int transparentOpaqueCount;
	int skyCount;
	int depthCount;
	int casterCount;
	int lightsLength;

public:
	double fov;


	//int numTriangles;
	//int numPolygons;

	static const T *TYPE;
	const T *__type_next;

	static uint32_t renderId;

	double viewSizeX;
	double viewSizeY;
	double correctionX;
	double correctionY;
	double deferredLightingStrength;
	double shadowMapStrength;
	double directionalLightStrength;
	double focalLength;
	double farClipping;
	double nearClipping;
	double projection[4];
	double correction[8];
	double transform[12];
	double shadowsStrength;

	double lightTransform[4];
	double fragmentConst[8];
	double depthParams[4];
	double lightParams[8];
	double fogFragment[4];
	double ssaoParams[4];
	double fogParams[4];



	double depthBufferScale;
	double softTransparencyStrength;
	double ssaoStrength;
	double ssaoAlpha;
	double fogNear;
	double fogFar;
	double fogAlpha;
	double fogStrength;
	double ssaoRange;
	double ssaoRadius;
	double shadowsDistanceMultiplier;


	Matrix4 matrix_g;



	int transparentCount;
	int omniesCount;
	int spotsCount;
	int tubesCount;
	int numOccluders;
	int numDraws;
	int numShadows;
	int numTriangles;
	int decalsCount;
	int ambientColor;
	int ssaoColor;
	int fogColor;


	QVector<Face*> transparentFaceLists;
	QVector<Object3D*> transparentObjects;
	QVector<OmniLight*> omnies;
	QVector<SpotLight*> spots;
	QVector<TubeLight*> tubes;
	QVector<std::shared_ptr<Vertex>> occluders;


	DirectionalLight *directionalLight;
	std::shared_ptr<Vertex> lastVertex;
	std::shared_ptr<Face> lastFace;

	QSet<IRender3D*> render3D;

	View *view;
	ShadowMap *shadowMap;
	Device *device;	
	TextureResource *lightMap;
	TextureResource *depthMap;

	bool deferredLighting;
	bool debug;
	bool occludedAll;
	bool softTransparency;
	bool ssao;







	Camera3D(const TT &t);
	virtual ~Camera3D();
	void render();
	void setViewSize(double w, double h);

	void addTransparent(std::shared_ptr<Face>&, std::shared_ptr<Object3D>&);
	int checkInDebug(std::shared_ptr<Object3D>&);


	QVector<std::shared_ptr<VertexBufferResource>> *receiversVertexBuffers;
	QVector<std::shared_ptr<IndexBufferResource>> *receiversIndexBuffers;



	Face* sortByAverageZ(Face*);
	Face* sortByDynamicBSP(Face*, double, Face * = nullptr);

	void clonePropertiesFromCamera3D(Camera3D *src);


	void addDecal(Decal *decal);

	void addShadow(Shadow *shadow);

	void addOpaque(Material *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *param6);
	std::shared_ptr<Face> cull(std::shared_ptr<Face> face, int param2);

	std::shared_ptr<Face> clip(std::shared_ptr<Face> face, int param2);


	void addSky(Material *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *param6);

	void deferredDestroy();

	void addRender(IRender3D *render);
	void removeRender(IRender3D *render);

	void removeAllShadows();
	void removeShadow(Shadow *shadow);
private:
	void drawTransparentList(Face *param1, Object3D *param2, bool param3);
	void clearOccluders();

	void lightweightRender();
	void graphicsRender();
};

#endif // CAMERA3D_H
