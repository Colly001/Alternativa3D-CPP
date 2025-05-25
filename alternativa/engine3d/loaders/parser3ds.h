#ifndef PARSER3DS_H
#define PARSER3DS_H

#include "flash/geom/matrix.h"
#include "alternativa/engine3d/materials/material.h"
#include <memory>
#include <QVector>
#include <QMap>

class Object3D;
class TextureMaterial;
class ObjectData;
class Mesh;
struct AnimationData;
struct MaterialData;
struct MapData;
class ChunkInfo;
class Parser3DS_StreamRead;
class Vector3;


class Parser3DS
{
	std::shared_ptr<QMap<QString, std::shared_ptr<ObjectData>>> objectDatas; //Object
	std::shared_ptr<QMap<QString, std::shared_ptr<MaterialData>>> materialDatas; //Object
	std::shared_ptr<QVector<std::shared_ptr<AnimationData>>> animationDatas; //Array
	Parser3DS_StreamRead *data;

public:
	Parser3DS();
	~Parser3DS();
	bool parse(const QByteArray &data, const QString &texturesBaseURL, double scale = 1.0);

	QVector<std::shared_ptr<Object3D>> objects;
	QVector<std::shared_ptr<Object3D>> parents;
	QVector<std::shared_ptr<Material>> materials; //+
	QVector<std::shared_ptr<TextureMaterial>> textureMaterials;

private:
	void buildContent(const QString &texturesBaseURL, double scale);
	void parse3DSChunk(int param1, int param2);
	void readChunkInfo(int param1, ChunkInfo *chunkinfo);
	void parseMainChunk(int param1, int param2);
	void buildMesh(Mesh *param1, ObjectData *param2, AnimationData *param3, double param4);
	void parse3DChunk(int param1, int param2);
	void parseMaterialChunk(std::shared_ptr<MaterialData> &param1, int param2, int param3);
	void parseMapChunk(const QString &param1, MapData *param2, int param3, int param4);
	QString getString(int param1);
	void parseMaterialName(std::shared_ptr<MaterialData>&);
	void parseObject(ChunkInfo *param1);
	void parseObjectChunk(std::shared_ptr<ObjectData> &param1, int param2, int param3);
	void parseMeshChunk(std::shared_ptr<ObjectData> &param1, int param2, int param3);
	void parseVertices(std::shared_ptr<ObjectData> &param1);
	void parseUVs(std::shared_ptr<ObjectData> &param1);
	void parseMatrix(std::shared_ptr<ObjectData> &param1);
	void parseFaces(std::shared_ptr<ObjectData> &param1, ChunkInfo *param2);
	void parseFacesChunk(std::shared_ptr<ObjectData> &param1, int param2, int param3);
	void parseSurface(std::shared_ptr<ObjectData> &param1);
	void parseSmoothingGroups(std::shared_ptr<ObjectData> &param1);
	void parseAnimationChunk(int param1, int param2);
	void parseObjectAnimationChunk(std::shared_ptr<AnimationData> &param1, int param2, int param3);
	std::shared_ptr<Vector3> getRotationFrom3DSAngleAxis(double param1, double param2, double param3, double param4);
};

#endif // PARSER3DS_H
