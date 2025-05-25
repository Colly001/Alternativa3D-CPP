#ifndef KDNODE_H
#define KDNODE_H

#include "receiver.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include <QVector>
#include <memory>

class Object3D;
class Shadow;
class Decal;


class KDNode
{
public:

	KDNode *negative;
	KDNode *positive;
	int axis;
	double coord;
	double minCoord;
	double maxCoord;
	BoundBox bound;
	std::shared_ptr<Object3D> objectList;
	std::shared_ptr<Object3D> objectBoundList;
	std::shared_ptr<Object3D> occluderList;
	std::shared_ptr<Object3D> occluderBoundList;
	std::shared_ptr<Receiver> receiverList;

	KDNode();

	void createReceivers(QVector<std::shared_ptr<QVector<double>>> *param1, QVector<std::shared_ptr<QVector<uint32_t>>> *param2);
	void collectReceivers(Shadow *param1);
	void collectPolygons(Decal *param1, double param2, double param3, double param4, double param5, double param6, double param7, double param8, double param9);
};

#endif // KDNODE_H
