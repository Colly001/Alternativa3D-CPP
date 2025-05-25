#ifndef OBJECT3D_CLONE_H
#define OBJECT3D_CLONE_H

#include <memory>

class Object3D;
class Mesh;
class Occluder;
class Object3DContainer;


std::shared_ptr<Occluder> Occluder_clone(const std::shared_ptr<Occluder> &in);
std::shared_ptr<Object3DContainer> Object3DContainer_clone(const std::shared_ptr<Object3DContainer> &in);
std::shared_ptr<Object3D> Object3DAll_clone(const std::shared_ptr<Object3D> &in);
std::shared_ptr<Mesh> MeshAll_clone(const std::shared_ptr<Mesh> &in);
std::shared_ptr<Mesh> Mesh_clone(Mesh *in);

#endif // OBJECT3D_CLONE_H
