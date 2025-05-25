#include "../objects/mesh.h"
#include "../objects/bsp.h"
#include "../objects/occluder.h"
#include "../objects/sprite3d.h"
#include "alternativa/tanks/camera/gamecamera.h"
#include "../lights/directionallight.h"
#include "../lights/omnilight.h"
#include "../lights/spotlight.h"
#include "../lights/tubelight.h"
#include "object3dcontainer.h"
#include "_global.h"


std::shared_ptr<Object3D> Object3DAll_clone(const std::shared_ptr<Object3D> &in)
{
	_function_name("Object3DAll_clone");

	if (in->isType(Object3D::TYPE)) {
		return Object3D_clone(in);
	}

	if (in->isType(Mesh::TYPE)) {
		return Mesh_clone(std::dynamic_pointer_cast<Mesh>(in));
	}

	if (in->isType(BSP::TYPE)) {
		return BSP_clone(std::dynamic_pointer_cast<BSP>(in));
	}

	if (in->isType(GameCamera::TYPE)) {
		return GameCamera_clone(std::dynamic_pointer_cast<GameCamera>(in));
	}

	if (in->isType(DirectionalLight::TYPE)) {
		return DirectionalLight_clone(std::dynamic_pointer_cast<DirectionalLight>(in));
	}

	if (in->isType(OmniLight::TYPE)) {
		return OmniLight_clone(std::dynamic_pointer_cast<OmniLight>(in));
	}

	if (in->isType(SpotLight::TYPE)) {
		return SpotLight_clone(std::dynamic_pointer_cast<SpotLight>(in));
	}

	if (in->isType(TubeLight::TYPE)) {
		return TubeLight_clone(std::dynamic_pointer_cast<TubeLight>(in));
	}

	if (in->isType(Occluder::TYPE)) {
		return Occluder_clone(std::dynamic_pointer_cast<Occluder>(in));
	}

	if (in->isType(Sprite3D::TYPE)) {
		return Sprite3D_clone(std::dynamic_pointer_cast<Sprite3D>(in));
	}

	if (in->isType(Object3DContainer::TYPE)) {
		return Object3DContainer_clone(std::dynamic_pointer_cast<Object3DContainer>(in));
	}

	throw 4113246;
}


std::shared_ptr<Mesh> MeshAll_clone(const std::shared_ptr<Mesh> &in)
{
	_function_name("Object3DAll_clone");

	if (in->isType(Mesh::TYPE)) {
		return Mesh_clone(std::dynamic_pointer_cast<Mesh>(in));
	}

	throw 4113246;
}

std::shared_ptr<Mesh> Mesh_clone(Mesh *in)
{
	if (in->isType(Mesh::TYPE)) {
		auto out = new_Mesh();
		out->clonePropertiesFromMesh(in);
		return out;
	}

	throw 41132422;
}
