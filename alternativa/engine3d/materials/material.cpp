#include "material.h"
#include <QString>

const Material::T *Material::TYPE = new Material::T("Material");


Material::Material(const T *type)
{
	this->_name = new QString();
	this->type = type;
	this->alphaTestThreshold = 0;
	this->zOffset = false;
	this->uploadEveryFrame = false;
	this->drawId = 0;
	this->useVerticesNormals = true;
	this->isTransparent = false;
}

Material::~Material()
{
	delete _name;
}

void Material::setName(const QString &name)
{
	*_name = name;
}

QString Material::getName() const
{
	return *this->_name;
}

bool Material::transparent()
{
	return this->isTransparent;
}

void Material::setTransparent(bool enable)
{
	this->isTransparent = enable;
}

void Material::clonePropertiesFromMaterial(Material *src)
{
	*this->_name = *src->_name;
	this->alphaTestThreshold = src->alphaTestThreshold;
	this->useVerticesNormals = src->useVerticesNormals;
}
