#include "animsprite.h"


const Object3D::T *AnimSprite::TYPE = new Object3D::T("AnimSprite");


AnimSprite::AnimSprite(const TT &t,
					   double width,
					   double height,
					   QVector<std::shared_ptr<Material>> *materials,
					   bool loop,
					   int frame) :
	Sprite3D(TT(t.type, this->TYPE), width, height, std::shared_ptr<Material>()),
	__type_next(t.type_next)
{
	this->_materials = materials;
	this->_loop = loop;
	this->setFrame(frame);
	this->softAttenuation = 140;
}

void AnimSprite::setMaterials(QVector<std::shared_ptr<Material> > *value)
{
	this->_materials = value;
	if (value != nullptr)
	{
		this->setFrame(this->_frame);
	}
	else
	{
		material = nullptr;
	}
}

void AnimSprite::setLoop(bool value)
{
	this->_loop = value;
	this->setFrame(this->_frame);
}

void AnimSprite::setFrame(int value)
{
	this->_frame = value;
	if (this->_materials != nullptr)
	{
		int materialsLength = this->_materials->length();
		int index = this->_frame;
		if (this->_frame < 0)
		{
			int mod = (this->_frame % materialsLength);
			index = (this->_loop && (!(mod == 0))) ? (mod + materialsLength) : 0;
		}
		else
		{
			if (this->_frame > (materialsLength - 1))
			{
				index = ((this->_loop) ? (this->_frame % materialsLength) : (materialsLength - 1));
			}
		}
		material = this->_materials->at(index);
	}
}


std::shared_ptr<AnimSprite> new_AnimSprite(double width,
										   double height,
										   QVector<std::shared_ptr<Material>> *materials,
										   bool loop,
										   int frame)
{
	return std::make_shared<AnimSprite>(Object3D::TT(AnimSprite::TYPE), width, height, materials, loop, frame);
}
