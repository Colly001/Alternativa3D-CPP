#ifndef ANIMSPRITE_H
#define ANIMSPRITE_H

#include "sprite3d.h"

class AnimSprite : public Sprite3D
{
	QVector<std::shared_ptr<Material>> *_materials;
	int _frame;
	bool _loop;
public:

	static const T *TYPE;
	const T *__type_next;


	AnimSprite(const TT &t,
			   double width,
			   double height,
			   QVector<std::shared_ptr<Material>> *materials = nullptr,
			   bool loop = false,
			   int frame = 0);

	/*
	public function get materials():Vector.<Material>
	{
		return (this._materials);
	}
	*/

	void setMaterials(QVector<std::shared_ptr<Material>> *value);


	bool loop() { return this->_loop; }
	void setLoop(bool value);
	int frame() { return this->_frame; }
	void setFrame(int value);
};

std::shared_ptr<AnimSprite> new_AnimSprite(double width,
										   double height,
										   QVector<std::shared_ptr<Material>> *materials = nullptr,
										   bool loop = false,
										   int frame = 0);

#endif // ANIMSPRITE_H
