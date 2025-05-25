#include "materialentry.h"

MaterialEntry::MaterialEntry(std::shared_ptr<BitmapData> &param1, std::shared_ptr<TextureMaterial> material)
{
	this->keyData = param1;
	this->texture = param1;
	this->material = material;
}

MaterialEntry::MaterialEntry(const QString &param1, std::shared_ptr<TextureMaterial> material)
{
	this->keyData = nullptr;
	this->keyDataString = param1;
	this->texture = nullptr;
	this->material = material;
}
