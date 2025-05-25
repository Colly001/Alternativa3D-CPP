#include "resource.h"

Resource::Resource()
{
	this->context = nullptr;
}

void Resource::dispose()
{
	this->context = nullptr;
}

void Resource::reset()
{
	this->context = nullptr;
}

bool Resource::available() const
{
	return false;
}

void Resource::create(Context3D *context)
{
	this->context = context;
}

bool Resource::upload()
{
	return true;
}

bool Resource::isCreated(Context3D *context) const
{
	return this->context != nullptr && this->context == context;
}

void Resource::destroy(bool)
{

}

bool Resource::isLoaded()
{
	return true;
}
