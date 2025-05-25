#include "bitmaptextureresource.h"
#include "mymath.h"
#include "flash/display/bitmapdata.h"
#include "alternativa/engine3d/materials/textureresourcesregistry.h"
#include "flash/display3D/textures/texture.h"
#include "flash/geom/geomrectangle.h"
#include "flash/geom/matrix.h"
#include "flash/display3D/context3d.h"
#include "file.h"
#include <QDebug>


static const Point point(0, 0);
static Context3D *nullTextureContext;
static std::shared_ptr<Texture> nullTexture;



BitmapTextureResource::BitmapTextureResource(std::shared_ptr<BitmapData> bitmapData, bool mipMapping, bool stretchNotPowerOf2Textures, bool calculateMipMapsUsingGPU)
{
	this->referencesCount = 1;
	this->_correctionU = 1;
	this->_correctionV = 1;

	this->_bitmapData = bitmapData;
	this->_mipMapping = mipMapping;
	this->_stretchNotPowerOf2Textures = stretchNotPowerOf2Textures;
	this->_calculateMipMapsUsingGPU = calculateMipMapsUsingGPU;
	this->correctedWidth = MyMath::pow(2, MyMath::ceil(MyMath::log(this->_bitmapData->width()) / MyMath::LN2));
	this->correctedHeight = MyMath::pow(2, MyMath::ceil(MyMath::log(this->_bitmapData->height()) / MyMath::LN2));
	this->loaded = false;


	if (this->correctedWidth > 0x0800)
	{
		this->correctedWidth = 0x0800;
	}

	if (this->correctedHeight > 0x0800)
	{
		this->correctedHeight = 0x0800;
	}

	if ((this->_bitmapData->width() != this->correctedWidth || this->_bitmapData->height() != this->correctedHeight) &&
		   !this->_stretchNotPowerOf2Textures && this->_bitmapData->width() <= 0x0800 && this->_bitmapData->height() <= 0x0800)
	{
		this->_correctionU = double(this->_bitmapData->width()) / double(this->correctedWidth);
		this->_correctionV = double(this->_bitmapData->height()) / double(this->correctedHeight);
	}
}

BitmapTextureResource::~BitmapTextureResource()
{

}

std::shared_ptr<BitmapData> BitmapTextureResource::bitmapData()
{
	return this->_bitmapData;
}

BitmapData* BitmapTextureResource::bitmapDataPtr()
{
	return this->_bitmapData.get();
}

bool BitmapTextureResource::mipMapping()
{
	return this->_mipMapping;
}

bool BitmapTextureResource::getStretchNotPowerOf2Textures()
{
	return this->_stretchNotPowerOf2Textures;
}





bool BitmapTextureResource::getCalculateMipMapsUsingGPU()
{
	return this->_calculateMipMapsUsingGPU;
}

void BitmapTextureResource::setCalculateMipMapsUsingGPU(bool b)
{
	this->_calculateMipMapsUsingGPU = b;
}

void BitmapTextureResource::forceDispose()
{
	/*
	this->referencesCount = 1;
	this->dispose();
	if (this->_bitmapData != nullptr)
	{
		//this->_bitmapData->dispose();
	}
	this->_bitmapData = nullptr;
	*/
}

void BitmapTextureResource::dispose()
{
	/*
	if (this->referencesCount == 0)
	{
		return;
	}
	this->referencesCount--;
	if (this->referencesCount == 0)
	{
		TextureResourcesRegistry::release(this->_bitmapData);
		this->_bitmapData = nullptr;
		//super.dispose();
	}
	*/
}

bool BitmapTextureResource::available() const
{
	return this->_bitmapData != nullptr;
}

bool BitmapTextureResource::isLoaded()
{
	return loaded;
}

Texture* BitmapTextureResource::getNullTexture() //override
{
	return nullTexture.get();
}

void BitmapTextureResource::freeMemory()
{
	useNullTexture = true;
	this->_mipMapping = false;
	this->forceDispose();
}

void BitmapTextureResource::create(Context3D *context)
{
	TextureResource::create(context);

	if (nullTexture == nullptr || nullTextureContext != context)
	{
		nullTexture = context->createTexture(1, 1, Context3DTextureFormat::BGRA, false);
		nullTexture->uploadFromBitmapData(new BitmapData(1, 1, true, 0x55CCCC00));
		nullTextureContext = context;
	}

	if (useNullTexture == false)
	{
		std::shared_ptr<Texture> texture = context->createTexture(this->correctedWidth, this->correctedHeight, Context3DTextureFormat::BGRA, false);
		this->setTexture(texture);
		if (texture == nullptr)
		{
			qDebug("error BitmapTextureResource::create");
		}

		/*
		try
		{
			return;
		}
		catch(e:Error)
		{
			freeMemory();
			return;
		}
		*/
	}
}

bool BitmapTextureResource::upload() //override
{
	std::shared_ptr<BitmapData> bitmapData_global;

	if (useNullTexture == true)
	{
		return true;
	}

	//qDebug() << "BitmapTextureResource::upload()" << this->_bitmapData->width() << this->_bitmapData->width();

	if (this->_bitmapData->width() == this->correctedWidth && this->_bitmapData->height() == this->correctedHeight)
	{
		bitmapData_global = std::make_shared<BitmapData>(this->correctedWidth, this->correctedHeight, this->_bitmapData->transparent(), 0);
		GeomRectangle _rectangle;
		this->_bitmapData->getRect(&_rectangle);
		bitmapData_global->copyPixels(this->_bitmapData.get(), &_rectangle, &point);
	}
	else
	{
		if ((this->_bitmapData->width() <= 0x0800) && (this->_bitmapData->height() <= 0x0800) && !this->_stretchNotPowerOf2Textures)
		{
			bitmapData_global = std::make_shared<BitmapData>(this->correctedWidth, this->correctedHeight, this->_bitmapData->transparent(), 0);

			GeomRectangle _rectangle;
			this->_bitmapData->getRect(&_rectangle);
			bitmapData_global->copyPixels(this->_bitmapData.get(), &_rectangle, &point);

			if (this->_bitmapData->width() < bitmapData_global->width())
			{
				int width = this->_bitmapData->width();
				int height = this->_bitmapData->height();
				int width_global = bitmapData_global->width();

				uint32_t *src_data = _bitmapData->data();
				uint32_t *dst_data = bitmapData_global->data();

				for (int y = 0; y < height; y++)
				{
					 uint32_t pixel = src_data[(y * width) + (width-1)];

					 for (int x = width; x < width_global; x++)
					 {
						 dst_data[(y * width_global) + x] = pixel;
					 }
				}
			}

			if (this->_bitmapData->height() < bitmapData_global->height())
			{
				int width = this->_bitmapData->width();
				int height = this->_bitmapData->height();
				int width_global = bitmapData_global->width();
				int height_global = bitmapData_global->height();
				uint32_t *dst_data = bitmapData_global->data();
				const uint32_t *src_y = this->_bitmapData->data() + ((height - 1) * width);
				for (int y = height-1; y < height_global; y++)
					memcpy(dst_data + (y * width_global), src_y, width * 4);
			}

			if (this->_bitmapData->width() < bitmapData_global->width() && this->_bitmapData->height() < bitmapData_global->height())
			{
				int width = this->_bitmapData->width();
				int height = this->_bitmapData->height();
				int width_global = bitmapData_global->width();
				int height_global = bitmapData_global->height();

				uint32_t *src_data = _bitmapData->data();
				uint32_t *dst_data = bitmapData_global->data();

				uint32_t pixel = src_data[((height-1) * width) + (width-1)];

				for (int y = height; y < height_global; y++)
					 for (int x = width; x < width_global; x++)
						 dst_data[(y * width_global) + x] = pixel;
			}
		}
		else
		{
			bitmapData_global = std::make_shared<BitmapData>();
			bitmapData_global->copy(this->_bitmapData.get());
			bitmapData_global->scaled(this->correctedWidth, this->correctedHeight);
		}

	}



	if (this->_mipMapping > 0)
	{
		this->uploadTexture(bitmapData_global.get(), 0);

		Matrix matrix;

		int i = 1;
		int width = bitmapData_global->width();
		int height = bitmapData_global->height();
		while ((width % 2) == 0 || (height % 2) == 0)
		{
			width = width >> 1;
			height = height >> 1;

			if (width == 0)
				width = 1;

			if (height == 0)
				height = 1;

			BitmapData bitmapData_local(width, height, bitmapData_global->transparent(), 0);
			matrix.a = double(width) / double(bitmapData_global->width());
			matrix.d = double(height) / double(bitmapData_global->height());
			bitmapData_local.draw(bitmapData_global.get(), &matrix, nullptr, BlendMode::NONE, nullptr, false);
			this->uploadTexture(&bitmapData_local, i++);
		}
	}


	this->uploadTexture(bitmapData_global.get(), 0);


	/*
	if (bitmapData_global != this->_bitmapData)
	{
		bitmapData_global->dispose();
		bitmapData_global = nullptr;
	}
	*/

	return true;
}

void BitmapTextureResource::uploadTexture(BitmapData *bitmapData, uint32_t mipLevel)
{
	BitmapData *source = bitmapData;

	Texture *texture = this->getTexture();

	if (texture != ::nullTexture.get())
	{
		if (!texture->uploadFromBitmapData(source, mipLevel))
		{
			//freeMemory();
		}
	}

	loaded = true;
}

void BitmapTextureResource::updateResource()
{
	loaded = false;
}

void BitmapTextureResource::clear()
{
	::nullTexture = nullptr;
}

void BitmapTextureResource::increaseReferencesCount()
{
	this->referencesCount++;
}

void BitmapTextureResource::destroy(bool b) //override
{
	(void)b;

	//this->freeMemory();
	//delete this[this];
}
