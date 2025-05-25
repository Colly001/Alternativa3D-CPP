#include "textureconstructor.h"
#include "alternativa/utils/texturebytedata.h"
#include "alternativa/utils/ITextureConstructorListener.h"
#include "flash/display/bitmapdata.h"
#include <QDebug>

void TextureConstructor::cancel()
{
	this->cancelled = true;
}

void TextureConstructor::createTexture(std::shared_ptr<TextureByteData> &textureData, ITextureConstructorListener *listener)
{
	//if (this->loader != nullptr)
		//throw (new Error("Construction in progress"));

	if (textureData == nullptr)
	{
		qDebug("Parameter textureData is null");
		return;
	}

	if (textureData->diffuseData.isEmpty())
	{
		qDebug("Diffuse data is null");
		return;
	}

	this->cancelled = false;
	this->textureData = textureData;
	this->listener = listener;
	//this->loadBytes(textureData->diffuseData, this->onDiffuseTextureLoadingComplete);

	//data.position = 0;
	//this->loader = new Loader();
	//this->loader.contentLoaderInfo.addEventListener(Event.COMPLETE, onDiffuseTextureLoadingComplete);
	//this->loader.loadBytes(data);

	_texture = std::make_shared<BitmapData>();
	if (!_texture->loadFromData(textureData->diffuseData))
	{
		qDebug("error 3243454");
		return;
	}

	if (!this->textureData->opacityData.isEmpty())
	{
		qDebug("error 54353232");
		/*
		BitmapData *alpha = new BitmapData;
		if (!alpha->loadFromData(textureData->opacityData))
		{
			return;
		}
		*/
	}

	//this->_texture = BitmapUtils.mergeBitmapAlpha(this->_texture, alpha, true);
	//this->complete();
	this->complete();
}


void TextureConstructor::onDiffuseTextureLoadingComplete()
{
	if (this->cancelled)
	{
		//Bitmap(this->loader.content).bitmapData.dispose();
		//this->loader.unload();
		//this->loader = nullptr;
	}
	else
	{
		//this->_texture = Bitmap(this->loader.content).bitmapData;
		//this->loader.contentLoaderInfo.removeEventListener(Event.COMPLETE, this->onDiffuseTextureLoadingComplete);
		//this->loader.unload();
		if (!this->textureData->opacityData.isEmpty())
		{
			//this->loadBytes(this->textureData.opacityData, this->onAlphaTextureLoadingComplete);

			//data.position = 0;
			//this->loader = new Loader();
			//this->loader.contentLoaderInfo.addEventListener(Event.COMPLETE, onAlphaTextureLoadingComplete);
			//this->loader.loadBytes(data);

		}
		else
		{
			this->complete();
		}
	}
}

void TextureConstructor::onAlphaTextureLoadingComplete()
{
	if (this->cancelled)
	{
		//Bitmap(this->loader.content).bitmapData.dispose();
		//this->loader.unload();
		//this->loader = nullptr;
	}
	else
	{
		//BitmapData *alpha = Bitmap(this->loader.content).bitmapData;
		//this->loader.contentLoaderInfo.removeEventListener(Event.COMPLETE, this->onAlphaTextureLoadingComplete);
		//this->loader.unload();
		//this->_texture = BitmapUtils.mergeBitmapAlpha(this->_texture, alpha, true);
		//this->complete();
	}
}

void TextureConstructor::complete()
{
	//this->loader = nullptr;
	//this->textureData = nullptr;
	//ITextureConstructorListener *listener = this->listener;
	//this->listener = nullptr;
	if (listener != nullptr)
		listener->onTextureReady(this);
}
