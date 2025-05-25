#include "device.h"
#include "flash/display3D/context3d.h"
#include "flash/display/stage3d.h"
#include "programresource.h"
#include "textureresource.h"
#include "flash/display3D/textures/texture.h"
#include "vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "flash/display/stage.h"
#include <QDebug>

//private static const RESOURCE_NOT_AVAILABLE_ERROR:String = "Resource is not available.";


void Device::setAntiAlias(int antiAlias)
{
	if (((((!(antiAlias == 0)) && (!(antiAlias == 2))) && (!(antiAlias == 4))) && (!(antiAlias == 16))))
	{
		throw 341341123;
		//throw (new Error("Invalid antialiasing value."));
	}
	this->_antiAlias = antiAlias;
}

void Device::setEnableErrorChecking(bool enable)
{
	this->_enableErrorChecking = enable;
	Context3D *context3D = this->_stage3D->context3D();
	if ((!(context3D == nullptr)) && (!(context3D->getEnableErrorChecking() == this->_enableErrorChecking)))
	{
		context3D->setEnableErrorChecking(this->_enableErrorChecking);
	}
}

bool Device::getReady()
{
	if (this->_stage3D == nullptr)
		return false;

	return this->_stage3D->context3D() != nullptr;
}

Device::Device(Stage *stage, Context3DRenderMode renderMode, DeviceProfile profile)
{	
	this->_stage = stage;
	this->_renderMode = renderMode;
	this->_profile = profile;
	this->_stage3D = this->_stage->stage3D();
	this->_x = _stage3D->x();;
	this->_y = _stage3D->y();;
	this->_width = stage->stageWidth();
	this->_height = stage->stageHeight();
	this->_antiAlias = 0;
	this->_enableDepthAndStencil = true;
	this->_enableErrorChecking = false;
	//this->_stage3D.addEventListener(Event::CONTEXT3D_CREATE, this->onContext3DCreate);
	//if (this->_stage3D.requestContext3D.length > 1)
	//this->_stage3D.requestContext3D(param2, param3);
	//else
	//this->_stage3D.requestContext3D(param2);

	if (this->_stage3D->createContext3D())
		this->onContext3DCreate();
	else
		qDebug("error createContext3D");
}

void Device::setCulling(Context3DTriangleFace triangleFaceToCull)
{
	//if (triangleFaceToCull != this->_renderState.culling)
	{
		this->_renderState.culling = triangleFaceToCull;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setCulling(triangleFaceToCull);
		}
	}
}


void Device::setBlendFactors(Context3DBlendFactor sourceFactor, Context3DBlendFactor destinationFactor)
{
	//if (((!(sourceFactor == this->_renderState.blendSourceFactor)) || (!(destinationFactor == this->_renderState.blendDestinationFactor))))
	{
		this->_renderState.blendSourceFactor = sourceFactor;
		this->_renderState.blendDestinationFactor = destinationFactor;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setBlendFactors(sourceFactor, destinationFactor);
		}
	}
}

void Device::setStencilActions(Context3DTriangleFace triangleFace,
							   Context3DCompareMode compareMode,
							   Context3DStencilAction actionOnBothPass,
							   Context3DStencilAction actionOnDepthFail,
							   Context3DStencilAction actionOnDepthPassStencilFail)
{
	/*
	if (((((!(triangleFace == this->_renderState.stencilActionTriangleFace)) ||
		   (!(compareMode == this->_renderState.stencilActionCompareMode))) ||
		  (!(actionOnBothPass == this->_renderState.stencilActionOnBothPass))) ||
		 (!(actionOnDepthFail == this->_renderState.stencilActionOnDepthFail))) ||
		(!(actionOnDepthPassStencilFail == this->_renderState.stencilActionOnDepthPassStencilFail)))
		*/
	{
		this->_renderState.stencilActionTriangleFace = triangleFace;
		this->_renderState.stencilActionCompareMode = compareMode;
		this->_renderState.stencilActionOnBothPass = actionOnBothPass;
		this->_renderState.stencilActionOnDepthFail = actionOnDepthFail;
		this->_renderState.stencilActionOnDepthPassStencilFail = actionOnDepthPassStencilFail;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setStencilActions(triangleFace, compareMode, actionOnBothPass, actionOnDepthFail, actionOnDepthPassStencilFail);
		}
	}
}

void Device::setStencilReferenceValue(uint32_t referenceValue, uint32_t readMask, uint32_t writeMask)
{
	/*
	if ((((!(referenceValue == this->_renderState.stencilReferenceValue)) ||
		  (!(readMask == this->_renderState.stencilReadMask))) ||
		 (!(writeMask == this->_renderState.stencilWriteMask))))
		 */
	{
		this->_renderState.stencilReferenceValue = referenceValue;
		this->_renderState.stencilReadMask = readMask;
		this->_renderState.stencilWriteMask = writeMask;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setStencilReferenceValue(referenceValue, readMask, writeMask);
		}
	}
}

void Device::setProgram(ProgramResource *program)
{
	//if (program != this->_renderState.program)
	{
		if (!program->available())
		{
			throw 52435435;
			//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
		}

		this->_renderState.program = program;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			this->prepareResource(context, program);
			context->setProgram(program->program);
		}
	}



	/*
	var _loc2_:Context3D;
	if (param1 != this->_renderState.program)
	{
		if ((!(param1.available)))
		{
			throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
		}
		this->_renderState.program = param1;
		_loc2_ = this->_stage3D.context3D;
		if (_loc2_ != null)
		{
			this->prepareResource(_loc2_, param1);
			_loc2_.setProgram(param1.program);
		}
	}
	*/
}

void Device::setDepthTest(bool depthMask, Context3DCompareMode passCompareMode)
{
	//if (((!(depthMask == this->_renderState.depthTestMask)) || (!(passCompareMode == this->_renderState.depthTestPassCompareMode))))
	{
		this->_renderState.depthTestMask = depthMask;
		this->_renderState.depthTestPassCompareMode = passCompareMode;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setDepthTest(depthMask, passCompareMode);
		}
	}
}

void Device::setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, const double *data, int numRegisters, bool forcibly)
{
	if (numRegisters < 0) {
		qDebug("error Device::setProgramConstantsFromVector");
		return;
	}

	//forcibly = false;

	int j = firstRegister << 2;
	int length = numRegisters << 2;
	double *array = (programType == Context3DProgramType::VERTEX) ? this->_renderState.vertexConstants : this->_renderState.fragmentConstants;
	if (forcibly)
	{
		int i = 0;
		bool changed = false;
		while (i < length)
		{
			double value = data[i];
			if (value != array[j])
			{
				array[j] = value;
				changed = true;
			}
			i++;
			j++;
		}


		if (changed == true)
		{
			Context3D *context = this->_stage3D->context3D();
			if (context != nullptr)
				context->setProgramConstantsFromVector(programType, firstRegister, data, numRegisters);
		}
	}
	else
	{
		int i = 0;
		while (i < length)
		{
			array[j] = data[i];
			i++;
			j++;
		}
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
			context->setProgramConstantsFromVector(programType, firstRegister, data, numRegisters);
	}
}

void Device::setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, QVector<double> data, int numRegisters, bool forcibly)
{
	if (numRegisters < 0) {
		qDebug("error Device::setProgramConstantsFromVector");
		return;
	}

	forcibly = false;

	int j = firstRegister << 2;
	int length = numRegisters << 2;
	double *array = (programType == Context3DProgramType::VERTEX) ? this->_renderState.vertexConstants : this->_renderState.fragmentConstants;
	if (forcibly)
	{
		int i = 0;
		bool changed = false;
		while (i < length)
		{
			double value = data[i];
			if (value != array[j])
			{
				array[j] = value;
				changed = true;
			}
			i++;
			j++;
		}
		if (changed == true)
		{
			Context3D *context = this->_stage3D->context3D();
			if (context != nullptr)
				context->setProgramConstantsFromVector(programType, firstRegister, data, numRegisters);
		}
	}
	else
	{
		int i = 0;
		while (i < length)
		{
			array[j] = data[i];
			i++;
			j++;
		}
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
			context->setProgramConstantsFromVector(programType, firstRegister, data, numRegisters);
	}
}

void Device::setTextureAt(int sampler, TextureResource *texture)
{
	//if (texture != this->_renderState.textures[sampler])
	{
		if (((!(texture == nullptr)) && (!(texture->available()))))
			throw 5134324; //(new Error(RESOURCE_NOT_AVAILABLE_ERROR));

		this->_renderState.textures[sampler] = texture;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			if (texture != nullptr)
			{
				this->prepareResource(context, texture);
				context->setTextureAt(sampler, texture->getTexture());
			}
			else
			{
				context->setTextureAt(sampler, nullptr);
			}
		}
	}

}

void Device::setVertexBufferAt(int index, VertexBufferResource *buffer, int bufferOffset, Context3DVertexBufferFormat format)
{
	/*
	if ((((!(buffer == this->_renderState.vertexBuffers[index])) ||
		  (!(bufferOffset == this->_renderState.vertexBuffersOffsets[index]))) ||
		 (!(format == this->_renderState.vertexBuffersFormats[index]))))
		 */
	{
		if (buffer != nullptr && !buffer->available())
		{
			throw 54345424; //(new Error(RESOURCE_NOT_AVAILABLE_ERROR));
		}

		this->_renderState.vertexBuffers[index] = buffer;
		this->_renderState.vertexBuffersOffsets[index] = bufferOffset;
		this->_renderState.vertexBuffersFormats[index] = format;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			if (buffer != nullptr)
			{
				this->prepareResource(context, buffer);
				context->setVertexBufferAt(index, buffer->buffer, bufferOffset, format);
			}
			else
			{
				context->setVertexBufferAt(index, nullptr);
			}
		}
	}
}

void Device::clear(double red, double green, double blue, double alpha, double depth, double stencil, uint32_t mask)
{
	Context3D *context = this->_stage3D->context3D();
	if (context != nullptr)
	{
		if (this->configured == false)
		{
			int _loc9_ = 50;
			int _loc10_width = this->_width;
			int _loc11_height = this->_height;
			if (this->_profile == DeviceProfile::BASELINECONSTRAINED)
			{
				int _loc12_x = this->_x;
				int _loc13_y = this->_y;
				if (_loc12_x < 0)
					_loc12_x = 0;

				if (_loc13_y < 0)
					_loc13_y = 0;

				if ((_loc12_x + _loc10_width) > this->stage()->stageWidth())
				{
					_loc10_width = this->stage()->stageWidth() - _loc12_x;
				}

				if ((_loc13_y + _loc11_height) > this->stage()->stageHeight())
				{
					_loc11_height = this->stage()->stageHeight() - _loc13_y;
				}

				if ((((((!(_loc12_x == this->_stage3D->x())) ||
						(!(_loc13_y == this->_stage3D->y()))) ||
					   (!(_loc10_width == this->backBufferWidth))) ||
					  (!(_loc11_height == this->backBufferHeight))) ||
					 (!(this->_enableDepthAndStencil == this->backBufferEnableDepthAndStencil))))
				{
					context->configureBackBuffer(_loc9_, _loc9_, 0, this->_enableDepthAndStencil);
					this->_stage3D->setPosition(_loc12_x, _loc13_y);
					context->configureBackBuffer(_loc10_width, _loc11_height, 0, this->_enableDepthAndStencil);
					this->backBufferWidth = _loc10_width;
					this->backBufferHeight = _loc11_height;
					this->backBufferAntiAlias = this->_antiAlias;
					this->backBufferEnableDepthAndStencil = this->_enableDepthAndStencil;
				}
			}
			else
			{
				if (this->_stage3D->x() != this->_x || this->_stage3D->y() != this->_y) {
					this->_stage3D->setPosition(this->_x, this->_y);
				}

				if (_loc10_width < _loc9_)
					_loc10_width = _loc9_;

				if (_loc11_height < _loc9_)
					_loc11_height = _loc9_;

				if (((((!(_loc10_width == this->backBufferWidth)) || (!(_loc11_height == this->backBufferHeight))) || (!(this->_antiAlias == this->backBufferAntiAlias))) || (!(this->_enableDepthAndStencil == this->backBufferEnableDepthAndStencil))))
				{
					context->configureBackBuffer(_loc10_width, _loc11_height, this->_antiAlias, this->_enableDepthAndStencil);
					this->backBufferWidth = _loc10_width;
					this->backBufferHeight = _loc11_height;
					this->backBufferAntiAlias = this->_antiAlias;
					this->backBufferEnableDepthAndStencil = this->_enableDepthAndStencil;
				}
			}
			this->configured = true;
		}
		context->clear(red, green, blue, alpha, depth, stencil, mask);
	}
}


void Device::setRenderToBackBuffer()
{
	//qDebug("used Device::setRenderToBackBuffer()");
	//if (this->_renderState.renderTarget != nullptr)
	{
		//this->_renderState.renderTarget = nullptr;
		Context3D *context = this->_stage3D->context3D();
		if (context != nullptr)
		{
			context->setRenderToBackBuffer();
		}
	}
}

void Device::uploadResource(Resource *resource)
{
	if (resource == nullptr)
	{
		qDebug() << "error Device::uploadResource null";
		throw 52345421;
	}


	if (!resource->available())
	{
		throw 52345423;
		//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
	}


	Context3D *context = this->_stage3D->context3D();
	if (context != nullptr)
	{
		if (!resource->isCreated(context))
		{
			resource->create(context);
		}
		//qDebug("Device::uploadResource");
		resource->upload();
	}


	/*
	if ((!(param1.available)))
	{
		throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
	}
	Context3D *context = this->_stage3D->context3D();
	if (context != null)
	{
		try
		{
			if ((!(param1.isCreated(context))))
			{
				param1.create(context);
			}
			param1.upload();
		}
		catch(e:Error)
		{
			trace("Engine error (329)! Unsafely ignoring...");
			this->resourcesToUpload[param1] = true;
			return;
		}
		if ((((!(this->bfModel == null)) && (this->bfModel.toDestroy.indexOf(param1) == -1)) && (param1 is BitmapTextureResource)))
		{
			this->bfModel.toDestroy.push(param1);
		}
	}
	else
	{
		this->resourcesToUpload[param1] = true;
	}
	*/
}

void Device::drawTriangles(IndexBufferResource *indexBuffer, int firstIndex, int numTriangles)
{
	//qDebug("Device::drawTriangles");

	if (!indexBuffer->available())
	{
		throw 1341234;
		//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
	}


	Context3D *context = this->_stage3D->context3D();
	if (context != nullptr)
	{
		this->prepareResource(context, indexBuffer);
		context->drawTriangles(indexBuffer->buffer, firstIndex, numTriangles);
	}

	/*
	Context3D *context = this->_stage3D->context3D();
	if (context != null)
	{
		this->prepareResource(context, param1);
		try
		{
			_loc4_.drawTriangles(param1.buffer, param2, param3);
			return;
		}
		catch(e:Error)
		{
			return;
		}
	}
	*/
}

void Device::onContext3DCreate()
{
	this->configured = false;
	this->backBufferWidth = -1;
	this->backBufferHeight = -1;
	this->backBufferAntiAlias = -1;
	this->backBufferEnableDepthAndStencil = false;

	Context3D *context = this->_stage3D->context3D();
	context->setEnableErrorChecking(this->_enableErrorChecking);

	/*
	for (_loc3_ in this->resourcesToUpload)
	{
		this->uploadResource(_loc3_);
		delete this->resourcesToUpload[_loc3_];
	}
	*/

	context->setBlendFactors(this->_renderState.blendSourceFactor, this->_renderState.blendDestinationFactor);
	context->setColorMask(this->_renderState.colorMaskRed, this->_renderState.colorMaskGreen, this->_renderState.colorMaskBlue, this->_renderState.colorMaskAlpha);
	context->setCulling(this->_renderState.culling);
	context->setDepthTest(this->_renderState.depthTestMask, this->_renderState.depthTestPassCompareMode);

	if (this->_renderState.program != nullptr)
	{
		if (!this->_renderState.program->available())
		{
			throw 1341234;
			//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
		}
		this->prepareResource(context, this->_renderState.program);
		context->setProgram(this->_renderState.program->program);
	}

	/*
	if (this->_renderState.renderTarget != nullptr)
	{
		if ((!(this->_renderState.renderTarget->available())))
		{
			throw 1341234;
			//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
		}
		this->prepareResource(context, this->_renderState.renderTarget);
		context->setRenderToTexture(this->_renderState.renderTarget.texture, this->_renderState.renderTargetEnableDepthAndStencil, this->_renderState.renderTargetAntiAlias, this->_renderState.renderTargetSurfaceSelector);
	}
	*/

	if (this->_renderState.scissor)
	{
		context->setScissorRectangle(&this->_renderState.scissorRectangle);
	}
	else
	{
		context->setScissorRectangle(nullptr);
	}

	context->setStencilActions(this->_renderState.stencilActionTriangleFace, this->_renderState.stencilActionCompareMode, this->_renderState.stencilActionOnBothPass, this->_renderState.stencilActionOnDepthFail, this->_renderState.stencilActionOnDepthPassStencilFail);
	context->setStencilReferenceValue(this->_renderState.stencilReferenceValue, this->_renderState.stencilReadMask, this->_renderState.stencilWriteMask);

	for (int i = 0; i < 8; i++)
	{
		TextureResource *textureResource = this->_renderState.textures[i];
		if (textureResource != nullptr)
		{
			if (!textureResource->available())
			{
				throw 343432423;
				//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
			}
			this->prepareResource(context, textureResource);
			context->setTextureAt(i, textureResource->getTexture());
		}

		VertexBufferResource *vertexBufferResource = this->_renderState.vertexBuffers[i];
		if (vertexBufferResource != nullptr)
		{
			if (!vertexBufferResource->available())
			{
				throw 343432423;
				//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
			}
			this->prepareResource(context, vertexBufferResource);
			context->setVertexBufferAt(i, vertexBufferResource->buffer, this->_renderState.vertexBuffersOffsets[i], this->_renderState.vertexBuffersFormats[i]);
		}
	}

	context->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, this->_renderState.vertexConstants, 128);
	context->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->_renderState.fragmentConstants, 28);
	//dispatchEvent(new Event(Event.CONTEXT3D_CREATE));
	//this->bfModel = BattlefieldModel(Main.osgi.getService(IBattleField));
}


void Device::prepareResource(Context3D *context, Resource *resource)
{
	//qDebug() << "Device::prepareResource" << context << resource;
	if (!resource->isCreated(context))
	{
		resource->create(context);

		if (resource->upload())
		{

		}
		else
		{
			qDebug("error Device::prepareResource resource->upload()");
		}

		//if ((((!(this->bfModel == nullptr)) && (this->bfModel->toDestroy->indexOf(resource) == -1)) && (context is BitmapTextureResource)))
		//{
		//this->bfModel.toDestroy.push(param2);
		//}
	} else if (!resource->isLoaded())
	{
		if (resource->upload())
		{

		}
		else
		{
			qDebug("error Device::prepareResource resource->upload()");
		}
	}
}

void Device::setScissorRectangle(GeomRectangle *rectangle)
{
	Context3D *context = this->_stage3D->context3D();
	if (rectangle != nullptr)
	{
		if (this->_renderState.scissor)
		{
			if (((((!(rectangle->x == this->_renderState.scissorRectangle.x)) || (!(rectangle->y == this->_renderState.scissorRectangle.y))) || (!(rectangle->width == this->_renderState.scissorRectangle.width))) || (!(rectangle->height == this->_renderState.scissorRectangle.height))))
			{
				this->_renderState.scissorRectangle.x = rectangle->x;
				this->_renderState.scissorRectangle.y = rectangle->y;
				this->_renderState.scissorRectangle.width = rectangle->width;
				this->_renderState.scissorRectangle.height = rectangle->height;
				if (context != nullptr)
				{
					context->setScissorRectangle(rectangle);
				}
			}
		}
		else
		{
			this->_renderState.scissor = true;
			this->_renderState.scissorRectangle.x = rectangle->x;
			this->_renderState.scissorRectangle.y = rectangle->y;
			this->_renderState.scissorRectangle.width = rectangle->width;
			this->_renderState.scissorRectangle.height = rectangle->height;
			if (context != nullptr)
			{
				context->setScissorRectangle(rectangle);
			}
		}
	}
	else
	{
		this->_renderState.scissor = false;
		if (context != nullptr)
		{
			context->setScissorRectangle(nullptr);
		}
	}
}

void Device::present()
{
	//qDebug("Device::present()");

	this->_renderState.renderTarget = nullptr;
	Context3D *context = this->_stage3D->context3D();
	if (context != nullptr)
		context->present();

	this->configured = false;
}

void Device::dispose()
{

}

bool Device::ready()
{
	return this->_stage3D->context3D() != nullptr;
}

void Device::draw3DLines(void *data, int count)
{
	Context3D *context = this->_stage3D->context3D();
	if (context != nullptr)
		context->draw3DLines(data, count);
}



void Device::setRenderToTexture(TextureResource *renderTarget,
								bool renderTargetEnableDepthAndStencil,
								int renderTargetAntiAlias,
								int renderTargetSurfaceSelector)
{
	Context3D *context = this->_stage3D->context3D();
	if (context == nullptr)
		return;

	//if (((((!(param1 == this._renderState.renderTarget)) || (!(param2 == this._renderState.renderTargetEnableDepthAndStencil))) || (!(param3 == this._renderState.renderTargetAntiAlias))) || (!(param4 == this._renderState.renderTargetSurfaceSelector))))
	{
		if ((!(renderTarget == nullptr)) && !renderTarget->available())
		{
			//throw (new Error(RESOURCE_NOT_AVAILABLE_ERROR));
			qDebug("error RESOURCE_NOT_AVAILABLE_ERROR");
			return;
		}
		//this._renderState.renderTarget = renderTarget;
		//this._renderState.renderTargetEnableDepthAndStencil = renderTargetEnableDepthAndStencil;
		//this._renderState.renderTargetAntiAlias = renderTargetAntiAlias;
		//this._renderState.renderTargetSurfaceSelector = renderTargetSurfaceSelector;
		if (context != nullptr)
		{
			if (renderTarget != nullptr)
			{
				this->prepareResource(context, renderTarget);
				context->setRenderToTexture(renderTarget->getTexture(), renderTargetEnableDepthAndStencil, renderTargetAntiAlias, renderTargetSurfaceSelector);
			}
			else
			{
				context->setRenderToBackBuffer();
			}
		}
	}
}
