#include "camera3d.h"
#include "vertex.h"
#include "face.h"
#include "wrapper.h"
#include "mymath.h"
#include "IMain.h"
#include "service.h"
#include "_global.h"
#include "alternativa/engine3d/materials/material.h"
#include "view.h"
#include "shadowatlas.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/engine3d/core/object3dcontainer.h"
#include "alternativa/engine3d/core/light3d.h"
#include "alternativa/engine3d/objects/sprite3d.h"
#include "alternativa/engine3d/lights/directionallight.h"
#include "depthrenderer.h"
#include "alternativa/engine3d/objects/decal.h"
#include "flash/display3D/Context3DClearMask.h"
#include "shadow.h"
#include "shadowmap.h"
#include "alternativa/gfx/core/rendertargettextureresource.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/tanks/engine3d/animatedsprite3d.h"
#include "alternativa/tanks/camera/IRender3D.h"
#include "alternativa/engine3d/lights/omnilight.h"
#include "alternativa/engine3d/lights/spotlight.h"
#include "alternativa/engine3d/lights/tubelight.h"


const Object3D::T *Camera3D::TYPE = new Object3D::T("Camera3D");


uint32_t Camera3D::renderId = 0;

static const int constantsAttributesCount = 8;
static const int constantsOffset = 16;
static const int constantsMaxTriangles = 18;

//18 * 3 = 54
//54 * 8 = 432
static double constants[432]; //((constantsMaxTriangles * 3) * constantsAttributesCount)


static VertexBufferResource* createConstantsVertexBuffer(int length)
{
	QVector<double> vertices;
	vertices.reserve(length);
	for (int i = 0; i < length; i++)
		vertices.append((i << 1) + constantsOffset);

	for (int i = 0; i < (length << 1); i++)
		constants[(i * 4) + 3] = 1;

	return new VertexBufferResource(vertices, 1);
}

static IndexBufferResource* createConstantsIndexBuffer(int length)
{
	QVector<uint32_t> indices;
	indices.reserve(length);
	for (int i = 0; i < length; i++)
		indices.append(i);
	return new IndexBufferResource(indices);
}

static VertexBufferResource *constantsVertexBuffer = createConstantsVertexBuffer(constantsMaxTriangles * 3);
static IndexBufferResource *constantsIndexBuffer = createConstantsIndexBuffer(constantsMaxTriangles * 3);

Camera3D::Camera3D(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	_function_name("Camera3D::Camera3D");

	fov = 1.5707963267949;
	//numTriangles = 0;
	//numPolygons = 0;

	//this->_diagram = this->createDiagram();

	this->firstVertex = Vertex::create();
	this->firstFace = Face::create();
	this->firstWrapper = Wrapper::create(28);
	this->lastWrapper = this->firstWrapper;
	this->lastVertex = this->firstVertex;
	this->lastFace = this->firstFace;

	this->transparentCount = 0;
	this->deferredLighting = false;
	this->deferredLightingStrength = 1;
	this->debug = false;
	this->omniesCount = 0;
	this->spotsCount = 0;
	this->directionalLight = nullptr;
	this->shadowMapStrength = 1;
	this->directionalLightStrength = 1;
	this->shadowMap = nullptr;
	this->tubesCount = 0;
	this->farClipping = 1000000;
	this->nearClipping = 1;
	this->occludedAll = false;
	this->numOccluders = 0;
	this->numDraws = 0;
	this->numShadows = 0;
	this->numTriangles = 0;
	this->lightsLength = 0;
	this->shadowsStrength = 1;
	this->decalsCount = 0;
	this->opaqueCount = 0;
	this->transparentOpaqueCount = 0;
	this->skyCount = 0;
	this->depthCount = 0;
	this->depthBufferScale = 1;
	this->softTransparency = false;
	this->softTransparencyStrength = 1;
	this->ssao = false;
	this->ssaoStrength = 1;
	this->lightMap = nullptr;
	this->casterCount = 0;
	this->ambientColor = 0;
	this->depthMap = nullptr;
	this->ssaoAlpha = 1;
	this->ssaoColor = 0;
	this->fogNear = 0;
	this->fogFar = 1000000;
	this->fogColor = 0x7F7F7F;
	this->fogAlpha = 0;
	this->fogStrength = 1;
	this->ssaoRange = 1000;
	this->ssaoRadius = 100;
	this->shadowsDistanceMultiplier = 1;



	this->lightTransform[0] = 0;
	this->lightTransform[1] = 0;
	this->lightTransform[2] = 0;
	this->lightTransform[3] = 1;

	//fragmentConst:Vector.<Number> = Vector.<Number>([0, 0, 0, 1, 0.5, 0.5, 0, (1 / 0x1000)]);
	this->fragmentConst[0] = 0;
	this->fragmentConst[1] = 0;
	this->fragmentConst[2] = 0;
	this->fragmentConst[3] = 1;
	this->fragmentConst[4] = 0.5;
	this->fragmentConst[5] = 0.5;
	this->fragmentConst[6] = 0;
	this->fragmentConst[7] = 1. / 0x1000;

	//depthParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->depthParams[0] = 0;
	this->depthParams[1] = 0;
	this->depthParams[2] = 0;
	this->depthParams[3] = 1;


	//lightParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1, 0, 0, 0, 1]);
	this->lightParams[0] = 0;
	this->lightParams[1] = 0;
	this->lightParams[2] = 0;
	this->lightParams[3] = 1;
	this->lightParams[4] = 0;
	this->lightParams[5] = 0;
	this->lightParams[6] = 0;
	this->lightParams[7] = 1;


	//fogFragment:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->fogFragment[0] = 0;
	this->fogFragment[1] = 0;
	this->fogFragment[2] = 0;
	this->fogFragment[3] = 1;

	//ssaoParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->ssaoParams[0] = 0;
	this->ssaoParams[1] = 0;
	this->ssaoParams[2] = 0;
	this->ssaoParams[3] = 1;

	//fogParams:Vector.<Number> = Vector.<Number>([1, 1, 0, 1]);
	this->fogParams[0] = 1;
	this->fogParams[1] = 1;
	this->fogParams[2] = 0;
	this->fogParams[3] = 1;

	depthRenderer = new DepthRenderer();
}

Camera3D::~Camera3D()
{
	if (this->firstVertex != nullptr)
		this->firstVertex->destroy();

	if (this->firstFace != nullptr)
		this->firstFace->destroy();

	if (this->firstWrapper != nullptr)
		Wrapper::destroy(this->firstWrapper);

	//delete this->firstVertex;
	//delete this->firstFace;
	//delete this->firstWrapper;

	delete depthRenderer;
}

void Camera3D::setViewSize(double w, double h)
{
	_function_name("Camera3D::setViewSize");
	this->viewSizeX = w * 0.5;
	this->viewSizeY = h * 0.5;
}

void Camera3D::addTransparent(std::shared_ptr<Face> &face, std::shared_ptr<Object3D> &object3D)
{
	//this->transparentFaceLists[this->transparentCount] = param1;
	//this->transparentObjects[this->transparentCount] = param2;
	transparentFaceLists.append(face.get());
	transparentObjects.append(object3D.get());
	this->transparentCount++;
}


void Camera3D::render()
{
	this->numDraws = 0;
	this->numShadows = 0;
	this->numTriangles = 0;

	if (this->view != nullptr && this->view->device != nullptr && this->view->device->getReady())
	{
		renderId++;
		this->device = this->view->device;
		this->view->configure();
		if (this->nearClipping < 1)
		{
			this->nearClipping = 1;
		}
		if (this->farClipping > 1000000)
		{
			this->farClipping = 1000000;
		}
		this->viewSizeX = this->view->_width * 0.5;
		this->viewSizeY = this->view->_height * 0.5;
		this->focalLength = MyMath::sqrt(((this->viewSizeX * this->viewSizeX) + (this->viewSizeY * this->viewSizeY))) / MyMath::tan((this->fov * 0.5));
		this->correctionX = this->viewSizeX / this->focalLength;
		this->correctionY = this->viewSizeY / this->focalLength;
		this->projection[0] = 1 << this->view->zBufferPrecision;
		this->projection[1] = 1;
		this->projection[2] = this->farClipping / (this->farClipping - this->nearClipping);
		this->projection[3] = (this->nearClipping * this->farClipping) / (this->nearClipping - this->farClipping);

		this->composeCameraMatrix();

		Object3D *object = this;
		while (object->_parent != nullptr)
		{
			object = object->_parent;
			object->composeMatrix();
			appendMatrix(object);
		}

		this->matrix_g.copy(&this->matrix);


		invertMatrix();
		this->transform[0] = this->matrix.ma;
		this->transform[1] = this->matrix.mb;
		this->transform[2] = this->matrix.mc;
		this->transform[3] = this->matrix.md;
		this->transform[4] = this->matrix.me;
		this->transform[5] = this->matrix.mf;
		this->transform[6] = this->matrix.mg;
		this->transform[7] = this->matrix.mh;
		this->transform[8] = this->matrix.mi;
		this->transform[9] = this->matrix.mj;
		this->transform[10] = this->matrix.mk;
		this->transform[11] = this->matrix.ml;
		this->numOccluders = 0;
		this->occludedAll = false;

		if (object != this && object->visible)
		{
			this->lightsLength = 0;
			this->lights.clear();


			Light3D *_loc6_ = ((Object3DContainer*)object)->lightList;

			while (_loc6_ != nullptr)
			{
				if (_loc6_->visible)
				{
					_loc6_->calculateCameraMatrix(this);
					if (_loc6_->checkFrustumCulling(this))
					{
						this->lights.append(_loc6_);
						this->lightsLength++;
						if ((((!(this->view->constrained)) && (this->deferredLighting)) && (this->deferredLightingStrength > 0)))
						{
							if (_loc6_->isType(OmniLight::TYPE))
							{
								this->omnies.append((OmniLight*)_loc6_);
								this->omniesCount++;
							}
							else
							{
								if (_loc6_->isType(SpotLight::TYPE))
								{
									this->spots.append((SpotLight*)_loc6_);
									this->spotsCount++;
								}
								else
								{
									if (_loc6_->isType(TubeLight::TYPE))
									{
										this->tubes.append((TubeLight*)_loc6_);
										this->tubesCount++;
									}
								}
							}
						}
					}
				}
				_loc6_ = _loc6_->nextLight;
			}


			object->appendMatrix(this);

			object->cullingInCamera(this, 63);


			/*
			if (this->debug)
			{
				int _loc1_ = 0;
				while (_loc1_ < this->lightsLength)
				{
					//(this->lights[_loc1_] as Light3D).drawDebug(this);
					_loc1_++;
				}
			}
			*/


			bool _loc8_ = false;
			if (!this->view->constrained && this->shadowsStrength > 0)
			{
				for (Shadow *shadow : this->shadows)
				{
					if (shadow->checkVisibility(this))
					{
						int index = shadow->mapSize + shadow->blur;

						ShadowAtlas *shadowAtlas = this->shadowAtlases[index];
						if (shadowAtlas == nullptr)
						{
							shadowAtlas = new ShadowAtlas(shadow->mapSize, shadow->blur);
							this->shadowAtlases[index] = shadowAtlas;
						}

						if (shadowAtlas->shadows.length() < (shadowAtlas->shadowsCount+1))
							shadowAtlas->shadows.resize(shadowAtlas->shadowsCount+1);

						shadowAtlas->shadows[shadowAtlas->shadowsCount] = shadow;
						shadowAtlas->shadowsCount++;
						_loc8_ = true;
					}
				}
			}


			this->device->setCulling(Context3DTriangleFace::FRONT);
			this->device->setBlendFactors(Context3DBlendFactor::ONE, Context3DBlendFactor::ZERO);
			this->device->setStencilActions(Context3DTriangleFace::NONE);
			this->device->setStencilReferenceValue(0);

			if (_loc8_)
			{
				this->device->setCulling(Context3DTriangleFace::BACK);
				this->device->setDepthTest(true, Context3DCompareMode::GREATER_EQUAL);
				this->device->setProgram(Shadow::getCasterProgram());

				for (ShadowAtlas *shadowAtlas : this->shadowAtlases)
				{
					if (shadowAtlas->shadowsCount > 0)
					{
						shadowAtlas->renderCasters(this);
					}
				}


				this->device->setCulling(Context3DTriangleFace::FRONT);
				this->device->setDepthTest(false, Context3DCompareMode::ALWAYS);

				for (ShadowAtlas *shadowAtlas : this->shadowAtlases)
				{
					if (shadowAtlas->shadowsCount > 0)
					{
						shadowAtlas->renderBlur(this);
					}
				}

				this->device->setTextureAt(0, nullptr);
				this->device->setVertexBufferAt(1, nullptr);
			}


			if (this->directionalLight != nullptr)
			{
				this->directionalLight->composeAndAppend(this);
				this->directionalLight->calculateInverseMatrix();
			}

			object->concatenatedAlpha = object->alpha;
			object->concatenatedBlendMode = object->blendMode;
			object->concatenatedColorTransform = object->colorTransform;
			object->draw(this);


			this->device->setDepthTest(true, Context3DCompareMode::LESS);
			if (this->view->constrained && this->shadowMap != nullptr && this->shadowMapStrength > 0)
			{
				this->shadowMap->calculateBounds(this);
				this->shadowMap->render(this, &this->casterObjects, this->casterCount);
			}


			this->depthMap = nullptr;
			this->lightMap = nullptr;
			if (((!(this->view->constrained)) && ((((this->softTransparency) && (this->softTransparencyStrength > 0)) || ((this->ssao) && (this->ssaoStrength > 0))) || ((this->deferredLighting) && (this->deferredLightingStrength > 0)))))
			{
				this->depthRenderer->render(this, this->view->_width, this->view->_height, this->depthBufferScale, ((this->ssao) && (this->ssaoStrength > 0)), ((this->deferredLighting) && (this->deferredLightingStrength > 0)), ((((!(this->directionalLight == nullptr)) && (this->directionalLightStrength > 0)) || ((!(this->shadowMap == nullptr)) && (this->shadowMapStrength > 0))) ? double(0) : double(0.5)), this->depthObjects, this->depthCount);
				if ((((this->softTransparency) && (this->softTransparencyStrength > 0)) || ((this->ssao) && (this->ssaoStrength > 0))))
				{
					this->depthMap = this->depthRenderer->depthBuffer;
				}
				if (((this->deferredLighting) && (this->deferredLightingStrength > 0)))
				{
					this->lightMap = this->depthRenderer->lightBuffer;
				}
			}
			else
			{
				this->depthRenderer->resetResources();
			}


			if ((_loc8_ || (!this->view->constrained && ((((this->softTransparency) && (this->softTransparencyStrength > 0)) || ((this->ssao) && (this->ssaoStrength > 0))) || ((this->deferredLighting) && (this->deferredLightingStrength > 0))))) || (((!(this->view->constrained)) && (!(this->shadowMap == nullptr))) && (this->shadowMapStrength > 0)))
			{
				this->device->setRenderToBackBuffer();
			}

			this->view->clearArea();
			this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 3, this->projection, 1);
			this->fragmentConst[0] = this->farClipping;
			this->fragmentConst[1] = (this->farClipping / 255.);
			this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 17, this->fragmentConst, 2);
			this->correction[0] = (this->view->rect.width / this->device->width());
			this->correction[1] = (this->view->rect.height / this->device->height());
			this->correction[2] = ((((this->view->rect.x * 2) + this->view->rect.width) - this->device->width()) / this->device->width());
			this->correction[3] = ((((this->view->rect.y * 2) + this->view->rect.height) - this->device->height()) / this->device->height());
			this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 13, this->correction, 1);
			if (((!(this->view->constrained)) && (((((this->softTransparency) && (this->softTransparencyStrength > 0)) || ((this->ssao) && (this->ssaoStrength > 0))) || ((this->deferredLighting) && (this->deferredLightingStrength > 0))) || ((!(this->shadowMap == nullptr)) && (this->shadowMapStrength > 0)))))
			{
				this->depthParams[0] = this->depthRenderer->correctionX;
				this->depthParams[1] = this->depthRenderer->correctionY;
				this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 4, this->depthParams, 1);
				if (((this->ssao) && (this->ssaoStrength > 0)))
				{
					this->ssaoParams[0] = (((1 - ((2 * ((this->ssaoColor >> 16) & 0xFF)) / 255.)) * this->ssaoAlpha) * this->ssaoStrength);
					this->ssaoParams[1] = (((1 - ((2 * ((this->ssaoColor >> 8) & 0xFF)) / 255.)) * this->ssaoAlpha) * this->ssaoStrength);
					this->ssaoParams[2] = (((1 - ((2 * (this->ssaoColor & 0xFF)) / 255.)) * this->ssaoAlpha) * this->ssaoStrength);
					this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 12, this->ssaoParams, 1);
				}
			}
			if ((((!(this->view->constrained)) && (!(this->shadowMap == nullptr))) && (this->shadowMapStrength > 0)))
			{
				this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 6, this->shadowMap->transform, 4);
				this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 5, this->shadowMap->params, 5);
			}
			if (((this->fogAlpha > 0) && (this->fogStrength > 0)))
			{
				this->fogParams[2] = this->fogNear;
				this->fogParams[3] = (this->fogFar - this->fogNear);
				this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 5, this->fogParams, 1);
				this->fogFragment[0] = (((this->fogColor >> 16) & 0xFF) / 255.);
				this->fogFragment[1] = (((this->fogColor >> 8) & 0xFF) / 255.);
				this->fogFragment[2] = ((this->fogColor & 0xFF) / 255.);
				this->fogFragment[3] = (this->fogAlpha * this->fogStrength);
				this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 2, this->fogFragment, 1);
			}

			if ((((!(this->view->constrained)) && (!(this->directionalLight == nullptr))) && (this->directionalLightStrength > 0)))
			{
				this->lightTransform[0] = -(this->directionalLight->matrix_i.mi);
				this->lightTransform[1] = -(this->directionalLight->matrix_i.mj);
				this->lightTransform[2] = -(this->directionalLight->matrix_i.mk);
				this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 10, this->lightTransform, 1);
				this->lightParams[0] = ((((this->directionalLight->intensity * ((this->directionalLight->color >> 16) & 0xFF)) * 2) * this->directionalLightStrength) / 255.);
				this->lightParams[1] = ((((this->directionalLight->intensity * ((this->directionalLight->color >> 8) & 0xFF)) * 2) * this->directionalLightStrength) / 255.);
				this->lightParams[2] = ((((this->directionalLight->intensity * (this->directionalLight->color & 0xFF)) * 2) * this->directionalLightStrength) / 255.);
				this->lightParams[4] = (1 + ((((((this->ambientColor >> 16) & 0xFF) * 2) / 255.) - 1) * this->directionalLightStrength));
				this->lightParams[5] = (1 + ((((((this->ambientColor >> 8) & 0xFF) * 2) / 255.) - 1) * this->directionalLightStrength));
				this->lightParams[6] = (1 + (((((this->ambientColor & 0xFF) * 2) / 255.) - 1) * this->directionalLightStrength));
				this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 10, this->lightParams, 2);
			}
			else
			{
				if ((((!(this->view->constrained)) && (!(this->shadowMap == nullptr))) && (this->shadowMapStrength > 0)))
				{
					this->lightParams[0] = 0;
					this->lightParams[1] = 0;
					this->lightParams[2] = 0;
					this->lightParams[4] = 1;
					this->lightParams[5] = 1;
					this->lightParams[6] = 1;
					this->device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 10, this->lightParams, 2);
				}
			}


			int _loc1_ = 0;
			while (_loc1_ < this->opaqueCount)
			{

				Material *material = this->opaqueMaterials[_loc1_];

				//auto a1 = this->opaqueVertexBuffers[_loc1_];
				//auto a2 = this->opaqueIndexBuffers[_loc1_];
				//auto a3 = this->opaqueFirstIndexes[_loc1_];
				//auto a4 = this->opaqueNumsTriangles[_loc1_];
				//auto a5 = this->opaqueObjects[_loc1_];


				material->drawOpaque(this,
									 this->opaqueVertexBuffers[_loc1_],
									 this->opaqueIndexBuffers[_loc1_],
									 this->opaqueFirstIndexes[_loc1_],
									 this->opaqueNumsTriangles[_loc1_],
									 this->opaqueObjects[_loc1_]);
				_loc1_++;
			}

			this->device->setDepthTest(false, Context3DCompareMode::LESS_EQUAL);
			_loc1_ = 0;
			while (_loc1_ < this->skyCount)
			{
				Material *_loc9_ = this->skyMaterials[_loc1_];
				_loc9_->drawOpaque(this, this->skyVertexBuffers[_loc1_], this->skyIndexBuffers[_loc1_], this->skyFirstIndexes[_loc1_], this->skyNumsTriangles[_loc1_], this->skyObjects[_loc1_]);
				_loc1_++;
			}
			this->device->setDepthTest(false, Context3DCompareMode::LESS);
			_loc1_ = (this->decalsCount - 1);
			while (_loc1_ >= 0)
			{
				Decal *_loc11_ = this->decals[_loc1_];
				if (_loc11_->concatenatedBlendMode != BlendMode::NORMAL)
				{
					this->device->setBlendFactors(Context3DBlendFactor::SOURCE_ALPHA, Context3DBlendFactor::ONE);
				}
				else
				{
					this->device->setBlendFactors(Context3DBlendFactor::SOURCE_ALPHA, Context3DBlendFactor::ONE_MINUS_SOURCE_ALPHA);
				}

				_loc11_->faceList()->material->drawOpaque(this, _loc11_->vertexBuffer.get(), _loc11_->indexBuffer.get(), 0, _loc11_->numTriangles, _loc11_);
				_loc1_--;
			}




			if (_loc8_)
			{
				this->device->setTextureAt(0, nullptr);
				this->device->setTextureAt(1, nullptr);
				this->device->setTextureAt(2, nullptr);
				this->device->setTextureAt(3, nullptr);
				this->device->setTextureAt(5, nullptr);
				this->device->setVertexBufferAt(1, nullptr);
				this->device->setVertexBufferAt(2, nullptr);
				int _loc12_ = 0;

				for (ShadowAtlas *shadowAtlas : this->shadowAtlases)
				{
					int i = 0;
					while (i < shadowAtlas->shadowsCount)
					{
						if (this->shadowList.length() < (_loc12_+1))
							this->shadowList.resize(_loc12_+1);

						this->shadowList[_loc12_] = shadowAtlas->shadows[i];
						_loc12_++;
						i++;
					}
				}

				this->device->setDepthTest(false, Context3DCompareMode::LESS);
				TextureResource *_loc15_ = nullptr;
				_loc1_ = 0;
				while (_loc1_ < _loc12_)
				{
					if (_loc1_ > 0)
					{
						this->device->clear(1, 1, 0, 0, 1, 0, (uint32_t)Context3DClearMask::STENCIL);
					}
					this->device->setBlendFactors(Context3DBlendFactor::ZERO, Context3DBlendFactor::ONE);
					this->device->setCulling(Context3DTriangleFace::NONE);
					this->device->setStencilActions(Context3DTriangleFace::FRONT_AND_BACK, Context3DCompareMode::ALWAYS, Context3DStencilAction::INVERT);
					int _loc13_ = _loc1_;
					int _loc14_ = 1;
					while ((_loc13_ < (_loc1_ + 8)) && (_loc13_ < _loc12_))
					{
						Shadow *shadow = this->shadowList[_loc13_];
						if (!shadow->cameraInside)
						{
							this->device->setStencilReferenceValue(_loc14_, _loc14_, _loc14_);
							shadow->renderVolume(this);
						}
						_loc13_++;
						_loc14_ = _loc14_ << 1;
					}
					this->device->setBlendFactors(Context3DBlendFactor::SOURCE_ALPHA, Context3DBlendFactor::ONE_MINUS_SOURCE_ALPHA);
					this->device->setCulling(Context3DTriangleFace::FRONT);
					this->device->setStencilActions(Context3DTriangleFace::BACK, Context3DCompareMode::EQUAL);
					_loc13_ = _loc1_;
					_loc14_ = 1;
					while (((_loc13_ < (_loc1_ + 8)) && (_loc13_ < _loc12_)))
					{
						Shadow *shadow = this->shadowList[_loc13_];
						if (shadow->texture != _loc15_)
						{
							this->device->setTextureAt(0, shadow->texture);
							_loc15_ = shadow->texture;
						}
						if (!shadow->cameraInside)
						{
							this->device->setStencilReferenceValue(_loc14_, _loc14_, _loc14_);
							shadow->renderReceivers(this);
						}
						else
						{
							this->device->setStencilActions(Context3DTriangleFace::BACK, Context3DCompareMode::ALWAYS);
							shadow->renderReceivers(this);
							this->device->setStencilActions(Context3DTriangleFace::BACK, Context3DCompareMode::EQUAL);
						}
						_loc13_++;
						_loc14_ = _loc14_ << 1;
					}
					this->device->setTextureAt(0, nullptr);
					_loc15_ = nullptr;
					_loc1_ += 8;
				}
				this->device->setStencilActions();
				this->device->setStencilReferenceValue(0);
			}
			this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 13, this->correction, 1);
			this->device->setCulling(Context3DTriangleFace::FRONT);
			_loc1_ = 0;




			while (_loc1_ < this->transparentOpaqueCount)
			{
				if (((_loc1_ < this->transparentOpaqueFaceLists.length()) && (_loc1_ < this->transparentOpaqueObjects.length())))
				{
					if (this->transparentFaceLists.length() < (this->transparentCount+1))
						this->transparentFaceLists.resize(this->transparentCount+1);

					if (this->transparentObjects.length() < (this->transparentCount+1))
						this->transparentObjects.resize(this->transparentCount+1);

					this->transparentFaceLists[this->transparentCount] = this->transparentOpaqueFaceLists[_loc1_];
					this->transparentObjects[this->transparentCount] = this->transparentOpaqueObjects[_loc1_];
					this->transparentCount++;
				}
				_loc1_++;
			}



			this->transparentOpaqueCount = (this->transparentCount - this->transparentOpaqueCount);

			this->device->setDepthTest(true, Context3DCompareMode::LESS);
			_loc1_ = (this->transparentCount - 1);
			while (_loc1_ >= 0)
			{
				if ((_loc1_ + 1) == this->transparentOpaqueCount)
				{
					this->device->setDepthTest(false, Context3DCompareMode::LESS);
				}
				Face *_loc16_ = this->transparentFaceLists[_loc1_];
				Object3D *_loc17_ = this->transparentObjects[_loc1_];

				if (_loc17_->concatenatedBlendMode != BlendMode::NORMAL)
				{
					this->device->setBlendFactors(Context3DBlendFactor::SOURCE_ALPHA, Context3DBlendFactor::ONE);
				}
				else
				{
					this->device->setBlendFactors(Context3DBlendFactor::SOURCE_ALPHA, Context3DBlendFactor::ONE_MINUS_SOURCE_ALPHA);
				}

				bool _loc18_ = _loc17_->isType(Sprite3D::TYPE);
				if (_loc18_)
				{
					int _loc20_ = 0;
					Sprite3D *_loc21_ = (Sprite3D*)_loc17_;
					Face *_loc22_ = _loc16_;
					while (_loc22_->processNext != nullptr)
					{
						_loc22_->distance = _loc20_;
						_loc22_ = _loc22_->processNext.get();
					}
					_loc22_->distance = _loc20_;
					if (this->transparentBatchObjects.length() < (_loc20_+1))
						this->transparentBatchObjects.resize(_loc20_+1);
					this->transparentBatchObjects[_loc20_] = _loc17_;
					_loc20_++;
					int _loc13_ = (_loc1_ - 1);
					while (_loc13_ >= 0)
					{
						Face *_loc23_ = this->transparentFaceLists[_loc13_];
						if (_loc16_->material != _loc23_->material)
						{
							break;
						}
						Object3D *_loc24_ = this->transparentObjects[_loc13_];
						if (_loc24_->isType(Sprite3D::TYPE))
						{
							Sprite3D *_loc25_ = (Sprite3D*)_loc24_;
							if ((((((((((!(_loc21_->useLight == _loc25_->useLight)) ||
										(!(_loc21_->useShadowMap == _loc25_->useShadowMap))) ||
									   (_loc21_->lighted)) || (_loc25_->lighted)) ||
									 (!(_loc21_->softAttenuation == _loc25_->softAttenuation))) ||
									(!(_loc21_->concatenatedAlpha == _loc25_->concatenatedAlpha))) ||
								   (!(_loc21_->concatenatedColorTransform == nullptr))) ||
								  (!(_loc25_->concatenatedColorTransform == nullptr))) ||
								 (!(_loc21_->concatenatedBlendMode == _loc25_->concatenatedBlendMode))))
							{
								break;
							}

							_loc22_->processNext = _loc23_->ptr();
							_loc22_ = _loc23_;
							while (_loc22_->processNext != nullptr)
							{
								_loc22_->distance = _loc20_;
								_loc22_ = _loc22_->processNext.get();
							}
							_loc22_->distance = _loc20_;
							if (this->transparentBatchObjects.length() < (_loc20_+1))
								this->transparentBatchObjects.resize(_loc20_+1);
							this->transparentBatchObjects[_loc20_] = _loc24_;
							_loc20_++;
							_loc1_--;
							_loc13_--;
						}
						else
						{
							break;
						}
					}
				}

				bool _loc19_ = _loc18_ && !((Sprite3D*)_loc17_)->depthTest;
				if (_loc19_)
				{
					this->device->setDepthTest(false, Context3DCompareMode::ALWAYS);
				}
				this->drawTransparentList(_loc16_, _loc17_, _loc18_);
				if (_loc19_)
				{
					this->device->setDepthTest(false, Context3DCompareMode::LESS);
				}
				_loc1_--;
			}




			this->device->setTextureAt(0, nullptr);
			this->device->setTextureAt(1, nullptr);
			this->device->setTextureAt(2, nullptr);
			this->device->setTextureAt(3, nullptr);
			this->device->setTextureAt(5, nullptr);
			this->device->setTextureAt(6, nullptr);
			this->device->setTextureAt(7, nullptr);
			this->device->setVertexBufferAt(1, nullptr);
			this->device->setVertexBufferAt(2, nullptr);
			this->device->setVertexBufferAt(3, nullptr);
			this->device->setVertexBufferAt(4, nullptr);
			this->device->setVertexBufferAt(5, nullptr);
			this->device->setVertexBufferAt(6, nullptr);
			this->device->setVertexBufferAt(7, nullptr);
			this->opaqueMaterials.clear();
			this->opaqueVertexBuffers.clear();
			this->opaqueIndexBuffers.clear();
			this->opaqueFirstIndexes.clear();
			this->opaqueNumsTriangles.clear();
			this->opaqueObjects.clear();
			this->opaqueCount = 0;
			this->skyMaterials.clear();
			this->skyVertexBuffers.clear();
			this->skyIndexBuffers.clear();
			this->skyFirstIndexes.clear();
			this->skyNumsTriangles.clear();
			this->skyObjects.clear();
			this->skyCount = 0;
			this->transparentFaceLists.clear();
			this->transparentObjects.clear();
			this->transparentCount = 0;
			this->transparentOpaqueFaceLists.clear();
			this->transparentOpaqueObjects.clear();
			this->transparentOpaqueCount = 0;
			this->transparentBatchObjects.clear();
			this->decals.clear();
			this->decalsCount = 0;
			this->depthObjects.clear();
			this->depthCount = 0;
			this->casterObjects.clear();
			this->casterCount = 0;
			this->omnies.clear();
			this->omniesCount = 0;
			this->spots.clear();
			this->spotsCount = 0;
			this->tubes.clear();
			this->tubesCount = 0;

			for (ShadowAtlas *shadowAtlas : this->shadowAtlases)
			{
				if (shadowAtlas->shadowsCount > 0)
				{
					shadowAtlas->clear();
				}
			}




			this->receiversVertexBuffers = nullptr;
			this->receiversIndexBuffers = nullptr;
			this->deferredDestroy();
			this->clearOccluders();
			//this->view->onRender(this);
			/*
				if (this->onRender != nullptr)
				{
					this->onRender();
				}
				*/

			if (!this->render3D.isEmpty())
			{
				QSetIterator<IRender3D *> i(this->render3D);
				while (i.hasNext())
					i.next()->render(device, this);
			}


			this->view->present();
		}
		else
		{
			this->view->clearArea();
			//if (this->onRender != nullptr)
			//this->onRender();
			this->view->present();
		}
		this->device = nullptr;
	}
}



void Camera3D::composeCameraMatrix()
{
	_function_name("Camera3D::composeCameraMatrix");

	double _loc1_ = this->viewSizeX / this->focalLength;
	double _loc2_ = this->viewSizeY / this->focalLength;
	double _loc3_ = MyMath::cos(rotation.x);
	double _loc4_ = MyMath::sin(rotation.x);
	double _loc5_ = MyMath::cos(rotation.y);
	double _loc6_ = MyMath::sin(rotation.y);
	double _loc7_ = MyMath::cos(rotation.z);
	double _loc8_ = MyMath::sin(rotation.z);
	double _loc9_ = _loc7_ * _loc6_;
	double _loc10_ = _loc8_ * _loc6_;
	double _loc11_ = _loc5_;
	double _loc12_ = _loc4_;
	double _loc13_ = _loc3_;
	double _loc14_ = _loc3_;
	double _loc15_ = _loc4_;
	matrix.ma = _loc7_ * _loc11_ * _loc1_;
	matrix.mb = (_loc9_ * _loc12_ - _loc8_ * _loc13_) * _loc2_;
	matrix.mc = _loc9_ * _loc14_ + _loc8_ * _loc15_;
	matrix.md = position.x;
	matrix.me = _loc8_ * _loc11_ * _loc1_;
	matrix.mf = (_loc10_ * _loc12_ + _loc7_ * _loc13_) * _loc2_;
	matrix.mg = _loc10_ * _loc14_ - _loc7_ * _loc15_;
	matrix.mh = position.y;
	matrix.mi = -_loc6_ * _loc1_;
	matrix.mj = _loc5_ * _loc12_ * _loc2_;
	matrix.mk = _loc5_ * _loc14_;
	matrix.ml = position.z;
	double _loc16_ = 0. / this->viewSizeX;
	double _loc17_ = 0. / this->viewSizeY;
	matrix.mc = matrix.mc - (matrix.ma * _loc16_ + matrix.mb * _loc17_);
	matrix.mg = matrix.mg - (matrix.me * _loc16_ + matrix.mf * _loc17_);
	matrix.mk = matrix.mk - (matrix.mi * _loc16_ + matrix.mj * _loc17_);
}



int Camera3D::checkInDebug(std::shared_ptr<Object3D> &param1)
{
	(void)param1;
	return 0;
	/*
	int _loc2_ = 0;
	int _loc3_ = 1;

	while (_loc3_ <= 0x0200)
	{
		if (this->debugSet[_loc3_])
		{
			if (((this->debugSet[_loc3_][Object3D]) || (this->debugSet[_loc3_][param1])))
			{
				_loc2_ = (_loc2_ | _loc3_);
			}
			else
			{
				Class _loc4_ = (getDefinitionByName(getQualifiedClassName(param1)) as Class);
				while (_loc4_ != Object3D)
				{
					if (this->debugSet[_loc3_][_loc4_])
					{
						(_loc2_ = (_loc2_ | _loc3_));
						break;
					}
					_loc4_ = Class(getDefinitionByName(getQualifiedSuperclassName(_loc4_)));
				}
			}
		}
		_loc3_ = _loc3_ << 1;
	}

	return _loc2_;
	*/
}


Face *Camera3D::sortByAverageZ(Face *param1)
{
	Face *_loc5_ = param1;
	Face *_loc6_ = param1->processNext.get();

	while (((!(_loc6_ == nullptr)) && (!(_loc6_->processNext == nullptr))))
	{
		param1 = param1->processNext.get();
		_loc6_ = _loc6_->processNext->processNext.get();
	}

	_loc6_ = param1->processNext.get();
	param1->processNext = nullptr;

	if (_loc5_->processNext != nullptr)
	{
		_loc5_ = this->sortByAverageZ(_loc5_);
	}
	else
	{
		int _loc2_ = 0;
		double _loc3_ = 0;
		Wrapper *_loc4_ = _loc5_->wrapper.get();
		while (_loc4_ != nullptr)
		{
			_loc2_++;
			_loc3_ = _loc3_ + _loc4_->vertex->cameraZ;
			_loc4_ = _loc4_->_next;
		}
		_loc5_->distance = (_loc3_ / _loc2_);
	}

	if (_loc6_->processNext != nullptr)
	{
		_loc6_ = this->sortByAverageZ(_loc6_);
	}
	else
	{
		int _loc2_ = 0;
		double _loc3_ = 0;
		Wrapper *_loc4_ = _loc6_->wrapper.get();
		while (_loc4_ != nullptr)
		{
			_loc2_++;
			_loc3_ = _loc3_ + _loc4_->vertex->cameraZ;
			_loc4_ = _loc4_->_next;
		}
		_loc6_->distance = (_loc3_ / _loc2_);
	}

	bool _loc7_ = (_loc5_->distance > _loc6_->distance);
	if (_loc7_)
	{
		param1 = _loc5_;
		_loc5_ = _loc5_->processNext.get();
	}
	else
	{
		param1 = _loc6_;
		_loc6_ = _loc6_->processNext.get();
	}

	Face *_loc8_ = param1;
	while (true)
	{
		if (_loc5_ == nullptr)
		{
			_loc8_->processNext = _loc6_->ptr();
			return (param1);
		}

		if (_loc6_ == nullptr)
		{
			_loc8_->processNext = _loc5_->ptr();
			return (param1);
		}

		if (_loc7_)
		{
			if (_loc5_->distance > _loc6_->distance)
			{
				_loc8_ = _loc5_;
				_loc5_ = _loc5_->processNext.get();
			}
			else
			{
				_loc8_->processNext = _loc6_->ptr();
				_loc8_ = _loc6_;
				_loc6_ = _loc6_->processNext.get();
				_loc7_ = false;
			}
		}
		else
		{
			if (_loc6_->distance > _loc5_->distance)
			{
				_loc8_ = _loc6_;
				_loc6_ = _loc6_->processNext.get();
			}
			else
			{
				_loc8_->processNext = _loc5_->ptr();
				_loc8_ = _loc5_;
				_loc5_ = _loc5_->processNext.get();
				_loc7_ = true;
			}
		}
	}

	return nullptr;
}

Face *Camera3D::sortByDynamicBSP(Face *param1, double param2, Face *param3)
{
	Face *_loc23_ = nullptr;
	Face *_loc24_ = nullptr;
	Face *_loc26_ = nullptr;
	Face *_loc27_ = nullptr;
	Face *_loc28_ = nullptr;

	Face *_loc9_ = param1;
	param1 = _loc9_->processNext.get();
	Wrapper *_loc4_ = _loc9_->wrapper.get();
	Vertex *_loc5_ = _loc4_->vertex.get();
	_loc4_ = _loc4_->_next;
	Vertex *_loc6_ = _loc4_->vertex.get();
	double _loc10_ = _loc5_->cameraX;
	double _loc11_ = _loc5_->cameraY;
	double _loc12_ = _loc5_->cameraZ;
	double _loc13_ = (_loc6_->cameraX - _loc10_);
	double _loc14_ = (_loc6_->cameraY - _loc11_);
	double _loc15_ = (_loc6_->cameraZ - _loc12_);
	double _loc16_ = 0;
	double _loc17_ = 0;
	double _loc18_ = 1;
	double _loc19_ = _loc12_;
	double _loc20_ = 0;
	_loc4_ = _loc4_->_next;

	std::shared_ptr<Wrapper> _loc47_;


	while (_loc4_ != nullptr)
	{
		Vertex *_loc8_ = _loc4_->vertex.get();
		double _loc30_ = (_loc8_->cameraX - _loc10_);
		double _loc31_ = (_loc8_->cameraY - _loc11_);
		double _loc32_ = (_loc8_->cameraZ - _loc12_);
		double _loc33_ = ((_loc32_ * _loc14_) - (_loc31_ * _loc15_));
		double _loc34_ = ((_loc30_ * _loc15_) - (_loc32_ * _loc13_));
		double _loc35_ = ((_loc31_ * _loc13_) - (_loc30_ * _loc14_));
		double _loc36_ = (((_loc33_ * _loc33_) + (_loc34_ * _loc34_)) + (_loc35_ * _loc35_));
		if (_loc36_ > param2)
		{
			(_loc36_ = (1. / MyMath::sqrt(_loc36_)));
			_loc16_ = (_loc33_ * _loc36_);
			(_loc17_ = (_loc34_ * _loc36_));
			(_loc18_ = (_loc35_ * _loc36_));
			(_loc19_ = (((_loc10_ * _loc16_) + (_loc11_ * _loc17_)) + (_loc12_ * _loc18_)));
			break;
		}
		if (_loc36_ > _loc20_)
		{
			(_loc36_ = (1. / MyMath::sqrt(_loc36_)));
			_loc16_ = (_loc33_ * _loc36_);
			(_loc17_ = (_loc34_ * _loc36_));
			(_loc18_ = (_loc35_ * _loc36_));
			(_loc19_ = (((_loc10_ * _loc16_) + (_loc11_ * _loc17_)) + (_loc12_ * _loc18_)));
			(_loc20_ = _loc36_);
		}
		_loc4_ = _loc4_->_next;
	}

	double _loc21_ = (_loc19_ - param2);
	double _loc22_ = (_loc19_ + param2);
	Face *_loc25_ = _loc9_;
	Face *_loc29_ = param1;
	while (_loc29_ != nullptr)
	{
		_loc28_ = _loc29_->processNext.get();
		_loc4_ = _loc29_->wrapper.get();
		_loc5_ = _loc4_->vertex.get();
		_loc4_ = _loc4_->_next;
		_loc6_ = _loc4_->vertex.get();
		_loc4_ = _loc4_->_next;
		Vertex *_loc7_ = _loc4_->vertex.get();
		_loc4_ = _loc4_->_next;
		double _loc37_ = (((_loc5_->cameraX * _loc16_) + (_loc5_->cameraY * _loc17_)) + (_loc5_->cameraZ * _loc18_));
		double _loc38_ = (((_loc6_->cameraX * _loc16_) + (_loc6_->cameraY * _loc17_)) + (_loc6_->cameraZ * _loc18_));
		double _loc39_ = (((_loc7_->cameraX * _loc16_) + (_loc7_->cameraY * _loc17_)) + (_loc7_->cameraZ * _loc18_));
		bool _loc40_ = (((_loc37_ < _loc21_) || (_loc38_ < _loc21_)) || (_loc39_ < _loc21_));
		bool _loc41_ = (((_loc37_ > _loc22_) || (_loc38_ > _loc22_)) || (_loc39_ > _loc22_));
		while (_loc4_ != nullptr)
		{
			Vertex *_loc8_ = _loc4_->vertex.get();
			double _loc42_ = (((_loc8_->cameraX * _loc16_) + (_loc8_->cameraY * _loc17_)) + (_loc8_->cameraZ * _loc18_));
			if (_loc42_ < _loc21_)
			{
				(_loc40_ = true);
			}
			else
			{
				if (_loc42_ > _loc22_)
				{
					(_loc41_ = true);
				}
			}
			_loc8_->offset = _loc42_;
			_loc4_ = _loc4_->_next;
		}
		if ((!(_loc40_)))
		{
			if ((!(_loc41_)))
			{
				_loc25_->processNext = _loc29_->ptr();
				(_loc25_ = _loc29_);
			}
			else
			{
				if (_loc26_ != nullptr)
				{
					_loc27_->processNext = _loc29_->ptr();
				}
				else
				{
					(_loc26_ = _loc29_);
				}
				(_loc27_ = _loc29_);
			}
		}
		else
		{
			if ((!(_loc41_)))
			{
				if (_loc23_ != nullptr)
				{
					_loc24_->processNext = _loc29_->ptr();
				}
				else
				{
					(_loc23_ = _loc29_);
				}
				(_loc24_ = _loc29_);
			}
			else
			{
				(_loc5_->offset = _loc37_);
				(_loc6_->offset = _loc38_);
				(_loc7_->offset = _loc39_);
				std::shared_ptr<Face> _loc43_ = Face::create();
				_loc43_->material = _loc29_->material;
				this->lastFace->next = _loc43_;
				this->lastFace = _loc43_;
				std::shared_ptr<Face> _loc44_ = Face::create();
				_loc44_->material = _loc29_->material;
				this->lastFace->next = _loc44_;
				this->lastFace = _loc44_;
				std::shared_ptr<Wrapper> _loc45_;
				std::shared_ptr<Wrapper> _loc46_;
				_loc4_ = _loc29_->wrapper->_next->_next;
				while (_loc4_->_next != nullptr)
				{
					_loc4_ = _loc4_->_next;
				}
				_loc5_ = _loc4_->vertex.get();
				_loc37_ = _loc5_->offset;
				bool _loc48_ = ((!(_loc29_->material == nullptr)) && (_loc29_->material->useVerticesNormals));
				_loc4_ = _loc29_->wrapper.get();
				while (_loc4_ != nullptr)
				{
					_loc6_ = _loc4_->vertex.get();
					_loc38_ = _loc6_->offset;
					if ((((_loc37_ < _loc21_) && (_loc38_ > _loc22_)) || ((_loc37_ > _loc22_) && (_loc38_ < _loc21_))))
					{
						double _loc49_ = ((_loc19_ - _loc37_) / (_loc38_ - _loc37_));

						std::shared_ptr<Vertex> _loc8_ = Vertex::create();
						(this->lastVertex->next = _loc8_);
						(this->lastVertex = _loc8_);
						(_loc8_->cameraX = (_loc5_->cameraX + ((_loc6_->cameraX - _loc5_->cameraX) * _loc49_)));
						(_loc8_->cameraY = (_loc5_->cameraY + ((_loc6_->cameraY - _loc5_->cameraY) * _loc49_)));
						(_loc8_->cameraZ = (_loc5_->cameraZ + ((_loc6_->cameraZ - _loc5_->cameraZ) * _loc49_)));
						(_loc8_->u = (_loc5_->u + ((_loc6_->u - _loc5_->u) * _loc49_)));
						(_loc8_->v = (_loc5_->v + ((_loc6_->v - _loc5_->v) * _loc49_)));

						if (_loc48_)
						{
							(_loc8_->x = (_loc5_->x + ((_loc6_->x - _loc5_->x) * _loc49_)));
							(_loc8_->y = (_loc5_->y + ((_loc6_->y - _loc5_->y) * _loc49_)));
							(_loc8_->z = (_loc5_->z + ((_loc6_->z - _loc5_->z) * _loc49_)));
							(_loc8_->normalX = (_loc5_->normalX + ((_loc6_->normalX - _loc5_->normalX) * _loc49_)));
							(_loc8_->normalY = (_loc5_->normalY + ((_loc6_->normalY - _loc5_->normalY) * _loc49_)));
							(_loc8_->normalZ = (_loc5_->normalZ + ((_loc6_->normalZ - _loc5_->normalZ) * _loc49_)));
						}
						_loc47_ = Wrapper::create(29);
						_loc47_->vertex = _loc8_;
						if (_loc45_ != nullptr)
						{
							_loc45_->setNext(_loc47_);
						}
						else
						{
							_loc43_->wrapper = _loc47_;
						}
						_loc45_ = _loc47_;
						_loc47_ = Wrapper::create(30);
						_loc47_->vertex = _loc8_;
						if (_loc46_ != nullptr)
						{
							_loc46_->setNext(_loc47_);
						}
						else
						{
							_loc44_->wrapper = _loc47_;
						}
						_loc46_ = _loc47_;
					}
					if (_loc38_ <= _loc22_)
					{
						_loc47_ = Wrapper::create(31);
						_loc47_->vertex = _loc6_->ptr();
						if (_loc45_ != nullptr)
						{
							_loc45_->setNext(_loc47_);
						}
						else
						{
							_loc43_->wrapper = _loc47_;
						}
						_loc45_ = _loc47_;
					}
					if (_loc38_ >= _loc21_)
					{
						_loc47_ = Wrapper::create(32);
						_loc47_->vertex = _loc6_->ptr();
						if (_loc46_ != nullptr)
						{
							_loc46_->setNext(_loc47_);
						}
						else
						{
							_loc44_->wrapper = _loc47_;
						}
						_loc46_ = _loc47_;
					}
					_loc5_ = _loc6_;
					_loc37_ = _loc38_;
					_loc4_ = _loc4_->_next;
				}
				if (_loc23_ != nullptr)
				{
					(_loc24_->processNext = _loc43_);
				}
				else
				{
					_loc23_ = _loc43_.get();
				}
				_loc24_ = _loc43_.get();
				if (_loc26_ != nullptr)
				{
					_loc27_->processNext = _loc44_;
				}
				else
				{
					_loc26_ = _loc44_.get();
				}
				_loc27_ = _loc44_.get();
				_loc29_->processNext = nullptr;
			}
		}
		(_loc29_ = _loc28_);
	}

	if (_loc26_ != nullptr)
	{
		(_loc27_->processNext = nullptr);
		if (_loc26_->processNext != nullptr)
		{
			(param3 = this->sortByDynamicBSP(_loc26_, param2, param3));
		}
		else
		{
			_loc26_->processNext = Face::ptr_safe(param3);
			param3 = _loc26_;
		}
	}
	_loc25_->processNext = Face::ptr_safe(param3);
	param3 = _loc9_;
	if (_loc23_ != nullptr)
	{
		(_loc24_->processNext = nullptr);
		if (_loc23_->processNext != nullptr)
		{
			param3 = this->sortByDynamicBSP(_loc23_, param2, param3);
		}
		else
		{
			_loc23_->processNext = Face::ptr_safe(param3);
			param3 = _loc23_;
		}
	}
	return param3;
}

void Camera3D::clonePropertiesFromCamera3D(Camera3D *src)
{
	clonePropertiesFromObject3D(src);
}

void Camera3D::addDecal(Decal *decal)
{
	if (this->decals.length() < (this->decalsCount + 1))
		this->decals.resize(this->decalsCount + 1);

	this->decals[this->decalsCount] = decal;
	this->decalsCount++;
}

void Camera3D::addShadow(Shadow *shadow)
{
	this->shadows.append(shadow);
}

void Camera3D::removeShadow(Shadow *shadow)
{
	this->shadows.removeOne(shadow);
}

void Camera3D::removeAllShadows()
{
	this->shadows.clear();
}

void Camera3D::drawTransparentList(Face *param1, Object3D *param2, bool param3)
{
	int _loc9_ = 0;
	int _loc10_ = 0;
	Material *_loc11_ = param1->material.get();
	while (param1 != nullptr)
	{
		Face *_loc8_ = param1->processNext.get();
		param1->processNext = nullptr;
		Wrapper *_loc7_ = param1->wrapper.get();
		Vertex *_loc4_ = _loc7_->vertex.get();
		_loc7_ = _loc7_->_next;
		Vertex *_loc5_ = _loc7_->vertex.get();
		if (param3)
		{
			int _loc12_ = param1->distance;
			Object3D *_loc13_ = this->transparentBatchObjects[_loc12_];
			_loc7_ = _loc7_->_next;
			while (_loc7_ != nullptr)
			{
				if (_loc10_ == constantsMaxTriangles)
				{
					if (_loc11_ != nullptr)
					{
						this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, constantsOffset, constants, (_loc10_ * 6), false);
						_loc11_->drawTransparent(this, constantsVertexBuffer, constantsIndexBuffer, 0, _loc10_, param2, true);
					}
					_loc10_ = 0;
					_loc9_ = 0;
				}
				Vertex *_loc6_ = _loc7_->vertex.get();
				constants[_loc9_] = _loc4_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc4_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc4_->cameraZ;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.md);
				_loc9_++;
				constants[_loc9_] = _loc4_->u;
				_loc9_++;
				constants[_loc9_] = _loc4_->v;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.mh);
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.ml);
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraZ;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.md);
				_loc9_++;
				constants[_loc9_] = _loc5_->u;
				_loc9_++;
				constants[_loc9_] = _loc5_->v;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.mh);
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.ml);
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraZ;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.md);
				_loc9_++;
				constants[_loc9_] = _loc6_->u;
				_loc9_++;
				constants[_loc9_] = _loc6_->v;
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.mh);
				_loc9_++;
				constants[_loc9_] = -(_loc13_->matrix.ml);
				_loc9_++;
				_loc10_++;
				_loc5_ = _loc6_;
				_loc7_ = _loc7_->_next;
			}
		}
		else
		{
			_loc7_ = _loc7_->_next;
			while (_loc7_ != nullptr)
			{
				if (_loc10_ == constantsMaxTriangles)
				{
					if (_loc11_ != nullptr)
					{
						this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, constantsOffset, constants, (_loc10_ * 6), false);
						_loc11_->drawTransparent(this, constantsVertexBuffer, constantsIndexBuffer, 0, _loc10_, param2, true);
					}
					_loc10_ = 0;
					_loc9_ = 0;
				}
				Vertex *_loc6_ = _loc7_->vertex.get();
				constants[_loc9_] = _loc4_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc4_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc4_->cameraZ;
				_loc9_++;
				constants[_loc9_] = _loc4_->normalX;
				_loc9_++;
				constants[_loc9_] = _loc4_->u;
				_loc9_++;
				constants[_loc9_] = _loc4_->v;
				_loc9_++;
				constants[_loc9_] = _loc4_->normalY;
				_loc9_++;
				constants[_loc9_] = _loc4_->normalZ;
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc5_->cameraZ;
				_loc9_++;
				constants[_loc9_] = _loc5_->normalX;
				_loc9_++;
				constants[_loc9_] = _loc5_->u;
				_loc9_++;
				constants[_loc9_] = _loc5_->v;
				_loc9_++;
				constants[_loc9_] = _loc5_->normalY;
				_loc9_++;
				constants[_loc9_] = _loc5_->normalZ;
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraX;
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraY;
				_loc9_++;
				constants[_loc9_] = _loc6_->cameraZ;
				_loc9_++;
				constants[_loc9_] = _loc6_->normalX;
				_loc9_++;
				constants[_loc9_] = _loc6_->u;
				_loc9_++;
				constants[_loc9_] = _loc6_->v;
				_loc9_++;
				constants[_loc9_] = _loc6_->normalY;
				_loc9_++;
				constants[_loc9_] = _loc6_->normalZ;
				_loc9_++;
				_loc10_++;
				_loc5_ = _loc6_;
				_loc7_ = _loc7_->_next;
			}
		}
		param1 = _loc8_;
	}
	if (_loc10_ > 0 && _loc11_ != nullptr)
	{
		this->device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, constantsOffset, constants, (_loc10_ * 6), false);
		_loc11_->drawTransparent(this, constantsVertexBuffer, constantsIndexBuffer, 0, _loc10_, param2, true);
	}
}


void Camera3D::deferredDestroy()
{
	Face *_loc1_ = this->firstFace->next.get();
	while (_loc1_ != nullptr)
	{
		Wrapper *_loc2_ = _loc1_->wrapper.get();
		if (_loc2_ != nullptr)
		{
			Wrapper *_loc3_ = nullptr;
			while (_loc2_ != nullptr)
			{
				_loc2_->vertex = nullptr;
				_loc3_ = _loc2_;
				_loc2_ = _loc2_->_next;
			}
			this->lastWrapper->setNext(_loc1_->wrapper);
			this->lastWrapper = _loc3_->ptr();
		}
		_loc1_->material = nullptr;
		_loc1_->wrapper = nullptr;
		_loc1_ = _loc1_->next.get();
	}

	if (this->firstFace != this->lastFace)
	{
		this->lastFace->next = Face::collector;
		Face::collector = this->firstFace->next;
		this->firstFace->next = nullptr;
		this->lastFace = this->firstFace;
	}

	if (this->firstWrapper != this->lastWrapper)
	{
		this->lastWrapper->setNext(Wrapper::collector);
		Wrapper::collector = this->firstWrapper->next();
		this->firstWrapper->setNext(nullptr);
		this->lastWrapper = this->firstWrapper;
	}

	if (this->firstVertex != this->lastVertex)
	{
		this->lastVertex->next = Vertex::collector;
		Vertex::collector = this->firstVertex->next;
		this->firstVertex->next = nullptr;
		this->lastVertex = this->firstVertex;
	}
}

void Camera3D::clearOccluders()
{
	for (int i = 0; i < this->numOccluders; i++)
	{
		std::shared_ptr<Vertex> vertex = this->occluders[i];

		std::shared_ptr<Vertex> temp = vertex;
		while (temp->next != nullptr)
			temp = temp->next;

		temp->next = Vertex::collector;

		Vertex::collector = vertex;
		this->occluders[i] = nullptr;
	}
	this->numOccluders = 0;
}




void Camera3D::addOpaque(Material *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *object)
{
	if (this->opaqueMaterials.length() < (this->opaqueCount+1))
		this->opaqueMaterials.resize(this->opaqueCount+1);

	if (this->opaqueVertexBuffers.length() < (this->opaqueCount+1))
		this->opaqueVertexBuffers.resize(this->opaqueCount+1);

	if (this->opaqueIndexBuffers.length() < (this->opaqueCount+1))
		this->opaqueIndexBuffers.resize(this->opaqueCount+1);

	if (this->opaqueFirstIndexes.length() < (this->opaqueCount+1))
		this->opaqueFirstIndexes.resize(this->opaqueCount+1);

	if (this->opaqueNumsTriangles.length() < (this->opaqueCount+1))
		this->opaqueNumsTriangles.resize(this->opaqueCount+1);

	if (this->opaqueNumsTriangles.length() < (this->opaqueCount+1))
		this->opaqueNumsTriangles.resize(this->opaqueCount+1);

	if (this->opaqueObjects.length() < (this->opaqueCount+1))
		this->opaqueObjects.resize(this->opaqueCount+1);

	this->opaqueMaterials[this->opaqueCount] = param1;
	this->opaqueVertexBuffers[this->opaqueCount] = param2;
	this->opaqueIndexBuffers[this->opaqueCount] = param3;
	this->opaqueFirstIndexes[this->opaqueCount] = param4;
	this->opaqueNumsTriangles[this->opaqueCount] = param5;
	this->opaqueObjects[this->opaqueCount] = object;
	this->opaqueCount++;
}


std::shared_ptr<Face> Camera3D::cull(std::shared_ptr<Face> param1, int param2)
{
	std::shared_ptr<Face> _loc3_;
	std::shared_ptr<Face> _loc4_;
	std::shared_ptr<Face> _loc5_;


	double _loc12_ = MyMath::nan(); //nan
	double _loc13_ = MyMath::nan(); //nan
	double _loc15_ = MyMath::nan(); //nan
	double _loc16_ = MyMath::nan(); //nan
	double _loc18_ = MyMath::nan(); //nan
	double _loc19_ = MyMath::nan(); //nan

	bool _loc21_ = ((param2 & 0x01) > 0);
	bool _loc22_ = ((param2 & 0x02) > 0);
	bool _loc23_ = ((param2 & 0x04) > 0);
	bool _loc24_ = ((param2 & 0x08) > 0);
	bool _loc25_ = ((param2 & 0x10) > 0);
	bool _loc26_ = ((param2 & 0x20) > 0);
	bool _loc27_ = this->nearClipping;
	bool _loc28_ = this->farClipping;
	bool _loc29_ = ((_loc23_) || (_loc24_));
	bool _loc30_ = ((_loc25_) || (_loc26_));

	std::shared_ptr<Face> _loc31_ = param1;

	for (; _loc31_ != nullptr; _loc31_ = _loc5_)
	{
		_loc5_ = _loc31_->processNext;
		Wrapper *_loc9_ = _loc31_->wrapper.get();
		Vertex *_loc6_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc7_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc8_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		if (_loc29_)
		{
			(_loc12_ = _loc6_->cameraX);
			(_loc15_ = _loc7_->cameraX);
			(_loc18_ = _loc8_->cameraX);
		}
		if (_loc30_)
		{
			(_loc13_ = _loc6_->cameraY);
			(_loc16_ = _loc7_->cameraY);
			(_loc19_ = _loc8_->cameraY);
		}
		double _loc14_ = _loc6_->cameraZ;
		double _loc17_ = _loc7_->cameraZ;
		double _loc20_ = _loc8_->cameraZ;

		if (_loc21_)
		{
			if ((((_loc14_ <= _loc27_) || (_loc17_ <= _loc27_)) || (_loc20_ <= _loc27_)))
			{
				_loc31_->processNext = nullptr;
				continue;
			}
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				if (_loc11_->vertex->cameraZ <= _loc27_)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ != nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc22_) && (_loc14_ >= _loc28_)) && (_loc17_ >= _loc28_)) && (_loc20_ >= _loc28_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				if (_loc11_->vertex->cameraZ < _loc28_)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}

		if (((((_loc23_) && (_loc14_ <= -(_loc12_))) && (_loc17_ <= -(_loc15_))) && (_loc20_ <= -(_loc18_))))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (-(_loc10_->cameraX) < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}

		if (((((_loc24_) && (_loc14_ <= _loc12_)) && (_loc17_ <= _loc15_)) && (_loc20_ <= _loc18_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (_loc10_->cameraX < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc25_) && (_loc14_ <= -(_loc13_))) && (_loc17_ <= -(_loc16_))) && (_loc20_ <= -(_loc19_))))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (-(_loc10_->cameraY) < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc26_) && (_loc14_ <= _loc13_)) && (_loc17_ <= _loc16_)) && (_loc20_ <= _loc19_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (_loc10_->cameraY < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (_loc3_ != nullptr)
		{
			_loc4_->processNext = _loc31_->ptr();
		}
		else
		{
			_loc3_ = _loc31_;
		}
		_loc4_ = _loc31_;
	}
	if (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
	}
	return _loc3_;
}

std::shared_ptr<Face> Camera3D::clip(std::shared_ptr<Face> param1, int param2)
{
	double _loc16_ = MyMath::nan(); //nan
	double _loc17_ = MyMath::nan(); //nan
	double _loc19_ = MyMath::nan(); //nan
	double _loc20_ = MyMath::nan(); //nan
	double _loc22_ = MyMath::nan(); //nan
	double _loc23_ = MyMath::nan(); //nan

	std::shared_ptr<Face> _loc3_;
	std::shared_ptr<Face> _loc4_;
	std::shared_ptr<Face> _loc5_;



	std::shared_ptr<Wrapper> _loc12_;
	std::shared_ptr<Wrapper> _loc13_;


	int _loc35_ = 0;

	bool _loc25_ = ((param2 & 0x01) > 0);
	bool _loc26_ = ((param2 & 0x02) > 0);
	bool _loc27_ = ((param2 & 0x04) > 0);
	bool _loc28_ = ((param2 & 0x08) > 0);
	bool _loc29_ = ((param2 & 0x10) > 0);
	bool _loc30_ = ((param2 & 0x20) > 0);

	double _loc31_ = this->nearClipping;
	double _loc32_ = this->farClipping;
	bool _loc33_ = (_loc27_) || (_loc28_);
	bool _loc34_ = (_loc29_) || (_loc30_);
	std::shared_ptr<Face> _loc37_ = param1;

	for (; _loc37_ != nullptr; _loc37_ = _loc5_)
	{
		_loc5_ = _loc37_->processNext;
		Wrapper *_loc9_ = _loc37_->wrapper.get();
		Vertex *_loc6_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc7_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc8_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		if (_loc33_)
		{
			(_loc16_ = _loc6_->cameraX);
			(_loc19_ = _loc7_->cameraX);
			(_loc22_ = _loc8_->cameraX);
		}
		if (_loc34_)
		{
			(_loc17_ = _loc6_->cameraY);
			(_loc20_ = _loc7_->cameraY);
			(_loc23_ = _loc8_->cameraY);
		}
		double _loc18_ = _loc6_->cameraZ;
		double _loc21_ = _loc7_->cameraZ;
		double _loc24_ = _loc8_->cameraZ;
		(_loc35_ = 0);
		if (_loc25_)
		{
			if ((((_loc18_ <= _loc31_) && (_loc21_ <= _loc31_)) && (_loc24_ <= _loc31_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					if (_loc11_->vertex->cameraZ > _loc31_)
					{
						(_loc35_ = (_loc35_ | 0x01));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc31_) && (_loc21_ > _loc31_)) && (_loc24_ > _loc31_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						if (_loc11_->vertex->cameraZ <= _loc31_)
						{
							_loc35_ = (_loc35_ | 0x01);
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					_loc35_ = (_loc35_ | 0x01);
				}
			}
		}
		if (_loc26_)
		{
			if ((((_loc18_ >= _loc32_) && (_loc21_ >= _loc32_)) && (_loc24_ >= _loc32_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					if (_loc11_->vertex->cameraZ < _loc32_)
					{
						(_loc35_ = (_loc35_ | 0x02));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ < _loc32_) && (_loc21_ < _loc32_)) && (_loc24_ < _loc32_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						if (_loc11_->vertex->cameraZ >= _loc32_)
						{
							(_loc35_ = (_loc35_ | 0x02));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x02));
				}
			}
		}
		if (_loc27_)
		{
			if ((((_loc18_ <= -(_loc16_)) && (_loc21_ <= -(_loc19_))) && (_loc24_ <= -(_loc22_))))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (-(_loc10_->cameraX) < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x04));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > -(_loc16_)) && (_loc21_ > -(_loc19_))) && (_loc24_ > -(_loc22_))))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (-(_loc10_->cameraX) >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x04));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x04));
				}
			}
		}
		if (_loc28_)
		{
			if ((((_loc18_ <= _loc16_) && (_loc21_ <= _loc19_)) && (_loc24_ <= _loc22_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (_loc10_->cameraX < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x08));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc16_) && (_loc21_ > _loc19_)) && (_loc24_ > _loc22_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (_loc10_->cameraX >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x08));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x08));
				}
			}
		}
		if (_loc29_)
		{
			if ((((_loc18_ <= -(_loc17_)) && (_loc21_ <= -(_loc20_))) && (_loc24_ <= -(_loc23_))))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (-(_loc10_->cameraY) < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x10));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > -(_loc17_)) && (_loc21_ > -(_loc20_))) && (_loc24_ > -(_loc23_))))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (-(_loc10_->cameraY) >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x10));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x10));
				}
			}
		}
		if (_loc30_)
		{
			if ((((_loc18_ <= _loc17_) && (_loc21_ <= _loc20_)) && (_loc24_ <= _loc23_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (_loc10_->cameraY < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x20));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc17_) && (_loc21_ > _loc20_)) && (_loc24_ > _loc23_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (_loc10_->cameraY >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x20));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x20));
				}
			}
		}
		if (_loc35_ > 0)
		{
			bool _loc38_ = ((!(_loc37_->material == nullptr)) && (_loc37_->material->useVerticesNormals));
			_loc12_ = nullptr;
			_loc13_ = nullptr;
			Wrapper *_loc11_ = _loc37_->wrapper.get();
			while (_loc11_ != nullptr)
			{
				std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(33);
				_loc15_->vertex = _loc11_->vertex;
				if (_loc12_ != nullptr)
				{
					_loc13_->setNext(_loc15_);
				}
				else
				{
					_loc12_ = _loc15_;
				}
				_loc13_ = _loc15_;
				_loc11_ = _loc11_->_next;
			}
			if ((_loc35_ & 0x01))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc31_) && (_loc18_ <= _loc31_)) || ((_loc21_ <= _loc31_) && (_loc18_ > _loc31_))))
					{
						double _loc36_ = ((_loc31_ - _loc18_) / (_loc21_ - _loc18_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));


						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(34);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > _loc31_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x02))
			{
				_loc6_ = _loc13_->vertex.get();
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ < _loc32_) && (_loc18_ >= _loc32_)) || ((_loc21_ >= _loc32_) && (_loc18_ < _loc32_))))
					{
						double _loc36_ = ((_loc32_ - _loc18_) / (_loc21_ - _loc18_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));

						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(35);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ < _loc32_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x04))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc16_ = _loc6_->cameraX;
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc19_ = _loc7_->cameraX;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > -(_loc19_)) && (_loc18_ <= -(_loc16_))) || ((_loc21_ <= -(_loc19_)) && (_loc18_ > -(_loc16_)))))
					{
						double _loc36_ = ((_loc16_ + _loc18_) / (((_loc16_ + _loc18_) - _loc19_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc16_ + ((_loc19_ - _loc16_) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));

						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(36);
						(_loc15_->vertex = _loc10_);
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > -(_loc19_))
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc16_ = _loc19_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x08))
			{
				(_loc6_ = _loc13_->vertex.get());
				(_loc16_ = _loc6_->cameraX);
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				(_loc12_ = nullptr);
				(_loc13_ = nullptr);
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc19_ = _loc7_->cameraX;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc19_) && (_loc18_ <= _loc16_)) || ((_loc21_ <= _loc19_) && (_loc18_ > _loc16_))))
					{
						double _loc36_ = ((_loc18_ - _loc16_) / (((_loc18_ - _loc16_) + _loc19_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						this->lastVertex->next = _loc10_;
						this->lastVertex = _loc10_;
						(_loc10_->cameraX = (_loc16_ + ((_loc19_ - _loc16_) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));

						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(37);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > _loc19_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					(_loc6_ = _loc7_);
					(_loc16_ = _loc19_);
					(_loc18_ = _loc21_);
					(_loc11_ = _loc14_);
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x10))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc17_ = _loc6_->cameraY;
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc20_ = _loc7_->cameraY;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > -(_loc20_)) && (_loc18_ <= -(_loc17_))) || ((_loc21_ <= -(_loc20_)) && (_loc18_ > -(_loc17_)))))
					{
						double _loc36_ = ((_loc17_ + _loc18_) / (((_loc17_ + _loc18_) - _loc20_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc17_ + ((_loc20_ - _loc17_) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));

						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(38);
						(_loc15_->vertex = _loc10_);
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > -(_loc20_))
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					(_loc6_ = _loc7_);
					(_loc17_ = _loc20_);
					(_loc18_ = _loc21_);
					(_loc11_ = _loc14_);
				}
				if (_loc12_ == nullptr)
				{
					(_loc37_->processNext = nullptr);
					continue;
				}
			}
			if ((_loc35_ & 0x20))
			{
				_loc6_ = _loc13_->vertex.get();
				(_loc17_ = _loc6_->cameraY);
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc20_ = _loc7_->cameraY;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc20_) && (_loc18_ <= _loc17_)) || ((_loc21_ <= _loc20_) && (_loc18_ > _loc17_))))
					{
						double _loc36_ = ((_loc18_ - _loc17_) / (((_loc18_ - _loc17_) + _loc20_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc17_ + ((_loc20_ - _loc17_) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));

						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(39);
						(_loc15_->vertex = _loc10_);
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > _loc20_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc17_ = _loc20_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			_loc37_->processNext = nullptr;
			std::shared_ptr<Face> _loc39_ = Face::create();
			_loc39_->material = _loc37_->material;
			this->lastFace->next = _loc39_;
			this->lastFace = _loc39_;
			_loc39_->wrapper = _loc12_;
			_loc37_ = _loc39_;
		}
		if (_loc3_ != nullptr)
		{
			(_loc4_->processNext = _loc37_);
		}
		else
		{
			_loc3_ = _loc37_;
		}
		_loc4_ = _loc37_;
	}
	if (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
	}
	return _loc3_;
}

void Camera3D::addSky(Material *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *param6)
{
	if (this->skyMaterials.length() < (this->skyCount+1))
		this->skyMaterials.resize(this->skyCount+1);

	if (this->skyVertexBuffers.length() < (this->skyCount+1))
		this->skyVertexBuffers.resize(this->skyCount+1);

	if (this->skyIndexBuffers.length() < (this->skyCount+1))
		this->skyIndexBuffers.resize(this->skyCount+1);

	if (this->skyFirstIndexes.length() < (this->skyCount+1))
		this->skyFirstIndexes.resize(this->skyCount+1);

	if (this->skyNumsTriangles.length() < (this->skyCount+1))
		this->skyNumsTriangles.resize(this->skyCount+1);

	if (this->skyObjects.length() < (this->skyCount+1))
		this->skyObjects.resize(this->skyCount+1);

	this->skyMaterials[this->skyCount] = param1;
	this->skyVertexBuffers[this->skyCount] = param2;
	this->skyIndexBuffers[this->skyCount] = param3;
	this->skyFirstIndexes[this->skyCount] = param4;
	this->skyNumsTriangles[this->skyCount] = param5;
	this->skyObjects[this->skyCount] = param6;
	this->skyCount++;
}


void Camera3D::addRender(IRender3D *render)
{
	render3D.insert(render);
}

void Camera3D::removeRender(IRender3D *render)
{
	render3D.remove(render);
}

