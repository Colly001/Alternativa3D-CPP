#include "view.h"
#include "alternativa/gfx/core/device.h"
#include "flash/display3D/Context3DClearMask.h"
#include "flash/display/stage.h"
#include "alternativa/gfx/core/device.h"


static QVector<View*> views;
static Device *staticDevice;




View::View(double x, double y, double width, double height, bool constrainedMode)
{
	//this->_stage = new Stage();
	this->device = nullptr;
	this->logo = nullptr;
	this->constrained = constrainedMode;
	this->zBufferPrecision = 16;
	this->quality = false;
	this->configured = false;
	this->globalCoords = nullptr;
	this->correction = false;
	this->antiAliasEnabled = true;
	this->presented = false;
	this->cleared = false;

	this->setPosition(x, y);
	this->setSize(width, height);

	//addEventListener(Event.ADDED_TO_STAGE, this.onAddToStage);
	//this->onAddToStage();
}


double View::getWidth()
{
	return this->_width;
}

double View::getHeight()
{
	return this->_height;
}

void View::setPosition(double x, double y)
{
	this->_x = x;
	this->_y = y;
}

void View::setSize(double width, double height)
{
	this->_width = width;
	this->_height = height;
	if (this->device != nullptr)
	{
		this->device->setWidth(width);
		this->device->setWidth(height);
	}
	//this->area.width = width;
	//this.area.height = height;
	//this->resizeLogo();
}


void View::configure()
{
	//public function get stage():Stage
	int _loc1_ = 4; //(stage->quality == "LOW") ? 0 : ((stage->quality == "MEDIUM") ? 2 : 4);
	int _loc2_ = ((this->antiAliasEnabled) && (!(this->constrained))) ? _loc1_ : 0;
	this->quality = _loc1_ > 0;

	if (configured == false)
	{
		double _loc3_ = 1000000;
		double _loc4_ = 1000000;
		double _loc5_ = -1000000;
		double _loc6_ = -1000000;

		int _loc7_ = 0;
		while (_loc7_ < views.length())
		{
			View *_loc8_ = views[_loc7_];

			Point coord(0, 0);

			if (_loc8_->globalCoords == nullptr)
				_loc8_->globalCoords = new Point();

			_loc8_->localToGlobal(&coord, _loc8_->globalCoords);

			if (_loc8_->globalCoords->x < _loc3_)
			{
				_loc3_ = _loc8_->globalCoords->x;
			}

			if (_loc8_->globalCoords->y < _loc4_)
			{
				_loc4_ = _loc8_->globalCoords->y;
			}

			if ((_loc8_->globalCoords->x + _loc8_->_width) > _loc5_)
			{
				_loc5_ = _loc8_->globalCoords->x + _loc8_->_width;
			}

			if ((_loc8_->globalCoords->y + _loc8_->_height) > _loc6_)
			{
				_loc6_ = _loc8_->globalCoords->y + _loc8_->_height;
			}

			_loc7_++;
		}

		if (this->device != nullptr)
		{
			this->device->setX(_loc3_);
			this->device->setY(_loc4_);
			this->device->setWidth(_loc5_ - _loc3_);
			this->device->setHeight(_loc6_ - _loc4_);
			this->device->setAntiAlias(_loc2_);
			this->device->setEnableDepthAndStencil(true);
			this->device->setEnableErrorChecking(false);
		}
		configured = true;
	}

	if (this->globalCoords == nullptr)
	{
		Point coord(0, 0);
		this->globalCoords = new Point;
		localToGlobal(&coord, this->globalCoords);
	}

	if (this->device != nullptr)
	{
		this->rect.x = int(this->globalCoords->x) - this->device->getX();
		this->rect.y = int(this->globalCoords->y) - this->device->getY();
	}
	else
	{
		this->rect.x = int(this->globalCoords->x);
		this->rect.y = int(this->globalCoords->y);
	}

	this->rect.width = int(this->_width);
	this->rect.height = int(this->_height);
	this->correction = false;
	//this->canvas.x = (this->_width / 2);
	//this->canvas.y = (this->_height / 2);
	//this->canvas.graphics.clear();
}

void View::clearArea()
{
	if (!cleared)
	{
		this->device->clear(1, 1, 1);
		cleared = true;
	}
	else
	{
		//double red, double green, double blue, double alpha, double depth, double stencil, uint32_t mask
		this->device->clear(1, 1, 1, 1, 1, 0, (int)Context3DClearMask::DEPTH | (int)Context3DClearMask::STENCIL);
	}

	if (((((!(this->rect.x == 0)) ||
		   (!(this->rect.y == 0))) ||
		  (!(this->rect.width == this->device->width()))) ||
		 (!(this->rect.height == this->device->height()))))
	{
		this->device->setScissorRectangle(&this->rect);
		this->correction = true;
	}
}

void View::localToGlobal(const Point *in, Point *out)
{
	out->x = this->x() + in->x;
	out->y = this->y() + in->y;
}

void View::hideLogo()
{
	if (this->logo != nullptr)
	{
		//super->removeChild(this->logo);
		this->logo = nullptr;
	}
}


void View::present()
{
	this->presented = true;
	this->device->setScissorRectangle(nullptr);
	this->correction = false;

	int i = 0;
	for (; i < views.length(); i++)
	{
		View *view = views[i];
		if (view->presented == false)
		{
			break;
		}
	}

	if (i == views.length())
	{
		this->device->present();
		configured = false;
		cleared = false;
		for (int i = 0; i <  views.length(); i++)
		{
			View *view = views[i];
			view->presented = false;
		}
	}
}

void View::addToStage(Stage *stage)
{
	//stage.addEventListener(KeyboardEvent.KEY_DOWN, this.onKeyDown);
	//stage.addEventListener(KeyboardEvent.KEY_UP, this.onKeyUp);
	//removeEventListener(Event.ADDED_TO_STAGE, this.onAddToStage);
	//addEventListener(Event.REMOVED_FROM_STAGE, this.onRemoveFromStage);

	if (staticDevice == nullptr)
	{
		staticDevice = new Device(stage,
								  Context3DRenderMode::AUTO,
								  this->constrained ? DeviceProfile::BASELINECONSTRAINED : DeviceProfile::BASELINE);
	}

	views.append(this);
	this->device = staticDevice;
}

void View::dispose()
{
	onRemoveFromStage();
	delete this;
}

void View::onRemoveFromStage()
{
	//stage.removeEventListener(KeyboardEvent.KEY_DOWN, this.onKeyDown);
	//stage.removeEventListener(KeyboardEvent.KEY_UP, this.onKeyUp);
	//addEventListener(Event.ADDED_TO_STAGE, this.onAddToStage);
	//removeEventListener(Event.REMOVED_FROM_STAGE, this.onRemoveFromStage);
	//this.canvas.graphics.clear();

	views.removeOne(this);
	if (views.length() == 0)
	{
		if (staticDevice != nullptr)
		{
			staticDevice->dispose();
			staticDevice = nullptr;
		}
	}
	this->device = nullptr;
}

void View::clear()
{
	if (((!(this->device == nullptr)) && (this->device->ready())))
	{
		this->device->clear();
	}
	//this->canvas->graphics->clear();
	//this->canvas->filters = [new GlowFilter(39202, 1, 100, 100, 10, 1)];
}
