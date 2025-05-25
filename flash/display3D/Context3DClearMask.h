#ifndef CONTEXT3DCLEARMASK_H
#define CONTEXT3DCLEARMASK_H

enum class Context3DClearMask
{
	ALL = 7, //Очистка всех буферов.
	COLOR = 1, //Очищает только буфер цвета.
	DEPTH = 2, //Очищает только буфер глубины.
	STENCIL = 4,//Очищает только буфер трафарета.
};

#endif // CONTEXT3DCLEARMASK_H
