#ifndef CONTEXT3DCOMPAREMODE_H
#define CONTEXT3DCOMPAREMODE_H

enum class Context3DCompareMode
{
	ALWAYS, //always Сравнение всегда возвращает значение true.
	EQUAL, //equal Равная (==).
	GREATER, //greater Больше (>).
	GREATER_EQUAL, //greaterEqual Больше или равно (>=).
	LESS, //less Меньше (<).
	LESS_EQUAL, //lessEqual Меньше или равно (<=).
	NEVER, //never Сравнение никогда не возвращает значение true.
	NOT_EQUAL, //notEqual Не равно (!=).
};

#endif // CONTEXT3DCOMPAREMODE_H
