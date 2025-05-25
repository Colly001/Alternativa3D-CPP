#ifndef CONTEXT3DSTENCILACTION_H
#define CONTEXT3DSTENCILACTION_H

enum class Context3DStencilAction
{
	DECREMENT_SATURATE, //decrementSaturate Уменьшает значение в буфере трафарета, минимальное значение 0.
	DECREMENT_WRAP, //decrementWrap Уменьшает значение буфера трафарета.
	INCREMENT_SATURATE, //incrementSaturate Увеличивает значение в буфере трафарета, максимальное значение 255.
	INCREMENT_WRAP, //incrementWrap Увеличивает значение буфера трафарета.
	INVERT, //invert Выполняет побитовую инверсию значения буфера трафарета.
	KEEP, //keep Сохраняет текущее значение буфера трафарета.
	SET, //set Заменяет значение буфера трафарета на эталонное значение.
	ZERO, //zero Задает нулевое значение буфер трафарета (0).
};

#endif // CONTEXT3DSTENCILACTION_H
