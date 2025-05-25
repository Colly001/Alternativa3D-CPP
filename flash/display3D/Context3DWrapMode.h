#ifndef CONTEXT3DWRAPMODE_H
#define CONTEXT3DWRAPMODE_H

enum class Context3DWrapMode
{
	CLAMP, //clamp Координаты фиксированной текстуры вне диапазона 0–1.
	CLAMP_U_REPEAT_V, //clamp_u_repeat_v Фиксация на оси U, но повтор на оси V.
	REPEAT, //repeat Координаты повторной (мозаичной) текстуры вне диапазона 0–1.
	REPEAT_U_CLAMP_V, //repeat_u_clamp_v Повтор на оси U, но фиксация на оси V.
};

#endif // CONTEXT3DWRAPMODE_H
