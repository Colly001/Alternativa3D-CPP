#ifndef CONTEXT3DMIPFILTER_H
#define CONTEXT3DMIPFILTER_H

enum class Context3DMipFilter
{
	MIPLINEAR, //miplinear Выбираются два ближайших MIP-уровня, и производится их линейное наложение друг на друга (режим наилучшего качества, но несколько затратный по производительности).
	MIPNEAREST, //mipnearest Для отбора MIP-уровней применяется метрика «ближайшего соседа» (наиболее быстрый режим визуализации).
	MIPNONE, //mipnone Всегда используется текстура верхнего уровня (режим потребляет много ресурсов при уменьшении разрешения текстур).
};

#endif // CONTEXT3DMIPFILTER_H
