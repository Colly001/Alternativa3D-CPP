#ifndef CONTEXT3DBLENDFACTOR_H
#define CONTEXT3DBLENDFACTOR_H

enum class Context3DBlendFactor
{
	DESTINATION_ALPHA, //destinationAlpha Коэффициент наложения — (Da,Da,Da,Da), где Da — это альфа-компонент цвета фрагмента, вычисленного пиксельной программой.
	DESTINATION_COLOR, //destinationColor Коэффициент наложения — (Dr,Dg,Db,Da), где Dr/g/b/a — это соответствующий компонент текущего цвета в буфере цвета.
	ONE,  //one Коэффициент наложения — (1,1,1,1).
	ONE_MINUS_DESTINATION_ALPHA, //oneMinusDestinationAlpha Коэффициент наложения — (1-Da,1-Da,1-Da,1-Da), где Da — это альфа-компонент текущего цвета в буфере цвета.
	ONE_MINUS_DESTINATION_COLOR, //oneMinusDestinationColor Коэффициент наложения — (1-Dr,1-Dg,1-Db,1-Da), где Dr/g/b/a — это соответствующий компонент текущего цвета в буфере цвета.
	ONE_MINUS_SOURCE_ALPHA, //oneMinusSourceAlpha Коэффициент наложения — (1-Sa,1-Sa,1-Sa,1-Sa), где Sa — это альфа-компонент цвета фрагмента, вычисленного пиксельной программой.
	ONE_MINUS_SOURCE_COLOR, //oneMinusSourceColor Коэффициент наложения — (1-Sr,1-Sg,1-Sb,1-Sa), где Sr/g/b/a — это соответствующий компонент цвета фрагмента, вычисленного пиксельной программой.
	SOURCE_ALPHA, //sourceAlpha Коэффициент наложения — (Sa,Sa,Sa,Sa), где Sa — это альфа-компонент цвета фрагмента, вычисленного пиксельной программой.
	SOURCE_COLOR, //sourceColor Коэффициент наложения — (Sr,Sg,Sb,Sa), где Sr/g/b/a — это соответствующий компонент цвета фрагмента, вычисленного пиксельной программой.
	ZERO, //zero Коэффициент наложения — (0,0,0,0).
};

#endif // CONTEXT3DBLENDFACTOR_H
