#include "gettimer.h"

#ifdef WIN32
#include <windows.h>
int getTimer()
{
	return GetTickCount();
}
#else
#include <ctime>
int getTimer()
{
	return std::clock();
}
#endif
