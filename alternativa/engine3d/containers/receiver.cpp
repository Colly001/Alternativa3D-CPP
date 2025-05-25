#include "receiver.h"

Receiver::Receiver()
{
	next = nullptr;
	transparent = false;
	buffer = -1;
	firstIndex = -1;
	numTriangles = 0;
}
