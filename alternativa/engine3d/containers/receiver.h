#ifndef RECEIVER_H
#define RECEIVER_H

#include <memory>

class Receiver
{
public:
	std::shared_ptr<Receiver> next;
	bool transparent;
	int buffer;
	int firstIndex;
	int numTriangles;

	Receiver();
};

#endif // RECEIVER_H
