#include "agaltest.h"
#include <QDebug>

void AgalTest::addData(const char *data)
{
	current = data;
}

void AgalTest::addKey(uint32_t k)
{
	hash.insert(k, current);
}

bool AgalTest::check(uint32_t k, const QByteArray &data)
{
	const char *d = hash.value(k);
	if (d == nullptr)
		return false;
	if (d == data)
		return true;

	//qDebug("AgalTest::check error %08X", k);
	//qDebug() << "happened" << data;
	//qDebug() << "required" << QString::fromLocal8Bit(d);

	return false;
}
