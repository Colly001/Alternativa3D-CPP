#ifndef AGALTEST_H
#define AGALTEST_H

#include <QHash>

class AgalTest
{
	char const *current;
	QHash<uint32_t, const char*> hash;
public:

	void addData(const char *data);
	void addKey(uint32_t k);

	bool check(uint32_t k, const QByteArray &data);
};

#endif // AGALTEST_H
