#ifndef TARAPARSER_H
#define TARAPARSER_H

#include <memory>

class ByteArrayMap;
class QByteArray;

std::shared_ptr<ByteArrayMap> TARAParse(const QByteArray &data);

#endif // TARAPARSER_H
