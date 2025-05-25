#ifndef BYTEARRAYMAP_H
#define BYTEARRAYMAP_H

#include <QHash>
#include <QByteArray>


class ByteArrayMap
{
	QHash<QString, QByteArray> _data;
public:
	ByteArrayMap();
	/*
	public function ByteArrayMap(data:Object=null)
			{
				this._data = ((data == null) ? {} : data);
			}
			*/

	QHash<QString, QByteArray> getData();
	void setData(QHash<QString, QByteArray> &data);
	QByteArray getValue(const QString &key);
	void putValue(const QString &key, const QByteArray &value);
	void destroy();
};

#endif // BYTEARRAYMAP_H
