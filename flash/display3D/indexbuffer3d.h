#ifndef INDEXBUFFER3D_H
#define INDEXBUFFER3D_H

#include <QVector>

class IDirect3DDevice9Ex;
class IDirect3DIndexBuffer9;

class IndexBuffer3D
{
	IDirect3DDevice9Ex *device;
	IDirect3DIndexBuffer9 *pIndexBuffer;
public:
	IndexBuffer3D(IDirect3DDevice9Ex *device, IDirect3DIndexBuffer9 *pIndexBuffer);


	IDirect3DIndexBuffer9* get() { return pIndexBuffer; }


	//Очистите все собственные ресурсы графического процессора, связанные с этим объектом.
	void dispose();

	//Сохраняйте в индексах вершин графической подсистемы.
	//void uploadFromByteArray(ByteArray data, int byteArrayOffset, int startOffset, int count);

	//Сохраняйте в индексах вершин графической подсистемы.
	void uploadFromVector(const QVector<uint32_t> &data, int startOffset, int count);
	void uploadFromVector(const uint16_t *data, int startOffset, int count);
};

#endif // INDEXBUFFER3D_H
