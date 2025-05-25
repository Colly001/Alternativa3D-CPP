#ifndef CONTEXT3DTEXTUREFORMAT_H
#define CONTEXT3DTEXTUREFORMAT_H

enum class Context3DTextureFormat
{
	BGRA, //"bgra"
	BGRA_PACKED, //"bgraPacked4444" 16 бит, формат bgra, упакованный в качестве 4:4:4:4
	BGR_PACKED, //"bgrPacked565" 6 бит, формат bgr, упакованный в качестве 5:6:5
	COMPRESSED, //"compressed"
	COMPRESSED_ALPHA, //"compressedAlpha"
	RGBA_HALF_FLOAT, //"rgbaHalfFloat"
};

#endif // CONTEXT3DTEXTUREFORMAT_H
