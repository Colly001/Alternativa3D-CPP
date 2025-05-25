#ifndef ITEXTURECONSTRUCTORLISTENER_H
#define ITEXTURECONSTRUCTORLISTENER_H

class TextureConstructor;

class ITextureConstructorListener
{
public:
	 virtual void onTextureReady(TextureConstructor*) = 0;
};

#endif // ITEXTURECONSTRUCTORLISTENER_H
