#ifndef IMAGENODES_H
#define IMAGENODES_H

class ImageItem;

class ImageNodes
{
public:

	ImageItem *first;
	ImageItem *last;

	ImageNodes();


	void add(ImageItem *item_new);
	void remove(ImageItem *node);
};

#endif // IMAGENODES_H
