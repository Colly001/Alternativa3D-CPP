#ifndef MATERIALENTRY_H
#define MATERIALENTRY_H


#include <QString>
#include <memory>

class BitmapData;
class TextureMaterial;


class MaterialEntry
{
public:
	QString keyDataString;
	std::shared_ptr<BitmapData> keyData;
	std::shared_ptr<BitmapData> texture;
	std::shared_ptr<TextureMaterial> material;
	int referenceCount;

	MaterialEntry(std::shared_ptr<BitmapData>&, std::shared_ptr<TextureMaterial> material);

	MaterialEntry(const QString&, std::shared_ptr<TextureMaterial> material);
};

#endif // MATERIALENTRY_H
