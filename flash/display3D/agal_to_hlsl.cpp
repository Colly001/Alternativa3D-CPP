#include "agal_to_hlsl.h"
#include "hash.h"
#include <QHash>
#include "HLSL/vertex_shader_data.h"
#include "HLSL/pixel_shader_data.h"


namespace
{
	class Convert
	{
		QHash<QByteArray, const DWORD*> hash;
	public:

		void add(const char *md5_agal, const uint32_t *code)
		{
			QByteArray md5;
			md5.setRawData(md5_agal, 32);
			hash.insert(md5, (const DWORD*)code);
		}

		Convert()
		{
			//TextureMaterialVertexShader

			add("e4a38fa4402b4879ed15036dce992a19", vertex_shader_data_c8006d829bbcbfeebf35a8fd5102c7fd);

			//TextureMaterialVertexShader TrackMaterialVertexShader
			add("53183f3551989a7c9078c7b36225abe8", vertex_shader_data_07e343cc8143f84da088eee109997bc9);

			//TextureMaterialVertexShader
			add("ccdb5d00d7eb87cc2cd8894d8f8f5aa3", vertex_shader_data_542b2f5f777cac375c47325be7d3d4fa);

			add("6d41c16b13630d3b2597b360efacabeb", vertex_shader_data_ec494106e8e3300c4e5797bfcd837f41);

			//PaintVertexShader

			add("fe1ae7c7d183777d8ac60ace6981c626", vertex_shader_data_60a356d56cfedc55f04f75f5648a281b);

			add("faaa9a1e99604a96ee32f890ea093a7d", vertex_shader_data_23fc30008fb031c1f2e19ff8f0ea23de);

			//TrackMaterialVertexShader
			add("0549248f494925814849dc82fad19e6d", vertex_shader_data_c95c4b989fd4927dba7168f0c9e5bbf3);

			//TextureMaterialFragmentShader

			add("0edd144fc71941450acca430eb11e70a", pixel_shader_data_650adeb7f50934a193ba98636c75e44a);
			add("676a8488359e491902e6adbced5531ff", pixel_shader_data_650adeb7f50934a193ba98636c75e44a);
			add("c69eb4b6e7fd8340775a9194b487e85b", pixel_shader_data_650adeb7f50934a193ba98636c75e44a);
			add("4671252066de3f39d6ef75bb64fe18e6", pixel_shader_data_650adeb7f50934a193ba98636c75e44a);

			add("a733cf7dd0f6283e87f682bf23e5f2ad", pixel_shader_data_14a8897639c6c26582c88dd45abd00c6);
			add("4eda3792cdb11f7b20bd6c06b5718232", pixel_shader_data_14a8897639c6c26582c88dd45abd00c6);

			add("41091e139e2039643e4c2d8e98c9be06", pixel_shader_data_457588a66c127c111fa4bbe7b10a4124);
			add("60516d5939d0790c4dd9ecbd2b24453e", pixel_shader_data_457588a66c127c111fa4bbe7b10a4124);

			add("7dfbd7dde4e4f8c17a95676d30087179", pixel_shader_data_3bcc9b53d244b3fd7c6589300dbf633e);
			add("faa50d6e21674f23b6c2d12bcf6bb3bd", pixel_shader_data_3bcc9b53d244b3fd7c6589300dbf633e);

			add("ebe149c6e791b710f1dec0aa0d150226", pixel_shader_data_faace72a085295b25cb279e86ccd0030);

			add("e087d5eee59062b88aa303acc8c363c1", pixel_shader_data_46cad4619021b137f363804d26dac5c1);
			add("7a752f800b660aa3caf8d44b7dab9559", pixel_shader_data_46cad4619021b137f363804d26dac5c1);
			add("331a5fe54e97dc47a53a9c1903817aef", pixel_shader_data_46cad4619021b137f363804d26dac5c1);

			//AnimatedPaintFragmentShader
			add("1d16d6cb5e404177da711c9459381be7", pixel_shader_data_1dcb657be5d54bd81e94cf5fecdf7366);

			add("31abf15878b758df428e625275afa66b", pixel_shader_data_4bee2b142a34e554277e7bfd64f994ed);

			add("ae19ef9604af61a0b30ca2174d2eecf1", pixel_shader_data_5d86f208206705904b8adf6ecf8a0c76);
		}

		const DWORD* get(const QByteArray &md5)
		{
			return hash.value(md5);
		}

	} convert;
}

const DWORD* AGAL_TO_HLSL(const QByteArray &agal)
{
	QByteArray md5_agal = Hash::md5(agal);
	const DWORD *code = convert.get(md5_agal);
	return code;
}
