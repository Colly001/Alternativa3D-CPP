#include "pixel_shader_data.h"



/*
 * TextureMaterialFragmentShader
 * agal 0edd144fc71941450acca430eb11e70a
 * agal 676a8488359e491902e6adbced5531ff
 * agal c69eb4b6e7fd8340775a9194b487e85b
 * agal 4671252066de3f39d6ef75bb64fe18e6

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     add r1.w, r0, c18
   1:     rcp r8, r1.w
   2:     mul r0.xyz, r0, r8
   3:     mov oC0, r0

// approximately 5 instruction slots used (1 texture, 4 arithmetic)
*/



const uint32_t pixel_shader_data_650adeb7f50934a193ba98636c75e44a[] =
{
	0xFFFF0201,

	//dcl t0
	0x0200001F, 0x80000000, 0xB00F0000,

	//dcl_2d s0
	0x0200001F, 0x90000000, 0xA00F0800,

	//texld r0, t0, s0
	0x03000042, 0x800F0000, 0xB0E40000, 0xA0E40800,

	//add r1.w, r0, c18
	0x03000002, 0x80080001, 0x80E40000, 0xA0E40012,

	//rcp r8, r1.w
	0x02000006, 0x800F0008, 0x80FF0001,

	//mul r0.xyz, r0, r8
	0x03000005, 0x80070000, 0x80E40000, 0x80E40008,

	//mov oC0, r0
	0x02000001, 0x800F0800, 0x80E40000,

	0x0000FFFF,
};

















/*
 * TextureMaterialFragmentShader
 * agal a733cf7dd0f6283e87f682bf23e5f2ad
 * agal 4eda3792cdb11f7b20bd6c06b5718232

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     mul r0.xyz, r0, c0
   1:     add r0.xyz, r0, c1
   2:     mov oC0, r0

// approximately 4 instruction slots used (1 texture, 3 arithmetic)
*/



const uint32_t pixel_shader_data_14a8897639c6c26582c88dd45abd00c6[] =
{
	0xFFFF0201,

	//dcl t0
	0x0200001F, 0x80000000, 0xB00F0000,

	//dcl_2d s0
	0x0200001F, 0x90000000, 0xA00F0800,

	//texld r0, t0, s0
	0x03000042, 0x800F0000, 0xB0E40000, 0xA0E40800,

	//mul r0.xyz, r0, c0
	0x03000005, 0x80070000, 0x80E40000, 0xA0E40000,

	//add r0.xyz, r0, c1
	0x03000002, 0x80070000, 0x80E40000, 0xA0E40001,

	//mov oC0, r0
	0x02000001, 0x800F0800, 0x80E40000,

	0x0000FFFF,
};












/*
 * TextureMaterialFragmentShader
 * agal 41091e139e2039643e4c2d8e98c9be06
 * agal 60516d5939d0790c4dd9ecbd2b24453e

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     add r1.w, r0, c18
   1:     rcp r8, r1.w
   2:     mul r0.xyz, r0, r8
   3:     mul r0.xyz, r0, c0
   4:     add r0.xyz, r0, c1
   5:     mul r0.w, r0, c0
   6:     mov oC0, r0

// approximately 8 instruction slots used (1 texture, 7 arithmetic)
*/



const uint32_t pixel_shader_data_457588a66c127c111fa4bbe7b10a4124[] =
{
	0xFFFF0201,

	//dcl t0
	0x0200001F, 0x80000000, 0xB00F0000,

	//dcl_2d s0
	0x0200001F, 0x90000000, 0xA00F0800,

	//texld r0, t0, s0
	0x03000042, 0x800F0000, 0xB0E40000, 0xA0E40800,

	//add r1.w, r0, c18
	0x03000002, 0x80080001, 0x80E40000, 0xA0E40012,

	//rcp r8, r1.w
	0x02000006, 0x800F0008, 0x80FF0001,

	//mul r0.xyz, r0, r8
	0x03000005, 0x80070000, 0x80E40000, 0x80E40008,

	//mul r0.xyz, r0, c0
	0x03000005, 0x80070000, 0x80E40000, 0xA0E40000,

	//add r0.xyz, r0, c1
	0x03000002, 0x80070000, 0x80E40000, 0xA0E40001,

	//mul r0.w, r0, c0
	0x03000005, 0x80080000, 0x80E40000, 0xA0E40000,

	//mov oC0, r0
	0x02000001, 0x800F0800, 0x80E40000,

	0x0000FFFF,
};


















/*
 * TextureMaterialFragmentShader
 * agal 7dfbd7dde4e4f8c17a95676d30087179
 * agal faa50d6e21674f23b6c2d12bcf6bb3bd

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     mov oC0, r0

// approximately 2 instruction slots used (1 texture, 1 arithmetic)
*/



const uint32_t pixel_shader_data_3bcc9b53d244b3fd7c6589300dbf633e[] =
{
	0xFFFF0201,
	0x0200001F,
	0x80000000,
	0xB00F0000,
	0x0200001F,
	0x90000000,
	0xA00F0800,
	0x03000042,
	0x800F0000,
	0xB0E40000,
	0xA0E40800,
	0x02000001,
	0x800F0800,
	0x80E40000,
	0x0000FFFF,
};











/*
 * TextureMaterialFragmentShader
 * agal ebe149c6e791b710f1dec0aa0d150226

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     add r1.w, r0, c18
   1:     rcp r8, r1.w
   2:     mul r0.xyz, r0, r8
   3:     abs r1, t0.z
   4:     mov_sat r1, r1
   5:     add r1, c17, -r1
   6:     mul r0.w, r0, r1
   7:     mov oC0, r0

// approximately 9 instruction slots used (1 texture, 8 arithmetic)
*/



const uint32_t pixel_shader_data_faace72a085295b25cb279e86ccd0030[] =
{
	0xFFFF0201,
	//dcl t0
	0x0200001F, 0x80000000, 0xB00F0000,
	//dcl_2d s0
	0x0200001F, 0x90000000, 0xA00F0800,
	//texld r0, t0, s0
	0x03000042, 0x800F0000, 0xB0E40000, 0xA0E40800,
	//add r1.w, r0, c18
	0x03000002, 0x80080001, 0x80E40000, 0xA0E40012,
	//rcp r8, r1.w
	0x02000006, 0x800F0008, 0x80FF0001,
	//mul r0.xyz, r0, r8
	0x03000005, 0x80070000, 0x80E40000, 0x80E40008,
	//abs r1, t0.z
	0x02000023, 0x800F0001, 0xB0AA0000,
	//mov_sat r1, r1
	0x02000001, 0x801F0001, 0x80E40001,
	//add r1, c17, -r1
	0x03000002, 0x800F0001, 0xA0E40011, 0x81E40001,
	//mul r0.w, r0, r1
	0x03000005, 0x80080000, 0x80E40000, 0x80E40001,
	//mov oC0, r0
	0x02000001, 0x800F0800, 0x80E40000,

	0x0000FFFF,
};















/*
 * TextureMaterialFragmentShader
 * agal e087d5eee59062b88aa303acc8c363c1
 * agal 7a752f800b660aa3caf8d44b7dab9559
 * agal 331a5fe54e97dc47a53a9c1903817aef

	ps_2_x
		  dcl t0
		  dcl_2d s0
   0:     texld r0, t0, s0
   0:     add r1.w, r0, c18
   1:     rcp r8, r1.w
   2:     mul r0.xyz, r0, r8
   3:     mul r0.w, r0, c0
   4:     mov oC0, r0

// approximately 6 instruction slots used (1 texture, 5 arithmetic)
*/



const uint32_t pixel_shader_data_46cad4619021b137f363804d26dac5c1[] =
{
	0xFFFF0201,
	0x0200001F,
	0x80000000,
	0xB00F0000,
	0x0200001F,
	0x90000000,
	0xA00F0800,
	0x03000042,
	0x800F0000,
	0xB0E40000,
	0xA0E40800,
	0x03000002,
	0x80080001,
	0x80E40000,
	0xA0E40012,
	0x02000006,
	0x800F0008,
	0x80FF0001,
	0x03000005,
	0x80070000,
	0x80E40000,
	0x80E40008,
	0x03000005,
	0x80080000,
	0x80E40000,
	0xA0E40000,
	0x02000001,
	0x800F0800,
	0x80E40000,
	0x0000FFFF,
};


































/*
 * AnimatedPaintFragmentShader
 * agal 1d16d6cb5e404177da711c9459381be7

	ps_2_x
		  dcl t0
		  dcl t6
		  dcl_2d s0
		  dcl_2d s4
		  dcl_2d s6
		  def c28, 0, 1, 0, 0
   0:     rcp r8.x, c24.x
   1:     rcp r8.y, c24.y
   2:     rcp r8.z, c24.z
   3:     rcp r8.w, c24.w
   4:     mul r4, t6, r8
   5:     frc r4, r4
   6:     mul r4, r4, c24
   7:     add r4.x, r4.x, c24.z
   8:     add r4.y, r4.y, c24.w
   9:     texld r0, r4, s4
   9:     texld r1, t0, s6
   9:     mul r2, r0, r1
  10:     mul r2, c23.w, r2
  11:     add r3, c23.z, -r0
  12:     add r4, c23.z, -r1
  13:     mul r3, c23.w, r3
  14:     mul r3, r3, r4
  15:     add r3, c23.z, -r3
  16:     setp_ge p0, r1.x, c23.z
  17:     (!p0) mov r4.x, c28.x
  19:     (p0) mov r4.x, c28.y
  21:     add r4.y, c23.z, -r4.x
  22:     mul r5, r4.x, r3
  23:     mul r6, r4.y, r2
  24:     add r0, r5, r6
  25:     mov r0.w, r1.w
  26:     texld r1, t0, s0
  26:     add r2.w, c23.z, -r1.w
  27:     mul r0.x, r0.x, r2.w
  28:     mul r0.y, r0.y, r2.w
  29:     mul r0.z, r0.z, r2.w
  30:     add r0.xyz, r0.xyzz, r1.xyzz
  31:     mov r1, c23.x
  32:     mov r2, c23.x
  33:     mov r3, c23.x
  34:     mov r4, c23.x
  35:     mov r5, c23.x
  36:     mov r6, c23.x
  37:     mov oC0, r0

// approximately 41 instruction slots used (3 texture, 38 arithmetic)
*/



const uint32_t pixel_shader_data_1dcb657be5d54bd81e94cf5fecdf7366[] =
{
	0xFFFF0201,
	0x0200001F,
	0x80000000,
	0xB00F0000,
	0x0200001F,
	0x80000000,
	0xB00F0006,
	0x0200001F,
	0x90000000,
	0xA00F0800,
	0x0200001F,
	0x90000000,
	0xA00F0804,
	0x0200001F,
	0x90000000,
	0xA00F0806,
	0x05000051,
	0xA00F001C,
	0x00000000,
	0x3F800000,
	0x00000000,
	0x00000000,
	0x02000006,
	0x80010008,
	0xA0000018,
	0x02000006,
	0x80020008,
	0xA0550018,
	0x02000006,
	0x80040008,
	0xA0AA0018,
	0x02000006,
	0x80080008,
	0xA0FF0018,
	0x03000005,
	0x800F0004,
	0xB0E40006,
	0x80E40008,
	0x02000013,
	0x800F0004,
	0x80E40004,
	0x03000005,
	0x800F0004,
	0x80E40004,
	0xA0E40018,
	0x03000002,
	0x80010004,
	0x80000004,
	0xA0AA0018,
	0x03000002,
	0x80020004,
	0x80550004,
	0xA0FF0018,
	0x03000042,
	0x800F0000,
	0x80E40004,
	0xA0E40804,
	0x03000042,
	0x800F0001,
	0xB0E40000,
	0xA0E40806,
	0x03000005,
	0x800F0002,
	0x80E40000,
	0x80E40001,
	0x03000005,
	0x800F0002,
	0xA0FF0017,
	0x80E40002,
	0x03000002,
	0x800F0003,
	0xA0AA0017,
	0x81E40000,
	0x03000002,
	0x800F0004,
	0xA0AA0017,
	0x81E40001,
	0x03000005,
	0x800F0003,
	0xA0FF0017,
	0x80E40003,
	0x03000005,
	0x800F0003,
	0x80E40003,
	0x80E40004,
	0x03000002,
	0x800F0003,
	0xA0AA0017,
	0x81E40003,
	0x0303005E,
	0xB00F1000,
	0x80000001,
	0xA0AA0017,
	0x13000001,
	0x80010004,
	0xBDE41000,
	0xA000001C,
	0x13000001,
	0x80010004,
	0xB0E41000,
	0xA055001C,
	0x03000002,
	0x80020004,
	0xA0AA0017,
	0x81000004,
	0x03000005,
	0x800F0005,
	0x80000004,
	0x80E40003,
	0x03000005,
	0x800F0006,
	0x80550004,
	0x80E40002,
	0x03000002,
	0x800F0000,
	0x80E40005,
	0x80E40006,
	0x02000001,
	0x80080000,
	0x80FF0001,
	0x03000042,
	0x800F0001,
	0xB0E40000,
	0xA0E40800,
	0x03000002,
	0x80080002,
	0xA0AA0017,
	0x81FF0001,
	0x03000005,
	0x80010000,
	0x80000000,
	0x80FF0002,
	0x03000005,
	0x80020000,
	0x80550000,
	0x80FF0002,
	0x03000005,
	0x80040000,
	0x80AA0000,
	0x80FF0002,
	0x03000002,
	0x80070000,
	0x80A40000,
	0x80A40001,
	0x02000001,
	0x800F0001,
	0xA0000017,
	0x02000001,
	0x800F0002,
	0xA0000017,
	0x02000001,
	0x800F0003,
	0xA0000017,
	0x02000001,
	0x800F0004,
	0xA0000017,
	0x02000001,
	0x800F0005,
	0xA0000017,
	0x02000001,
	0x800F0006,
	0xA0000017,
	0x02000001,
	0x800F0800,
	0x80E40000,
	0x0000FFFF,
};











/*
 * AnimatedPaintFragmentShader
 * agal 31abf15878b758df428e625275afa66b

	ps_2_x
		  dcl t0
		  dcl t6
		  dcl_2d s0
		  dcl_2d s4
		  dcl_2d s6
		  def c28, 0, 1, 0, 0
   0:     rcp r8.x, c24.x
   1:     rcp r8.y, c24.y
   2:     rcp r8.z, c24.z
   3:     rcp r8.w, c24.w
   4:     mul r4, t6, r8
   5:     frc r4, r4
   6:     mul r4, r4, c24
   7:     add r4.x, r4.x, c24.z
   8:     add r4.y, r4.y, c24.w
   9:     texld r0, r4, s4
   9:     texld r1, t0, s6
   9:     mul r2, r0, r1
  10:     mul r2, c23.w, r2
  11:     add r3, c23.z, -r0
  12:     add r4, c23.z, -r1
  13:     mul r3, c23.w, r3
  14:     mul r3, r3, r4
  15:     add r3, c23.z, -r3
  16:     setp_ge p0, r1.x, c23.z
  17:     (!p0) mov r4.x, c28.x
  19:     (p0) mov r4.x, c28.y
  21:     add r4.y, c23.z, -r4.x
  22:     mul r5, r4.x, r3
  23:     mul r6, r4.y, r2
  24:     add r0, r5, r6
  25:     mov r0.w, r1.w
  26:     texld r1, t0, s0
  26:     add r2.w, c23.z, -r1.w
  27:     mul r0.x, r0.x, r2.w
  28:     mul r0.y, r0.y, r2.w
  29:     mul r0.z, r0.z, r2.w
  30:     add r0.xyz, r0.xyzz, r1.xyzz
  31:     mov r1, c23.x
  32:     mov r2, c23.x
  33:     mov r3, c23.x
  34:     mov r4, c23.x
  35:     mov r5, c23.x
  36:     mov r6, c23.x
  37:     mul r0.xyz, r0, c0
  38:     add r0.xyz, r0, c1
  39:     mov oC0, r0

// approximately 43 instruction slots used (3 texture, 40 arithmetic)
*/



const uint32_t pixel_shader_data_4bee2b142a34e554277e7bfd64f994ed[] =
{
	0xFFFF0201,
	0x0200001F,
	0x80000000,
	0xB00F0000,
	0x0200001F,
	0x80000000,
	0xB00F0006,
	0x0200001F,
	0x90000000,
	0xA00F0800,
	0x0200001F,
	0x90000000,
	0xA00F0804,
	0x0200001F,
	0x90000000,
	0xA00F0806,
	0x05000051,
	0xA00F001C,
	0x00000000,
	0x3F800000,
	0x00000000,
	0x00000000,
	0x02000006,
	0x80010008,
	0xA0000018,
	0x02000006,
	0x80020008,
	0xA0550018,
	0x02000006,
	0x80040008,
	0xA0AA0018,
	0x02000006,
	0x80080008,
	0xA0FF0018,
	0x03000005,
	0x800F0004,
	0xB0E40006,
	0x80E40008,
	0x02000013,
	0x800F0004,
	0x80E40004,
	0x03000005,
	0x800F0004,
	0x80E40004,
	0xA0E40018,
	0x03000002,
	0x80010004,
	0x80000004,
	0xA0AA0018,
	0x03000002,
	0x80020004,
	0x80550004,
	0xA0FF0018,
	0x03000042,
	0x800F0000,
	0x80E40004,
	0xA0E40804,
	0x03000042,
	0x800F0001,
	0xB0E40000,
	0xA0E40806,
	0x03000005,
	0x800F0002,
	0x80E40000,
	0x80E40001,
	0x03000005,
	0x800F0002,
	0xA0FF0017,
	0x80E40002,
	0x03000002,
	0x800F0003,
	0xA0AA0017,
	0x81E40000,
	0x03000002,
	0x800F0004,
	0xA0AA0017,
	0x81E40001,
	0x03000005,
	0x800F0003,
	0xA0FF0017,
	0x80E40003,
	0x03000005,
	0x800F0003,
	0x80E40003,
	0x80E40004,
	0x03000002,
	0x800F0003,
	0xA0AA0017,
	0x81E40003,
	0x0303005E,
	0xB00F1000,
	0x80000001,
	0xA0AA0017,
	0x13000001,
	0x80010004,
	0xBDE41000,
	0xA000001C,
	0x13000001,
	0x80010004,
	0xB0E41000,
	0xA055001C,
	0x03000002,
	0x80020004,
	0xA0AA0017,
	0x81000004,
	0x03000005,
	0x800F0005,
	0x80000004,
	0x80E40003,
	0x03000005,
	0x800F0006,
	0x80550004,
	0x80E40002,
	0x03000002,
	0x800F0000,
	0x80E40005,
	0x80E40006,
	0x02000001,
	0x80080000,
	0x80FF0001,
	0x03000042,
	0x800F0001,
	0xB0E40000,
	0xA0E40800,
	0x03000002,
	0x80080002,
	0xA0AA0017,
	0x81FF0001,
	0x03000005,
	0x80010000,
	0x80000000,
	0x80FF0002,
	0x03000005,
	0x80020000,
	0x80550000,
	0x80FF0002,
	0x03000005,
	0x80040000,
	0x80AA0000,
	0x80FF0002,
	0x03000002,
	0x80070000,
	0x80A40000,
	0x80A40001,
	0x02000001,
	0x800F0001,
	0xA0000017,
	0x02000001,
	0x800F0002,
	0xA0000017,
	0x02000001,
	0x800F0003,
	0xA0000017,
	0x02000001,
	0x800F0004,
	0xA0000017,
	0x02000001,
	0x800F0005,
	0xA0000017,
	0x02000001,
	0x800F0006,
	0xA0000017,
	0x03000005,
	0x80070000,
	0x80E40000,
	0xA0E40000,
	0x03000002,
	0x80070000,
	0x80E40000,
	0xA0E40001,
	0x02000001,
	0x800F0800,
	0x80E40000,
	0x0000FFFF,
};











/*
 * AnimatedPaintFragmentShader
 * agal ae19ef9604af61a0b30ca2174d2eecf1

	ps_2_x
		  dcl t0
		  dcl t6
		  dcl_2d s0
		  dcl_2d s4
		  dcl_2d s6
		  def c28, 0, 1, 0, 0
   0:     rcp r8.x, c24.x
   1:     rcp r8.y, c24.y
   2:     rcp r8.z, c24.z
   3:     rcp r8.w, c24.w
   4:     mul r4, t6, r8
   5:     frc r4, r4
   6:     mul r4, r4, c24
   7:     add r4.x, r4.x, c24.z
   8:     add r4.y, r4.y, c24.w
   9:     texld r0, r4, s4
   9:     texld r1, t0, s6
   9:     mul r2, r0, r1
  10:     mul r2, c23.w, r2
  11:     add r3, c23.z, -r0
  12:     add r4, c23.z, -r1
  13:     mul r3, c23.w, r3
  14:     mul r3, r3, r4
  15:     add r3, c23.z, -r3
  16:     setp_ge p0, r1.x, c23.z
  17:     (!p0) mov r4.x, c28.x
  19:     (p0) mov r4.x, c28.y
  21:     add r4.y, c23.z, -r4.x
  22:     mul r5, r4.x, r3
  23:     mul r6, r4.y, r2
  24:     add r0, r5, r6
  25:     mov r0.w, r1.w
  26:     texld r1, t0, s0
  26:     add r2.w, c23.z, -r1.w
  27:     mul r0.x, r0.x, r2.w
  28:     mul r0.y, r0.y, r2.w
  29:     mul r0.z, r0.z, r2.w
  30:     add r0.xyz, r0.xyzz, r1.xyzz
  31:     mov r1, c23.x
  32:     mov r2, c23.x
  33:     mov r3, c23.x
  34:     mov r4, c23.x
  35:     mov r5, c23.x
  36:     mov r6, c23.x
  37:     add r1.w, r0, c18
  38:     rcp r8, r1.w
  39:     mul r0.xyz, r0, r8
  40:     mul r0.xyz, r0, c0
  41:     add r0.xyz, r0, c1
  42:     mul r0.w, r0, c0
  43:     mov oC0, r0

// approximately 47 instruction slots used (3 texture, 44 arithmetic)
*/



const uint32_t pixel_shader_data_5d86f208206705904b8adf6ecf8a0c76[] =
{
	0xFFFF0201,
	//dcl t0
	0x0200001F, 0x80000000, 0xB00F0000,
	//dcl t6
	0x0200001F, 0x80000000, 0xB00F0006,
	//dcl_2d s0
	0x0200001F, 0x90000000, 0xA00F0800,
	//dcl_2d s4
	0x0200001F, 0x90000000, 0xA00F0804,
	//dcl_2d s6
	0x0200001F, 0x90000000, 0xA00F0806,
	//def c28, 0, 1, 0, 0
	0x05000051, 0xA00F001C, 0x00000000, 0x3F800000,0x00000000, 0x00000000,
	//rcp r8.x, c24.x
	0x02000006, 0x80010008, 0xA0000018,
	//rcp r8.y, c24.y
	0x02000006, 0x80020008, 0xA0550018,
	//rcp r8.z, c24.z
	0x02000006, 0x80040008, 0xA0AA0018,
	//rcp r8.w, c24.w
	0x02000006, 0x80080008, 0xA0FF0018,
	//mul r4, t6, r8
	0x03000005, 0x800F0004, 0xB0E40006, 0x80E40008,
	//frc r4, r4
	0x02000013, 0x800F0004, 0x80E40004,
	//mul r4, r4, c24
	0x03000005, 0x800F0004, 0x80E40004, 0xA0E40018,
	//add r4.x, r4.x, c24.z
	0x03000002, 0x80010004, 0x80000004, 0xA0AA0018,
	//add r4.y, r4.y, c24.w
	0x03000002, 0x80020004, 0x80550004, 0xA0FF0018,
	//texld r0, r4, s4
	0x03000042, 0x800F0000, 0x80E40004, 0xA0E40804,
	//texld r1, t0, s6
	0x03000042, 0x800F0001, 0xB0E40000, 0xA0E40806,
	//mul r2, r0, r1
	0x03000005, 0x800F0002, 0x80E40000, 0x80E40001,
	//mul r2, c23.w, r2
	0x03000005, 0x800F0002, 0xA0FF0017, 0x80E40002,
	//add r3, c23.z, -r0
	0x03000002, 0x800F0003, 0xA0AA0017, 0x81E40000,
	//add r4, c23.z, -r1
	0x03000002, 0x800F0004, 0xA0AA0017, 0x81E40001,
	//mul r3, c23.w, r3
	0x03000005, 0x800F0003, 0xA0FF0017, 0x80E40003,
	//mul r3, r3, r4
	0x03000005, 0x800F0003, 0x80E40003, 0x80E40004,
	//add r3, c23.z, -r3
	0x03000002, 0x800F0003, 0xA0AA0017, 0x81E40003,
	//setp_ge p0, r1.x, c23.z
	0x0303005E, 0xB00F1000, 0x80000001, 0xA0AA0017,
	//(!p0) mov r4.x, c28.x
	0x13000001, 0x80010004, 0xBDE41000, 0xA000001C,
	//(p0) mov r4.x, c28.y
	0x13000001, 0x80010004, 0xB0E41000, 0xA055001C,
	//add r4.y, c23.z, -r4.x
	0x03000002, 0x80020004, 0xA0AA0017, 0x81000004,
	//mul r5, r4.x, r3
	0x03000005, 0x800F0005, 0x80000004, 0x80E40003,
	//mul r6, r4.y, r2
	0x03000005, 0x800F0006, 0x80550004, 0x80E40002,
	//add r0, r5, r6
	0x03000002, 0x800F0000, 0x80E40005, 0x80E40006,
	//mov r0.w, r1.w
	0x02000001, 0x80080000, 0x80FF0001,
	//texld r1, t0, s0
	0x03000042, 0x800F0001, 0xB0E40000, 0xA0E40800,
	//add r2.w, c23.z, -r1.w
	0x03000002, 0x80080002, 0xA0AA0017, 0x81FF0001,
	//mul r0.x, r0.x, r2.w
	0x03000005, 0x80010000, 0x80000000, 0x80FF0002,
	//mul r0.y, r0.y, r2.w
	0x03000005, 0x80020000, 0x80550000, 0x80FF0002,
	//mul r0.z, r0.z, r2.w
	0x03000005, 0x80040000, 0x80AA0000, 0x80FF0002,
	//add r0.xyz, r0.xyzz, r1.xyzz
	0x03000002, 0x80070000, 0x80A40000, 0x80A40001,
	//mov r1, c23.x
	0x02000001, 0x800F0001, 0xA0000017,
	//mov r2, c23.x
	0x02000001, 0x800F0002, 0xA0000017,
	//mov r3, c23.x
	0x02000001, 0x800F0003, 0xA0000017,
	//mov r4, c23.x
	0x02000001, 0x800F0004, 0xA0000017,
	//mov r5, c23.x
	0x02000001, 0x800F0005, 0xA0000017,
	//mov r6, c23.x
	0x02000001, 0x800F0006, 0xA0000017,
	//add r1.w, r0, c18
	0x03000002, 0x80080001, 0x80E40000, 0xA0E40012,
	//rcp r8, r1.w
	0x02000006, 0x800F0008, 0x80FF0001,
	//mul r0.xyz, r0, r8
	0x03000005, 0x80070000, 0x80E40000, 0x80E40008,
	//mul r0.xyz, r0, c0
	0x03000005, 0x80070000, 0x80E40000, 0xA0E40000,
	//add r0.xyz, r0, c1
	0x03000002, 0x80070000, 0x80E40000,0xA0E40001,
	//mul r0.w, r0, c0
	0x03000005, 0x80080000, 0x80E40000, 0xA0E40000,
	//mov oC0, r0
	0x02000001, 0x800F0800, 0x80E40000,

	0x0000FFFF,
};


