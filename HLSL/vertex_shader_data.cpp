#include "vertex_shader_data.h"

/*
 * TextureMaterialVertexShader
 * agal e4a38fa4402b4879ed15036dce992a19
 *
	vs_2_x
		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c4
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mul oT0, v1, c4
   5:     mov r12.xw, r0.xzzz
   6:     mov r12.y, -r0
   7:     mul r0.z, r0, c3
   8:     add r12.z, r0, c3.w
   9:     mad oPos, r12.w, c254, r12

   // approximately 10 instruction slots used
*/

const uint32_t vertex_shader_data_c8006d829bbcbfeebf35a8fd5102c7fd[] =
{
	//begin
	0xFFFE0201,
	//dcl_texcoord v0
	0x0200001F, 0x80000005, 0x900F0000,
	//dcl_texcoord1 v1
	0x0200001F, 0x80010005, 0x900F0001,
	//mov r0, c4
	0x02000001, 0x800F0000, 0xA0E40004,
	//dp4 r0.x, v0, c0
	0x03000009, 0x80010000, 0x90E40000, 0xA0E40000,
	//dp4 r0.y, v0, c1
	0x03000009, 0x80020000, 0x90E40000, 0xA0E40001,
	//dp4 r0.z, v0, c2
	0x03000009, 0x80040000, 0x90E40000, 0xA0E40002,
	//mul oT0, v1, c4
	0x03000005, 0xE00F0000,0x90E40001, 0xA0E40004,
	//mov r12.xw, r0.xzzz
	0x02000001, 0x8009000C, 0x80A80000,
	//mov r12.y, -r0
	0x02000001, 0x8002000C, 0x81E40000,
	//mul r0.z, r0, c3
	0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,
	//add r12.z, r0, c3.w
	0x03000002, 0x8004000C, 0x80E40000, 0xA0FF0003,
	//mad oPos, r12.w, c254, r12
	0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,
	//end
	0x0000FFFF,
};


/*
 * TextureMaterialVertexShader
 * agal 53183f3551989a7c9078c7b36225abe8
 *
 * TrackMaterialVertexShader
 * agal 53183f3551989a7c9078c7b36225abe8


	vs_2_x
		  dcl_texcoord v0
   0:     mov r0, c4
   1:     mova a0.x, v0.x
   2:     mov r0.xyz, c0[a0.x]
   3:     mov r1, c1[a0.x]
   4:     mul oT0, r1, c4
   5:     mov r12.xw, r0.xzzz
   6:     mov r12.y, -r0
   7:     mul r0.z, r0, c3
   8:     add r12.z, r0, c3.w
   9:     mad oPos, r12.w, c254, r12

// approximately 10 instruction slots used
*/


const uint32_t vertex_shader_data_07e343cc8143f84da088eee109997bc9[] =
{
	//begin
	0xFFFE0201,

	//dcl_texcoord v0
	0x0200001F, 0x80000005, 0x900F0000,

	//mov r0, c4
	0x02000001, 0x800F0000, 0xA0E40004,

	//mova a0.x, v0.x
	0x0200002E, 0xB0010000, 0x90000000,

	//mov r0.xyz, c0[a0.x]
	0x03000001, 0x80070000, 0xA0E42000, 0xB0000000,

	//mov r1, c1[a0.x]
	0x03000001, 0x800F0001, 0xA0E42001, 0xB0000000,

	//mul oT0, r1, c4
	0x03000005, 0xE00F0000, 0x80E40001, 0xA0E40004,

	//mov r12.xw, r0.xzzz
	0x02000001, 0x8009000C, 0x80A80000,

	//mov r12.y, -r0
	0x02000001, 0x8002000C, 0x81E40000,

	//mul r0.z, r0, c3
	0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,

	//add r12.z, r0, c3.w
	0x03000002, 0x8004000C, 0x80E40000, 0xA0FF0003,

	//mad oPos, r12.w, c254, r12
	0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,

	//end
	0x0000FFFF,
};





/*
 * TextureMaterialVertexShader
 * agal ccdb5d00d7eb87cc2cd8894d8f8f5aa3

	vs_2_x
		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c4
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mul oT0, v1, c4
   5:     mul r0.xyz, r0, c11
   6:     mov r12.xw, r0.xzzz
   7:     mov r12.y, -r0
   8:     mul r0.z, r0, c3
   9:     add r12.z, r0, c3.w
  10:     mad oPos, r12.w, c254, r12

// approximately 11 instruction slots used
*/


const uint32_t vertex_shader_data_542b2f5f777cac375c47325be7d3d4fa[] =
{
	0xFFFE0201,
	0x0200001F,
	0x80000005,
	0x900F0000,
	0x0200001F,
	0x80010005,
	0x900F0001,
	0x02000001,
	0x800F0000,
	0xA0E40004,
	0x03000009,
	0x80010000,
	0x90E40000,
	0xA0E40000,
	0x03000009,
	0x80020000,
	0x90E40000,
	0xA0E40001,
	0x03000009,
	0x80040000,
	0x90E40000,
	0xA0E40002,
	0x03000005,
	0xE00F0000,
	0x90E40001,
	0xA0E40004,
	0x03000005,
	0x80070000,
	0x80E40000,
	0xA0E4000B,
	0x02000001,
	0x8009000C,
	0x80A80000,
	0x02000001,
	0x8002000C,
	0x81E40000,
	0x03000005,
	0x80040000,
	0x80E40000,
	0xA0E40003,
	0x03000002,
	0x8004000C,
	0x80E40000,
	0xA0FF0003,
	0x04000004,
	0xC00F0000,
	0x80FF000C,
	0xA0E400FE,
	0x80E4000C,
	0x0000FFFF,
};




/*
 * TextureMaterialVertexShader
 *  agal 6d41c16b13630d3b2597b360efacabeb

	vs_2_x
		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c4
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mul oT0, v1, c4
   5:     mov r1, c4
   6:     mov r1, r0
   7:     mul r1.x, r1, c11.w
   8:     mul r1.y, r1, c12.w
   9:     add r1, r1, -c11
  10:     dp3 oT0.z, r1, c12
  11:     rcp r8.x, r0.x
  12:     rcp r8.y, r0.y
  13:     rcp r8.z, r0.z
  14:     rcp r8.w, r0.w
  15:     mul r1.z, c3.w, r8
  16:     add r1.z, r1, c3
  17:     mul r1.z, r1, c3.x
  18:     add r1.z, r1, -c3.y
  19:     rcp r8, c3.x
  20:     mul r1.z, r1, r8
  21:     add r1.z, r1, -c3
  22:     rcp r8.x, r1.x
  23:     rcp r8.y, r1.y
  24:     rcp r8.z, r1.z
  25:     rcp r8.w, r1.w
  26:     mul r1.z, c3.w, r8
  27:     mov r2, c4
  28:     nrm r2.xyz, r0.xyzz
  31:     add r1.z, r0, -r1
  32:     rcp r8.x, r2.x
  33:     rcp r8.y, r2.y
  34:     rcp r8.z, r2.z
  35:     rcp r8.w, r2.w
  36:     mul r1.z, r1, r8
  37:     mul r2, r2, r1.z
  38:     add r0, r0, -r2
  39:     mov r12.xw, r0.xzzz
  40:     mov r12.y, -r0
  41:     mul r0.z, r0, c3
  42:     add r12.z, r0, c3.w
  43:     mad oPos, r12.w, c254, r12

// approximately 44 instruction slots used
*/

const uint32_t vertex_shader_data_ec494106e8e3300c4e5797bfcd837f41[] =
{
	0xFFFE0201,

	//dcl_texcoord v0
	0x0200001F, 0x80000005, 0x900F0000,

	//dcl_texcoord1 v1
	0x0200001F, 0x80010005, 0x900F0001,

	//mov r0, c4
	0x02000001, 0x800F0000, 0xA0E40004,

	//dp4 r0.x, v0, c0
	0x03000009, 0x80010000, 0x90E40000, 0xA0E40000,

	//dp4 r0.y, v0, c1
	0x03000009, 0x80020000, 0x90E40000, 0xA0E40001,

	//dp4 r0.z, v0, c2
	0x03000009, 0x80040000, 0x90E40000, 0xA0E40002,

	//mul oT0, v1, c4
	0x03000005, 0xE00F0000, 0x90E40001, 0xA0E40004,

	//mov r1, c4
	0x02000001, 0x800F0001, 0xA0E40004,

	//mov r1, r0
	0x02000001, 0x800F0001, 0x80E40000,

	//mul r1.x, r1, c11.w
	0x03000005, 0x80010001, 0x80E40001, 0xA0FF000B,

	//mul r1.y, r1, c12.w
	0x03000005, 0x80020001, 0x80E40001, 0xA0FF000C,

	//add r1, r1, -c11
	0x03000002, 0x800F0001, 0x80E40001, 0xA1E4000B,

	//dp3 oT0.z, r1, c12
	0x03000008, 0xE0040000, 0x80E40001, 0xA0E4000C,

	//rcp r8.x, r0.x
	0x02000006, 0x80010008, 0x80000000,

	//rcp r8.y, r0.y
	0x02000006, 0x80020008, 0x80550000,

	//rcp r8.z, r0.z
	0x02000006, 0x80040008, 0x80AA0000,

	//rcp r8.w, r0.w
	0x02000006, 0x80080008, 0x80FF0000,

	//mul r1.z, c3.w, r8
	0x03000005, 0x80040001, 0xA0FF0003, 0x80E40008,

	//add r1.z, r1, c3
	0x03000002, 0x80040001, 0x80E40001, 0xA0E40003,

	//mul r1.z, r1, c3.x
	0x03000005, 0x80040001, 0x80E40001, 0xA0000003,

	//add r1.z, r1, -c3.y
	0x03000002, 0x80040001, 0x80E40001, 0xA1550003,

	//rcp r8, c3.x
	0x02000006, 0x800F0008, 0xA0000003,

	//mul r1.z, r1, r8
	0x03000005, 0x80040001, 0x80E40001, 0x80E40008,

	//add r1.z, r1, -c3
	0x03000002, 0x80040001, 0x80E40001, 0xA1E40003,

	//rcp r8.x, r1.x
	0x02000006, 0x80010008, 0x80000001,

	//rcp r8.y, r1.y
	0x02000006, 0x80020008, 0x80550001,

	//rcp r8.z, r1.z
	0x02000006, 0x80040008, 0x80AA0001,

	//rcp r8.w, r1.w
	0x02000006, 0x80080008, 0x80FF0001,

	//mul r1.z, c3.w, r8
	0x03000005, 0x80040001, 0xA0FF0003, 0x80E40008,

	//mov r2, c4
	0x02000001, 0x800F0002, 0xA0E40004,

	//nrm r2.xyz, r0.xyzz
	0x02000024, 0x80070002, 0x80A40000,

	//add r1.z, r0, -r1
	0x03000002, 0x80040001, 0x80E40000, 0x81E40001,

	//rcp r8.x, r2.x
	0x02000006, 0x80010008, 0x80000002,

	//rcp r8.y, r2.y
	0x02000006, 0x80020008, 0x80550002,

	//rcp r8.z, r2.z
	0x02000006, 0x80040008, 0x80AA0002,

	//rcp r8.w, r2.w
	0x02000006, 0x80080008, 0x80FF0002,

	//mul r1.z, r1, r8
	0x03000005, 0x80040001, 0x80E40001, 0x80E40008,

	//mul r2, r2, r1.z
	0x03000005, 0x800F0002, 0x80E40002, 0x80AA0001,

	//add r0, r0, -r2
	0x03000002, 0x800F0000, 0x80E40000, 0x81E40002,

	//mov r12.xw, r0.xzzz
	0x02000001, 0x8009000C, 0x80A80000,

	//mov r12.y, -r0
	0x02000001, 0x8002000C, 0x81E40000,

	//mul r0.z, r0, c3
	0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,

	//add r12.z, r0, c3.w
	0x03000002, 0x8004000C, 0x80E40000, 0xA0FF0003,

	//mad oPos, r12.w, c254, r12
	0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,

	0x0000FFFF,
};


/*
 * PaintVertexShader
 * agal fe1ae7c7d183777d8ac60ace6981c626

	vs_2_x
		  dcl_texcoord v0
   0:     mov r0, c4
   1:     mova a0.x, v0.x
   2:     mov r0.xyz, c0[a0.x]
   3:     mov r1, c1[a0.x]
   4:     mul oT0, r1, c4
   5:     mov r2, c4
   6:     dp3 r2.x, r1, c14
   7:     dp3 r2.y, r1, c15
   8:     add r2.x, r2.x, c14.z
   9:     add r2.y, r2.y, c15.z
  10:     mov oT6, r2
  11:     mov r12.xw, r0.xzzz
  12:     mov r12.y, -r0
  13:     mul r0.z, r0, c3
  14:     add r12.z, r0, c3.w
  15:     mad oPos, r12.w, c254, r12

// approximately 16 instruction slots used
*/


const uint32_t vertex_shader_data_60a356d56cfedc55f04f75f5648a281b[] =
{
	0xFFFE0201,
	0x0200001F,
	0x80000005,
	0x900F0000,
	0x02000001,
	0x800F0000,
	0xA0E40004,
	0x0200002E,
	0xB0010000,
	0x90000000,
	0x03000001,
	0x80070000,
	0xA0E42000,
	0xB0000000,
	0x03000001,
	0x800F0001,
	0xA0E42001,
	0xB0000000,
	0x03000005,
	0xE00F0000,
	0x80E40001,
	0xA0E40004,
	0x02000001,
	0x800F0002,
	0xA0E40004,
	0x03000008,
	0x80010002,
	0x80E40001,
	0xA0E4000E,
	0x03000008,
	0x80020002,
	0x80E40001,
	0xA0E4000F,
	0x03000002,
	0x80010002,
	0x80000002,
	0xA0AA000E,
	0x03000002,
	0x80020002,
	0x80550002,
	0xA0AA000F,
	0x02000001,
	0xE00F0006,
	0x80E40002,
	0x02000001,
	0x8009000C,
	0x80A80000,
	0x02000001,
	0x8002000C,
	0x81E40000,
	0x03000005,
	0x80040000,
	0x80E40000,
	0xA0E40003,
	0x03000002,
	0x8004000C,
	0x80E40000,
	0xA0FF0003,
	0x04000004,
	0xC00F0000,
	0x80FF000C,
	0xA0E400FE,
	0x80E4000C,
	0x0000FFFF,
};




















/*
 * PaintVertexShader
 * faaa9a1e99604a96ee32f890ea093a7d

	vs_2_x
		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c4
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mul oT0, v1, c4
   5:     mov r1, c4
   6:     dp3 r1.x, v1, c14
   7:     dp3 r1.y, v1, c15
   8:     add r1.x, r1.x, c14.z
   9:     add r1.y, r1.y, c15.z
  10:     mov oT6, r1
  11:     mov r12.xw, r0.xzzz
  12:     mov r12.y, -r0
  13:     mul r0.z, r0, c3
  14:     add r12.z, r0, c3.w
  15:     mad oPos, r12.w, c254, r12

// approximately 16 instruction slots used
*/


const uint32_t vertex_shader_data_23fc30008fb031c1f2e19ff8f0ea23de[] =
{
	0xFFFE0201,
	0x0200001F,
	0x80000005,
	0x900F0000,
	0x0200001F,
	0x80010005,
	0x900F0001,
	0x02000001,
	0x800F0000,
	0xA0E40004,
	0x03000009,
	0x80010000,
	0x90E40000,
	0xA0E40000,
	0x03000009,
	0x80020000,
	0x90E40000,
	0xA0E40001,
	0x03000009,
	0x80040000,
	0x90E40000,
	0xA0E40002,
	0x03000005,
	0xE00F0000,
	0x90E40001,
	0xA0E40004,
	0x02000001,
	0x800F0001,
	0xA0E40004,
	0x03000008,
	0x80010001,
	0x90E40001,
	0xA0E4000E,
	0x03000008,
	0x80020001,
	0x90E40001,
	0xA0E4000F,
	0x03000002,
	0x80010001,
	0x80000001,
	0xA0AA000E,
	0x03000002,
	0x80020001,
	0x80550001,
	0xA0AA000F,
	0x02000001,
	0xE00F0006,
	0x80E40001,
	0x02000001,
	0x8009000C,
	0x80A80000,
	0x02000001,
	0x8002000C,
	0x81E40000,
	0x03000005,
	0x80040000,
	0x80E40000,
	0xA0E40003,
	0x03000002,
	0x8004000C,
	0x80E40000,
	0xA0FF0003,
	0x04000004,
	0xC00F0000,
	0x80FF000C,
	0xA0E400FE,
	0x80E4000C,
	0x0000FFFF,
};















/*
 * TrackMaterialVertexShader
 * agal 0549248f494925814849dc82fad19e6d

	vs_2_x
		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c4
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mov r1, c4
   5:     dp3 r1.x, v1, c14
   6:     dp3 r1.y, v1, c15
   7:     add r1.x, r1.x, c14.z
   8:     add r1.y, r1.y, c15.z
   9:     mov oT0, r1
  10:     mov r12.xw, r0.xzzz
  11:     mov r12.y, -r0
  12:     mul r0.z, r0, c3
  13:     add r12.z, r0, c3.w
  14:     mad oPos, r12.w, c254, r12

// approximately 15 instruction slots used
*/


const uint32_t vertex_shader_data_c95c4b989fd4927dba7168f0c9e5bbf3[] =
{
	0xFFFE0201,
	//dcl_texcoord v0
	0x0200001F, 0x80000005, 0x900F0000,
	//dcl_texcoord1 v1
	0x0200001F, 0x80010005, 0x900F0001,
	//mov r0, c4
	0x02000001, 0x800F0000, 0xA0E40004,
	//dp4 r0.x, v0, c0
	0x03000009, 0x80010000, 0x90E40000, 0xA0E40000,
	//dp4 r0.y, v0, c1
	0x03000009, 0x80020000, 0x90E40000, 0xA0E40001,
	//dp4 r0.z, v0, c2
	0x03000009, 0x80040000, 0x90E40000, 0xA0E40002,
	//mov r1, c4
	0x02000001, 0x800F0001, 0xA0E40004,
	//dp3 r1.x, v1, c14
	0x03000008, 0x80010001, 0x90E40001, 0xA0E4000E,
	//dp3 r1.y, v1, c15
	0x03000008, 0x80020001, 0x90E40001, 0xA0E4000F,
	//add r1.x, r1.x, c14.z
	0x03000002, 0x80010001, 0x80000001, 0xA0AA000E,
	//add r1.y, r1.y, c15.z
	0x03000002, 0x80020001, 0x80550001, 0xA0AA000F,
	//mov oT0, r1
	0x02000001, 0xE00F0000, 0x80E40001,
	//mov r12.xw, r0.xzzz
	0x02000001, 0x8009000C, 0x80A80000,
	//mov r12.y, -r0
	0x02000001, 0x8002000C, 0x81E40000,
	//mul r0.z, r0, c3
	0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,
	//add r12.z, r0, c3.w
	0x03000002, 0x8004000C, 0x80E40000, 0xA0FF0003,
	//mad oPos, r12.w, c254, r12
	0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,

	0x0000FFFF,
};


