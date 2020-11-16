/*
 * ov2640.c
 *
 *  Created on: 16 oct. 2018
 *      Author: alexandre.ferroni
 */

#include "ov2640.h"

const unsigned char OV2640_JPEG_INIT[][2]=
{
  { 0xff, 0x00 },
  { 0x2c, 0xff },
  { 0x2e, 0xdf },
  { 0xff, 0x01 },
  { 0x3c, 0x32 },
  { 0x11, 0x00 },
  { 0x09, 0x02 },
  { 0x04, 0x28 },
  { 0x13, 0xe5 },
  { 0x14, 0x48 },
  { 0x2c, 0x0c },
  { 0x33, 0x78 },
  { 0x3a, 0x33 },
  { 0x3b, 0xfB },
  { 0x3e, 0x00 },
  { 0x43, 0x11 },
  { 0x16, 0x10 },
  { 0x39, 0x92 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0x48, 0x00 },
  { 0x5B, 0x00 },
  { 0x42, 0x03 },
  { 0x4a, 0x81 },
  { 0x21, 0x99 },
  { 0x24, 0x40 },
  { 0x25, 0x38 },
  { 0x26, 0x82 },
  { 0x5c, 0x00 },
  { 0x63, 0x00 },
  { 0x61, 0x70 },
  { 0x62, 0x80 },
  { 0x7c, 0x05 },
  { 0x20, 0x80 },
  { 0x28, 0x30 },
  { 0x6c, 0x00 },
  { 0x6d, 0x80 },
  { 0x6e, 0x00 },
  { 0x70, 0x02 },
  { 0x71, 0x94 },
  { 0x73, 0xc1 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x37, 0xc0 },
  { 0x4f, 0x60 },
  { 0x50, 0xa8 },
  { 0x6d, 0x00 },
  { 0x3d, 0x38 },
  { 0x46, 0x3f },
  { 0x4f, 0x60 },
  { 0x0c, 0x3c },
  { 0xff, 0x00 },
  { 0xe5, 0x7f },
  { 0xf9, 0xc0 },
  { 0x41, 0x24 },
  { 0xe0, 0x14 },
  { 0x76, 0xff },
  { 0x33, 0xa0 },
  { 0x42, 0x20 },
  { 0x43, 0x18 },
  { 0x4c, 0x00 },
  { 0x87, 0xd5 },
  { 0x88, 0x3f },
  { 0xd7, 0x03 },
  { 0xd9, 0x10 },
  { 0xd3, 0x82 },
  { 0xc8, 0x08 },
  { 0xc9, 0x80 },
  { 0x7c, 0x00 },
  { 0x7d, 0x00 },
  { 0x7c, 0x03 },
  { 0x7d, 0x48 },
  { 0x7d, 0x48 },
  { 0x7c, 0x08 },
  { 0x7d, 0x20 },
  { 0x7d, 0x10 },
  { 0x7d, 0x0e },
  { 0x90, 0x00 },
  { 0x91, 0x0e },
  { 0x91, 0x1a },
  { 0x91, 0x31 },
  { 0x91, 0x5a },
  { 0x91, 0x69 },
  { 0x91, 0x75 },
  { 0x91, 0x7e },
  { 0x91, 0x88 },
  { 0x91, 0x8f },
  { 0x91, 0x96 },
  { 0x91, 0xa3 },
  { 0x91, 0xaf },
  { 0x91, 0xc4 },
  { 0x91, 0xd7 },
  { 0x91, 0xe8 },
  { 0x91, 0x20 },
  { 0x92, 0x00 },
  { 0x93, 0x06 },
  { 0x93, 0xe3 },
  { 0x93, 0x05 },
  { 0x93, 0x05 },
  { 0x93, 0x00 },
  { 0x93, 0x04 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x96, 0x00 },
  { 0x97, 0x08 },
  { 0x97, 0x19 },
  { 0x97, 0x02 },
  { 0x97, 0x0c },
  { 0x97, 0x24 },
  { 0x97, 0x30 },
  { 0x97, 0x28 },
  { 0x97, 0x26 },
  { 0x97, 0x02 },
  { 0x97, 0x98 },
  { 0x97, 0x80 },
  { 0x97, 0x00 },
  { 0x97, 0x00 },
  { 0xc3, 0xed },
  { 0xa4, 0x00 },
  { 0xa8, 0x00 },
  { 0xc5, 0x11 },
  { 0xc6, 0x51 },
  { 0xbf, 0x80 },
  { 0xc7, 0x10 },
  { 0xb6, 0x66 },
  { 0xb8, 0xA5 },
  { 0xb7, 0x64 },
  { 0xb9, 0x7C },
  { 0xb3, 0xaf },
  { 0xb4, 0x97 },
  { 0xb5, 0xFF },
  { 0xb0, 0xC5 },
  { 0xb1, 0x94 },
  { 0xb2, 0x0f },
  { 0xc4, 0x5c },
  { 0xc0, 0x64 },
  { 0xc1, 0x4B },
  { 0x8c, 0x00 },
  { 0x86, 0x3D },
  { 0x50, 0x00 },
  { 0x51, 0xC8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5a, 0xC8 },
  { 0x5b, 0x96 },
  { 0x5c, 0x00 },
  { 0xd3, 0x00 },	//{ 0xd3, 0x7f },
  { 0xc3, 0xed },
  { 0x7f, 0x00 },
  { 0xda, 0x00 },
  { 0xe5, 0x1f },
  { 0xe1, 0x67 },
  { 0xe0, 0x00 },
  { 0xdd, 0x7f },
  { 0x05, 0x00 },

  { 0x12, 0x40 },
  { 0xd3, 0x04 },	//{ 0xd3, 0x7f },
  { 0xc0, 0x16 },
  { 0xC1, 0x12 },
  { 0x8c, 0x00 },
  { 0x86, 0x3d },
  { 0x50, 0x00 },
  { 0x51, 0x2C },
  { 0x52, 0x24 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5A, 0x2c },
  { 0x5b, 0x24 },
  { 0x5c, 0x00 },
  //{ 0xff, 0xff },
};

const unsigned char OV2640_YUV422[][2]=
{
  {0xFF, 0x00},
  {0x05, 0x00},
  {0xDA, 0x10},
  {0xD7, 0x03},
  {0xDF, 0x00},
  {0x33, 0x80},
  {0x3C, 0x40},
  {0xe1, 0x77},
  {0x00, 0x00},
  //{0xff, 0xff},
};

const unsigned char OV2640_rgb565[][2] = {
  {0xFF, 0x00},
  {0xE0, 0x04},
  {0xDA, 0x08},
  {0xD7, 0x03},
  {0xE1, 0x77},
  {0xE0, 0x00},
};

const unsigned char OV2640_JPEG[][2]=
{
  {0xe0, 0x14},
  {0xe1, 0x77},
  {0xe5, 0x1f},
  {0xd7, 0x03},
  {0xda, 0x10},
  {0xe0, 0x00},
  {0xFF, 0x01},
  {0x04, 0x08},
  //{0xff, 0xff},
};

/* JPG 160x120 */
const unsigned char OV2640_160x120_JPEG[][2]=
{
  {0xff, 0x01},
  {0x12, 0x40},
  {0x17, 0x11},
  {0x18, 0x43},
  {0x19, 0x00},
  {0x1a, 0x4b},
  {0x32, 0x09},
  {0x4f, 0xca},
  {0x50, 0xa8},
  {0x5a, 0x23},
  {0x6d, 0x00},
  {0x39, 0x12},
  {0x35, 0xda},
  {0x22, 0x1a},
  {0x37, 0xc3},
  {0x23, 0x00},
  {0x34, 0xc0},
  {0x36, 0x1a},
  {0x06, 0x88},
  {0x07, 0xc0},
  {0x0d, 0x87},
  {0x0e, 0x41},
  {0x4c, 0x00},
  {0xff, 0x00},
  {0xe0, 0x04},
  {0xc0, 0x64},
  {0xc1, 0x4b},
  {0x86, 0x35},
  {0x50, 0x92},
  {0x51, 0xc8},
  {0x52, 0x96},
  {0x53, 0x00},
  {0x54, 0x00},
  {0x55, 0x00},
  {0x57, 0x00},
  {0x5a, 0x28},
  {0x5b, 0x1e},
  {0x5c, 0x00},
  {0xe0, 0x00},
  //{0xff, 0xff},
};


/* JPG 320x240 */
const unsigned char OV2640_320x240_JPEG[][2]=
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x89 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x50 },
  { 0x5b, 0x3c },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },
  //{ 0xff, 0xff },
};

/* JPG 640x480 */
const unsigned char OV2640_640x480_JPEG[][2]=
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},

	{0xff, 0x00},
	{0xe0, 0x04},
	{0xc0, 0xc8},
	{0xc1, 0x96},
	{0x86, 0x3d},
	{0x50, 0x89},
	{0x51, 0x90},
	{0x52, 0x2c},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x88},
	{0x57, 0x00},
	{0x5a, 0xa0},
	{0x5b, 0x78},
	{0x5c, 0x00},
	{0xd3, 0x04},
	{0xe0, 0x00},
};




/* JPG 800x600 */
const unsigned char OV2640_800x600_JPEG[][2] =
{
		//HrefEnd - HrefStart = 800 && Vend - Vstrt = 600
	{0xff, 0x01},
	{0x11, 0x01}, // Clock divider (clk = XVCLK/(0x01 + 1)
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40}, //?
	{0x4f, 0xca}, // {0x4f, 0xbb}, test
	{0x50, 0xa8}, // {0x50, 0x9c},
	{0x5a, 0x57}, //?
	{0x6d, 0x80}, // {0x6d, 0x80},
	{0x3d, 0x34}, //?
	{0x39, 0x02}, //?
	{0x35, 0x88}, //?
	{0x22, 0x0a}, //?
	{0x37, 0x40}, //?
	{0x34, 0xa0}, //?
	{0x06, 0x02}, //?
	{0x0d, 0xb7}, //?
	{0x0e, 0x01}, //?

	{0xff, 0x00},
	{0xe0, 0x04},//? DVP -> reset
	{0xc0, 0xc8},//Hsize[10-3]
	{0xc1, 0x96},//Vsize[10-3]
	{0x86, 0x35},//?
	{0x50, 0x89},//?
	{0x51, 0x90},//c8
	{0x52, 0x2c},// 2c = 44 *4 = 176 faux ?
	{0x53, 0x00},//no offset
	{0x54, 0x00},//no offset
	{0x55, 0x88},//7->Vsize(8), 6:4->offsetY(10:8), Hsize(8), offsetX(10:8)
	{0x57, 0x00},//7->Hsize(9)
	{0x5a, 0xc8},//outW[7:0](real/4) OK
	{0x5b, 0x96},//outH[7:0](real/4) OK
	{0x5c, 0x00},//2->outH(8), 1:0->outW(9:8)
	{0xd3, 0x02},//?
	{0xe0, 0x00},//reset 00
};

/* JPG 1024x768 */
const unsigned char OV2640_1024x768_JPEG[][2]=
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},

	{0xff, 0x00},
	{0xc0, 0xC8},
	{0xc1, 0x96},
	{0x8c, 0x00},
	{0x86, 0x3D},
	{0x50, 0x00},
	{0x51, 0x90},
	{0x52, 0x2C},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x88},
	{0x5a, 0x00},
	{0x5b, 0xC0},
	{0x5c, 0x01},
	{0xd3, 0x02},


};

/* JPG 1280x1024 */
const unsigned char OV2640_1280x1024_JPEG[][2]=
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},

	{0xff, 0x00},
	{0xe0, 0x04},
	{0xc0, 0xc8},
	{0xc1, 0x96},
	{0x86, 0x3d},
	{0x50, 0x00},
	{0x51, 0x90},
	{0x52, 0x2c},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x88},
	{0x57, 0x00},
	{0x5a, 0x40},
	{0x5b, 0xf0},
	{0x5c, 0x01},
	{0xd3, 0x02},
	{0xe0, 0x00},
};

/* JPG 1600x1200 */
const unsigned char OV2640_1600x1200_JPEG[][2] =
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},

	{0xff, 0x00},
	{0xe0, 0x04},
	{0xc0, 0xc8},
	{0xc1, 0x96},
	{0x86, 0x3d},
	{0x50, 0x00},
	{0x51, 0x90},
	{0x52, 0x2c},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x88},
	{0x57, 0x00},
	{0x5a, 0x90},
	{0x5b, 0x2C},
	{0x5c, 0x05},              //bit2->1;bit[1:0]->1
	{0xd3, 0x02},
	{0xe0, 0x00},
};


 void OV2640_WriteReg(uint8_t addr, uint8_t data)
 {
	 sccb_WriteNReg(addr, data);
	 DELAY_TIM_us(100);
 }


//UXGA ou Ultra Extended Graphics Array est une norme d'affichage dont la d�finition est de 1600 � 1200 pixels

void OV2640_JPEGConfig()
{

  uint32_t i;
  OV2640_WriteReg(0xff, 0x01);
  OV2640_WriteReg(0x12, 0x80);

  /* Initialize OV2640 */
  for(i=0; i<(sizeof(OV2640_JPEG_INIT)/2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG_INIT[i][0], OV2640_JPEG_INIT[i][1]);
  }

  /* Set to output YUV422 */
  for(i=0; i<(sizeof(OV2640_YUV422)/2); i++)
  {
    OV2640_WriteReg(OV2640_YUV422[i][0], OV2640_YUV422[i][1]);
  }


  OV2640_WriteReg(0xff, 0x01);
  OV2640_WriteReg(0x15, 0x00);

  /* Set to output JPEG */
  for(i=0; i<(sizeof(OV2640_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG[i][0], OV2640_JPEG[i][1]);
  }

  /* Set JPEG size 160*120 */
  /*for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
  }*/

  /* Set JPEG size 320*240 */
  /*for(i=0; i<(sizeof(OV2640_320x240_JPEG)/2); i++)
  {
   OV2640_WriteReg(OV2640_320x240_JPEG[i][0], OV2640_320x240_JPEG[i][1]);
  }*/

  /* Set JPEG size 640*480 */
  /*for(i=0; i<(sizeof(OV2640_640x480_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_640x480_JPEG[i][0], OV2640_640x480_JPEG[i][1]);
  }*/

  /* Set JPEG size 800*600 */
  /*for(i=0; i<(sizeof(OV2640_800x600_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_800x600_JPEG[i][0], OV2640_800x600_JPEG[i][1]);
  }*/

  /* Set JPEG size 1024*768 */
  /*for(i=0; i<(sizeof(OV2640_1024x768_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_1024x768_JPEG[i][0], OV2640_1024x768_JPEG[i][1]);
  }*/

  /* Set JPEG size 1280*1024 */
  /*for(i=0; i<(sizeof(OV2640_1280x1024_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_1280x1024_JPEG[i][0], OV2640_1280x1024_JPEG[i][1]);
  }*/

  /* Set JPEG size 1600*1200 */
  /*for(i=0; i<(sizeof(OV2640_1600x1200_JPEG)/2); i++)
  {
	  OV2640_WriteReg(OV2640_1600x1200_JPEG[i][0], OV2640_1600x1200_JPEG[i][1]);
  }*/


}


void OV2640_Reset()
{
  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
  OV2640_WriteReg(OV2640_SENSOR_COM7, 0x80);
}

void OV2640_LQ()
{
	uint32_t i;
	/* Set JPEG size 160*120 */
	for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
	{
		OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
	}
}

void OV2640_HQ()
{
	uint32_t i;

	 /* Set JPEG size 800*600 */
	 for(i=0; i<(sizeof(OV2640_800x600_JPEG)/2); i++)
	 {
		 OV2640_WriteReg(OV2640_800x600_JPEG[i][0], OV2640_800x600_JPEG[i][1]);
	 }

	/*
	OV2640_WriteReg(0xff, 0x00);
	OV2640_WriteReg(0xc7, 0x40);
	OV2640_WriteReg(0xcc, 0x52);
	OV2640_WriteReg(0xcd, 0x41);
	OV2640_WriteReg(0xce, 0x66);
	*/

	/* Set JPEG size 1024*768 */
	/*for(i=0; i<(sizeof(OV2640_1024x768_JPEG)/2); i++)
	{
		OV2640_WriteReg(OV2640_1024x768_JPEG[i][0], OV2640_1024x768_JPEG[i][1]);
	}*/


}

