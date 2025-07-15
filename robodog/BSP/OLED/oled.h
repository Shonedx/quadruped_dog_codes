#ifndef __OLED_H
#define __OLED_H
#include "Font.h"          // 包含 OLED 字体相关的头文件，可能包含自定义字体（如中文）数据
#include "stm32f4xx.h"

typedef enum {
  OLED_COLOR_NORMAL = 0, // 正常模式 黑底白字
  OLED_COLOR_REVERSED    // 反色模式 白底黑字
} OLED_ColorMode;


// ========================== OLED 显示参数 ==========================
void oledSetCursor(u8 Y, u8 X);
// ========================== 底层通信函数 ==========================
void oledSend(u8 *data, u8 len) ;
// ========================== OLED 驱动函数 ==========================
void oledDisplayOn();
void oledDisplayOff();
void oledSetColorMode(OLED_ColorMode mode) ;
// ========================== 显存操作函数 ==========================
void oledNewFrame(void) ;
void oledShowFrame(void) ;
void oledSetPixel(u8 x, u8 y, OLED_ColorMode color) ;
void oledSetBitsInPage(u8 page, u8 column, u8 data, u8 start, u8 end, OLED_ColorMode color) ;
void oledSetBitsCrossPage(u8 x, u8 y, u8 data, u8 len, OLED_ColorMode color);
void oledSetByteInPage(u8 page, u8 column, u8 data, OLED_ColorMode color) ;
void oledSetByteCrossPage(u8 x, u8 y, u8 data, OLED_ColorMode color) ;
void oledSetBlock(u8 x, u8 y, const u8 *data, u8 w, u8 h, OLED_ColorMode color);
// ========================== 图形绘制函数 ==========================
void oledDrawImage(u8 x, u8 y, const Image_t *img, OLED_ColorMode color) ;
// ================================ 文字绘制 ================================
void oledPrintNumASCII(u8 x, u8 y, u32 Number,u8 Length, const ASCIIFont *font, OLED_ColorMode color);
void oledPrintCharASCII(u8 x, u8 y, char ch, const ASCIIFont *font, OLED_ColorMode color) ;
void oledPrintStringASCII(u8 x, u8 y, char *str, const ASCIIFont *font, OLED_ColorMode color);
u8 oledGetLenUTF8(char *string) ;
void oledClear(void) ;
u32 oledPow(u32 X, u32 Y);
void oledInit(void);


#endif
