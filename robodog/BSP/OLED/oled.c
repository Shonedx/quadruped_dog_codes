#include "stm32f4xx.h"     // <--- **从 stm32f10x.h 更改为 stm32f4xx.h**
#include "OLED.h"          // 包含 OLED 驱动函数声明的头文件
#include "string.h"        // 包含字符串处理函数（如 memset, memcpy）的头文件
#include "oled_iic.h"      // <--- **新添加的头文件，用于调用硬件 I2C 函数**
#include "font.h"          // 包含字体相关的头文件
// OLED 器件地址
#define OLED_ADDRESS 0x78 // <--- **根据你的OLED模块地址调整，这里设置为0x78，因为I2C地址在i2c_oled.h中已定义，但这里保留以防万一**

// OLED 参数
#define OLED_PAGE 8            // OLED 总页数（例如 128x64 屏幕有 8 页）
#define OLED_ROW 8 * OLED_PAGE // OLED 总行数（8页 * 每页8行 = 64行）
#define OLED_COLUMN 128        // OLED 总列数

// 显存缓冲区：用于在发送到 OLED 前存储要显示的数据
u8 oled_gram[OLED_PAGE][OLED_COLUMN]; // 定义一个 8x128 的二维数组作为显存缓冲区 //一页包含八行对应oled_gram的类型u8，每一位对应一行
void oledSetCursor(u8 Y, u8 X)
{
	iicWriteCmd(0xB0 | Y);					// 设置页地址 (Page Address) 命令，B0-B7 对应页 0-7
	iicWriteCmd(0x10 | ((X & 0xF0) >> 4));	// 设置列地址高 4 位命令 (Column High Address)，10-1F
	iicWriteCmd(0x00 | (X & 0x0F));			// 设置列地址低 4 位命令 (Column Low Address)，00-0F
}
// ========================== 底层通信函数 ==========================
void oledSend(u8 *data, u8 len) {
    // <--- **调用硬件 I2C1 的多字节写入函数**
    // 假设 data[0] 是控制字节 (0x40), data+1 是实际像素数据，len-1 是数据长度
    iicWriteMultipleBit(OLED_I2C_ADDRESS, data[0], data + 1, len - 1);
}
// ========================== OLED 驱动函数 ==========================
void oledDisplayOn() {
  iicWriteCmd(0x8D); // Charge Pump Setting (电荷泵设置)
  iicWriteCmd(0x14); // Enable Charge Pump (开启电荷泵)
  iicWriteCmd(0xAF); // Display On (点亮屏幕)
}
void oledDisplayOff() {
  iicWriteCmd(0x8D); // Charge Pump Setting (电荷泵设置)
  iicWriteCmd(0x10); // Disable Charge Pump (关闭电荷泵)
  iicWriteCmd(0xAE); // Display Off (关闭屏幕)
}
void oledSetColorMode(OLED_ColorMode mode) {
  if (mode == OLED_COLOR_NORMAL) {
    iicWriteCmd(0xA6); // Normal Display (正常显示，1 显示，0 不显示)
  }
  if (mode == OLED_COLOR_REVERSED) {
    iicWriteCmd(0xA7); // Inverse Display (反色显示，0 显示，1 不显示)
  }
}
// ========================== 显存操作函数 ==========================
void oledNewFrame(void) {
  memset(oled_gram, 0, sizeof(oled_gram)); // 使用 memset 将整个 oled_gram 缓冲区清零
}
void oledShowFram(void) {
  // 用于存储控制字节和当前页数据的发送缓冲区
  static u8 send_buffer[OLED_COLUMN + 1]; 
  // send_buffer[0] = 0x40; // <-- **此行被移动到 memcpy 之后**

  for (u8 i = 0; i < OLED_PAGE; i++) { // 遍历每一页 (0-7)
    iicWriteCmd(0xB0 + i); // 设置当前要写入的页地址
    iicWriteCmd(0x02);     // 设置列地址低 4 位为 0x02 (通常设置为 0x00或0x02取决于具体型号和连接)
    iicWriteCmd(0x10);     // 设置列地址高 4 位为 0x10 (通常设置为 0x10)
    
    // <--- **调整拷贝和控制字节位置**
    // 将当前页的显存数据拷贝到发送缓冲区中，从 send_buffer[1] 开始
    // 这样 send_buffer[0] 可以作为控制字节。
    send_buffer[0] = 0x40; // 第一个字节为控制字节，0x40 表示后续是数据流
    memcpy(&send_buffer[1], oled_gram[i], OLED_COLUMN); 
    
    // 发送包含控制字节和当前页数据的缓冲区到 OLED
    oledSend(send_buffer, OLED_COLUMN + 1);
  }
}
void oledSetPixel(u8 x, u8 y, OLED_ColorMode color) { //设置某一点的像素
  // 检查坐标是否超出屏幕范围
  if (x >= OLED_COLUMN || y >= OLED_ROW) return;
  
  // 根据 y 坐标计算所在的页 (y / 8) 和页内的位 (y % 8)
  // 每个字节垂直对应 8 个像素
  if (!color) { // 如果是正常颜色模式 (COLOR_NORMAL)，对应点亮像素 (设置为 1)
    oled_gram[y / 8][x] |= (1 << (y % 8)); // 将对应字节的对应位设置为 1
  } else { // 如果是反色模式 (COLOR_REVERSED)，对应熄灭像素 (设置为 0)
    oled_gram[y / 8][x] &= ~(1 << (y % 8)); // 将对应字节的对应位设置为 0
  }
}
void oledSetBitsInPage(u8 page, u8 column, u8 data, u8 start, u8 end, OLED_ColorMode color) { //设置显存中某一个字节中的某几位 from start to end
  for (u8 i = start; i <= end; i++) {
    OLED_SetPixel(column, page * 8 + i, !((data >> i) & 0x01));
  }
}
void oledSetBitsCrossPage(u8 x, u8 y, u8 data, u8 len, OLED_ColorMode color) { //跨页设置一个字节的数据
  for (u8 i = 0; i < len; i++) {
    OLED_SetPixel(x, y + i, !((data >> i) & 0x01));
  }
}
void oledSetByteInPage(u8 page, u8 column, u8 data, OLED_ColorMode color) { //设置显存中某一个字节 只能设置页内的数据
  // 检查页和列是否超出范围
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  if (color) data = ~data; // 如果是反色模式，将数据按位取反
  oled_gram[page][column] = data; // 直接将数据写入显存对应的字节
}

void oledSetByteCrossPage(u8 x, u8 y, u8 data, OLED_ColorMode color) {
  u8 page = y / 8; // 计算起始像素所在的页
  u8 bit = y % 8;  // 计算起始像素在页内的位 (0-7)
  // 设置当前页的部分位：从 bit 到 7，共 8 - bit 位
  // 需要将 data 左移 bit 位，使其有效位与页内起始位对齐
  oledSetBitsInPage(page, x, data << bit, bit, 7, color); //写入的长度是8-bit
  
  if (bit) { // 如果起始位不是 0 (即 y 不是 8 的倍数，跨页)
    // 设置下一页的部分位：从 0 到 bit - 1，共 bit 位
    // 需要将 data 右移 8 - bit 位，获取下一页对应的位
    oledSetBitsInPage(page + 1, x, data >> (8 - bit), 0, bit - 1, color); //下一页长度是bit位
  }
}
void oledSetBlock(u8 x, u8 y, const u8 *data, u8 w, u8 h, OLED_ColorMode color) { //设置一块显存区域
  for (u8 i = 0; i < w; i++) {
    for (u8 j = 0; j < h; j++) {
      for (u8 k = 0; k < 8; k++) {
        if (j * 8 + k >= h) break; // 防止越界 (不完整的字节)
        OLED_SetPixel(x + i, y + j * 8 + k, !((data[i + j * w] >> k) & 0x01));
      }
    }
  }
}

// ========================== 图形绘制函数 ==========================

void oledDrawImage(u8 x, u8 y, const Image_t *img, OLED_ColorMode color) { //绘制图片
  oledSetBlock(x, y, img->data, img->w, img->h, color); // 调用 SetBlock 函数绘制图片
}
// ================================ 文字绘制 ================================

// 绘制一个 ASCII 数字 (使用像素坐标和指定字体)
void oledPrintNumASCII(u8 x, u8 y, u32 Number,u8 Length, const ASCIIFont *font, OLED_ColorMode color)
{
	u8 i;
    // 循环 Length 次，从最高位开始打印数字的每一位
	for(int i = 0; i < Length; i++)
	{
		oledPrintCharASCII(x + i * font->w, y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', font, color); // 修正: x 坐标应随字符宽度增加，y 坐标保持不变
	}
}
void oledPrintCharASCII(u8 x, u8 y, char ch, const ASCIIFont *font, OLED_ColorMode color) {
  // 检查字符是否在可显示 ASCII 范围 (' ' 到 '~')
  if (ch < ' ' || ch > '~') return;
  const u8 *char_data = font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w);
  // 使用 SetBlock 函数绘制字符位图
  oledSetBlock(x, y, char_data, font->w, font->h, color);
}

void oledPrintStringASCII(u8 x, u8 y, char *str, const ASCIIFont *font, OLED_ColorMode color) {
  u8 x0 = x; // 记录起始 x 坐标，用于按字体宽度累加
  while (*str) { // 遍历字符串直到遇到空字符 '\0'
    oledPrintCharASCII(x0, y, *str, font, color); // 绘制当前字符
    x0 += font->w; // x 坐标向右移动一个字符的宽度
    str++;       // 指向下一个字符
  }
}

u8 oledGetLenUTF8(char *string) {
  if ((string[0] & 0x80) == 0x00) { // 0xxxxxxx - 单字节 ASCII
    return 1;
  } else if ((string[0] & 0xE0) == 0xC0) { // 110xxxxx - 双字节 UTF-8 开头
    return 2;
  } else if ((string[0] & 0xF0) == 0xE0) { // 1110xxxx - 三字节 UTF-8 开头
    return 3;
  } else if ((string[0] & 0xF8) == 0xF0) { // 11110xxx - 四字节 UTF-8 开头
    return 4;
  }
  return 0; // 不是有效的 UTF-8 开头
}
void oledClear(void) //清除数据
{  
	u8 i, j;
    // 从页 0 遍历到页 7
	for (j = 0; j < 8; j++)
	{
		oledSetCursor(j, 0); // 设置光标到当前页的起始列
		for(i = 0; i < 128; i++) // 遍历当前页的所有列
		{
			iicWriteData(0x00); // 写入 0x00，清除该列的 8 个像素
		}
	}
}
u32 oledPow(u32 X, u32 Y)
{
	u32 result = 1; // 结果初始化为 1
	while (Y--) // 循环 Y 次
	{
		result *= X; // 每次循环乘以 X
	}
	return result; // 返回计算结果
}
void oledInit(void)
{
	u32 i, j;
	// 上电延时，等待 OLED 稳定
	for (i = 0; i < 1000; i++)			
	{
		for (j = 0; j < 1000; j++);
	}
	
	iicOledInit();			// 初始化 I2C 硬件配置

	// 发送初始化命令序列
	iicWriteCmd(0xAE);	// Set Display OFF (关闭显示)
	
	iicWriteCmd(0xD5);	// Set Display Clock Divide Ratio/Oscillator Frequency (设置显示时钟分频比/振荡器频率)
	iicWriteCmd(0x80);    // 默认值
	
	iicWriteCmd(0xA8);	// Set Multiplex Ratio (设置多路复用率)
	iicWriteCmd(0x3F);    // 64 行显示 (0-63)
	
	iicWriteCmd(0xD3);	// Set Display Offset (设置显示偏移)
	iicWriteCmd(0x00);    // 偏移 0
	
	iicWriteCmd(0x40);	// Set Display Start Line (设置显示起始行) - 0x40 + 0 (行 0)
	
	iicWriteCmd(0xA1);	// Set Segment Re-map (设置段重映射)，A0 正常，A1 左右翻转
	
	iicWriteCmd(0xC8);	// Set COM Output Scan Direction (设置 COM 扫描方向)，C0 正常，C8 上下翻转

	iicWriteCmd(0xDA);	// Set COM Pins Hardware Configuration (设置 COM 引脚硬件配置)
	iicWriteCmd(0x12);    // (对于 128x64 屏幕)
	
	iicWriteCmd(0x81);	// Set Contrast Control (设置对比度控制)
	iicWriteCmd(0xCF);    // 设置对比度值 (0x00 到 0xFF，CF 是一个中间值)

	iicWriteCmd(0xD9);	// Set Pre-charge Period (设置预充电周期)
	iicWriteCmd(0xF1);    // 默认值

	iicWriteCmd(0xDB);	// Set VCOMH Deselect Level (设置 VCOMH 去选择级别)
	iicWriteCmd(0x30);    // 默认值

	iicWriteCmd(0xA4);	// Set Entire Display On/Off (设置整个显示开启/关闭，A4 正常显示，A5 忽略显存，全屏点亮)

	iicWriteCmd(0xA6);	// Set Normal/Inverse Display (设置正常/反色显示，A6 正常，A7 反色)

	iicWriteCmd(0x8D);	// Set Charge Pump Setting (设置电荷泵)
	iicWriteCmd(0x14);    // Enable Charge Pump (开启电荷泵)

	iicWriteCmd(0xAF);	// Set Display ON (开启显示)
		
	OLED_Clear();				// 清空 OLED 屏幕内容 (清除页 2-7)
}