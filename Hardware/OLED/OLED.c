#include "stm32f4xx.h"     // <--- **从 stm32f10x.h 更改为 stm32f4xx.h**

#include "OLED.h"          // 包含 OLED 驱动函数声明的头文件

#include "string.h"        // 包含字符串处理函数（如 memset, memcpy）的头文件
#include "IIC_OLED.h"      // <--- **新添加的头文件，用于调用硬件 I2C 函数**

/*引脚配置*/
// 定义控制 OLED SCL (时钟) 引脚的宏，使用 GPIOB 的 Pin 8
// #define OLED_W_SCL(x)		GPIO_WriteBit(GPIOA, GPIO_Pin_5, (BitAction)(x)) // GPIOA Pin 5 <-- **这些软件I2C宏已被移除**
// 定义控制 OLED SDA (数据) 引脚的宏，使用 GPIOB 的 Pin 9
// #define OLED_W_SDA(x)		GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(x)) // GPIOA Pin 7 <-- **这些软件I2C宏已被移除**

// 设置OLED屏幕纵轴方向为y轴，横轴为x轴，x表示第几列 0-128，y表示第几页 0-8
// 注意：OLED 屏幕的寻址是按页进行的，每页 8 行为一个字节

// OLED 器件地址
#define OLED_ADDRESS 0x78 // <--- **根据你的OLED模块地址调整，这里设置为0x78，因为I2C地址在i2c_oled.h中已定义，但这里保留以防万一**

// OLED 参数
#define OLED_PAGE 8            // OLED 总页数（例如 128x64 屏幕有 8 页）
#define OLED_ROW 8 * OLED_PAGE // OLED 总行数（8页 * 每页8行 = 64行）
#define OLED_COLUMN 128        // OLED 总列数

// 显存缓冲区：用于在发送到 OLED 前存储要显示的数据
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN]; // 定义一个 8x128 的二维数组作为显存缓冲区

/*引脚初始化*/
// 初始化用于 I2C 通信的 GPIO 引脚
void OLED_I2C_Init(void)
{
    // <--- **原始的软件 I2C GPIO 初始化代码已移除**
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);
    // OLED_W_SCL(1);
    // OLED_W_SDA(1);

    // <--- **调用硬件 I2C1 的初始化函数**
    I2C1_OLED_Init();
	
	// 上电延时，等待 OLED 稳定 (从原代码中保留)
	uint32_t i, j;
	for (i = 0; i < 1000; i++)			
	{
		for (j = 0; j < 1000; j++);
	}
}

/**
  * @brief  I2C 开始信号
  * @param  无
  * @retval 无
  */
// void OLED_I2C_Start(void) { /* 此函数已被移除，由硬件I2C处理 */ }

/**
  * @brief  I2C 停止信号
  * @param  无
  * @retval 无
  */
// void OLED_I2C_Stop(void) { /* 此函数已被移除，由硬件I2C处理 */ }

/**
  * @brief  I2C 发送一个字节
  * @param  Byte 要发送的一个字节数据
  * @retval 无
  */
// void OLED_I2C_SendByte(uint8_t Byte) { /* 此函数已被移除，由硬件I2C处理 */ }

/**
  * @brief  OLED 写命令
  * @param  Command 要写入的命令字节
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
    // <--- **调用硬件 I2C1 的写入命令函数**
	I2C1_WriteCommand(Command);
}

/**
  * @brief  OLED 写数据
  * @param  Data 要写入的数据字节
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
    // <--- **调用硬件 I2C1 的写入数据函数**
	I2C1_WriteData(Data);
}

/**
  * @brief  OLED 设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7 (页)
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127 (列)
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					// 设置页地址 (Page Address) 命令，B0-B7 对应页 0-7
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// 设置列地址高 4 位命令 (Column High Address)，10-1F
	OLED_WriteCommand(0x00 | (X & 0x0F));			// 设置列地址低 4 位命令 (Column Low Address)，00-0F
}

// ========================== 底层通信函数 ==========================

/**
 * @brief 向OLED发送数据的函数
 * @param data 要发送的数据指针
 * @param len 要发送的数据长度
 * @return None
 * @note 此函数是移植本驱动时的重要函数，将本驱动库移植到其他平台时应根据实际情况修改此函数
 * 目前实现是循环调用 OLED_WriteData 发送数据
 */
void OLED_Send(uint8_t *data, uint8_t len) {
    // <--- **调用硬件 I2C1 的多字节写入函数**
    // 假设 data[0] 是控制字节 (0x40), data+1 是实际像素数据，len-1 是数据长度
    I2C1_WriteMultiByte(OLED_I2C_ADDRESS, data[0], data + 1, len - 1);
}

// ========================== OLED 驱动函数 ==========================
/**
 * @brief 开启OLED显示
 */
void OLED_DisPlay_On() {
  OLED_WriteCommand(0x8D); // Charge Pump Setting (电荷泵设置)
  OLED_WriteCommand(0x14); // Enable Charge Pump (开启电荷泵)
  OLED_WriteCommand(0xAF); // Display On (点亮屏幕)
}
/**
 * @brief 关闭OLED显示
 */
void OLED_DisPlay_Off() {
  OLED_WriteCommand(0x8D); // Charge Pump Setting (电荷泵设置)
  OLED_WriteCommand(0x10); // Disable Charge Pump (关闭电荷泵)
  OLED_WriteCommand(0xAE); // Display Off (关闭屏幕)
}

/**
 * @brief 设置颜色模式 黑底白字或白底黑字
 * @param mode 颜色模式枚举 COLOR_NORMAL/COLOR_REVERSED
 * @note 此函数直接设置屏幕的颜色模式
 */
void OLED_SetColorMode(OLED_ColorMode mode) {
  if (mode == OLED_COLOR_NORMAL) {
    OLED_WriteCommand(0xA6); // Normal Display (正常显示，1 显示，0 不显示)
  }
  if (mode == OLED_COLOR_REVERSED) {
    OLED_WriteCommand(0xA7); // Inverse Display (反色显示，0 显示，1 不显示)
  }
}

// ========================== 显存操作函数 ==========================

/**
 * @brief 清空显存，绘制新的一帧
 * @note 将显存缓冲区 OLED_GRAM 的所有数据清零
 */
void OLED_NewFrame(void) {
  memset(OLED_GRAM, 0, sizeof(OLED_GRAM)); // 使用 memset 将整个 OLED_GRAM 缓冲区清零
}

/**
 * @brief 将当前显存显示到屏幕上
 * @note 此函数是移植本驱动时的重要函数，将本驱动库移植到其他驱动芯片时应根据实际情况修改此函数
 * 目前实现是逐页将显存数据发送到 OLED
 */
void OLED_ShowFrame(void) {
  // 用于存储控制字节和当前页数据的发送缓冲区
  static uint8_t sendBuffer[OLED_COLUMN + 1]; 
  // sendBuffer[0] = 0x40; // <-- **此行被移动到 memcpy 之后**

  for (uint8_t i = 0; i < OLED_PAGE; i++) { // 遍历每一页 (0-7)
    OLED_WriteCommand(0xB0 + i); // 设置当前要写入的页地址
    OLED_WriteCommand(0x02);     // 设置列地址低 4 位为 0x02 (通常设置为 0x00或0x02取决于具体型号和连接)
    OLED_WriteCommand(0x10);     // 设置列地址高 4 位为 0x10 (通常设置为 0x10)
    
    // <--- **调整拷贝和控制字节位置**
    // 将当前页的显存数据拷贝到发送缓冲区中，从 sendBuffer[1] 开始
    // 这样 sendBuffer[0] 可以作为控制字节。
    sendBuffer[0] = 0x40; // 第一个字节为控制字节，0x40 表示后续是数据流
    memcpy(&sendBuffer[1], OLED_GRAM[i], OLED_COLUMN); 
    
    // 发送包含控制字节和当前页数据的缓冲区到 OLED
    OLED_Send(sendBuffer, OLED_COLUMN + 1);
  }
}

/**
 * @brief 设置一个像素点
 * @param x 横坐标 (列)，范围：0~127
 * @param y 纵坐标 (行)，范围：0~63
 * @param color 颜色模式 (COLOR_NORMAL: 亮/白，COLOR_REVERSED: 灭/黑)
 * @note 在显存缓冲区中设置或清除一个像素点对应的位
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color) {
  // 检查坐标是否超出屏幕范围
  if (x >= OLED_COLUMN || y >= OLED_ROW) return;
  
  // 根据 y 坐标计算所在的页 (y / 8) 和页内的位 (y % 8)
  // 每个字节垂直对应 8 个像素
  if (!color) { // 如果是正常颜色模式 (COLOR_NORMAL)，对应点亮像素 (设置为 1)
    OLED_GRAM[y / 8][x] |= (1 << (y % 8)); // 将对应字节的对应位设置为 1
  } else { // 如果是反色模式 (COLOR_REVERSED)，对应熄灭像素 (设置为 0)
    OLED_GRAM[y / 8][x] &= ~(1 << (y % 8)); // 将对应字节的对应位设置为 0
  }
}

/**
 * @brief 设置显存中一个字节数据的某几位
 * @param page 页地址，范围：0~7
 * @param column 列地址，范围：0~127
 * @param data 数据字节
 * @param start 起始位 (0-7)
 * @param end 结束位 (0-7)
 * @param color 颜色模式
 * @note 此函数将显存中某一字节的第 start 位到第 end 位设置为与 data 对应的位相同
 * @note start 和 end 的范围为 0-7, start 必须小于等于 end
 * @note 此函数与 OLED_SetBits_Fine 的区别在于此函数只能设置显存中的某一真实字节
 */
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color) {
  static uint8_t temp;
  // 检查页和列是否超出范围
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  
  if (color) data = ~data; // 如果是反色模式，将数据按位取反

  // 创建一个掩码：高位 (end+1 到 7) 和低位 (0 到 start-1) 置为 1，中间位为 0
  temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
  // 使用掩码清除显存中对应字节的中间位（start 到 end）
  OLED_GRAM[page][column] &= temp;
  
  // 创建一个掩码：高位 (end+1 到 7) 和低位 (0 到 start-1) 置为 0，中间位与 data 对应
  temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
  // 使用掩码设置显存中对应字节的中间位（start 到 end）为 data 对应的值
  OLED_GRAM[page][column] |= temp;
  
  // 下面是使用 OLED_SetPixel 实现的逻辑，被注释掉了
  // for (uint8_t i = start; i <= end; i++) {
  //   OLED_SetPixel(column, page * 8 + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief 设置显存中的一个字节数据
 * @param page 页地址，范围：0~7
 * @param column 列地址，范围：0~127
 * @param data 数据字节
 * @param color 颜色模式
 * @note 此函数将显存中的某一字节设置为 data 的值
 */
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color) {
  // 检查页和列是否超出范围
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  
  if (color) data = ~data; // 如果是反色模式，将数据按位取反
  
  OLED_GRAM[page][column] = data; // 直接将数据写入显存对应的字节
}

/**
 * @brief 设置显存中的一个字节数据的某几位
 * @param x 横坐标 (列)
 * @param y 纵坐标 (行)
 * @param data 数据字节 (低 len 位有效)
 * @param len 位数，范围：1-8
 * @param color 颜色模式
 * @note 此函数将显存中从 (x,y) 开始向下数 len 位设置为与 data 的低 len 位相同
 * @note len 的范围为 1-8
 * @note 此函数与 OLED_SetByte_Fine 的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color) {
  uint8_t page = y / 8; // 计算起始像素所在的页
  uint8_t bit = y % 8;  // 计算起始像素在页内的位 (0-7)
  
  // 如果需要设置的位跨越当前页和下一页的边界 (当前页剩余的位不足 len)
  if (bit + len > 8) {
    // 先设置当前页的部分位：从 bit 到 7，共 8 - bit 位
    // 需要将 data 左移 bit 位，使其有效位与页内起始位对齐
    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    // 然后设置下一页的部分位：从 0 到 len + bit - 1 - 8
    // 需要将 data 右移 8 - bit 位，获取剩余的有效位
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8, color);
  } else { // 如果需要设置的位都在当前页内
    // 从 bit 到 bit + len - 1，共 len 位
    // 需要将 data 左移 bit 位，使其有效位与页内起始位对齐
    OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
  }
  
  // 下面是使用 OLED_SetPixel 实现的逻辑，被注释掉了
  // for (uint8_t i = 0; i < len; i++) {
  //   OLED_SetPixel(x, y + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief 设置显存中一个字节长度的数据 (8 位垂直像素)
 * @param x 横坐标 (列)
 * @param y 纵坐标 (行)，必须是 8 的倍数 (即页的起始行)
 * @param data 数据字节
 * @param color 颜色模式
 * @note 此函数将显存中从 (x,y) 开始向下数 8 位设置为与 data 相同
 * @note 此函数与 OLED_SetByte 的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color) {
  uint8_t page = y / 8; // 计算起始像素所在的页
  uint8_t bit = y % 8;  // 计算起始像素在页内的位 (0-7)
  
  // 设置当前页的部分位：从 bit 到 7，共 8 - bit 位
  // 需要将 data 左移 bit 位，使其有效位与页内起始位对齐
  OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
  
  if (bit) { // 如果起始位不是 0 (即 y 不是 8 的倍数，跨页)
    // 设置下一页的部分位：从 0 到 bit - 1，共 bit 位
    // 需要将 data 右移 8 - bit 位，获取下一页对应的位
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
  }
}

/**
 * @brief 设置一块显存区域
 * @param x 起始横坐标
 * @param y 起始纵坐标
 * @param data 数据的起始地址 (数据按列优先排列)
 * @param w 宽度 (像素)
 * @param h 高度 (像素)
 * @param color 颜色模式
 * @note 此函数将显存中从 (x,y) 开始的 w*h 个像素设置为与 data 中的数据相同
 * @note data 中的数据应采用列优先排列（即先写完一列的垂直像素数据，再写下一列）
 */
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color) {
  uint8_t fullRow = h / 8; // 计算完整的字节行数 (每行 8 像素)
  uint8_t partBit = h % 8; // 计算不完整的字节行中的有效像素位数
  
  for (uint8_t i = 0; i < w; i++) { // 遍历每一列
    for (uint8_t j = 0; j < fullRow; j++) { // 遍历每一整页行
      // 设置当前列 (x+i)，当前页行 (y + j*8) 的一个完整字节 (8 位垂直像素)
      // data 的索引为 i + j * w，因为数据是列优先排列
      OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
    }
  }
  
  if (partBit) { // 如果有不完整的字节行
    uint16_t fullNum = w * fullRow; // 计算完整字节行占用的数据字节数
    for (uint8_t i = 0; i < w; i++) { // 遍历每一列
      // 设置当前列 (x+i)，最后不完整页行 (y + fullRow * 8) 的 partialBit 位
      // data 的索引为 fullNum + i，跳过前面完整的字节数据
      OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
    }
  }
  
  // 下面是使用 OLED_SetPixel 实现的逻辑，被注释掉了
  // for (uint8_t i = 0; i < w; i++) {
  //   for (uint8_t j = 0; j < h; j++) {
  //     for (uint8_t k = 0; k < 8; k++) {
  //       if (j * 8 + k >= h) break; // 防止越界 (不完整的字节)
  //       OLED_SetPixel(x + i, y + j * 8 + k, !((data[i + j * w] >> k) & 0x01));
  //     }
  //   }
  // }
}

// ========================== 图形绘制函数 ==========================

/**
 * @brief 绘制一条线段
 * @param x1 起始点横坐标
 * @param y1 起始点纵坐标
 * @param x2 终止点横坐标
 * @param y2 终止点纵坐标
 * @param color 颜色模式
 * @note 此函数使用 Bresenham 算法绘制线段
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color) {
  static uint8_t temp = 0;
  // 处理垂直线
  if (x1 == x2) {
    if (y1 > y2) { // 确保 y1 小于等于 y2
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++) { // 逐点绘制
      OLED_SetPixel(x1, y, color);
    }
  }
  // 处理水平线
  else if (y1 == y2) {
    if (x1 > x2) { // 确保 x1 小于等于 x2
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++) { // 逐点绘制
      OLED_SetPixel(x, y1, color);
    }
  }
  // 处理斜线 (Bresenham 直线算法)
  else {
    int16_t dx = x2 - x1; // x 方向距离
    int16_t dy = y2 - y1; // y 方向距离
    // 确定 x, y 方向的步进方向 (1 或 -1)
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0; // 当前点坐标和误差项

    // 取 dx 和 dy 的绝对值
	if(dx<0) dx=-dx;
	//else dx=dx; // 这行是多余的
	if(dy<0) dy=-dy;
	//else dy=dy; // 这行是多余的
    // dx = abs(dx); // 原注释中被注释掉的 stdlib 函数版本
    // dy = abs(dy); // 原注释中被注释掉的 stdlib 函数版本

    // 根据 dx 和 dy 的大小选择主方向
    if (dx > dy) { // x 是主方向
      for (x = x1; x != x2; x += ux) { // 沿 x 方向逐点
        OLED_SetPixel(x, y, color); // 绘制当前点
        eps += dy; // 误差项增加 dy
        if ((eps << 1) >= dx) { // 如果误差项达到阈值
          y += uy; // 沿 y 方向步进
          eps -= dx; // 误差项减去 dx
        }
      }
    } else { // y 是主方向
      for (y = y1; y != y2; y += uy) { // 沿 y 方向逐点
        OLED_SetPixel(x, y, color); // 绘制当前点
        eps += dx; // 误差项增加 dx
        if ((eps << 1) >= dy) { // 如果误差项达到阈值
          x += ux; // 沿 x 方向步进
          eps -= dy; // 误差项减去 dy
        }
      }
    }
  }
}

/**
 * @brief 绘制一个矩形框
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param w 矩形宽度
 * @param h 矩形高度
 * @param color 颜色模式
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  // 绘制四条边线
  OLED_DrawLine(x, y, x + w, y, color);       // 顶边
  OLED_DrawLine(x, y + h, x + w, y + h, color); // 底边
  OLED_DrawLine(x, y, x, y + h, color);       // 左边
  OLED_DrawLine(x + w, y, x + w, y + h, color); // 右边
}

/**
 * @brief 绘制一个填充矩形
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param w 矩形宽度
 * @param h 矩形高度
 * @param color 颜色模式
 */
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  // 从上到下逐行绘制水平线来填充矩形
  for (uint8_t i = 0; i < h; i++) {
    OLED_DrawLine(x, y+i, x+w-1, y+i, color); // 绘制一行水平线
  }
}

/**
 * @brief 绘制一个三角形框
 * @param x1 第一个点横坐标
 * @param y1 第一个点纵坐标
 * @param x2 第二个点横坐标
 * @param y2 第二个点纵坐标
 * @param x3 第三个点横坐标
 * @param y3 第三个点纵坐标
 * @param color 颜色模式
 */
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  // 绘制三条边线
  OLED_DrawLine(x1, y1, x2, y2, color);
  OLED_DrawLine(x2, y2, x3, y3, color);
  OLED_DrawLine(x3, y3, x1, y1, color);
}

/**
 * @brief 绘制一个填充三角形
 * @param x1 第一个点横坐标
 * @param y1 第一个点纵坐标
 * @param x2 第二个点横坐标
 * @param y2 第二个点纵坐标
 * @param x3 第三个点横坐标
 * @param y3 第三个点纵坐标
 * @param color 颜色模式
 * @note 此处的填充三角形算法可能不完整或存在问题，可能无法正确处理所有三角形类型。
 */
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  uint8_t a = 0, b = 0, y = 0, last = 0;
  // 找到 y1 和 y2 中的最小值和最大值
  if (y1 > y2) {
    a = y2;
    b = y1;
  } else {
    a = y1;
    b = y2;
  }
  y = a;
  // 绘制从 y=a 到 y=b 之间的水平线段
  for (; y <= b; y++) {
    if (y <= y3) { // 如果当前行在点3的上方或水平位置
      // 绘制连接 (x1, y1) - (x2, y2) 和 (x1, y1) - (x3, y3) 线上对应 y 坐标的水平线段
      // 注意：这里的插值计算 (y - y1) * (x2 - x1) / (y2 - y1) 可能会因为整数除法损失精度，对于斜率较小的线段可能不准确
      OLED_DrawLine(x1 + (y - y1) * (x2 - x1) / (y2 - y1), y, x1 + (y - y1) * (x3 - x1) / (y3 - y1), y, color);
    } else { // 如果当前行在点3的下方
      last = y - 1; // 记录上一个 y 坐标
      break; // 退出循环
    }
  }
  // 绘制剩余部分的水平线段
  for (; y <= b; y++) {
    // 绘制连接 (x2, y2) - (x3, y3) 和 (x1, last) - (x3, y3) (似乎逻辑有点问题) 线上对应 y 坐标的水平线段
     OLED_DrawLine(x2 + (y - y2) * (x3 - x2) / (y3 - y2), y, x1 + (y - last) * (x3 - x1) / (y3 - last), y, color);
  }
}

/**
 * @brief 绘制一个圆框
 * @param x 圆心横坐标
 * @param y 圆心纵坐标
 * @param r 圆半径
 * @param color 颜色模式
 * @note 此函数使用 Bresenham 算法绘制圆
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  // Bresenham 圆算法变量
  int16_t a = 0, b = r, di = 3 - (r << 1);
  // 绘制圆弧的八分之一部分，并通过对称得到整个圆
  while (a <= b) {
    // 绘制对称的八个点
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x + b, y - a, color);
    OLED_SetPixel(x - a, y + b, color);
    OLED_SetPixel(x - b, y - a, color); // 重复绘制，此处可能是复制错误
    OLED_SetPixel(x - a, y - b, color);
    OLED_SetPixel(x + b, y + a, color);
    OLED_SetPixel(x + a, y - b, color);
    OLED_SetPixel(x + a, y + b, color);
    OLED_SetPixel(x - b, y + a, color); // 重复绘制，此处可能是复制错误
    
    a++; // 沿 x 方向步进
    // 更新决策参数 di
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--; // 沿 y 方向步进
    }
    OLED_SetPixel(x + a, y + b, color); // 绘制下一个点 (这个点应该在循环外处理或包含在对称点中)
  }
}

/**
 * @brief 绘制一个填充圆
 * @param x 圆心横坐标
 * @param y 圆心纵坐标
 * @param r 圆半径
 * @param color 颜色模式
 * @note 此函数使用类似圆算法的方法绘制填充圆，通过绘制水平线段填充
 */
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  // Bresenham 圆算法变量
  int16_t a = 0, b = r, di = 3 - (r << 1);
  while (a <= b) {
    // 绘制对称的水平线段
    for (int16_t i = x - b; i <= x + b; i++) { // 绘制垂直于 y 轴的水平线段
      OLED_SetPixel(i, y + a, color);
      OLED_SetPixel(i, y - a, color);
    }
    for (int16_t i = x - a; i <= x + a; i++) { // 绘制垂直于 x 轴的水平线段
      OLED_SetPixel(i, y + b, color);
      OLED_SetPixel(i, y - b, color);
    }
    a++; // 沿 x 方向步进
    // 更新决策参数 di
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--; // 沿 y 方向步进
    }
  }
}

/**
 * @brief 绘制一个椭圆框
 * @param x 椭圆中心横坐标
 * @param y 椭圆中心纵坐标
 * @param a 椭圆长轴或短轴的一半长度 (水平方向)
 * @param b 椭圆长轴或短轴的另一半长度 (垂直方向)
 * @param color 颜色模式
 * @note 使用中点椭圆算法绘制
 */
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color) {
  int xpos = 0, ypos = b; // 起始点 (0, b)
  int a2 = a * a, b2 = b * b; // a^2 和 b^2
  int d = b2 + a2 * (0.25 - b); // 决策参数初始值 (区域 1)
  // 区域 1 (斜率绝对值 < 1)
  while (a2 * ypos > b2 * xpos) {
    // 绘制对称的四个点
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    
    if (d < 0) { // 决策参数 < 0，下一个点取 E
      d = d + b2 * ((xpos << 1) + 3);
      xpos += 1;
    } else { // 决策参数 >= 0，下一个点取 SE
      d = d + b2 * ((xpos << 1) + 3) + a2 * (-(ypos << 1) + 2);
      xpos += 1;
      ypos -= 1;
    }
  }
  
  // 区域 2 (斜率绝对值 > 1)
  d = b2 * (xpos + 0.5) * (xpos + 0.5) + a2 * (ypos - 1) * (ypos - 1) - a2 * b2; // 决策参数初始值 (区域 2)
  while (ypos > 0) {
    // 绘制对称的四个点
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    
    if (d < 0) { // 决策参数 < 0，下一个点取 SE
      d = d + b2 * ((xpos << 1) + 2) + a2 * (-(ypos << 1) + 3);
      xpos += 1;
      ypos -= 1;
    } else { // 决策参数 >= 0，下一个点取 S
      d = d + a2 * (-(ypos << 1) + 3);
      ypos -= 1;
    }
  }
}

/**
 * @brief 绘制一张图片
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param img 图片结构体指针，包含数据、宽度和高度
 * @param color 颜色模式
 * @note 图片数据应为列优先排列的单色位图数据
 */
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color) {
  OLED_SetBlock(x, y, img->data, img->w, img->h, color); // 调用 SetBlock 函数绘制图片
}

// ================================ 文字绘制 ================================

// 绘制一个 ASCII 数字 (使用像素坐标和指定字体)
void OLED_PrintASCIINum(uint8_t x, uint8_t y, uint32_t Number,uint8_t Length, const ASCIIFont *font, OLED_ColorMode color)
{
	uint8_t i;
    // 循环 Length 次，从最高位开始打印数字的每一位
	for(int i = 0; i < Length; i++)
	{
        // 计算当前位的数字，并转换为对应的 ASCII 字符
        // 例如，对于数字 123, Length=3:
        // i=0: 123 / 100 % 10 + '0' = 1 + '0' -> '1'
        // i=1: 123 / 10 % 10 + '0' = 2 + '0' -> '2'
        // i=2: 123 / 1 % 10 + '0' = 3 + '0' -> '3'
		OLED_PrintASCIIChar(x + i * font->w, y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', font, color); // 修正: x 坐标应随字符宽度增加，y 坐标保持不变
	}
}

/**
 * @brief 绘制一个 ASCII 字符
 * @param x 起始点横坐标 (像素)
 * @param y 起始点纵坐标 (像素)
 * @param ch 字符
 * @param font 字体结构体指针
 * @param color 颜色模式
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color) {
  // 检查字符是否在可显示 ASCII 范围 (' ' 到 '~')
  if (ch < ' ' || ch > '~') return;
  
  // 计算字符在字体数据中的偏移量
  // (ch - ' ') 得到字符相对于空格的索引
  // 每个字符占用的字节数 = (字体高度 + 7) / 8 * 字体宽度 (向上取整到字节)
  // chars + 偏移量 指向该字符的位图数据
  const uint8_t *char_data = font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w);
  // 使用 SetBlock 函数绘制字符位图
  OLED_SetBlock(x, y, char_data, font->w, font->h, color);
}

/**
 * @brief 绘制一个 ASCII 字符串
 * @param x 起始点横坐标 (像素)
 * @param y 起始点纵坐标 (像素)
 * @param str 字符串
 * @param font 字体结构体指针
 * @param color 颜色模式
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color) {
  uint8_t x0 = x; // 记录起始 x 坐标，用于按字体宽度累加
  while (*str) { // 遍历字符串直到遇到空字符 '\0'
    OLED_PrintASCIIChar(x0, y, *str, font, color); // 绘制当前字符
    x0 += font->w; // x 坐标向右移动一个字符的宽度
    str++;       // 指向下一个字符
  }
}

/**
 * @brief 获取 UTF-8 编码的字符长度 (字节数)
 * @param string 指向 UTF-8 编码字符串的指针
 * @return 字符的字节长度 (1-4)，如果不是有效的 UTF-8 开头则返回 0
 */
uint8_t _OLED_GetUTF8Len(char *string) {
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

/**
 * @brief 绘制字符串 (支持 ASCII 和 UTF-8 编码的多字节字符，如中文)
 * @param x 起始点横坐标 (像素)
 * @param y 起始点纵坐标 (像素)
 * @param str 字符串 (UTF-8 编码)
 * @param font 字体结构体指针 (包含 ASCII 字体和自定义字体数据)
 * @param color 颜色模式
 *
 * @note 为确保字符串中的中文会被自动识别并绘制，需:
 * 1. 编译器字符集设置为 UTF-8
 * 2. 使用特定的取模工具生成字体 (如 https://led.baud-dance.com)
 */
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color) {
  uint16_t i = 0;                                       // 字符串当前处理的索引
  // 计算一个完整字模（包含编码和位图数据）占用的字节数
  // = ((字体高度 + 7) / 8) * 字体宽度 (位图数据字节数) + 4 (编码字节数，假设为 4)
  uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; 
  uint8_t found;                                        // 标志，表示是否在自定义字体数据中找到字符
  uint8_t utf8Len;                                      // 当前字符的 UTF-8 编码长度 (字节数)
  uint8_t *head;                                        // 指向字体数据中当前字模的头部

  while (str[i]) { // 遍历字符串直到遇到空字符 '\0'
    found = 0; // 假设未找到
    utf8Len = _OLED_GetUTF8Len(str + i); // 获取当前字符的 UTF-8 编码长度
    if (utf8Len == 0) { // 如果不是有效的 UTF-8 字符
        // 打印一个空格占位并跳过此无效字节 (或者可以打印问号等)
        OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
        x += font->ascii->w; // 移动 x 坐标
        i++; // 只跳过一个字节，因为不知道无效字符的实际长度
        continue; // 处理下一个字符
    }

    // 搜索自定义字体数据
    // TODO 优化查找算法, 可以使用二分查找或 hash 表提高效率，目前是线性查找
    for (uint8_t j = 0; j < font->len; j++) { // 遍历自定义字体数据中的所有字模
      head = (uint8_t *)(font->chars) + (j * oneLen); // 计算当前字模的起始地址
      // 比较当前字符串中的字符编码与字模中的编码 (前 utf8Len 个字节)
      if (memcmp(str + i, head, utf8Len) == 0) { // 如果找到匹配的字模
        // 字模数据通常包含编码和位图，位图数据在编码之后，偏移 4 个字节 (根据工具生成格式确定)
        OLED_SetBlock(x, y, head + 4, font->w, font->h, color); // 绘制字模的位图数据
        x += font->w;     // x 坐标向右移动一个字模的宽度
        i += utf8Len;   // 字符串索引跳过当前字符的字节数
        found = 1;      // 标记为已找到
        break;          // 退出字模查找循环
      }
    }

    // 如果在自定义字体中未找到，且当前字符是 ASCII 字符 (长度为 1)
    if (found == 0) {
      if (utf8Len == 1) {
        // 绘制 ASCII 字符
        OLED_PrintASCIIChar(x, y, str[i], font->ascii, color);
        x += font->ascii->w; // x 坐标向右移动一个 ASCII 字符的宽度
        i += utf8Len;     // 字符串索引加 1
      } else { // 如果是多字节字符但未找到字模
        // 绘制一个空格占位
        OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
        x += font->ascii->w; // x 坐标向右移动
        i += utf8Len;     // 字符串索引跳过当前字符的字节数
      }
    }
  }
}

/**
  * @brief  OLED 清屏
  * @param  无
  * @retval 无
  * @note 此函数仅清除了页面 0 到 7 的内容
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
    // 从页 0 遍历到页 7
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0); // 设置光标到当前页的起始列
		for(i = 0; i < 128; i++) // 遍历当前页的所有列
		{
			OLED_WriteData(0x00); // 写入 0x00，清除该列的 8 个像素
		}
	}
}
/**
  * @brief  OLED 次方函数 (计算 X 的 Y 次幂)
  * @param  X 底数
  * @param  Y 指数
  * @retval 返回值等于 X 的 Y 次幂
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1; // 结果初始化为 1
	while (Y--) // 循环 Y 次
	{
		Result *= X; // 每次循环乘以 X
	}
	return Result; // 返回计算结果
}

/**
  * @brief  OLED 初始化
  * @param  无
  * @retval 无
  * @note 发送一系列命令配置 OLED 控制器
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	// 上电延时，等待 OLED 稳定
	for (i = 0; i < 1000; i++)			
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			// 初始化 I2C 引脚

	// 发送初始化命令序列
	OLED_WriteCommand(0xAE);	// Set Display OFF (关闭显示)
	
	OLED_WriteCommand(0xD5);	// Set Display Clock Divide Ratio/Oscillator Frequency (设置显示时钟分频比/振荡器频率)
	OLED_WriteCommand(0x80);    // 默认值
	
	OLED_WriteCommand(0xA8);	// Set Multiplex Ratio (设置多路复用率)
	OLED_WriteCommand(0x3F);    // 64 行显示 (0-63)
	
	OLED_WriteCommand(0xD3);	// Set Display Offset (设置显示偏移)
	OLED_WriteCommand(0x00);    // 偏移 0
	
	OLED_WriteCommand(0x40);	// Set Display Start Line (设置显示起始行) - 0x40 + 0 (行 0)
	
	OLED_WriteCommand(0xA1);	// Set Segment Re-map (设置段重映射)，A0 正常，A1 左右翻转
	
	OLED_WriteCommand(0xC8);	// Set COM Output Scan Direction (设置 COM 扫描方向)，C0 正常，C8 上下翻转

	OLED_WriteCommand(0xDA);	// Set COM Pins Hardware Configuration (设置 COM 引脚硬件配置)
	OLED_WriteCommand(0x12);    // (对于 128x64 屏幕)
	
	OLED_WriteCommand(0x81);	// Set Contrast Control (设置对比度控制)
	OLED_WriteCommand(0xCF);    // 设置对比度值 (0x00 到 0xFF，CF 是一个中间值)

	OLED_WriteCommand(0xD9);	// Set Pre-charge Period (设置预充电周期)
	OLED_WriteCommand(0xF1);    // 默认值

	OLED_WriteCommand(0xDB);	// Set VCOMH Deselect Level (设置 VCOMH 去选择级别)
	OLED_WriteCommand(0x30);    // 默认值

	OLED_WriteCommand(0xA4);	// Set Entire Display On/Off (设置整个显示开启/关闭，A4 正常显示，A5 忽略显存，全屏点亮)

	OLED_WriteCommand(0xA6);	// Set Normal/Inverse Display (设置正常/反色显示，A6 正常，A7 反色)

	OLED_WriteCommand(0x8D);	// Set Charge Pump Setting (设置电荷泵)
	OLED_WriteCommand(0x14);    // Enable Charge Pump (开启电荷泵)

	OLED_WriteCommand(0xAF);	// Set Display ON (开启显示)
		
	OLED_Clear();				// 清空 OLED 屏幕内容 (清除页 2-7)
}