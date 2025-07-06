#include "stm32f4xx.h"     // <--- **�� stm32f10x.h ����Ϊ stm32f4xx.h**

#include "OLED.h"          // ���� OLED ��������������ͷ�ļ�

#include "string.h"        // �����ַ������������� memset, memcpy����ͷ�ļ�
#include "IIC_OLED.h"      // <--- **����ӵ�ͷ�ļ������ڵ���Ӳ�� I2C ����**

/*��������*/
// ������� OLED SCL (ʱ��) ���ŵĺ꣬ʹ�� GPIOB �� Pin 8
// #define OLED_W_SCL(x)		GPIO_WriteBit(GPIOA, GPIO_Pin_5, (BitAction)(x)) // GPIOA Pin 5 <-- **��Щ���I2C���ѱ��Ƴ�**
// ������� OLED SDA (����) ���ŵĺ꣬ʹ�� GPIOB �� Pin 9
// #define OLED_W_SDA(x)		GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(x)) // GPIOA Pin 7 <-- **��Щ���I2C���ѱ��Ƴ�**

// ����OLED��Ļ���᷽��Ϊy�ᣬ����Ϊx�ᣬx��ʾ�ڼ��� 0-128��y��ʾ�ڼ�ҳ 0-8
// ע�⣺OLED ��Ļ��Ѱַ�ǰ�ҳ���еģ�ÿҳ 8 ��Ϊһ���ֽ�

// OLED ������ַ
#define OLED_ADDRESS 0x78 // <--- **�������OLEDģ���ַ��������������Ϊ0x78����ΪI2C��ַ��i2c_oled.h���Ѷ��壬�����ﱣ���Է���һ**

// OLED ����
#define OLED_PAGE 8            // OLED ��ҳ�������� 128x64 ��Ļ�� 8 ҳ��
#define OLED_ROW 8 * OLED_PAGE // OLED ��������8ҳ * ÿҳ8�� = 64�У�
#define OLED_COLUMN 128        // OLED ������

// �Դ滺�����������ڷ��͵� OLED ǰ�洢Ҫ��ʾ������
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN]; // ����һ�� 8x128 �Ķ�ά������Ϊ�Դ滺����

/*���ų�ʼ��*/
// ��ʼ������ I2C ͨ�ŵ� GPIO ����
void OLED_I2C_Init(void)
{
    // <--- **ԭʼ����� I2C GPIO ��ʼ���������Ƴ�**
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);
    // OLED_W_SCL(1);
    // OLED_W_SDA(1);

    // <--- **����Ӳ�� I2C1 �ĳ�ʼ������**
    I2C1_OLED_Init();
	
	// �ϵ���ʱ���ȴ� OLED �ȶ� (��ԭ�����б���)
	uint32_t i, j;
	for (i = 0; i < 1000; i++)			
	{
		for (j = 0; j < 1000; j++);
	}
}

/**
  * @brief  I2C ��ʼ�ź�
  * @param  ��
  * @retval ��
  */
// void OLED_I2C_Start(void) { /* �˺����ѱ��Ƴ�����Ӳ��I2C���� */ }

/**
  * @brief  I2C ֹͣ�ź�
  * @param  ��
  * @retval ��
  */
// void OLED_I2C_Stop(void) { /* �˺����ѱ��Ƴ�����Ӳ��I2C���� */ }

/**
  * @brief  I2C ����һ���ֽ�
  * @param  Byte Ҫ���͵�һ���ֽ�����
  * @retval ��
  */
// void OLED_I2C_SendByte(uint8_t Byte) { /* �˺����ѱ��Ƴ�����Ӳ��I2C���� */ }

/**
  * @brief  OLED д����
  * @param  Command Ҫд��������ֽ�
  * @retval ��
  */
void OLED_WriteCommand(uint8_t Command)
{
    // <--- **����Ӳ�� I2C1 ��д�������**
	I2C1_WriteCommand(Command);
}

/**
  * @brief  OLED д����
  * @param  Data Ҫд��������ֽ�
  * @retval ��
  */
void OLED_WriteData(uint8_t Data)
{
    // <--- **����Ӳ�� I2C1 ��д�����ݺ���**
	I2C1_WriteData(Data);
}

/**
  * @brief  OLED ���ù��λ��
  * @param  Y �����Ͻ�Ϊԭ�㣬���·�������꣬��Χ��0~7 (ҳ)
  * @param  X �����Ͻ�Ϊԭ�㣬���ҷ�������꣬��Χ��0~127 (��)
  * @retval ��
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					// ����ҳ��ַ (Page Address) ���B0-B7 ��Ӧҳ 0-7
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// �����е�ַ�� 4 λ���� (Column High Address)��10-1F
	OLED_WriteCommand(0x00 | (X & 0x0F));			// �����е�ַ�� 4 λ���� (Column Low Address)��00-0F
}

// ========================== �ײ�ͨ�ź��� ==========================

/**
 * @brief ��OLED�������ݵĺ���
 * @param data Ҫ���͵�����ָ��
 * @param len Ҫ���͵����ݳ���
 * @return None
 * @note �˺�������ֲ������ʱ����Ҫ������������������ֲ������ƽ̨ʱӦ����ʵ������޸Ĵ˺���
 * Ŀǰʵ����ѭ������ OLED_WriteData ��������
 */
void OLED_Send(uint8_t *data, uint8_t len) {
    // <--- **����Ӳ�� I2C1 �Ķ��ֽ�д�뺯��**
    // ���� data[0] �ǿ����ֽ� (0x40), data+1 ��ʵ���������ݣ�len-1 �����ݳ���
    I2C1_WriteMultiByte(OLED_I2C_ADDRESS, data[0], data + 1, len - 1);
}

// ========================== OLED �������� ==========================
/**
 * @brief ����OLED��ʾ
 */
void OLED_DisPlay_On() {
  OLED_WriteCommand(0x8D); // Charge Pump Setting (��ɱ�����)
  OLED_WriteCommand(0x14); // Enable Charge Pump (������ɱ�)
  OLED_WriteCommand(0xAF); // Display On (������Ļ)
}
/**
 * @brief �ر�OLED��ʾ
 */
void OLED_DisPlay_Off() {
  OLED_WriteCommand(0x8D); // Charge Pump Setting (��ɱ�����)
  OLED_WriteCommand(0x10); // Disable Charge Pump (�رյ�ɱ�)
  OLED_WriteCommand(0xAE); // Display Off (�ر���Ļ)
}

/**
 * @brief ������ɫģʽ �ڵװ��ֻ�׵׺���
 * @param mode ��ɫģʽö�� COLOR_NORMAL/COLOR_REVERSED
 * @note �˺���ֱ��������Ļ����ɫģʽ
 */
void OLED_SetColorMode(OLED_ColorMode mode) {
  if (mode == OLED_COLOR_NORMAL) {
    OLED_WriteCommand(0xA6); // Normal Display (������ʾ��1 ��ʾ��0 ����ʾ)
  }
  if (mode == OLED_COLOR_REVERSED) {
    OLED_WriteCommand(0xA7); // Inverse Display (��ɫ��ʾ��0 ��ʾ��1 ����ʾ)
  }
}

// ========================== �Դ�������� ==========================

/**
 * @brief ����Դ棬�����µ�һ֡
 * @note ���Դ滺���� OLED_GRAM ��������������
 */
void OLED_NewFrame(void) {
  memset(OLED_GRAM, 0, sizeof(OLED_GRAM)); // ʹ�� memset ������ OLED_GRAM ����������
}

/**
 * @brief ����ǰ�Դ���ʾ����Ļ��
 * @note �˺�������ֲ������ʱ����Ҫ������������������ֲ����������оƬʱӦ����ʵ������޸Ĵ˺���
 * Ŀǰʵ������ҳ���Դ����ݷ��͵� OLED
 */
void OLED_ShowFrame(void) {
  // ���ڴ洢�����ֽں͵�ǰҳ���ݵķ��ͻ�����
  static uint8_t sendBuffer[OLED_COLUMN + 1]; 
  // sendBuffer[0] = 0x40; // <-- **���б��ƶ��� memcpy ֮��**

  for (uint8_t i = 0; i < OLED_PAGE; i++) { // ����ÿһҳ (0-7)
    OLED_WriteCommand(0xB0 + i); // ���õ�ǰҪд���ҳ��ַ
    OLED_WriteCommand(0x02);     // �����е�ַ�� 4 λΪ 0x02 (ͨ������Ϊ 0x00��0x02ȡ���ھ����ͺź�����)
    OLED_WriteCommand(0x10);     // �����е�ַ�� 4 λΪ 0x10 (ͨ������Ϊ 0x10)
    
    // <--- **���������Ϳ����ֽ�λ��**
    // ����ǰҳ���Դ����ݿ��������ͻ������У��� sendBuffer[1] ��ʼ
    // ���� sendBuffer[0] ������Ϊ�����ֽڡ�
    sendBuffer[0] = 0x40; // ��һ���ֽ�Ϊ�����ֽڣ�0x40 ��ʾ������������
    memcpy(&sendBuffer[1], OLED_GRAM[i], OLED_COLUMN); 
    
    // ���Ͱ��������ֽں͵�ǰҳ���ݵĻ������� OLED
    OLED_Send(sendBuffer, OLED_COLUMN + 1);
  }
}

/**
 * @brief ����һ�����ص�
 * @param x ������ (��)����Χ��0~127
 * @param y ������ (��)����Χ��0~63
 * @param color ��ɫģʽ (COLOR_NORMAL: ��/�ף�COLOR_REVERSED: ��/��)
 * @note ���Դ滺���������û����һ�����ص��Ӧ��λ
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color) {
  // ��������Ƿ񳬳���Ļ��Χ
  if (x >= OLED_COLUMN || y >= OLED_ROW) return;
  
  // ���� y ����������ڵ�ҳ (y / 8) ��ҳ�ڵ�λ (y % 8)
  // ÿ���ֽڴ�ֱ��Ӧ 8 ������
  if (!color) { // �����������ɫģʽ (COLOR_NORMAL)����Ӧ�������� (����Ϊ 1)
    OLED_GRAM[y / 8][x] |= (1 << (y % 8)); // ����Ӧ�ֽڵĶ�Ӧλ����Ϊ 1
  } else { // ����Ƿ�ɫģʽ (COLOR_REVERSED)����ӦϨ������ (����Ϊ 0)
    OLED_GRAM[y / 8][x] &= ~(1 << (y % 8)); // ����Ӧ�ֽڵĶ�Ӧλ����Ϊ 0
  }
}

/**
 * @brief �����Դ���һ���ֽ����ݵ�ĳ��λ
 * @param page ҳ��ַ����Χ��0~7
 * @param column �е�ַ����Χ��0~127
 * @param data �����ֽ�
 * @param start ��ʼλ (0-7)
 * @param end ����λ (0-7)
 * @param color ��ɫģʽ
 * @note �˺������Դ���ĳһ�ֽڵĵ� start λ���� end λ����Ϊ�� data ��Ӧ��λ��ͬ
 * @note start �� end �ķ�ΧΪ 0-7, start ����С�ڵ��� end
 * @note �˺����� OLED_SetBits_Fine ���������ڴ˺���ֻ�������Դ��е�ĳһ��ʵ�ֽ�
 */
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color) {
  static uint8_t temp;
  // ���ҳ�����Ƿ񳬳���Χ
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  
  if (color) data = ~data; // ����Ƿ�ɫģʽ�������ݰ�λȡ��

  // ����һ�����룺��λ (end+1 �� 7) �͵�λ (0 �� start-1) ��Ϊ 1���м�λΪ 0
  temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
  // ʹ����������Դ��ж�Ӧ�ֽڵ��м�λ��start �� end��
  OLED_GRAM[page][column] &= temp;
  
  // ����һ�����룺��λ (end+1 �� 7) �͵�λ (0 �� start-1) ��Ϊ 0���м�λ�� data ��Ӧ
  temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
  // ʹ�����������Դ��ж�Ӧ�ֽڵ��м�λ��start �� end��Ϊ data ��Ӧ��ֵ
  OLED_GRAM[page][column] |= temp;
  
  // ������ʹ�� OLED_SetPixel ʵ�ֵ��߼�����ע�͵���
  // for (uint8_t i = start; i <= end; i++) {
  //   OLED_SetPixel(column, page * 8 + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief �����Դ��е�һ���ֽ�����
 * @param page ҳ��ַ����Χ��0~7
 * @param column �е�ַ����Χ��0~127
 * @param data �����ֽ�
 * @param color ��ɫģʽ
 * @note �˺������Դ��е�ĳһ�ֽ�����Ϊ data ��ֵ
 */
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color) {
  // ���ҳ�����Ƿ񳬳���Χ
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  
  if (color) data = ~data; // ����Ƿ�ɫģʽ�������ݰ�λȡ��
  
  OLED_GRAM[page][column] = data; // ֱ�ӽ�����д���Դ��Ӧ���ֽ�
}

/**
 * @brief �����Դ��е�һ���ֽ����ݵ�ĳ��λ
 * @param x ������ (��)
 * @param y ������ (��)
 * @param data �����ֽ� (�� len λ��Ч)
 * @param len λ������Χ��1-8
 * @param color ��ɫģʽ
 * @note �˺������Դ��д� (x,y) ��ʼ������ len λ����Ϊ�� data �ĵ� len λ��ͬ
 * @note len �ķ�ΧΪ 1-8
 * @note �˺����� OLED_SetByte_Fine ���������ڴ˺����ĺ��������������������Ϊ��λ��, ���ܳ��ֿ�������ʵ�ֽڵ����(��ҳ)
 */
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color) {
  uint8_t page = y / 8; // ������ʼ�������ڵ�ҳ
  uint8_t bit = y % 8;  // ������ʼ������ҳ�ڵ�λ (0-7)
  
  // �����Ҫ���õ�λ��Խ��ǰҳ����һҳ�ı߽� (��ǰҳʣ���λ���� len)
  if (bit + len > 8) {
    // �����õ�ǰҳ�Ĳ���λ���� bit �� 7���� 8 - bit λ
    // ��Ҫ�� data ���� bit λ��ʹ����Чλ��ҳ����ʼλ����
    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    // Ȼ��������һҳ�Ĳ���λ���� 0 �� len + bit - 1 - 8
    // ��Ҫ�� data ���� 8 - bit λ����ȡʣ�����Чλ
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8, color);
  } else { // �����Ҫ���õ�λ���ڵ�ǰҳ��
    // �� bit �� bit + len - 1���� len λ
    // ��Ҫ�� data ���� bit λ��ʹ����Чλ��ҳ����ʼλ����
    OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
  }
  
  // ������ʹ�� OLED_SetPixel ʵ�ֵ��߼�����ע�͵���
  // for (uint8_t i = 0; i < len; i++) {
  //   OLED_SetPixel(x, y + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief �����Դ���һ���ֽڳ��ȵ����� (8 λ��ֱ����)
 * @param x ������ (��)
 * @param y ������ (��)�������� 8 �ı��� (��ҳ����ʼ��)
 * @param data �����ֽ�
 * @param color ��ɫģʽ
 * @note �˺������Դ��д� (x,y) ��ʼ������ 8 λ����Ϊ�� data ��ͬ
 * @note �˺����� OLED_SetByte ���������ڴ˺����ĺ��������������������Ϊ��λ��, ���ܳ��ֿ�������ʵ�ֽڵ����(��ҳ)
 */
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color) {
  uint8_t page = y / 8; // ������ʼ�������ڵ�ҳ
  uint8_t bit = y % 8;  // ������ʼ������ҳ�ڵ�λ (0-7)
  
  // ���õ�ǰҳ�Ĳ���λ���� bit �� 7���� 8 - bit λ
  // ��Ҫ�� data ���� bit λ��ʹ����Чλ��ҳ����ʼλ����
  OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
  
  if (bit) { // �����ʼλ���� 0 (�� y ���� 8 �ı�������ҳ)
    // ������һҳ�Ĳ���λ���� 0 �� bit - 1���� bit λ
    // ��Ҫ�� data ���� 8 - bit λ����ȡ��һҳ��Ӧ��λ
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
  }
}

/**
 * @brief ����һ���Դ�����
 * @param x ��ʼ������
 * @param y ��ʼ������
 * @param data ���ݵ���ʼ��ַ (���ݰ�����������)
 * @param w ��� (����)
 * @param h �߶� (����)
 * @param color ��ɫģʽ
 * @note �˺������Դ��д� (x,y) ��ʼ�� w*h ����������Ϊ�� data �е�������ͬ
 * @note data �е�����Ӧ�������������У�����д��һ�еĴ�ֱ�������ݣ���д��һ�У�
 */
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color) {
  uint8_t fullRow = h / 8; // �����������ֽ����� (ÿ�� 8 ����)
  uint8_t partBit = h % 8; // ���㲻�������ֽ����е���Ч����λ��
  
  for (uint8_t i = 0; i < w; i++) { // ����ÿһ��
    for (uint8_t j = 0; j < fullRow; j++) { // ����ÿһ��ҳ��
      // ���õ�ǰ�� (x+i)����ǰҳ�� (y + j*8) ��һ�������ֽ� (8 λ��ֱ����)
      // data ������Ϊ i + j * w����Ϊ����������������
      OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
    }
  }
  
  if (partBit) { // ����в��������ֽ���
    uint16_t fullNum = w * fullRow; // ���������ֽ���ռ�õ������ֽ���
    for (uint8_t i = 0; i < w; i++) { // ����ÿһ��
      // ���õ�ǰ�� (x+i)���������ҳ�� (y + fullRow * 8) �� partialBit λ
      // data ������Ϊ fullNum + i������ǰ���������ֽ�����
      OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
    }
  }
  
  // ������ʹ�� OLED_SetPixel ʵ�ֵ��߼�����ע�͵���
  // for (uint8_t i = 0; i < w; i++) {
  //   for (uint8_t j = 0; j < h; j++) {
  //     for (uint8_t k = 0; k < 8; k++) {
  //       if (j * 8 + k >= h) break; // ��ֹԽ�� (���������ֽ�)
  //       OLED_SetPixel(x + i, y + j * 8 + k, !((data[i + j * w] >> k) & 0x01));
  //     }
  //   }
  // }
}

// ========================== ͼ�λ��ƺ��� ==========================

/**
 * @brief ����һ���߶�
 * @param x1 ��ʼ�������
 * @param y1 ��ʼ��������
 * @param x2 ��ֹ�������
 * @param y2 ��ֹ��������
 * @param color ��ɫģʽ
 * @note �˺���ʹ�� Bresenham �㷨�����߶�
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color) {
  static uint8_t temp = 0;
  // ����ֱ��
  if (x1 == x2) {
    if (y1 > y2) { // ȷ�� y1 С�ڵ��� y2
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++) { // ������
      OLED_SetPixel(x1, y, color);
    }
  }
  // ����ˮƽ��
  else if (y1 == y2) {
    if (x1 > x2) { // ȷ�� x1 С�ڵ��� x2
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++) { // ������
      OLED_SetPixel(x, y1, color);
    }
  }
  // ����б�� (Bresenham ֱ���㷨)
  else {
    int16_t dx = x2 - x1; // x �������
    int16_t dy = y2 - y1; // y �������
    // ȷ�� x, y ����Ĳ������� (1 �� -1)
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0; // ��ǰ������������

    // ȡ dx �� dy �ľ���ֵ
	if(dx<0) dx=-dx;
	//else dx=dx; // �����Ƕ����
	if(dy<0) dy=-dy;
	//else dy=dy; // �����Ƕ����
    // dx = abs(dx); // ԭע���б�ע�͵��� stdlib �����汾
    // dy = abs(dy); // ԭע���б�ע�͵��� stdlib �����汾

    // ���� dx �� dy �Ĵ�Сѡ��������
    if (dx > dy) { // x ��������
      for (x = x1; x != x2; x += ux) { // �� x �������
        OLED_SetPixel(x, y, color); // ���Ƶ�ǰ��
        eps += dy; // ��������� dy
        if ((eps << 1) >= dx) { // ��������ﵽ��ֵ
          y += uy; // �� y ���򲽽�
          eps -= dx; // ������ȥ dx
        }
      }
    } else { // y ��������
      for (y = y1; y != y2; y += uy) { // �� y �������
        OLED_SetPixel(x, y, color); // ���Ƶ�ǰ��
        eps += dx; // ��������� dx
        if ((eps << 1) >= dy) { // ��������ﵽ��ֵ
          x += ux; // �� x ���򲽽�
          eps -= dy; // ������ȥ dy
        }
      }
    }
  }
}

/**
 * @brief ����һ�����ο�
 * @param x ��ʼ�������
 * @param y ��ʼ��������
 * @param w ���ο��
 * @param h ���θ߶�
 * @param color ��ɫģʽ
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  // ������������
  OLED_DrawLine(x, y, x + w, y, color);       // ����
  OLED_DrawLine(x, y + h, x + w, y + h, color); // �ױ�
  OLED_DrawLine(x, y, x, y + h, color);       // ���
  OLED_DrawLine(x + w, y, x + w, y + h, color); // �ұ�
}

/**
 * @brief ����һ��������
 * @param x ��ʼ�������
 * @param y ��ʼ��������
 * @param w ���ο��
 * @param h ���θ߶�
 * @param color ��ɫģʽ
 */
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  // ���ϵ������л���ˮƽ����������
  for (uint8_t i = 0; i < h; i++) {
    OLED_DrawLine(x, y+i, x+w-1, y+i, color); // ����һ��ˮƽ��
  }
}

/**
 * @brief ����һ�������ο�
 * @param x1 ��һ���������
 * @param y1 ��һ����������
 * @param x2 �ڶ����������
 * @param y2 �ڶ�����������
 * @param x3 �������������
 * @param y3 ��������������
 * @param color ��ɫģʽ
 */
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  // ������������
  OLED_DrawLine(x1, y1, x2, y2, color);
  OLED_DrawLine(x2, y2, x3, y3, color);
  OLED_DrawLine(x3, y3, x1, y1, color);
}

/**
 * @brief ����һ�����������
 * @param x1 ��һ���������
 * @param y1 ��һ����������
 * @param x2 �ڶ����������
 * @param y2 �ڶ�����������
 * @param x3 �������������
 * @param y3 ��������������
 * @param color ��ɫģʽ
 * @note �˴�������������㷨���ܲ�������������⣬�����޷���ȷ�����������������͡�
 */
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  uint8_t a = 0, b = 0, y = 0, last = 0;
  // �ҵ� y1 �� y2 �е���Сֵ�����ֵ
  if (y1 > y2) {
    a = y2;
    b = y1;
  } else {
    a = y1;
    b = y2;
  }
  y = a;
  // ���ƴ� y=a �� y=b ֮���ˮƽ�߶�
  for (; y <= b; y++) {
    if (y <= y3) { // �����ǰ���ڵ�3���Ϸ���ˮƽλ��
      // �������� (x1, y1) - (x2, y2) �� (x1, y1) - (x3, y3) ���϶�Ӧ y �����ˮƽ�߶�
      // ע�⣺����Ĳ�ֵ���� (y - y1) * (x2 - x1) / (y2 - y1) ���ܻ���Ϊ����������ʧ���ȣ�����б�ʽ�С���߶ο��ܲ�׼ȷ
      OLED_DrawLine(x1 + (y - y1) * (x2 - x1) / (y2 - y1), y, x1 + (y - y1) * (x3 - x1) / (y3 - y1), y, color);
    } else { // �����ǰ���ڵ�3���·�
      last = y - 1; // ��¼��һ�� y ����
      break; // �˳�ѭ��
    }
  }
  // ����ʣ�ಿ�ֵ�ˮƽ�߶�
  for (; y <= b; y++) {
    // �������� (x2, y2) - (x3, y3) �� (x1, last) - (x3, y3) (�ƺ��߼��е�����) ���϶�Ӧ y �����ˮƽ�߶�
     OLED_DrawLine(x2 + (y - y2) * (x3 - x2) / (y3 - y2), y, x1 + (y - last) * (x3 - x1) / (y3 - last), y, color);
  }
}

/**
 * @brief ����һ��Բ��
 * @param x Բ�ĺ�����
 * @param y Բ��������
 * @param r Բ�뾶
 * @param color ��ɫģʽ
 * @note �˺���ʹ�� Bresenham �㷨����Բ
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  // Bresenham Բ�㷨����
  int16_t a = 0, b = r, di = 3 - (r << 1);
  // ����Բ���İ˷�֮һ���֣���ͨ���ԳƵõ�����Բ
  while (a <= b) {
    // ���ƶԳƵİ˸���
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x + b, y - a, color);
    OLED_SetPixel(x - a, y + b, color);
    OLED_SetPixel(x - b, y - a, color); // �ظ����ƣ��˴������Ǹ��ƴ���
    OLED_SetPixel(x - a, y - b, color);
    OLED_SetPixel(x + b, y + a, color);
    OLED_SetPixel(x + a, y - b, color);
    OLED_SetPixel(x + a, y + b, color);
    OLED_SetPixel(x - b, y + a, color); // �ظ����ƣ��˴������Ǹ��ƴ���
    
    a++; // �� x ���򲽽�
    // ���¾��߲��� di
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--; // �� y ���򲽽�
    }
    OLED_SetPixel(x + a, y + b, color); // ������һ���� (�����Ӧ����ѭ���⴦�������ڶԳƵ���)
  }
}

/**
 * @brief ����һ�����Բ
 * @param x Բ�ĺ�����
 * @param y Բ��������
 * @param r Բ�뾶
 * @param color ��ɫģʽ
 * @note �˺���ʹ������Բ�㷨�ķ����������Բ��ͨ������ˮƽ�߶����
 */
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  // Bresenham Բ�㷨����
  int16_t a = 0, b = r, di = 3 - (r << 1);
  while (a <= b) {
    // ���ƶԳƵ�ˮƽ�߶�
    for (int16_t i = x - b; i <= x + b; i++) { // ���ƴ�ֱ�� y ���ˮƽ�߶�
      OLED_SetPixel(i, y + a, color);
      OLED_SetPixel(i, y - a, color);
    }
    for (int16_t i = x - a; i <= x + a; i++) { // ���ƴ�ֱ�� x ���ˮƽ�߶�
      OLED_SetPixel(i, y + b, color);
      OLED_SetPixel(i, y - b, color);
    }
    a++; // �� x ���򲽽�
    // ���¾��߲��� di
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--; // �� y ���򲽽�
    }
  }
}

/**
 * @brief ����һ����Բ��
 * @param x ��Բ���ĺ�����
 * @param y ��Բ����������
 * @param a ��Բ���������һ�볤�� (ˮƽ����)
 * @param b ��Բ�����������һ�볤�� (��ֱ����)
 * @param color ��ɫģʽ
 * @note ʹ���е���Բ�㷨����
 */
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color) {
  int xpos = 0, ypos = b; // ��ʼ�� (0, b)
  int a2 = a * a, b2 = b * b; // a^2 �� b^2
  int d = b2 + a2 * (0.25 - b); // ���߲�����ʼֵ (���� 1)
  // ���� 1 (б�ʾ���ֵ < 1)
  while (a2 * ypos > b2 * xpos) {
    // ���ƶԳƵ��ĸ���
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    
    if (d < 0) { // ���߲��� < 0����һ����ȡ E
      d = d + b2 * ((xpos << 1) + 3);
      xpos += 1;
    } else { // ���߲��� >= 0����һ����ȡ SE
      d = d + b2 * ((xpos << 1) + 3) + a2 * (-(ypos << 1) + 2);
      xpos += 1;
      ypos -= 1;
    }
  }
  
  // ���� 2 (б�ʾ���ֵ > 1)
  d = b2 * (xpos + 0.5) * (xpos + 0.5) + a2 * (ypos - 1) * (ypos - 1) - a2 * b2; // ���߲�����ʼֵ (���� 2)
  while (ypos > 0) {
    // ���ƶԳƵ��ĸ���
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    
    if (d < 0) { // ���߲��� < 0����һ����ȡ SE
      d = d + b2 * ((xpos << 1) + 2) + a2 * (-(ypos << 1) + 3);
      xpos += 1;
      ypos -= 1;
    } else { // ���߲��� >= 0����һ����ȡ S
      d = d + a2 * (-(ypos << 1) + 3);
      ypos -= 1;
    }
  }
}

/**
 * @brief ����һ��ͼƬ
 * @param x ��ʼ�������
 * @param y ��ʼ��������
 * @param img ͼƬ�ṹ��ָ�룬�������ݡ���Ⱥ͸߶�
 * @param color ��ɫģʽ
 * @note ͼƬ����ӦΪ���������еĵ�ɫλͼ����
 */
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color) {
  OLED_SetBlock(x, y, img->data, img->w, img->h, color); // ���� SetBlock ��������ͼƬ
}

// ================================ ���ֻ��� ================================

// ����һ�� ASCII ���� (ʹ�����������ָ������)
void OLED_PrintASCIINum(uint8_t x, uint8_t y, uint32_t Number,uint8_t Length, const ASCIIFont *font, OLED_ColorMode color)
{
	uint8_t i;
    // ѭ�� Length �Σ������λ��ʼ��ӡ���ֵ�ÿһλ
	for(int i = 0; i < Length; i++)
	{
        // ���㵱ǰλ�����֣���ת��Ϊ��Ӧ�� ASCII �ַ�
        // ���磬�������� 123, Length=3:
        // i=0: 123 / 100 % 10 + '0' = 1 + '0' -> '1'
        // i=1: 123 / 10 % 10 + '0' = 2 + '0' -> '2'
        // i=2: 123 / 1 % 10 + '0' = 3 + '0' -> '3'
		OLED_PrintASCIIChar(x + i * font->w, y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', font, color); // ����: x ����Ӧ���ַ�������ӣ�y ���걣�ֲ���
	}
}

/**
 * @brief ����һ�� ASCII �ַ�
 * @param x ��ʼ������� (����)
 * @param y ��ʼ�������� (����)
 * @param ch �ַ�
 * @param font ����ṹ��ָ��
 * @param color ��ɫģʽ
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color) {
  // ����ַ��Ƿ��ڿ���ʾ ASCII ��Χ (' ' �� '~')
  if (ch < ' ' || ch > '~') return;
  
  // �����ַ������������е�ƫ����
  // (ch - ' ') �õ��ַ�����ڿո������
  // ÿ���ַ�ռ�õ��ֽ��� = (����߶� + 7) / 8 * ������ (����ȡ�����ֽ�)
  // chars + ƫ���� ָ����ַ���λͼ����
  const uint8_t *char_data = font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w);
  // ʹ�� SetBlock ���������ַ�λͼ
  OLED_SetBlock(x, y, char_data, font->w, font->h, color);
}

/**
 * @brief ����һ�� ASCII �ַ���
 * @param x ��ʼ������� (����)
 * @param y ��ʼ�������� (����)
 * @param str �ַ���
 * @param font ����ṹ��ָ��
 * @param color ��ɫģʽ
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color) {
  uint8_t x0 = x; // ��¼��ʼ x ���꣬���ڰ��������ۼ�
  while (*str) { // �����ַ���ֱ���������ַ� '\0'
    OLED_PrintASCIIChar(x0, y, *str, font, color); // ���Ƶ�ǰ�ַ�
    x0 += font->w; // x ���������ƶ�һ���ַ��Ŀ��
    str++;       // ָ����һ���ַ�
  }
}

/**
 * @brief ��ȡ UTF-8 ������ַ����� (�ֽ���)
 * @param string ָ�� UTF-8 �����ַ�����ָ��
 * @return �ַ����ֽڳ��� (1-4)�����������Ч�� UTF-8 ��ͷ�򷵻� 0
 */
uint8_t _OLED_GetUTF8Len(char *string) {
  if ((string[0] & 0x80) == 0x00) { // 0xxxxxxx - ���ֽ� ASCII
    return 1;
  } else if ((string[0] & 0xE0) == 0xC0) { // 110xxxxx - ˫�ֽ� UTF-8 ��ͷ
    return 2;
  } else if ((string[0] & 0xF0) == 0xE0) { // 1110xxxx - ���ֽ� UTF-8 ��ͷ
    return 3;
  } else if ((string[0] & 0xF8) == 0xF0) { // 11110xxx - ���ֽ� UTF-8 ��ͷ
    return 4;
  }
  return 0; // ������Ч�� UTF-8 ��ͷ
}

/**
 * @brief �����ַ��� (֧�� ASCII �� UTF-8 ����Ķ��ֽ��ַ���������)
 * @param x ��ʼ������� (����)
 * @param y ��ʼ�������� (����)
 * @param str �ַ��� (UTF-8 ����)
 * @param font ����ṹ��ָ�� (���� ASCII ������Զ�����������)
 * @param color ��ɫģʽ
 *
 * @note Ϊȷ���ַ����е����Ļᱻ�Զ�ʶ�𲢻��ƣ���:
 * 1. �������ַ�������Ϊ UTF-8
 * 2. ʹ���ض���ȡģ������������ (�� https://led.baud-dance.com)
 */
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color) {
  uint16_t i = 0;                                       // �ַ�����ǰ���������
  // ����һ��������ģ�����������λͼ���ݣ�ռ�õ��ֽ���
  // = ((����߶� + 7) / 8) * ������ (λͼ�����ֽ���) + 4 (�����ֽ���������Ϊ 4)
  uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; 
  uint8_t found;                                        // ��־����ʾ�Ƿ����Զ��������������ҵ��ַ�
  uint8_t utf8Len;                                      // ��ǰ�ַ��� UTF-8 ���볤�� (�ֽ���)
  uint8_t *head;                                        // ָ�����������е�ǰ��ģ��ͷ��

  while (str[i]) { // �����ַ���ֱ���������ַ� '\0'
    found = 0; // ����δ�ҵ�
    utf8Len = _OLED_GetUTF8Len(str + i); // ��ȡ��ǰ�ַ��� UTF-8 ���볤��
    if (utf8Len == 0) { // ���������Ч�� UTF-8 �ַ�
        // ��ӡһ���ո�ռλ����������Ч�ֽ� (���߿��Դ�ӡ�ʺŵ�)
        OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
        x += font->ascii->w; // �ƶ� x ����
        i++; // ֻ����һ���ֽڣ���Ϊ��֪����Ч�ַ���ʵ�ʳ���
        continue; // ������һ���ַ�
    }

    // �����Զ�����������
    // TODO �Ż������㷨, ����ʹ�ö��ֲ��һ� hash �����Ч�ʣ�Ŀǰ�����Բ���
    for (uint8_t j = 0; j < font->len; j++) { // �����Զ������������е�������ģ
      head = (uint8_t *)(font->chars) + (j * oneLen); // ���㵱ǰ��ģ����ʼ��ַ
      // �Ƚϵ�ǰ�ַ����е��ַ���������ģ�еı��� (ǰ utf8Len ���ֽ�)
      if (memcmp(str + i, head, utf8Len) == 0) { // ����ҵ�ƥ�����ģ
        // ��ģ����ͨ�����������λͼ��λͼ�����ڱ���֮��ƫ�� 4 ���ֽ� (���ݹ������ɸ�ʽȷ��)
        OLED_SetBlock(x, y, head + 4, font->w, font->h, color); // ������ģ��λͼ����
        x += font->w;     // x ���������ƶ�һ����ģ�Ŀ��
        i += utf8Len;   // �ַ�������������ǰ�ַ����ֽ���
        found = 1;      // ���Ϊ���ҵ�
        break;          // �˳���ģ����ѭ��
      }
    }

    // ������Զ���������δ�ҵ����ҵ�ǰ�ַ��� ASCII �ַ� (����Ϊ 1)
    if (found == 0) {
      if (utf8Len == 1) {
        // ���� ASCII �ַ�
        OLED_PrintASCIIChar(x, y, str[i], font->ascii, color);
        x += font->ascii->w; // x ���������ƶ�һ�� ASCII �ַ��Ŀ��
        i += utf8Len;     // �ַ��������� 1
      } else { // ����Ƕ��ֽ��ַ���δ�ҵ���ģ
        // ����һ���ո�ռλ
        OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
        x += font->ascii->w; // x ���������ƶ�
        i += utf8Len;     // �ַ�������������ǰ�ַ����ֽ���
      }
    }
  }
}

/**
  * @brief  OLED ����
  * @param  ��
  * @retval ��
  * @note �˺����������ҳ�� 0 �� 7 ������
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
    // ��ҳ 0 ������ҳ 7
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0); // ���ù�굽��ǰҳ����ʼ��
		for(i = 0; i < 128; i++) // ������ǰҳ��������
		{
			OLED_WriteData(0x00); // д�� 0x00��������е� 8 ������
		}
	}
}
/**
  * @brief  OLED �η����� (���� X �� Y ����)
  * @param  X ����
  * @param  Y ָ��
  * @retval ����ֵ���� X �� Y ����
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1; // �����ʼ��Ϊ 1
	while (Y--) // ѭ�� Y ��
	{
		Result *= X; // ÿ��ѭ������ X
	}
	return Result; // ���ؼ�����
}

/**
  * @brief  OLED ��ʼ��
  * @param  ��
  * @retval ��
  * @note ����һϵ���������� OLED ������
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	// �ϵ���ʱ���ȴ� OLED �ȶ�
	for (i = 0; i < 1000; i++)			
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			// ��ʼ�� I2C ����

	// ���ͳ�ʼ����������
	OLED_WriteCommand(0xAE);	// Set Display OFF (�ر���ʾ)
	
	OLED_WriteCommand(0xD5);	// Set Display Clock Divide Ratio/Oscillator Frequency (������ʾʱ�ӷ�Ƶ��/����Ƶ��)
	OLED_WriteCommand(0x80);    // Ĭ��ֵ
	
	OLED_WriteCommand(0xA8);	// Set Multiplex Ratio (���ö�·������)
	OLED_WriteCommand(0x3F);    // 64 ����ʾ (0-63)
	
	OLED_WriteCommand(0xD3);	// Set Display Offset (������ʾƫ��)
	OLED_WriteCommand(0x00);    // ƫ�� 0
	
	OLED_WriteCommand(0x40);	// Set Display Start Line (������ʾ��ʼ��) - 0x40 + 0 (�� 0)
	
	OLED_WriteCommand(0xA1);	// Set Segment Re-map (���ö���ӳ��)��A0 ������A1 ���ҷ�ת
	
	OLED_WriteCommand(0xC8);	// Set COM Output Scan Direction (���� COM ɨ�跽��)��C0 ������C8 ���·�ת

	OLED_WriteCommand(0xDA);	// Set COM Pins Hardware Configuration (���� COM ����Ӳ������)
	OLED_WriteCommand(0x12);    // (���� 128x64 ��Ļ)
	
	OLED_WriteCommand(0x81);	// Set Contrast Control (���öԱȶȿ���)
	OLED_WriteCommand(0xCF);    // ���öԱȶ�ֵ (0x00 �� 0xFF��CF ��һ���м�ֵ)

	OLED_WriteCommand(0xD9);	// Set Pre-charge Period (����Ԥ�������)
	OLED_WriteCommand(0xF1);    // Ĭ��ֵ

	OLED_WriteCommand(0xDB);	// Set VCOMH Deselect Level (���� VCOMH ȥѡ�񼶱�)
	OLED_WriteCommand(0x30);    // Ĭ��ֵ

	OLED_WriteCommand(0xA4);	// Set Entire Display On/Off (����������ʾ����/�رգ�A4 ������ʾ��A5 �����Դ棬ȫ������)

	OLED_WriteCommand(0xA6);	// Set Normal/Inverse Display (��������/��ɫ��ʾ��A6 ������A7 ��ɫ)

	OLED_WriteCommand(0x8D);	// Set Charge Pump Setting (���õ�ɱ�)
	OLED_WriteCommand(0x14);    // Enable Charge Pump (������ɱ�)

	OLED_WriteCommand(0xAF);	// Set Display ON (������ʾ)
		
	OLED_Clear();				// ��� OLED ��Ļ���� (���ҳ 2-7)
}