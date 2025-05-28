#ifndef __FONT_H
#define __FONT_H
#include "stdint.h"
#include "string.h"
typedef struct ASCIIFont {
  uint8_t h;
  uint8_t w;
  uint8_t *chars;
} ASCIIFont;

extern const ASCIIFont afont8x6;
extern const ASCIIFont afont12x6;
extern const ASCIIFont afont16x8;
extern const ASCIIFont afont24x12;

/**
 * @brief ����ṹ��
 * @note  �ֿ�ǰ4�ֽڴ洢utf8���� ʣ���ֽڴ洢��ģ����
 * @note �ֿ����ݿ���ʹ�ò����ɶ�LEDȡģ��������(https://led.baud-dance.com)
 */
typedef struct Font {
  uint8_t h;              // �ָ߶�
  uint8_t w;              // �ֿ��
  const uint8_t *chars;   // �ֿ� �ֿ�ǰ4�ֽڴ洢utf8���� ʣ���ֽڴ洢��ģ����
  uint8_t len;            // �ֿⳤ�� ����256�����Ϊuint16_t
  const ASCIIFont *ascii; // ȱʡASCII���� ���ֿ���û�ж�Ӧ�ַ�����Ҫ��ʾASCII�ַ�ʱʹ��
} Font;

extern const Font font16x16;
extern const Font font14x14;
/**
 * @brief ͼƬ�ṹ��
 * @note  ͼƬ���ݿ���ʹ�ò����ɶ�LEDȡģ��������(https://led.baud-dance.com)
 */
typedef struct Image {
  uint8_t w;           // ͼƬ���
  uint8_t h;           // ͼƬ�߶�
  const uint8_t *data; // ͼƬ����
} Image;

extern const Image circleImg;
extern const Image rockerImg;
extern const Image buttomImg;
#endif // __FONT_H
