/* Blink Example

这个例子是点亮LCD屏的例子
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0xF81F
#define GRED 0xFFE0
#define GBLUE 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
or you can edit the following line and set a number here.
*/



uint8_t number0_18[] = {
0x00,0x7E,0x00,0x80,0xFF,0x01,0xC0,0xFF,0x03,0xE0,0xFF,0x07,0xF0,0xC3,0x0F,0xF0,
0x80,0x0F,0xF0,0x00,0x0F,0xF8,0x00,0x1F,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,
0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,
0x78,0x00,0x1E,0x78,0x00,0x1E,0xF8,0x00,0x1F,0xF0,0x00,0x0F,0xF0,0x00,0x0F,0xF0,
0xC3,0x0F,0xE0,0xFF,0x07,0xE0,0xFF,0x03,0x80,0xFF,0x01,0x00,0x7E,0x00,/*"0",0*/
};

uint8_t number1_18[] = {
0x00,0x70,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x7C,0x00,0x00,0x7F,0x00,0x80,
0x7F,0x00,0xE0,0x7B,0x00,0xE0,0x79,0x00,0x60,0x78,0x00,0x00,0x78,0x00,0x00,0x78,
0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,
0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,
0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,/*"1",0*/
}; // 24 * 26


uint8_t number2_18[] = {
0x00,0x7F,0x00,0xE0,0xFF,0x03,0xF0,0xFF,0x07,0xF8,0xFF,0x0F,0xF8,0xC0,0x0F,0x7C,
0x00,0x1F,0x3C,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,
0x0F,0x00,0x80,0x0F,0x00,0xC0,0x07,0x00,0xE0,0x03,0x00,0xF0,0x01,0x00,0xF8,0x00,
0x00,0x7E,0x00,0x00,0x3F,0x00,0xC0,0x0F,0x00,0xE0,0x07,0x00,0xF0,0x03,0x00,0xF0,
0x00,0x00,0x78,0x00,0x00,0xFC,0xFF,0x1F,0xFC,0xFF,0x1F,0xFC,0xFF,0x1F,/*"2",0*/
};

uint8_t number3_18[] = {
0x00,0x3F,0x00,0xC0,0xFF,0x00,0xE0,0xFF,0x03,0xF0,0xFF,0x03,0xF0,0xE1,0x07,0xF8,
0x80,0x07,0x78,0x80,0x07,0x00,0x80,0x07,0x00,0x80,0x03,0x00,0xE0,0x03,0x00,0xFC,
0x00,0x00,0xFC,0x00,0x00,0xFC,0x03,0x00,0x80,0x07,0x00,0x00,0x0F,0x00,0x00,0x1E,
0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x78,0x00,0x1E,0xF8,0x00,0x0F,0xF0,
0x81,0x0F,0xF0,0xFF,0x07,0xE0,0xFF,0x03,0xC0,0xFF,0x01,0x00,0x7F,0x00,/*"3",0*/
};

uint8_t number4_18[] = {
0x00,0xC0,0x01,0x00,0xE0,0x01,0x00,0xF0,0x01,0x00,0xF0,0x01,0x00,0xF8,0x01,0x00,
0xFC,0x01,0x00,0xFE,0x01,0x00,0xFE,0x01,0x00,0xFF,0x01,0x80,0xEF,0x01,0x80,0xE7,
0x01,0xC0,0xE7,0x01,0xE0,0xE3,0x01,0xF0,0xE1,0x01,0xF0,0xE1,0x01,0xF8,0xE0,0x01,
0x7C,0xE0,0x01,0xFC,0xFF,0x1F,0xFC,0xFF,0x1F,0xFC,0xFF,0x1F,0x00,0xE0,0x01,0x00,
0xE0,0x01,0x00,0xE0,0x01,0x00,0xE0,0x01,0x00,0xE0,0x01,0x00,0xE0,0x01,/*"4",0*/
};

uint8_t number5_18[] = {
0xC0,0xFF,0x07,0xC0,0xFF,0x07,0xE0,0xFF,0x07,0xE0,0x01,0x00,0xE0,0x01,0x00,0xE0,
0x01,0x00,0xF0,0x01,0x00,0xF0,0x00,0x00,0xF0,0xFC,0x00,0xF0,0xFE,0x03,0xF0,0xFF,
0x07,0xF8,0xFF,0x0F,0xF8,0x80,0x0F,0x00,0x00,0x1F,0x00,0x00,0x1E,0x00,0x00,0x1E,
0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x78,0x00,0x1E,0xF8,0x00,0x0F,0xF0,
0x81,0x0F,0xF0,0xFF,0x07,0xE0,0xFF,0x03,0xC0,0xFF,0x01,0x00,0x7F,0x00,/*"5",0*/
};

uint8_t number6_18[] = {
0x00,0xFE,0x00,0x80,0xFF,0x03,0xC0,0xFF,0x07,0xE0,0xFF,0x0F,0xF0,0x81,0x0F,0xF8,
0x00,0x1F,0x78,0x00,0x1E,0x78,0x00,0x00,0x3C,0x00,0x00,0x3C,0xFC,0x00,0x3C,0xFF,
0x03,0xBC,0xFF,0x07,0xFC,0xFF,0x0F,0xFC,0x81,0x0F,0x7C,0x00,0x1F,0x3C,0x00,0x1E,
0x3C,0x00,0x1E,0x3C,0x00,0x1E,0x3C,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x0F,0xF0,
0x81,0x0F,0xF0,0xFF,0x07,0xE0,0xFF,0x07,0xC0,0xFF,0x01,0x00,0x7E,0x00,/*"6",0*/
};

uint8_t number7_18[] = {
0xF8,0xFF,0x1F,0xF8,0xFF,0x1F,0xF8,0xFF,0x1F,0x00,0x00,0x0E,0x00,0x00,0x07,0x00,
0x80,0x07,0x00,0xC0,0x03,0x00,0xE0,0x01,0x00,0xE0,0x01,0x00,0xF0,0x00,0x00,0xF0,
0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x1E,0x00,
0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x0F,0x00,0x00,0x0F,0x00,0x00,0x0F,0x00,0x00,
0x0F,0x00,0x80,0x07,0x00,0x80,0x07,0x00,0x80,0x07,0x00,0x80,0x07,0x00,/*"7",0*/
};

uint8_t number8_18[] = {
0x00,0x7E,0x00,0x80,0xFF,0x01,0xE0,0xFF,0x07,0xE0,0xFF,0x07,0xF0,0x81,0x0F,0xF0,
0x00,0x0F,0xF0,0x00,0x0F,0xF0,0x00,0x0F,0xE0,0x81,0x07,0xC0,0xFF,0x03,0x80,0xFF,
0x01,0x80,0xFF,0x01,0xE0,0xFF,0x07,0xF0,0x81,0x0F,0xF0,0x00,0x0F,0x78,0x00,0x1E,
0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0xF8,0x00,0x1F,0xF0,
0x81,0x0F,0xF0,0xFF,0x0F,0xE0,0xFF,0x07,0x80,0xFF,0x01,0x00,0x7E,0x00,/*"8",0*/
};

uint8_t number9_18[] = {
0x00,0x7F,0x00,0x80,0xFF,0x01,0xE0,0xFF,0x03,0xF0,0xFF,0x07,0xF0,0x81,0x0F,0xF8,
0x00,0x0F,0x78,0x00,0x0E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,
0x1E,0xF8,0x00,0x1F,0xF0,0x81,0x1F,0xF0,0xFF,0x1F,0xE0,0xFF,0x1E,0xC0,0x7F,0x1E,
0x00,0x1F,0x1E,0x00,0x00,0x1E,0x00,0x00,0x0F,0x78,0x00,0x0F,0xF8,0x80,0x0F,0xF0,
0xC1,0x07,0xF0,0xFF,0x03,0xE0,0xFF,0x03,0xC0,0xFF,0x00,0x00,0x3F,0x00,/*"9",0*/
};



#define SIZE096 1
#define SIZE35 2
#define SIZE20 3 // st7789V

// #define width 320
// #define height 480
// #define width 160
// #define height 104 // 80 + 24（24是一个偏移量，具体与硬件有关系，这是一个坑，影响坐标）

#define width 320
#define height 240

#define wramcmd 0X2C
#define setxcmd 0X2A
#define setycmd 0X2B

#define SIZE SIZE20
#if SIZE == SIZE35

  #define BACKLIGHT_GPIO GPIO_NUM_2
  #define CS_GPIO GPIO_NUM_12
  #define SCL_GPIO GPIO_NUM_14
  #define SDA_GPIO GPIO_NUM_13
  #define RS_GPIO GPIO_NUM_15
  #define RST_GPIO GPIO_NUM_4

#elif SIZE == SIZE096

  #define BACKLIGHT_GPIO GPIO_NUM_12//低电平有效
  #define CS_GPIO GPIO_NUM_32
  #define SCL_GPIO GPIO_NUM_2
  #define SDA_GPIO GPIO_NUM_3
  #define RS_GPIO GPIO_NUM_33 //dc
  #define RST_GPIO GPIO_NUM_15

#elif SIZE == SIZE20

  #define BACKLIGHT_GPIO GPIO_NUM_2//低电平有效
  #define CS_GPIO GPIO_NUM_12
  #define SCL_GPIO GPIO_NUM_14
  #define SDA_GPIO GPIO_NUM_13
  #define RS_GPIO GPIO_NUM_15 //dc
  #define RST_GPIO GPIO_NUM_5
  #define SPI8 1
#else

#endif

#define CS0_L gpio_set_level(CS_GPIO, 0)  //片选
#define CS0_H gpio_set_level(CS_GPIO, 1)
#define SCL_L gpio_set_level(SCL_GPIO, 0)  //时钟
#define SCL_H gpio_set_level(SCL_GPIO, 1)
#define SDA_L gpio_set_level(SDA_GPIO, 0)//MOSI
#define SDA_H  gpio_set_level(SDA_GPIO, 1)
#define RS_L gpio_set_level(RS_GPIO, 0)// D/C
#define RS_H gpio_set_level(RS_GPIO, 1)//

#if SIZE == SIZE35
  #define BL_C gpio_set_level(BACKLIGHT_GPIO, 0)//背光
  #define BL_O gpio_set_level(BACKLIGHT_GPIO, 1)
  #define RST_L Nop()//gpio_set_level(RST_GPIO, 0) //
  #define RST_H Nop()//gpio_set_level(RST_GPIO, 1)

#elif SIZE == SIZE096
  #define BL_C gpio_set_level(BACKLIGHT_GPIO, 0)//背光
  #define BL_O gpio_set_level(BACKLIGHT_GPIO, 1)
  #define RST_L gpio_set_level(RST_GPIO, 0) //
  #define RST_H gpio_set_level(RST_GPIO, 1)
#elif SIZE == SIZE20
  #define BL_C gpio_set_level(BACKLIGHT_GPIO, 0)//背光
  #define BL_O gpio_set_level(BACKLIGHT_GPIO, 1)
  #define RST_L gpio_set_level(RST_GPIO, 0) //
  #define RST_H gpio_set_level(RST_GPIO, 1)
#endif


//------------------------------------------
void Nop(){

}

void  SendDataSPI(unsigned char dat)
{  
unsigned char i;

for(i=0; i<8; i++)
    {  
    if( (dat&0x80)!=0 ) SDA_H;
    else SDA_L;

    dat <<= 1;

        SCL_L;
    SCL_H;
    }
}

void WriteComm(unsigned int i)
{

  CS0_L;
	RS_L;
	SendDataSPI(i);

	CS0_H;

}
void WriteData(unsigned int i)
{
  CS0_L;
	RS_H;

	SendDataSPI(i);

	CS0_H;
}

// void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
// {
// 	//ILI9163C
// 	WriteComm(0x2A);
// 	WriteData(Xstart>>8);
// 	WriteData(Xstart);
// 	WriteData(Xend>>8);
// 	WriteData(Xend);
	
// 	WriteComm(0x2B);
// 	WriteData(Ystart>>8);
// 	WriteData(Ystart);
// 	WriteData(Yend>>8);
// 	WriteData(Yend);
	
// 	WriteComm(0x2c);
// }

void LCD_Init(void)
{
    CS0_L;
    // // RST=1;  //复位
    // Delay(200);
    RST_H; 
    vTaskDelay(100 / portTICK_PERIOD_MS);
    RST_L;//复位
    // Delay(800);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    RST_H;//复位
    // Delay(800);

#if SIZE == SIZE35
    WriteComm(0xE0);
    WriteData(0x00);
    WriteData(0x07);
    WriteData(0x0f);
    WriteData(0x0D);
    WriteData(0x1B);
    WriteData(0x0A);
    WriteData(0x3c);
    WriteData(0x78);
    WriteData(0x4A);
    WriteData(0x07);
    WriteData(0x0E);
    WriteData(0x09);
    WriteData(0x1B);
    WriteData(0x1e);
    WriteData(0x0f);
    
    WriteComm(0xE1);
    WriteData(0x00);
    WriteData(0x22);
    WriteData(0x24);
    WriteData(0x06);
    WriteData(0x12);
    WriteData(0x07);
    WriteData(0x36);
    WriteData(0x47);
    WriteData(0x47);
    WriteData(0x06);
    WriteData(0x0a);
    WriteData(0x07);
    WriteData(0x30);
    WriteData(0x37);
    WriteData(0x0f);

    WriteComm(0xC0);
    WriteData(0x10);
    WriteData(0x10);

    WriteComm(0xC1);
    WriteData(0x41);

    WriteComm(0xC5);
    WriteData(0x00);
    WriteData(0x22);
    WriteData(0x80);
    
    WriteComm(0x36);
    WriteData(0x48);
    
    WriteComm(0x3A); //Interface Mode Control
    WriteData(0x66);
    
    WriteComm(0XB0);  //Interface Mode Control  
    WriteData(0x00);
    WriteComm(0xB1);//Frame rate 70HZ  
    WriteData(0xB0);
    WriteData(0x11);
    WriteComm(0xB4);
    WriteData(0x02);
    WriteComm(0xB6); //RGB/MCU Interface Control
    WriteData(0x02);
    WriteData(0x02);
    
    WriteComm(0xB7);
    WriteData(0xC6);

    //WriteComm(0XBE);
    //WriteData(0x00);
    //WriteData(0x04);

    WriteComm(0xE9);
    WriteData(0x00);

    WriteComm(0XF7);
    WriteData(0xA9);
    WriteData(0x51);
    WriteData(0x2C);
    WriteData(0x82);

    WriteComm(0x11);
    // delay_ms(120);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    WriteComm(0x29);
    WriteComm(0x2c);

#elif SIZE == SIZE096
    WriteComm(0x11);
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    WriteComm(0xB1);//------------------------------------ST7735S Frame Rate-----------------------------------------//
    WriteData(0x05);
    WriteData(0x3C);
    WriteData(0x3C);

    WriteComm(0xB2);
    WriteData(0x05);
    WriteData(0x3C);
    WriteData(0x3C);

    WriteComm(0xB3);
    WriteData(0x05);
    WriteData(0x3C);
    WriteData(0x3C);
    WriteData(0x05);
    WriteData(0x3C);
    WriteData(0x3C);//------------------------------------End ST7735S Frame Rate-----------------------------------------//

    WriteComm(0xB4);//Dot inversion
    WriteData(0x03);

    WriteComm(0xC0);//------------------------------------ST7735S Power Sequence-----------------------------------------//
    WriteData(0x0E);
    WriteData(0x0E);
    WriteData(0x04);

    WriteComm(0xC1);
    WriteData(0xC0);

    WriteComm(0xC2);
    WriteData(0x0D);
    WriteData(0x00);

    WriteComm(0xC3);
    WriteData(0x8D);
    WriteData(0x2A);

    WriteComm(0xC4);
    WriteData(0x8D);
    WriteData(0xEE);//---------------------------------End ST7735S Power Sequence-------------------------------------//

    WriteComm(0xC5);//VCOM
    WriteData(0x04);

    WriteComm(0x36);//MX, MY, RGB mode  刷屏模式 A8： 横屏  68：横屏翻转
    WriteData(0x68);

    WriteComm(0x3a);  // 颜色配置05H： 56506H： 666
    WriteData(0x05);


    WriteComm(0xE0);
    WriteData(0x05);
    WriteData(0x1A);
    WriteData(0x0B);
    WriteData(0x15);
    WriteData(0x3D);
    WriteData(0x38);
    WriteData(0x2E);
    WriteData(0x30);
    WriteData(0x2D);
    WriteData(0x28);
    WriteData(0x30);
    WriteData(0x3B);
    WriteData(0x00);
    WriteData(0x01);
    WriteData(0x02);
    WriteData(0x10);

    WriteComm(0xE1);
    WriteData(0x05);
    WriteData(0x1A);
    WriteData(0x0B);
    WriteData(0x15);
    WriteData(0x36);
    WriteData(0x2E);
    WriteData(0x28);
    WriteData(0x2B);
    WriteData(0x2B);
    WriteData(0x28);
    WriteData(0x30);
    WriteData(0x3B);
    WriteData(0x00);
    WriteData(0x01);
    WriteData(0x02);
    WriteData(0x10);
    WriteComm(0x29);

#elif SIZE == SIZE20

    WriteComm(0x11);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    WriteComm(0x36);
    WriteData(0xA0); // 扫描方向 0xA0 翻转：0x70

    WriteComm(0x3A);
    WriteData(0x05);  //0x05( 65K Color)

    WriteComm(0x21);

    WriteComm(0xB2);
    WriteData(0x05);
    WriteData(0x05);
    WriteData(0x00);
    WriteData(0x33);
    WriteData(0x33);

    WriteComm(0xB7);
    WriteData(0x23);

    WriteComm(0xBB);
    WriteData(0x22);

    WriteComm(0xC0);
    WriteData(0x2C);

    WriteComm(0xC2);
    WriteData(0x01);

    WriteComm(0xC3);
    WriteData(0x13);

    WriteComm(0xC4);
    WriteData(0x20);

    WriteComm(0xC6);
    WriteData(0x0F);

    WriteComm(0xD0);
    WriteData(0xA4);
    WriteData(0xA1);

    WriteComm(0xD6);
    WriteData(0xA1);

    WriteComm(0xE0);
    WriteData(0x70);
    WriteData(0x06);
    WriteData(0x0C);
    WriteData(0x08);
    WriteData(0x09);
    WriteData(0x27);
    WriteData(0x2E);
    WriteData(0x34);
    WriteData(0x46);
    WriteData(0x37);
    WriteData(0x13);
    WriteData(0x13);
    WriteData(0x25);
    WriteData(0x2A);

    WriteComm(0xE1);
    WriteData(0x70);
    WriteData(0x04);
    WriteData(0x08);
    WriteData(0x09);
    WriteData(0x07);
    WriteData(0x03);
    WriteData(0x2C);
    WriteData(0x42);
    WriteData(0x42);
    WriteData(0x38);
    WriteData(0x14);
    WriteData(0x14);
    WriteData(0x27);
    WriteData(0x2C);

    WriteComm(0x29);     //Display on


#endif
}


void drawFrameBufferBigger(uint8_t* buff,uint16_t size,uint16_t color,uint16_t bgcolor)
{
        for (uint16_t i = 0 ;i < size ;i++){
            for(uint8_t j = 0;j < 8;j++){
                if(((buff[i] >> j) & 0x01)){
#if SPI8
                    WriteData(color >> 8);
                    WriteData(color & 0xff);
                }
                else{
                    WriteData(bgcolor >> 8);
                    WriteData(bgcolor & 0xff);
                }
            }
#else
                    WriteData(color);
                }
                else{
                    WriteData(bgcolor);
                }
            }
#endif

        }
}




void drawPicture(uint16_t xStar,uint16_t yStar,uint16_t weight,uint16_t hight,uint8_t* buff,uint16_t color,uint16_t bgcolor){
#if SPI8
    WriteComm(setxcmd);
    WriteData(xStar >> 8);WriteData(xStar & 0xff);
    WriteData((xStar + weight - 1)>>8);WriteData((xStar + weight - 1)&0xff);
    WriteComm(setycmd);
    WriteData(yStar >> 8);WriteData(yStar & 0xff);
    WriteData((yStar + hight - 1) >> 8);WriteData((yStar + hight -1) &0Xff);

	WriteComm(wramcmd);

#else
    WriteComm(setxcmd);
    WriteData(xStar);
    WriteData((xStar + weight - 1));
    WriteComm(setycmd);
    WriteData(yStar);
    WriteData((yStar + hight - 1));

	WriteComm(wramcmd);
#endif
    drawFrameBufferBigger(buff,(weight * hight/8),color,bgcolor);
}
//------------------------------------------
void gpio_init(){
      // PIN_FUNC_SELECT( IO_MUX_GPIO13_REG, PIN_FUNC_GPIO)
      gpio_reset_pin(CS_GPIO);
      gpio_set_direction(CS_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(SCL_GPIO);
      gpio_set_direction(SCL_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(SDA_GPIO);
      gpio_set_direction(SDA_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(RS_GPIO);
      gpio_set_direction(RS_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(BACKLIGHT_GPIO);
      gpio_set_direction(BACKLIGHT_GPIO, GPIO_MODE_OUTPUT);
      // gpio_reset_pin(RST_GPIO);
      // gpio_set_direction(RST_GPIO, GPIO_MODE_OUTPUT); 
      BL_O;
      CS0_H;
      SCL_H;
      SDA_L;  //MOSI io13
      RS_H; 
      RS_L; 
}

// void DispColor(unsigned int color)
// {
// 	unsigned int i,j;

// 	BlockWrite(0,COL-1,0,ROW-1);	
// 	CS0_L; 
// 	RS_H;

// 	for(i=0;i<ROW;i++)
// 	{
// 	 for(j=0;j<COL;j++)
// 		{ 
// 			 WriteData(color>>8);  	
// WriteData(color);  
// 		}
// 	}

// 	CS0_H; 
// }

void LCD_Clear(uint32_t color)
{
	uint32_t index=0; 
	uint32_t totalpoint=width;
	totalpoint*=height; 	

  WriteComm(setxcmd); 
  WriteData(0);WriteData(0);
  WriteData((width-1)>>8);WriteData((width-1)&0XFF);
  WriteComm(setycmd); 
  WriteData(0);WriteData(0);
  WriteData((height-1)>>8);WriteData((height-1)&0XFF);  

	WriteComm(0x2c);
	for(index=0;index<totalpoint;index++)
	{
    // WriteData(color>>16);
    WriteData(color>>8);
    WriteData(color & 0xff);
	}
}

void setSetMemoryArea(uint16_t xStar,uint16_t xEnd,uint16_t yStar,uint16_t yEnd){

  WriteComm(setxcmd);
  WriteData(xStar >> 8);WriteData(xStar & 0xff);
  WriteData((xStar)>>8);WriteData((xStar)&0xff);
  WriteComm(setycmd);
  WriteData(yStar >> 8);WriteData(yStar & 0xff);
  WriteData((yEnd) >> 8);WriteData((yEnd)  &0Xff);

	WriteComm(0x2c);

}

// void writeMemory(uint16_t xStar,uint16_t xEnd,uint16_t yStar,uint16_t yEnd){
//   WriteComm(0x2c);

//   WriteData();
// }

void drawBlock(uint16_t xStar,uint16_t yStar,uint16_t xEnd,uint16_t yEnd,uint32_t color)
{
	uint32_t index = 0; 
	uint32_t totalpoint = (xEnd - xStar + 1) * (yEnd - yStar + 1);

  WriteComm(setxcmd);
  WriteData(xStar >> 8);WriteData(xStar & 0xff);
  WriteData((xEnd)>>8);WriteData((xEnd)&0xff);
  WriteComm(setycmd);
  WriteData(yStar >> 8);WriteData(yStar & 0xff);
  WriteData((yEnd) >> 8);WriteData((yEnd)  &0Xff);

	WriteComm(0x2c);

	for(index=0;index < totalpoint;index++)
	{
    // WriteData(color>>16);
    WriteData(color >> 8);
    WriteData(color & 0xff);
	}

}

void rollInit(void){ // 滚动的初始化，设置滚动区域，在lcd 初始化之后调用
  WriteComm(0x33); 
  WriteData(0);
  WriteData(0);
  WriteData(0>>8);
  WriteData((162) & 0xff);
  WriteData(0);
  WriteData(0);
}
void startRoll(uint8_t i){
  WriteComm(0x37); 
  WriteData(0);
  WriteData(i);
}

void keepRoll1(void){  // 开始滚动，1和2的区别是方向不同
    uint8_t time = 0;
    while(1) {
      startRoll(time);
      time++;
      if(time > 160)
        time = 0;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      }
}

void keepRoll2(void){
    uint8_t time = 160;
    while(1) {
      startRoll(time);
      time--;
      if(time == 0)
        time = 160;
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void uart_init(int baud_rate, int tx_pin, int rx_pin) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1,tx_pin ,-1 , UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}



#define I2C_MASTER_SCL_IO           33          /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           32          /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */

#define ADDR_CONFIG 0x24
#define ADDR_DIGIT1  0x34
#define ADDR_DIGIT2  0x35
#define ADDR_DIGIT3  0x36
#define ADDR_DIGIT4  0x37

// ADDR_CONFIG 0xf1
// ADDR_DIGIT1 0x00

// void i2c_master_init(void)
// {
//     i2c_config_t conf;
//     conf.mode = 1;
//     conf.sda_io_num = I2C_MASTER_SDA_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_io_num = I2C_MASTER_SCL_IO;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode,
//                        I2C_MASTER_RX_BUF_DISABLE,
//                        I2C_MASTER_TX_BUF_DISABLE, 0);
// }

// void i2c_master_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
// }

// uint8_t i2c_master_read_byte(uint8_t dev_addr, uint8_t reg_addr)
// {
//     uint8_t data = 0;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return data;
// }

// #define IIC_SCL_PIN 33
// #define IIC_SDA_PIN 32
// #define IIC_FREQ_HZ 400000

// void iic_init(void)
// {
//     i2c_config_t iic_config = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = IIC_SDA_PIN,
//         .scl_io_num = IIC_SCL_PIN,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = IIC_FREQ_HZ
//     };

//     i2c_param_config(I2C_NUM_0, &iic_config);
//     i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
// }

// void iic_send_data(uint8_t *data, size_t len,uint8_t add)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, add,true);
//     i2c_master_write(cmd, data, len, true);
//     i2c_master_stop(cmd);

//     esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_RATE_MS);
//     if (ret != ESP_OK) {
//     }
//     printf("ret %x,add = %d\r\n",ret,add);
//     i2c_cmd_link_delete(cmd);
// }

void app_main(void)
{
      gpio_init();
      LCD_Init();
      uart_init(19200,19,18);
      uint8_t data0[2] = {0x08,0x00};
      uart_write_bytes(UART_NUM_1, data0, 2);
      vTaskDelay(20/ portTICK_PERIOD_MS);
      uint8_t data1[2] = {0x08,0xFF};
      uart_write_bytes(UART_NUM_1, data1, 2);
      vTaskDelay(20/ portTICK_PERIOD_MS);
      uint8_t data2[2] = {0x18,0xfe};
      uart_write_bytes(UART_NUM_1, data2, 2);
      // rollInit();
      // i2c_master_init();
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      // i2c_master_start(cmd);
      // i2c_master_write_byte(cmd, (ADDR_CONFIG << 1) | I2C_MASTER_WRITE, true);
      // i2c_master_write_byte(cmd, 0xf1, true);
      iic_init();
      // uint8_t A = 0xF1;
      uint8_t B = 0x00;
      // iic_send_data(&A,1,ADDR_CONFIG);
      // iic_send_data(&B,1,ADDR_DIGIT1);
      // iic_send_data(&B,1,ADDR_DIGIT2);
      // iic_send_data(&B,1,ADDR_DIGIT3);
      // iic_send_data(&B,1,ADDR_DIGIT4);
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      // uint8_t data_rd[257];
      for(uint16_t i = 0; i < 256;i++){
      // data_rd[i]= i2c_master_write_byte(cmd, i, true);
      // i2c_master_stop(cmd);
      // i2c_master_read(cmd, &data_rd[i], 1, I2C_MASTER_ACK);
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      iic_send_data(&B,1,i);
      }
      // for(uint16_t i = 0; i < 256;i++){
      //   printf("data_rd[%d] = %x\r\n",i,data_rd[i]);
      // }
      // i2c_master_write_byte(cmd, (ADDR_DIGIT1 << 1) | I2C_MASTER_WRITE, true);
      // i2c_master_write_byte(cmd, 0x00, true); 

      // i2c_master_write_byte(cmd, (ADDR_DIGIT2 << 1) | I2C_MASTER_WRITE, true);
      // i2c_master_write_byte(cmd, 0x00, true);

      // i2c_master_write_byte(cmd, (ADDR_DIGIT3 << 1) | I2C_MASTER_WRITE, true);
      // i2c_master_write_byte(cmd, 0x00, true);

      // i2c_master_write_byte(cmd, (ADDR_DIGIT4 << 1) | I2C_MASTER_WRITE, true);
      // i2c_master_write_byte(cmd, 0x00, true); 


      while(1) {
      // LCD_Init();
        LCD_Clear(RED);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // LCD_Clear(GBLUE);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // drawBlock(50,50,200,60,WHITE);
        // drawBlock(50,80,200,90,WHITE);
        // drawBlock(50,100,200,120,WHITE);
        // keepRoll1();
      drawPicture(15,15,24,26,number0_18,WHITE,BLACK);
      drawPicture(45,15,24,26,number1_18,WHITE,BLACK);
      drawPicture(75,15,24,26,number2_18,WHITE,BLACK);
      drawPicture(105,15,24,26,number3_18,WHITE,BLACK);
      drawPicture(135,15,24,26,number4_18,WHITE,BLACK);
      drawPicture(165,15,24,26,number5_18,WHITE,BLACK);
      drawPicture(195,15,24,26,number6_18,WHITE,BLACK);
      drawPicture(225,15,24,26,number7_18,WHITE,BLACK);
      drawPicture(255,15,24,26,number8_18,WHITE,BLACK);
      drawPicture(285,15,24,26,number9_18,WHITE,BLACK);

      drawPicture(15,46,24,26,number0_18,WHITE,BLACK);
      drawPicture(45,46,24,26,number1_18,WHITE,BLACK);
      drawPicture(75,46,24,26,number2_18,WHITE,BLACK);
      drawPicture(105,46,24,26,number3_18,WHITE,BLACK);
      drawPicture(135,46,24,26,number4_18,WHITE,BLACK);
      drawPicture(165,46,24,26,number5_18,WHITE,BLACK);
      drawPicture(195,46,24,26,number6_18,WHITE,BLACK);
      drawPicture(225,46,24,26,number7_18,WHITE,BLACK);
      drawPicture(255,46,24,26,number8_18,WHITE,BLACK);
      drawPicture(285,46,24,26,number9_18,WHITE,BLACK);

      drawPicture(15,80,24,26,number0_18,RED,BLACK);
      drawPicture(45,80,24,26,number1_18,GREEN,BLACK);
      drawPicture(75,80,24,26,number2_18,BLUE,BLACK);
      drawPicture(105,80,24,26,number3_18,YELLOW,BLACK);
      drawPicture(135,80,24,26,number4_18,BROWN,BLACK);
      drawPicture(165,80,24,26,number5_18,BRRED,BLACK);
      drawPicture(195,80,24,26,number6_18,GRAY,BLACK);
      drawPicture(225,80,24,26,number7_18,MAGENTA,BLACK);
      drawPicture(255,80,24,26,number8_18,CYAN,BLACK);
      drawPicture(285,80,24,26,number9_18,WHITE,BLACK);
      // // printf("Turning on the LED\n");
      // LCD_Clear(GREEN);
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // LCD_Clear(YELLOW);
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // // LCD_Clear(BROWN);
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // LCD_Clear(BRRED);
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // LCD_Clear(GRAY);
      while(1) {
        printf("hello word\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
 }
}


/* 0.96 寸屏幕
#define LCD_NOP			0x00	//空命令
#define LCD_SWRESET		0x01	//软件复位，在睡眠和显示模式下，重置软件后需等待120ms后方可执行下一条指令
#define LCD_RDDID		0x04	//读取LCD的制造商ID（8位）、驱动版本ID（最高位为1，7位）、驱动程序ID（8位）
#define LCD_RDDST		0x09	//读取显示屏所有状态参数
#define LCD_RDDPM		0x0A	//读取显示屏能量模式
#define LCD_RDDMADCTL	0x0B	//读取显示屏MADCTL
#define LCD_RDDCOLMOD	0x0C	//读取显示屏像素定义
#define LCD_RDDIM		0x0D	//读取显示屏图片模式
#define LCD_RDDSM		0x0E	//读取显示屏单信号模式
#define LCD_RDDSDR		0x0F	//读取显示屏自我诊断结果
#define LCD_SLPIN		0x10	//进入最小功耗模式
#define LCD_SLPOUT 		0x11	//关闭睡眠模式
#define LCD_PTLON		0x12	//打开Partial模式
#define LCD_NORON		0x13	//恢复到正常模式
#define LCD_INVOFF		0x20	//显示反转模式中恢复
#define LCD_INVON		0x21	//进入反向显示模式
#define LCD_GAMSET		0x26	//当前显示选择所需的伽马曲线
#define LCD_DISPOFF		0x28	//关闭显示，帧内存的输出被禁用
#define LCD_DISPON		0x29	//开启显示，帧内存的输出被启用
#define LCD_CASET		0x2A	//列地址设置，每个值代表帧内存中的一列
#define LCD_RASET		0x2B	//行地址设置，每个值代表帧内存中的一列
#define LCD_RAMWR		0x2C	//写入内存
#define LCD_RGBSET		0x2D	//颜色模式设置
#define LCD_RAMRD		0x2E	//读取内存
#define LCD_PTLAR		0x30	//部分模式的显示区域设置
#define LCD_SCRLAR		0x33	//定义垂直滚动区域的显示
#define LCD_TEOFF		0x34	//关闭(Active Low) TE信号线的撕裂效应输出信号
#define LCD_TEON		0x35	//打开TE信号线的撕裂效果输出信号
#define LCD_MADCTL		0x36	//定义帧内存的读写扫描方向
#define LCD_VSCSAD		0x37	//设置垂直滚动起始地址，此命令与垂直滚动定义(33h)一起使用
#define LCD_IDMOFF		0x38	//关闭空闲模式
#define LCD_IDMON		0x39	//开启空闲模式
#define LCD_COLMOD		0x3A	//定义通过MCU接口传输的RGB图片数据的格式
#define LCD_FRMCTR1		0xB1	//设置全色正常模式的帧频
#define LCD_FRMCTR2 	0xB2	//设置空闲模式的帧频
#define LCD_FRMCTR3 	0xB3	//设置部分模式/全色的帧频率
#define LCD_INVCRT 		0xB4	//反转模式控制
#define LCD_PWCTR1 		0xC0	//设置AVDD、MODE、VRHP、VRHN
#define LCD_PWCTR2 		0xC1	//设置VGH与VGL的供电功率
#define LCD_PWCTR3 		0xC2	//设置正常模式/全色模式下的运放的电流
#define LCD_PWCTR4 		0xC3	//设置空闲模式/八色模式下的运放的电流
#define LCD_PWCTR5 		0xC4	//设置部分模式/全色模式下的运放的电流
#define LCD_VMCTR1 		0xC5	//设置VCOM电压电平以减少闪烁问题
#define LCD_VMOFCTR		0xC7	//VCOM偏移控制，在使用命令0xC7之前，命令0xD9的位VMF_EN必须启用(设置为1)
#define LCD_WRID2		0xD1	//写入LCD模块版本的7位数据，保存到NVM
#define LCD_WRID3		0xD2	//写入项目代码模块的8位数据，保存到NVM
#define LCD_NVFCTR1		0xD9	//NVM状态控制
#define LCD_RDID1		0xDA	//读字节返回8位LCD模块的制造商ID
#define LCD_RDID2		0xDB	//读字节返回8位LCD模块/驱动程序版本ID
#define LCD_RDID3		0xDC	//读字节返回8位LCD模块/驱动ID
#define LCD_NVFCTR2		0xDE	//NVM读取命令
#define LCD_NVFCTR3		0xDF	//NVM写取命令
#define LCD_GMCTRP1		0xE0	//Gamma ‘+’ Polarity Correction Characteristics Setting
#define LCD_GMCTRN1		0xE1	//Gamma ‘+’ Polarity Correction Characteristics Setting
#define LCD_GCV			0xFC	//自动调节门泵时钟，节省功耗
*/
