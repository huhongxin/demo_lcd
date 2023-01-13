/* Blink Example

这个例子是点亮LCD屏的例子
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

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
#define SIZE096 1
#define SIZE35 2

#define SIZE SIZE096
#if SIZE == SIZE35

  #define BACKLIGHT_GPIO GPIO_NUM_2
  #define CS_GPIO GPIO_NUM_12
  #define SCL_GPIO GPIO_NUM_14
  #define SDA_GPIO GPIO_NUM_13
  #define RS_GPIO GPIO_NUM_15
  #define RST_GPIO GPIO_NUM_4

#elif SIZE == SIZE096

  #define BACKLIGHT_GPIO GPIO_NUM_12//低电平有效
  #define CS_GPIO GPIO_NUM_3
  #define SCL_GPIO GPIO_NUM_14
  #define SDA_GPIO GPIO_NUM_13
  #define RS_GPIO GPIO_NUM_2
  #define RST_GPIO GPIO_NUM_15

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
  #define BL_C gpio_set_level(BACKLIGHT_GPIO, 1)//背光
  #define BL_O gpio_set_level(BACKLIGHT_GPIO, 0)
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
#endif
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

// #define width 320
// #define height 480
#define width 132
#define height 162

#define wramcmd 0X2C
#define setxcmd 0X2A
#define setycmd 0X2B
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
		
    WriteData(color>>16);
    WriteData(color>>8);
    WriteData(color);	
	}
}  
void app_main(void)
{
      gpio_init();
      LCD_Init();

    while(1) {
      // LCD_Init();
      LCD_Clear(BLUE);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(GBLUE);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      // printf("Turning on the LED\n");
      LCD_Clear(GREEN);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(YELLOW);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(BROWN);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(BRRED);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(GRAY);
 }
}