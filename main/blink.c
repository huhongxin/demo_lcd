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
  #define CS_GPIO GPIO_NUM_32
  #define SCL_GPIO GPIO_NUM_2
  #define SDA_GPIO GPIO_NUM_3
  #define RS_GPIO GPIO_NUM_33
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
#define width 160
#define height 120

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
    // WriteData(color>>16);
    WriteData(color>>8);
    WriteData(color & 0xff);
	}
}

void setSetMemoryArea(uint16_t xStar,uint16_t xEnd,uint16_t yStar,uint16_t yEnd){

	uint32_t index=0;
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

void drawLine(uint16_t xStar,uint16_t xEnd,uint16_t yStar,uint16_t yEnd,uint32_t color)
{
	uint32_t index=0; 
	uint32_t totalpoint=(xEnd - xStar) * (yEnd - yStar);

  WriteComm(setxcmd);
  WriteData(xStar >> 8);WriteData(xStar & 0xff);
  WriteData((xEnd)>>8);WriteData((xEnd)&0xff);
  WriteComm(setycmd);
  WriteData(yStar >> 8);WriteData(yStar & 0xff);
  WriteData((yEnd) >> 8);WriteData((yEnd)  &0Xff);

	WriteComm(0x2c);

	for(index=0;index< totalpoint;index++)
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

void app_main(void)
{
      gpio_init();
      LCD_Init();
      rollInit();
    while(1) {
      // LCD_Init();
      LCD_Clear(YELLOW);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LCD_Clear(GBLUE);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      drawLine(0,80,60,62,RED);
      uint8_t time = 0;
      while(1){
      startRoll(time);
      time++;
      if(time >160)
        time = 0;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      }

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
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
 }
}


/*
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
