// Color TFT DEMO program by takuya matsubara
// for Akizuki AE-ATM0177B3A
// 1.77inch ZETTLER COLOR TFT LCD(ILI9163V / 128x160 pixel）
// https://akizukidenshi.com/catalog/g/gK-14032/

// Reference : Max MC Costa(sumotoy)'s "TFT_ILI9163C"
// https://github.com/sumotoy/TFT_ILI9163C

#define SOFT_SPI  0 // software SPI(0=fast / 1=slow)

#if SOFT_SPI==0
#include <SPI.h>
#endif

#include "8x8font.h"

// AE-ATM0177B3A pin assign
//1: NC
//2: NC
//3: NC
//4: NC
//5: LED Anode
//6: NC
//7: NC
//8: GND
//9: VCC
//10: SDA
//11: SCK
//12: CD(H=Data/L=Command)
//13: RESET
//14: LED Cathode
//15: CS(L=active)

//Seeeduino XIAO - Arduino Microcontroller
//CPU：Arm Cortex M0+ 32bit 48 MHz（SAMD21）
//Flash:256 KB 
//SRAM:32 KB

// Seeeduino XIAO pin assign
//    [USB]
//D0         5V
//D1(RST)    GND
//D2(CS)     3V3
//D3(CD)     D10(MOSI)
//D4         D9(MISO)
//D5         D8(SCK)
//D6(TX)     D7(RX)

#define TFTCS     2   // chip select
#define TFTCD     3   // command/data
#define SPICLK    8   // clock
#define SPIDAT   10   // data
#define TFTRST    1   // reset

#define SOFTWARE_RESET            0x01
#define SLEEP_OUT                 0x11
#define PARTIAL_MODE_ON           0x12
#define NORMAL_DISPLAY_MODE_ON    0x13
#define DISPLAY_INVERSION_OFF     0x20
#define DISPLAY_INVERSION_ON      0x21
#define GAMMA_SET                 0x26
#define DISPLAY_OFF               0x28
#define DISPLAY_ON                0x29
#define COLUMN_ADDRESS_SET        0x2A
#define PAGE_ADDRESS_SET          0x2B
#define MEMORY_WRITE              0x2C
#define V_SCROLL_DEFINITION       0x33
#define MEMORY_ACCESS_CONTROL     0x36
#define V_SCROLL_START_ADDRESS    0x37
#define INTERFACE_PIXEL_FORMAT    0x3A
#define FRAME_RATE_CONTROL_1      0xB1
#define DISPLAY_INVERSION_CONTROL 0xB4
#define DISPLAY_FUCTION_SET       0xb6
#define POWER_CONTROL_1           0xC0
#define POWER_CONTROL_2           0xC1
#define VCOM_CONTROL_1            0xC5
#define VCOM_OFFSET_CONTROL       0xC7
#define POSITIVE_GAMMA            0xE0
#define NEGATIVE_GAMMA            0xE1
#define GAM_R_SEL                 0xF2

#define VRAMWIDTH        128  // width[pixel]
#define VRAMHEIGHT       160  // height[pixel]
#define VRAMXMAX  (VRAMWIDTH-1)
#define VRAMYMAX  (VRAMHEIGHT-1)
#define VRAMSIZE   (VRAMWIDTH*VRAMHEIGHT)

unsigned char vram[VRAMSIZE];

const unsigned char p_gamma_table[] = {
  0x36,0x29,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x12,0x0A,0x11,0x0B,0x06
};
     
const unsigned char n_gamma_table[] = {
  0x09,0x16,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x34,0x39
};

const unsigned int colortable[]={
/*  rrrrrgggggbbbbbb */
  0b0000000000000000, /*color 0*/
  0b0000000000111111, /*color 1*/
  0b1111100000000000, /*color 2*/
  0b1111100000111111, /*color 3*/
  0b0000011111000000, /*color 4*/
  0b0000011111111111, /*color 5*/
  0b1111111111000000, /*color 6*/
  0b1111111111111111  /*color 7*/
};

void tft_init(void)
{
  pinMode(SPIDAT, OUTPUT);
  pinMode(SPICLK, OUTPUT);
  pinMode(TFTCD, OUTPUT);
  pinMode(TFTCS, OUTPUT);
  pinMode(TFTRST, OUTPUT);
  digitalWrite(SPIDAT, HIGH);
  digitalWrite(SPICLK, LOW);
  digitalWrite(TFTCD, HIGH);
  digitalWrite(TFTCS, HIGH);
  digitalWrite(TFTRST, HIGH);

#if SOFT_SPI==0
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
//  SPI.setClockDivider(SPI_CLOCK_DIV4); 
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
#endif

  delay(500);
  tft_sendcmd(SOFTWARE_RESET);
  delay(500);
  tft_sendcmd(SLEEP_OUT);
  delay(5);
  tft_sendcmd_byte(INTERFACE_PIXEL_FORMAT, 0x05);  //16bit
  tft_sendcmd_byte(GAMMA_SET,0x04);
  tft_sendcmd_byte(GAM_R_SEL,0x01);
  tft_sendcmd(NORMAL_DISPLAY_MODE_ON);
  tft_sendcmd_word(DISPLAY_FUCTION_SET,0xff06);
//  tft_sendcmd_15byte(POSITIVE_GAMMA,(unsigned char *)p_gamma_table);
//  tft_sendcmd_15byte(NEGATIVE_GAMMA,(unsigned char *)n_gamma_table);
  tft_sendcmd_word(FRAME_RATE_CONTROL_1,0x0802);
  tft_sendcmd_byte(DISPLAY_INVERSION_CONTROL,0x07);
  tft_sendcmd_word(POWER_CONTROL_1,0x0A02);
  tft_sendcmd_byte(POWER_CONTROL_2,0x02);
  tft_sendcmd_word(VCOM_CONTROL_1,0x5063);
  tft_sendcmd_byte(VCOM_OFFSET_CONTROL,0x00);
  tft_sendcmd_long(COLUMN_ADDRESS_SET,VRAMWIDTH-1); 
  tft_sendcmd_long(PAGE_ADDRESS_SET,VRAMHEIGHT-1); 
//  tft_sendcmd_long(V_SCROLL_DEFINITION,VRAMHEIGHT);
//  spi_sendbyte(0);
//  spi_sendbyte(0);
//  tft_sendcmd_byte(MEMORY_ACCESS_CONTROL,0x00); // RGB
//  tft_sendcmd_word(V_SCROLL_START_ADDRESS,0x0000);
  tft_sendcmd(DISPLAY_ON);
  delay(25);
}

void spi_sendbyte(unsigned char data)
{
#if SOFT_SPI==0
  SPI.transfer(data);
#else
  unsigned char bitmask;
  digitalWrite(SPICLK, LOW);
  bitmask = 0x80;
  while(bitmask){
    digitalWrite(SPIDAT, ((bitmask & data) != 0));
    digitalWrite(SPICLK, HIGH);
    digitalWrite(SPICLK, LOW);
    bitmask >>= 1;
  }
#endif
}

void tft_sendcmd(unsigned char data)
{
  digitalWrite(TFTCD,LOW);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  digitalWrite(TFTCD,HIGH);
}

void tft_sendcmd_15byte(unsigned char cmd,unsigned char *data)
{
  int cnt;
  digitalWrite(TFTCS,LOW);
  tft_sendcmd(cmd);
  cnt=15;
  while(cnt--){
    spi_sendbyte(*data++);
  }
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

void tft_sendcmd_byte(unsigned char cmd,unsigned char data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

void tft_sendcmd_word(unsigned char cmd,unsigned int data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte((unsigned char)(data >> 8));
  spi_sendbyte((unsigned char)(data & 0xff));
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

void tft_sendcmd_long(unsigned char cmd,unsigned long data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte((unsigned char)(data >> 24));
  spi_sendbyte((unsigned char)((data >> 16)& 0xff));
  spi_sendbyte((unsigned char)((data >> 8)& 0xff));
  spi_sendbyte((unsigned char)(data & 0xff));
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

// 
void tft_from_vram(void)
{
  int x,y,i;
  unsigned int bitmask;
  unsigned int r,g,b;
  unsigned int color;
  unsigned char *ptr;
  
  tft_sendcmd(MEMORY_WRITE);
  digitalWrite(TFTCS,LOW);
  ptr = (unsigned char *)vram;
  for(y=0; y<VRAMHEIGHT; y++){
    for(x=0; x<VRAMWIDTH; x++){
      color = colortable[*ptr++];
      spi_sendbyte(color >> 8);
      spi_sendbyte(color & 0xff);
    }
  }
  digitalWrite(TFTCS,HIGH);
  delay(1);
}


void vram_cls(void)
{
  int i;

  for(i=0; i<VRAMSIZE; i++){
    vram[i] = 0;
  }
}


//ピクセル取得(X座標,Y座標)
unsigned char vram_point(int x,int y)
{
  unsigned char mask;
  int i;

  if(x<0)return(0);
  if(y<0)return(0);
  if(x>=VRAMWIDTH)return(0);
  if(y>=VRAMHEIGHT)return(0);

  i = (y*VRAMWIDTH)+ x;
  return(vram[i]);
}


//ピクセル描画(X座標,Y座標,カラーコード)
void vram_pset(int x,int y,unsigned char color)
{
  int i;

  if(x<0)return;
  if(y<0)return;
  if(x>=VRAMWIDTH)return;
  if(y>=VRAMHEIGHT)return;

  i = (y*VRAMWIDTH)+ x;
  vram[i] = color;
}


//ライン描画(X1座標,Y1座標,X2座標,Y2座標,カラーコード)
void vram_line(int x1 ,int y1 ,int x2 ,int y2 ,char color)
{
  int xd;    // X2-X1座標の距離
  int yd;    // Y2-Y1座標の距離
  int xs=1;  // X方向の1pixel移動量
  int ys=1;  // Y方向の1pixel移動量
  int e;

  xd = x2 - x1;  // X2-X1座標の距離
  if(xd < 0){
    xd = -xd;  // X2-X1座標の絶対値
    xs = -1;    // X方向の1pixel移動量
  }
  yd = y2 - y1;  // Y2-Y1座標の距離
  if(yd < 0){
    yd = -yd;  // Y2-Y1座標の絶対値
    ys = -1;    // Y方向の1pixel移動量
  }
  vram_pset (x1, y1 ,color); //ドット描画
  e = 0;
  if( yd < xd ) {
    while( x1 != x2) {
      x1 += xs;
      e += (2 * yd);
      if(e >= xd) {
        y1 += ys;
        e -= (2 * xd);
      }
      vram_pset (x1, y1 ,color); //ドット描画
    }
  }else{
    while( y1 != y2) {
      y1 += ys;
      e += (2 * xd);
      if(e >= yd) {
        x1 += xs;
        e -= (2 * yd);
      }
      vram_pset (x1, y1 ,color); //ドット描画
    }
  }
}


void setup(void)
{
  tft_init();

//  Serial.begin(115200); //debug
//  while (!Serial) {
//  }
}


// 1キャラクタをVRAM転送
// 引数 x,y,chキャラクターコード
void vram_putch(int textx, int texty, unsigned char ch,unsigned char color)
{
  char i,j;
  unsigned char bitdata;
  int p;

  if(ch < 0x20)return;
  p = ((int)(ch - 0x20) * 8);
  for(i=0 ;i<8 ;i++) {
    bitdata = font[p++];
    for(j=0; j<8; j++){
      if(bitdata & 0x80)vram_pset(textx+j,texty+i,color);
      bitdata <<= 1;
    }
  }
}

void vram_scroll(int x1,int y1){
  int x,y;
  unsigned char color;

  for(y=0;y<VRAMHEIGHT;y++){
    for(x=0;x<VRAMWIDTH;x++){
      if(((x+x1)>=VRAMWIDTH)||((y+y1)>=VRAMHEIGHT)){
        color = 0;
      }else{
        color = vram_point(x+x1, y+y1);
      }
      vram_pset(x,y,color);
    }
  }
}


void chartest()
{
  int x,y;
  unsigned char chrnum;
  unsigned char color;
  int timeout;

  timeout=1000;
  x=0;
  y=VRAMHEIGHT-8;
  chrnum=0x20;
  color=7;
  while(timeout--){
    vram_putch(x,y,chrnum,color);
    chrnum++;
    if(chrnum > 0x7f){
      chrnum=0x20;
      color++;
      if(color > 7)color=1;
    }
    x = (x+8) % VRAMWIDTH;
    if(x == 0){
      vram_scroll(0,8);
    }  
    tft_from_vram();
  }
}
  
void bound()
{
  int xa,ya,xb,yb;
  int xa1,ya1,xb1,yb1;
  unsigned char color;
  int timeout;

  timeout = 1000;
  xa =random(VRAMWIDTH-2)+1;
  ya =random(VRAMHEIGHT-2)+1;
  xb =random(VRAMWIDTH-2)+1;
  yb =random(VRAMHEIGHT-2)+1;
  xa1 =random(5)-2;
  ya1 =random(5)-2;
  xb1 =random(5)-2;
  yb1 =random(5)-2;

  color=0;
  while(timeout--){
    xa += xa1;
    ya += ya1;
    xb += xb1;
    yb += yb1;
    vram_line(xa ,ya ,xb ,yb ,(color/4)& 7);
    if((xa<=0)||(xa>=VRAMXMAX))xa1 = -xa1;
    if((ya<=0)||(ya>=VRAMYMAX))ya1 = -ya1;
    if((xb<=0)||(xb>=VRAMXMAX))xb1 = -xb1;
    if((yb<=0)||(yb>=VRAMYMAX))yb1 = -yb1;
    color++;
    tft_from_vram();
  }
}

void loop()
{
  chartest();
  bound();
}
