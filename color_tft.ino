// Color TFT test program by takuya matsubara
// for Akizuki AE-ATM0177B3A
// 1.77inch ZETTLER COLOR TFT LCD(ILI9163V / 128x160 pixel）
// https://akizukidenshi.com/catalog/g/gK-14032/

// Reference : Max MC Costa(sumotoy)'s "TFT_ILI9163C"
// https://github.com/sumotoy/TFT_ILI9163C

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

#define GRAMWIDTH        128  // width[pixel]
#define GRAMHEIGHT       160  // height[pixel]

const unsigned char p_gamma_table[] = {
  0x36,0x29,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x12,0x0A,0x11,0x0B,0x06
};
     
const unsigned char n_gamma_table[] = {
  0x09,0x16,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x34,0x39
};

void spi_sendbyte(unsigned char data)
{
  unsigned char bitmask;
  digitalWrite(TFTCS,LOW);
  digitalWrite(SPICLK, LOW);
  bitmask = 0x80;
  while(bitmask){
    if(bitmask & data){
      digitalWrite(SPIDAT, HIGH);
    }else{
      digitalWrite(SPIDAT, LOW);
    }    
    digitalWrite(SPICLK, HIGH);
    digitalWrite(SPICLK, LOW);
    bitmask >>= 1;
  }
  digitalWrite(TFTCS,HIGH);
}

void tft_sendcmd(unsigned char data)
{
  delay(1);
  digitalWrite(TFTCD,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCD,HIGH);
}

void tft_send_data(unsigned char data)
{
  digitalWrite(TFTCD,HIGH);
  spi_sendbyte(data);
}

void tft_sendcmd_15byte(unsigned char cmd,unsigned char *data)
{
  int cnt;
  tft_sendcmd(cmd);
  cnt=15;
  while(cnt--){
    tft_send_data(*data++);
  }
}

void tft_sendcmd_byte(unsigned char cmd,unsigned char data)
{
  tft_sendcmd(cmd);
  tft_send_data(data);
}

void tft_sendcmd_word(unsigned char cmd,unsigned int data)
{
  tft_sendcmd(cmd);
  tft_send_data((unsigned char)(data >> 8));
  tft_send_data((unsigned char)(data & 0xff));
}

void tft_sendcmd_long(unsigned char cmd,unsigned long data)
{
  tft_sendcmd(cmd);
  tft_send_data((unsigned char)(data >> 24));
  tft_send_data((unsigned char)((data >> 16)& 0xff));
  tft_send_data((unsigned char)((data >> 8)& 0xff));
  tft_send_data((unsigned char)(data & 0xff));
}

// test pattern
void tft_display_test(void){
  int x,y;
  char mode;
  unsigned int r,g,b,color;
  tft_sendcmd(MEMORY_WRITE);
  delay(150);  
  color=0;
  for(y=0; y<160; y++){
    for(x=0; x<128; x++){
      mode = (y / 40);
      if(mode==0) color = ((y % 40)*0x1f/39);     //blue(5bit)
      if(mode==1) color = ((y % 40)*0x3f/39)<<5;  //green(6bit)
      if(mode==2) color = ((y % 40)*0x1f/39)<<11; //red(5bit)
      if(mode==3){ //grayscale
        r=(y % 40)*0x1f/39;
        g=(y % 40)*0x3f/39;
        b=r;
        color = (r<<11)+(g<<5)+b; 
      }
      tft_send_data(color >> 8);
      tft_send_data(color & 0xff);
    }
  }
}

void setup(void)
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

//  Serial.begin(115200);
//  while (!Serial) {
//  }

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
  tft_sendcmd_long(COLUMN_ADDRESS_SET,GRAMWIDTH-1); 
  tft_sendcmd_long(PAGE_ADDRESS_SET,GRAMHEIGHT-1); 
//  tft_sendcmd_long(V_SCROLL_DEFINITION,GRAMHEIGHT);
//  tft_send_data(0);
//  tft_send_data(0);
//  tft_sendcmd_byte(MEMORY_ACCESS_CONTROL,0x00); // RGB
//  tft_sendcmd_word(V_SCROLL_START_ADDRESS,0x0000);
  tft_sendcmd(DISPLAY_ON);
}
 
void loop()
{
  delay(500);  
  tft_display_test();
}
