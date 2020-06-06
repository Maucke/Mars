# include <Adafruit_NeoPixel.h>
# ifdef __AVR__
# include <avr/power.h>
#endif
#include <EEPROM.h>

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

int pinInterrupt = 2; //接中断信号的脚
#define STRIPNUM 8

Adafruit_NeoPixel strip[STRIPNUM];

int MotionIndex=0;
void onChange()
{
    if (digitalRead(pinInterrupt) == LOW)
        Serial.println("Key Down");
    else
    {
        Serial.println("Key UP");
        MotionIndex++;
        if(MotionIndex>14)
          MotionIndex = 0;
        EEPROM.write(address, MotionIndex);
    }
}

#define DATA_PACKAGE_MIN_LEN    5
#define DATA_PACKAGE_MAX_LEN    512
#define DATA_PACKAGE_FFT_LEN    200
// 同步帧头
#define CMD_HEAD1 0xFF
#define CMD_HEAD2 0x55

#define MAKEWORD(low, high)    (((byte)(low)) | (((byte)(high)) << 8))

byte Uart_Data[DATA_PACKAGE_MAX_LEN] = { 0xFF, 0x55, 0x00, 0x00, 0x02, 0x00, 0x00 };
bool Uart_Overflow_Flag = false;

void Analysismsg(uint8_t* Buf)
{
    int i;
    if (Buf[0] == CMD_HEAD1 && Buf[1] == CMD_HEAD2)
    {
        {
            switch (MAKEWORD(Buf[3], Buf[2]))
            {
                case 0: for (i = 0; i < Buf[4]; i++) Buf[i] = Buf[i + 5]; break;
                case 1: Buf[0] = MAKEWORD(Buf[6], Buf[5]); break;
            }
        }
    }
}

void USART_Handler(void)                  //串口1中断服务程序
{
    byte Uart_Recv_Data = 0;
    static byte Uart_Recv_Step = 0;
    static byte Uart_Recv_Count = 0;

    while (Serial.available() > 0)
    {
        Uart_Recv_Data = (byte)Serial.read();
        if (!Uart_Overflow_Flag)
        {
            switch (Uart_Recv_Step)
            {
                case 0: if (Uart_Recv_Data == CMD_HEAD1) Uart_Recv_Step++; break;
                case 1: if (Uart_Recv_Data == CMD_HEAD2) Uart_Recv_Step++; else Uart_Recv_Step = 0; break;
                case 2: Uart_Data[2] = Uart_Recv_Data; Uart_Recv_Step++; break;
                case 3: Uart_Data[3] = Uart_Recv_Data; Uart_Recv_Step++; break;
                case 4: Uart_Data[4] = Uart_Recv_Data; Uart_Recv_Step++; break;
                case 5: Uart_Data[Uart_Recv_Count + DATA_PACKAGE_MIN_LEN] = Uart_Recv_Data; Uart_Recv_Count++; if (Uart_Recv_Count >= Uart_Data[4]) { Uart_Recv_Step = 0; Uart_Recv_Count = 0; Analysismsg(Uart_Data); } break;
            }
        }
    }
}

void setup()
{
    int i;
    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif

    Serial.begin(115200); //打开串口

    pinMode(pinInterrupt, INPUT);//设置管脚为输入
    attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, CHANGE);

    for (i = 0; i < 5; i++)
        strip[i] = Adafruit_NeoPixel(8, i + 4, NEO_GRB + NEO_KHZ800);
    strip[5] = Adafruit_NeoPixel(64, 9, NEO_GRB + NEO_KHZ800);
    for (i = 6; i < 7; i++)
        strip[i] = Adafruit_NeoPixel(8, i + 4, NEO_GRB + NEO_KHZ800);

    for (i = 0; i < 7; i++)
    {
        strip[i].begin();
        strip[i].show(); // Initialize all pixels to 'off'
    }
//    EEPROM.write(address, MotionIndex);
    MotionIndex = EEPROM.read(address);
}

int MotionRun = 0;

void loop()
{
    MotionRun++;
    switch(MotionIndex)
    {
   case 0:if (MotionRun > 256 * 2) MotionRun = 0; colorWipe(Wheel(0,MotionRun/ 2)); break;
    case 1:if (MotionRun > 256 * 2) MotionRun = 0; rainbow(MotionRun / 2); break;
    case 2:if (MotionRun > 256 * 3) MotionRun = 0; rainbow(MotionRun / 3); break;
    case 3:if (MotionRun > 256 * 5) MotionRun = 0; rainbowCycle(MotionRun); break;
    case 4:if (MotionRun > 256 * 5 * 2) MotionRun = 0; rainbowCycle(MotionRun / 2); break;
    case 5:if (MotionRun > 256 * 5) MotionRun = 0; negrainbowCycle(MotionRun); break;
    case 6:if (MotionRun > 256 * 5 * 2) MotionRun = 0; negrainbowCycle(MotionRun / 2); break;
    case 7:MotionRun = 0; colorWipe(0xFF0000); break;
    case 8:MotionRun = 0; colorWipe(0x00FF00); break;
    case 9:MotionRun = 0; colorWipe(0x0000FF); break;
    case 10:MotionRun = 0; colorWipe(0x55FF55); break;
    case 11:MotionRun = 0; colorWipe(0xFF5555); break;
    case 12:MotionRun = 0; colorWipe(0x5555FF); break;
    case 13:MotionRun = 0; colorWipe(0xFFFFFF); break;
   case 14:MotionRun = 0; colorWipe(0); break;
    }
    
    
    //    
    delay(1);
}

void colorWipe(uint32_t c)
{
    for (int index = 0; index < 7; index++)
    {
        for (uint16_t i = 0; i < strip[index].numPixels(); i++)
        {
            strip[index].setPixelColor(i, c);
        }
            strip[index].show();
            delay(1);
    }
}

void rainbow(uint8_t j)
{
    uint16_t i;

        for (int index = 0; index < 7; index++)
        {
            for (i = 0; i < strip[index].numPixels(); i++)
            {
                strip[index].setPixelColor(i, Wheel(index, (i + j) & 255));
            }
            strip[index].show();
            delay(1);
        }
}

void rainbowCycle(uint8_t j)
{
    uint16_t i;

        for (int index = 0; index < 7; index++)
        {
            for (i = 0; i < strip[index].numPixels(); i++)
            {
                strip[index].setPixelColor(i, Wheel(index, ((i * 256 / strip[index].numPixels()) + j) & 255));
            }
            strip[index].show();
            delay(1);
        }
}

void negrainbowCycle(uint8_t j)
{
    uint16_t i;

        for (int index = 0; index < 7; index++)
        {
            for (i = 0; i < strip[index].numPixels(); i++)
            {
                strip[index].setPixelColor(i, Wheel(index, ((i * 256 / strip[index].numPixels()) + 255 - j) & 255));
            }
            strip[index].show();
            delay(1);
        }
}

void theaterChase(uint32_t c, uint8_t wait)
{
    for (int j = 0; j < 10; j++)
    {  //do 10 cycles of chasing
        for (int q = 0; q < 3; q++)
        {
            for (int index = 0; index < 7; index++)
                for (uint16_t i = 0; i < strip[index].numPixels(); i = i + 3)
                {
                    strip[index].setPixelColor(i + q, c);    //turn every third pixel on
                    strip[index].show();
                }

            delay(wait);

            for (int index = 0; index < 7; index++)
                for (uint16_t i = 0; i < strip[index].numPixels(); i = i + 3)
                {
                    strip[index].setPixelColor(i + q, 0);        //turn every third pixel off
                }
        }
    }
}

void theaterChaseRainbow(uint8_t j)
{
        for (int q = 0; q < 3; q++)
        {
            for (int index = 0; index < 7; index++)
            {
                for (uint16_t i = 0; i < strip[index].numPixels(); i = i + 3)
                {
                        strip[index].setPixelColor(i + q, Wheel(index, (i + j) % 255));    //turn every third pixel on
                }
                strip[index].show();
                delay(1);
            }
            delay(40);


            for (int index = 0; index < 7; index++)
            {
                for (uint16_t i = 0; i < strip[index].numPixels(); i = i + 3)
                {
                    strip[index].setPixelColor(i + q, 0);        //turn every third pixel off
                }
            }
        }
}



uint32_t Wheel(int index, byte WheelPos)
{
    WheelPos = WheelPos%255;
    if (WheelPos < 85)
    {
        return strip[index].Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return strip[index].Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip[index].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
uint32_t negWheel(int index, byte WheelPos)
{
    WheelPos = WheelPos%255;
    if (WheelPos < 85)
    {
        return strip[index].Color(WheelPos * 3, 255 - 0, 255 - WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return strip[index].Color(255 - 0, 255 - WheelPos * 3, WheelPos * 3);
    }
    WheelPos -= 170;
    return strip[index].Color(255 - WheelPos * 3, WheelPos * 3, 255 - 0);
}
