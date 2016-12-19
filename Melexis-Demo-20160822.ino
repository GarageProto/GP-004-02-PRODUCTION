/*
* Adapted by Josh Long (https://github.com/longjos) Oct 2015
* Based on a https://github.com/robinvanemden/MLX90621_Arduino_Processing
* Modified by Robert Chapman and Mike Blankenship to work with the GP-001-01 (MLX906XX EVAL Arduino shield) and GP-004-02 (Thermal Imager)
* TFT graphics and Interpolation sketch by Sandbox Electronics http://sandboxelectronics.com/
* Verified to work on the Arduino UNO R3 and Leonardo
* Original work by:
* 2013 by Felix Bonowski
* Based on a forum post by maxbot: http://forum.arduino.cc/index.php/topic,126244.msg949212.html#msg949212
* This code is in the public domain.
*/

#include <SPI.h>
#include <Wire.h>
#include "MLX90621.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define RANGE_MIN       (-20)
#define RANGE_MAX       (300)
#define TEMP_STEPS     (1020)
#define BLOCK_SIZE       (11)

#define TFT_CS            (8)
#define TFT_DC            (7)
#define TFT_RST           (0)

#define TFT_CS_PORT     PORTB
#define TFT_CS_PIN        (0)
#define TFT_DC_PORT     PORTD
#define TFT_DC_PIN        (7)

#define MLX_VOLTAGE      (A7) //added by tom
#define BAT_VOLTAGE      (A6) //added by tom
#define PERIPHERAL_POWER  (3) //added by tom
#define LASER_PIN         (5) //added by tom

#define TrackballUp      (A3)
#define TrackballLeft    (A2)
#define TrackballDown    (A1)
#define TrackballRight   (A0)
#define TrackballButton   (2)
#define LED_RED           (6)
#define LED_GREEN         (9)
#define LED_BLUE         (10)
#define CURSOR_TH         (3)

#define BTN_W            (41)
#define BTN_H            (19)
#define BTN_X            (76)
#define BTN_Y            (88)

#define RANGE             (0)
#define CURSOR            (1)
#define LASER             (2)
#define SLEEP             (3)

#define CURSOR_ROW_MIN    (0)
#define CURSOR_ROW_MAX    (6)
#define CURSOR_COL_MIN    (0)
#define CURSOR_COL_MAX   (15)

#define FOCUS        (0xFC00)
#define BLUR    (ST7735_CYAN)

struct {
    uint8_t prr_val;
    uint8_t didr0_val;
    uint8_t didr1_val;
    uint8_t acsr;
    uint8_t adcsra_val;
    uint8_t pcicr;
} PowerDown;

struct {
    uint8_t  scroll;
    int8_t   row;
    int8_t   col;
    uint8_t  counter_up;
    uint8_t  counter_down;
    uint8_t  counter_left;
    uint8_t  counter_right;
    uint16_t timer_up;
    uint16_t timer_down;
    uint16_t timer_left;
    uint16_t timer_right;
    uint32_t millis;
    uint8_t  range;;
    uint8_t  cursor;
    uint8_t  laser;
    uint8_t  focus;
    int16_t  rangeMin;
    int16_t  rangeMax;
    uint8_t  rangeFocus;
    uint8_t  press;
} Trackball;

Adafruit_ST7735   tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
MLX90621          Sensor21;
Adafruit_MLX90614 Sensor14;
float             AmbientTemp;
float             CursorTemp = 0;
uint32_t          LoopTimer;
byte              LoopCounter = 0;
char             *ButtonText[]   = {"Range", "Cursor", "Laser", "Sleep"};
uint8_t           ButtonActive[] = {false, false, false, false};
uint8_t           SensorType;

void setup() {
    CursorSetup();
    PeripheralInitialize();
    attachInterrupt(digitalPinToInterrupt(TrackballButton), pressISR, FALLING);
}

void loop() {
    byte  x, y;
    float t;
    uint16_t grids[4][16];

    PressControl();

    if (SensorType == 21) {
        Sensor21.measure(true);
        AmbientTemp = Sensor21.getAmbient();
    } else {
        AmbientTemp = Sensor14.readAmbientTempC();
    }

    if (LoopCounter == 0) {
        showVBATvolt();
        showMLXvolt();
        showAmbientTemp();
        LoopTimer = millis();
    }

    Serial.println("IRSTART");
    Serial.print("MLX906"); Serial.println(SensorType);

    if (SensorType == 21) {
        for(y=0; y<4; y++){ //go through all the rows
            for(x=0; x<16; x++){ //go through all the columns
                t = Sensor21.getTemperature(y+x*4);
                grids[y][15-x] = mapT(t);
                Serial.print(t); Serial.print(",");

                if (Trackball.col == 15-x) {
                    if (Trackball.row%2) {
                        if ((Trackball.row-1)/2 == y) {
                            CursorTemp = t;
                        } else if ((Trackball.row+1)/2 == y) {
                            CursorTemp = (CursorTemp + t) / 2;
                        }
                    } else {
                        if (Trackball.row/2 == y) {
                            CursorTemp = t;
                        }
                    }
                }
            }
            Serial.println();
        }

        for (y=0; y<3; y++) {
            for (x=0; x<15; x++) {
                drawBlock(y, x, grids[y][x], grids[y][x+1], grids[y+1][x], grids[y+1][x+1]);
            }
        }
    } else {
        CursorTemp = Sensor14.readObjectTempC();
        tft.fillRect(5, 0, 151, 62, mapC(mapT(CursorTemp)));
    }

    Serial.print("TA=");
    Serial.print(AmbientTemp);
    Serial.print(",");

    Serial.print("CPIX=");
    Serial.print(Sensor21.get_CPIX()); // GET VALUE MIKEY red shOW
    Serial.print(",");

    Serial.print("PTAT=");
//    Serial.print(Sensor21.get_PTAT()); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("EMISSIVITY=");
    Serial.print("1"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("V_TH=");
    //Serial.print(Sensor21.get_KT1());
    Serial.print("6760"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("K_T1=");
    Serial.print("23.03"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("K_T2=");
    Serial.print("0.02"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("MY_TEMP="); //DS18B20 TEMP
    Serial.print(0);
    Serial.print(",");

    Serial.println("IREND");

    if (++LoopCounter == 10) {
        showFPS();
        LoopCounter = 0;
    }

    CursorControl();

    if (Trackball.focus == CURSOR && ButtonActive[CURSOR]) {
        CursorDisplay();
    }
}


void showVBATvolt() {
    char  buf[8];

    dtostrf(analogRead(BAT_VOLTAGE) * (5.0 / 1023), 5, 2, buf);
    buf[5] = 0x00;
    tft.fillRect(30, 90, 30, 7, ST7735_BLACK);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(30, 90);
    tft.print(buf);
}


void showMLXvolt() {
    char  buf[8];

    dtostrf(analogRead(MLX_VOLTAGE) * (5.0 / 1023), 5, 2, buf);
    buf[5] = 0x00;
    tft.fillRect(30, 100, 30, 7, ST7735_BLACK);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(30, 100);
    tft.print(buf);
}


void showFPS() {
    char  buf[8];

    dtostrf(10 * 1000.0 / (millis()-LoopTimer), 5, 2, buf);
    buf[5] = 0x00;
    tft.setTextColor(ST7735_RED);
    tft.fillRect(30, 120, 30, 7, ST7735_BLACK);
    tft.setCursor(30, 120);
    tft.print(buf);
}


void showAmbientTemp() {
    char buf[8];

    dtostrf(AmbientTemp, 5, 2, buf);
    buf[5] = 0x00;
    tft.fillRect(30, 110, 30, 7, ST7735_BLACK);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(30, 110);
    tft.print(buf);
}


uint16_t mapT(float temp) {
    int16_t t;

    t = (int16_t)((temp - Trackball.rangeMin) / (Trackball.rangeMax - Trackball.rangeMin) * TEMP_STEPS);

    if (t > TEMP_STEPS) {
        t = TEMP_STEPS;
    } else if (t < 0) {
        t = 0;
    }

    return t;
}


void drawBlock(byte row, byte column, uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    byte i, j, x, y;
    byte red, green, blue;
    uint16_t color;
    byte i_start = 2;
    byte j_start = 1;

    if (row == 0) {
        i_start = 0;
    }

    if (column == 0) {
        j_start = 0;
    }

    uint16_t ac[BLOCK_SIZE];
    uint16_t bd[BLOCK_SIZE];
    uint16_t line[BLOCK_SIZE];

    interpolateT(ac, a, c);
    interpolateT(bd, b, d);

    x = column * (BLOCK_SIZE-1) + 5 + j_start;
    y = row * (BLOCK_SIZE-1) * 2 + i_start;

    tft.setAddrWindow(x, y, x + BLOCK_SIZE - 1 - j_start, y + BLOCK_SIZE * 2 - 1 - i_start);

    for (i=i_start; i<BLOCK_SIZE * 2; i++) {
        if (i%2 == 0) {//uint32_t start = micros();
            interpolateC(line, ac[i/2], bd[i/2], BLOCK_SIZE);//Serial.println(micros() - start);
        }

        for (j=j_start; j<BLOCK_SIZE; j++) {
            TFT_DC_PORT |= 1 << TFT_DC_PIN;
            TFT_CS_PORT &= ~(1 << TFT_CS_PIN);

            SPDR = (line[j] >> 8);
            while(!(SPSR & _BV(SPIF)));
            SPDR = (line[j]);
            while(!(SPSR & _BV(SPIF)));

            TFT_CS_PORT |= 1 << TFT_CS_PIN;
        }
    }
}


void interpolateT(uint16_t *line, uint16_t p, uint16_t q) {
    byte i;

    for (i=0; i<BLOCK_SIZE; i++) {
        line[i] = (p * (BLOCK_SIZE - i) + q * i) / BLOCK_SIZE;
    }
}


void interpolateC(uint16_t *line, uint16_t p, uint16_t q, uint8_t block_size) {
    byte     i, r, g, b;
    uint16_t t;

    for (i=0; i<block_size; i++) {
        t = (p * (block_size - i) + q * i) / block_size;

        //RED
        if (t >= 765) {
            r = 255;
        } else if (t > 510) {
            r = t - 510;
        } else {
            r = 0;
        }

        //GREEN
        if (t > 765) {
            g = 1020 - t;
        } else if (t >= 255) {
            g = 255;
        } else {
            g = t;
        }

        //BLUE
        if (t >= 510) {
            b = 0;
        } else if (t > 255) {
            b = 510 - t;
        } else {
            b = 255;
        }

        line[i] = r >> 3 << 11 | g >> 2 << 5 | b >> 3;
    }
}


uint16_t mapC(uint16_t t) {
    byte r, g, b;

    //RED
    if (t >= 765) {
        r = 255;
    } else if (t > 510) {
        r = t - 510;
    } else {
        r = 0;
    }

    //GREEN
    if (t > 765) {
        g = 1020 - t;
    } else if (t >= 255) {
        g = 255;
    } else {
        g = t;
    }

    //BLUE
    if (t >= 510) {
        b = 0;
    } else if (t > 255) {
        b = 510 - t;
    } else {
        b = 255;
    }

    return r >> 3 << 11 | g >> 2 << 5 | b >> 3;
}


void PeripheralPowerControl(uint8_t power_status) {
    if (power_status !=0) {
        pinMode(PERIPHERAL_POWER, OUTPUT);
        digitalWrite(PERIPHERAL_POWER, LOW);
    } else {
        pinMode(PERIPHERAL_POWER,OUTPUT);
        digitalWrite(PERIPHERAL_POWER, HIGH);
    }
}


void LaserControl(uint8_t laser_status) {
    // SB orignal digitalWrite(LASER_PIN, laser_status);

    // RED 08-21/2016
     if(laser_status){
    analogWrite(LASER_PIN, 32);
  } else {
    analogWrite(LASER_PIN, 0);
  }
}


void CursorDisplay() {
    byte row = Trackball.row;
    byte col = Trackball.col;
    byte x   = Trackball.col * 10 + 5;
    byte y   = Trackball.row * 10;
    byte x_size = 3;
    byte y_size = 3;
    char buf[7];
    //uint16_t t;

    if (SensorType == 21) {
        if (Trackball.row == CURSOR_ROW_MIN) {
            y += 1;
        } else if (Trackball.row == CURSOR_ROW_MAX) {
            //y -= 1;
        }

        if (Trackball.col == CURSOR_COL_MIN) {
            x += 1;
        } else if (Trackball.col == CURSOR_COL_MAX) {
            x -= 1;
        }

        tft.drawRect(x-1, y-1, x_size, y_size, ST7735_WHITE);
    }

    dtostrf(CursorTemp, 1, 2, buf);
    buf[7] = '\0';
    drawButton(1, buf);
    TrackballDisplay(mapC(mapT(CursorTemp)));
}


void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


void TrackballInitialize(void) {
    pinMode(TrackballUp,INPUT);
    pinMode(TrackballLeft,INPUT);
    pinMode(TrackballDown,INPUT);
    pinMode(TrackballRight,INPUT);

    pciSetup(TrackballUp);
    pciSetup(TrackballLeft);
    pciSetup(TrackballDown);
    pciSetup(TrackballRight);

    Trackball.scroll = PINC&0x0F;
}


void TrackballDisplay(uint16_t color) {
    uint8_t r, g, b;

    r = color >> 11;
    g = color >> 5 & 0x3F;
    b = color & 0x1F;

    analogWrite(LED_RED,   r * 40/255);
    analogWrite(LED_GREEN, g << 2);
    analogWrite(LED_BLUE,  b * 80/255);
}


ISR (PCINT1_vect) { // handle pin change interrupt for A0 to A5 here
    uint8_t key;

    key = PINC & 0x0F;
    key = key^Trackball.scroll;
    Trackball.scroll = PINC&0x0F;

    if (key&0x01) {
        Trackball.counter_left++;
        Trackball.timer_left = 200;
    }

    if (key&0x02) {
        Trackball.counter_up++;
        Trackball.timer_up = 200;
    }

    if (key&0x04) {
        Trackball.counter_right++;
        Trackball.timer_right = 200;
    }

    if (key&0x08) {
        Trackball.counter_down++;
        Trackball.timer_down = 200;
    }
}


void PeripheralInitialize(void) {
    analogWrite(LED_BLUE, 0);
    analogWrite(LED_RED, 0);
    analogWrite(LED_GREEN, 0);
    pinMode(LED_RED,OUTPUT);
    pinMode(LED_GREEN,OUTPUT);
    pinMode(LED_BLUE,OUTPUT);

    pinMode(PERIPHERAL_POWER, OUTPUT);
    PeripheralPowerControl(1);

    pinMode(LASER_PIN, OUTPUT);
    if (ButtonActive[LASER]) {
        LaserControl(1);
    } else {
        LaserControl(0);
    }

    Serial.begin(115200);
    Wire.begin();

    DisplaySetup();

    digitalWrite(TrackballButton, 1);
    pinMode(TrackballButton, INPUT);
    TrackballInitialize();

    analogReference(INTERNAL);analogRead(BAT_VOLTAGE);analogRead(BAT_VOLTAGE);

    if (SensorType == 21) {
        Sensor21.initialise(16);
    } else {
        Sensor14.begin();
    }
}


void DisplaySetup() {
    uint8_t i;
    char    buf[9] = "MLX906";

    tft.initR(INITR_BLACKTAB);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    tft.setRotation(3);

    //Display Splash Screen
    tft.fillScreen(ST7735_WHITE);//0x128C
    tft.fillRect(0, 0, 160, 56, Color16(23, 88, 77));
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(3);
    tft.setCursor(19, 18);
    tft.print("Melexis");
    tft.setTextColor(Color16(21, 80, 70));
    tft.setTextSize(1);
    tft.setCursor(25, 75);
    tft.print("Detecting Sensor...");
    delay(1000);

    //Check Sensor Type
    Wire.beginTransmission(0x50);
    if (Wire.endTransmission() == 0) {
        SensorType = 21;
        buf[6] = '2';
        buf[7] = '1';
        buf[8] = '\0';
    } else {
        Wire.beginTransmission(0x5A);
        if (Wire.endTransmission() == 0) {
            SensorType = 14;
            buf[6] = '1';
            buf[7] = '4';
            buf[8] = '\0';
        } else {
            tft.fillRect(25, 75, 115, 36, ST7735_WHITE);
            tft.setTextColor(ST7735_RED);
            tft.setCursor(30, 86);
            tft.print("No Sensor");
            while(1);
        }
    }

    if (SensorType) {
        tft.drawRect(54, 94, 57, 17, Color16(23, 88, 77));
        tft.setCursor(59, 99);
        tft.print(buf);
        delay(500);
        tft.fillRect(25, 75, 115, 36, ST7735_WHITE);
    }

    //MLX906XX Demo
    tft.setTextSize(2);
    tft.setCursor(20, 73);

    for (i=0; i<8; i++) {
        tft.print(buf[i]);
        delay(100);
    }

    tft.setCursor(95, 99);
    tft.print("Demo");
    delay(1500);

    //Main Screen
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(1);
    //VBAT
    tft.setTextColor(ST7735_BLUE);
    tft.setCursor(0, 90);
    tft.print("VBAT:");
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(66, 90);
    tft.print("V");
    //VMLX
    tft.setTextColor(ST7735_BLUE);
    tft.setCursor(0, 100);
    tft.print("VMLX:");
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(66, 100);
    tft.print("V");
    //AmbT
    tft.setTextColor(ST7735_BLUE);
    tft.setCursor(0, 110);
    tft.print("AmbT:");
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(66, 110);
    tft.print("C");
    tft.fillRect(63, 110, 2, 2, ST7735_YELLOW);
    //FPS
    tft.setTextColor(ST7735_BLUE);
    tft.setCursor(0, 120);
    tft.print("FPS :");
    //Hue
    for (uint8_t i=0; i<101; i++) {
        tft.drawFastVLine(30+i, 66, 18, mapC((TEMP_STEPS*(uint32_t)i)/101));
    }
    //Range
    drawRange();
    //Buttons
    drawButton(RANGE, 0);
    drawButton(CURSOR,0);
    drawButton(LASER, 0);
    drawButton(SLEEP, 0);
}


void MCUPowerDown() {
    PeripheralPowerControl(0);
    //Disable ADC
    PowerDown.adcsra_val = ADCSRA;
    PowerDown.acsr = ACSR;
    ADCSRA = 0;
    ACSR  &= ~_BV(ACIE);
    ACSR  |=  _BV(ACD);

    //Disable Input Buffer
    PowerDown.didr0_val = DIDR0;
    PowerDown.didr1_val = DIDR1;
    DIDR0 = 0x3F;
    DIDR1 = 0x03;

    //Disable pin on change interrupt;
    PowerDown.pcicr = PCICR;
    PCICR = 0;

    DDRC  = 0;
    PORTC = 0;

    PORTD = 0x04;
    DDRD  = 0x20;
    PORTB = 0x00;
    DDRB  = 0x04;

    delay(1000);
    PCIFR = 0X07;
    Wire.end();
    SPI.end();
    Serial.end();

    // Disable unnecessary clocks
    PowerDown.prr_val = PRR;
    PRR = 0xFF;

    noInterrupts();
    SMCR  = _BV(SM1) | _BV(SE);
    MCUCR = 0x60;
    MCUCR = 0x40;
    interrupts();
    //MCUCR |= 0x10;
    sleep_cpu();

    PRR = PowerDown.prr_val;

    ADCSRA = PowerDown.adcsra_val;
    ACSR   = PowerDown.acsr;

    //delay(500);
    PeripheralInitialize();

    PCIFR = 0X07;
    PCICR = PowerDown.pcicr;

    LoopCounter = 0;
}


void PressControl(void) {
    if ((PIND & _BV(TrackballButton)) && Trackball.press) {
        switch (Trackball.focus) {
        case RANGE:
            ButtonActive[RANGE] = !ButtonActive[RANGE];
            drawButton(RANGE, 0);
            drawRange();
            break;

        case CURSOR:
            ButtonActive[CURSOR] = !ButtonActive[CURSOR];
            drawButton(CURSOR, 0);

            if (!ButtonActive[CURSOR]) {
                TrackballDisplay(ST7735_BLACK);
            }
            break;

        case LASER:
            ButtonActive[LASER] = !ButtonActive[LASER];
            LaserControl(ButtonActive[LASER]);
            drawButton(LASER, 0);
            break;

        case SLEEP:
            MCUPowerDown();
            break;
        }

        Trackball.press = false;
    }
}


void CursorSetup() {
    Trackball.row = 3;
    Trackball.col = 8;
    Trackball.counter_up    = 0;
    Trackball.counter_down  = 0;
    Trackball.counter_left  = 0;
    Trackball.counter_right = 0;
    Trackball.millis = millis();
    Trackball.range  = false;
    Trackball.cursor = true;
    Trackball.laser  = true;
    Trackball.focus  = SLEEP;
    // SB default Trackball.rangeMin = 0;
    // SB default Trackball.rangeMax = 50;
    Trackball.rangeMin = 10; //MIKEY
    Trackball.rangeMax = 35; //MIKEY
    Trackball.rangeFocus = 0;
    Trackball.press = false;
}


void CursorControl() {
    uint32_t elapsed = millis() - Trackball.millis;
    uint8_t  up      = Trackball.counter_up    / CURSOR_TH;
    uint8_t  down    = Trackball.counter_down  / CURSOR_TH;
    uint8_t  left    = Trackball.counter_left  / CURSOR_TH;
    uint8_t  right   = Trackball.counter_right / CURSOR_TH;

    if (up) {
        ScrollHandler_Up(up);
    }

    if (down) {
        ScrollHandler_Down(down);
    }

    if (left) {
        ScrollHandler_Left(left);
    }

    if (right) {
        ScrollHandler_Right(right);
    }

    //Clear counter when timer expires
    if (Trackball.timer_up <= elapsed) {
        Trackball.timer_up   = 0;
        Trackball.counter_up = 0;
    } else {
        Trackball.timer_up  -= elapsed;
        Trackball.counter_up = Trackball.counter_up % CURSOR_TH;
    }

    if (Trackball.timer_down <= elapsed) {
        Trackball.timer_down   = 0;
        Trackball.counter_down = 0;
    } else {
        Trackball.timer_down  -= elapsed;
        Trackball.counter_down = Trackball.counter_down % CURSOR_TH;
    }

    if (Trackball.timer_left <= elapsed) {
        Trackball.timer_left   = 0;
        Trackball.counter_left = 0;
    } else {
        Trackball.timer_left  -= elapsed;
        Trackball.counter_left = Trackball.counter_left % CURSOR_TH;
    }

    if (Trackball.timer_right <= elapsed) {
        Trackball.timer_right   = 0;
        Trackball.counter_right = 0;
    } else {
        Trackball.timer_right  -= elapsed;
        Trackball.counter_right = Trackball.counter_right % CURSOR_TH;
    }
}


void ScrollHandler_Up(uint8_t n) {
    if (ButtonActive[RANGE]) {
        if (Trackball.rangeFocus == 0) {
            Trackball.rangeMin += n * 5;
            if (Trackball.rangeMin >= Trackball.rangeMax) {
                Trackball.rangeMin  = Trackball.rangeMax - 5;
            }
        } else {
            Trackball.rangeMax += n * 5;
            if (Trackball.rangeMax > RANGE_MAX) {
                Trackball.rangeMax = RANGE_MAX;
            }
        }
        drawRange();
    } else if (ButtonActive[CURSOR]) {
        Trackball.row -= n;
        if (Trackball.row < CURSOR_ROW_MIN) {
            Trackball.row = CURSOR_ROW_MIN;
        }
    } else {
        if (Trackball.focus == LASER) {
            Trackball.focus = RANGE;
            drawButton(LASER, 0);
            drawButton(RANGE, 0);
        } else if (Trackball.focus == SLEEP) {
            Trackball.focus = CURSOR;
            drawButton(SLEEP, 0);
            drawButton(CURSOR,0);
        }
    }
}


void ScrollHandler_Down(uint8_t n) {
    if (ButtonActive[RANGE]) {
        if (Trackball.rangeFocus == 0) {
            Trackball.rangeMin -= n * 5;
            if (Trackball.rangeMin < RANGE_MIN) {
                Trackball.rangeMin = RANGE_MIN;
            }
        } else {
            Trackball.rangeMax -= n * 5;
            if (Trackball.rangeMax <= Trackball.rangeMin) {
                Trackball.rangeMax  = Trackball.rangeMin + 5;
            }
        }
        drawRange();
    } else if (ButtonActive[CURSOR]) {
        Trackball.row += n;
        if (Trackball.row > CURSOR_ROW_MAX) {
            Trackball.row = CURSOR_ROW_MAX;
        }
    } else {
        if (Trackball.focus == RANGE) {
            Trackball.focus = LASER;
            drawButton(RANGE, 0);
            drawButton(LASER, 0);
        } else if (Trackball.focus == CURSOR) {
            Trackball.focus = SLEEP;
            drawButton(CURSOR,0);
            drawButton(SLEEP, 0);
        }
    }
}


void ScrollHandler_Left(uint8_t n) {
    if (ButtonActive[RANGE]) {
        if (Trackball.rangeFocus != 0) {
            Trackball.rangeFocus = 0;
            drawRange();
        }
    } else if (ButtonActive[CURSOR]) {
        Trackball.col -= n;
        if (Trackball.col < CURSOR_COL_MIN) {
            Trackball.col = CURSOR_COL_MIN;
        }
    } else {
        if (Trackball.focus == CURSOR) {
            Trackball.focus = RANGE;
            drawButton(CURSOR,0);
            drawButton(RANGE, 0);
        } else if (Trackball.focus == SLEEP) {
            Trackball.focus = LASER;
            drawButton(SLEEP, 0);
            drawButton(LASER, 0);
        }
    }
}


void ScrollHandler_Right(uint8_t n) {
    if (ButtonActive[RANGE]) {
        if (Trackball.rangeFocus != 1) {
            Trackball.rangeFocus = 1;
            drawRange();
        }
    } else if (ButtonActive[CURSOR]) {
        Trackball.col += n;
        if (Trackball.col > CURSOR_COL_MAX) {
            Trackball.col = CURSOR_COL_MAX;
        }
    } else {
        if (Trackball.focus == RANGE) {
            Trackball.focus = CURSOR;
            drawButton(RANGE, 0);
            drawButton(CURSOR,0);
        } else if (Trackball.focus == LASER) {
            Trackball.focus = SLEEP;
            drawButton(LASER, 0);
            drawButton(SLEEP, 0);
        }
    }
}


void drawButton(byte btn, char *text) {
    byte     x      = BTN_X;
    byte     y      = BTN_Y;
    byte     active = false;
    uint16_t color;

    if (btn%2) {
        x += BTN_W + 2;
    }

    if (btn/2) {
        y += BTN_H + 2;
    }

    if (!text) {
        text = ButtonText[btn];
    }

    if (Trackball.focus == btn) {
        color = FOCUS;
    } else {
        color = BLUR;
    }

    if (ButtonActive[btn]) {
        tft.fillRect(x, y, BTN_W, BTN_H, color);
        tft.setTextColor(ST7735_BLACK);
    } else {
        tft.fillRect(x, y, BTN_W, BTN_H, ST7735_BLACK);
        tft.drawRect(x, y, BTN_W, BTN_H, color);
        tft.setTextColor(color);
    }

    tft.setCursor(x+1+BTN_W/2-strlen(text)*3, y+6);
    tft.setTextSize(1);
    tft.print(text);
}


void drawRange() {
    char     buf[5];
    uint16_t color0, color1;

    if (ButtonActive[RANGE]) {
        if (Trackball.rangeFocus == 0) {
            color0 = FOCUS;
            color1 = BLUR;
        } else {
            color0 = BLUR;
            color1 = FOCUS;
        }
    } else {
        color0 = BLUR;
        color1 = BLUR;
    }

    //Range_Min
    tft.setTextColor(color0);
    sprintf(buf, "%d", Trackball.rangeMin);
    tft.setCursor(18-strlen(buf)*3, 72);
    tft.fillRect(0, 72, 30, 7, ST7735_BLACK);
    tft.print(buf);
    tft.fillTriangle(17, 65, 13, 69, 21, 69, color0);
    tft.fillTriangle(17, 85, 13, 81, 21, 81, color0);

    //Range_Max
    tft.setTextColor(color1);
    sprintf(buf, "%d", Trackball.rangeMax);
    tft.setCursor(145-strlen(buf)*3, 72);
    tft.fillRect(131, 72, 29, 7, ST7735_BLACK);
    tft.print(buf);
    tft.fillTriangle(144, 65, 140, 69, 148, 69, color1);
    tft.fillTriangle(144, 85, 140, 81, 148, 81, color1);
}


uint16_t Color16(uint8_t r, uint8_t g, uint8_t b) {
    return r >> 3 << 11 | g >> 2 << 5 | b >> 3;
}


void pressISR() {
    int16_t counter = 1000;

    while (!(PIND & _BV(2)) && counter) {
        counter--;
    }

    if (counter == 0) {
        Trackball.press = true;
    }
}




