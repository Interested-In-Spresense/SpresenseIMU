/*
 *  DispCore.ino - Gyro compass sample on MultiCore with Display.
 *  Author Interested-In-Spresense
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <MP.h>

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <cmath>

U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/****************************************************************************
 * drawTriangleLine
 ****************************************************************************/
void drawTriangleLine(int x0,int y0,int x1,int y1,int x2,int y2)
{
  u8g2.drawLine(x0,y0, x1,y1);
  u8g2.drawLine(x1,y1, x2,y2);
  u8g2.drawLine(x2,y2, x0,y0);
}

/****************************************************************************
 * calibration
 ****************************************************************************/
void calibration()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
  u8g2.drawStr( 0, 20, "Now calibration!");
  u8g2.sendBuffer();
  delay(100);


}

void display(float angle)
{
  printf("Angle = %F\n", angle);
  float x = cos((angle+90.0) * M_PI / 180);
  float y = sin((angle+90.0) * M_PI / 180);
  int x0 = x*48+64;

  int y0 = y*48+48;
  int x1 = (-y*48)/4+64;
  int y1 = (x*48)/4+48;
  int x2 = (y*48)/4+64;
  int y2 = (-x*48)/4+48;
  int x3 = -x*48+64;

  int y3 = -y*48+48;

//  printf("+ x,y = %d,%d\n", x0,y0);
//  printf("- x,y = %d,%d\n", x3,y3);
  u8g2.clearBuffer();
  u8g2.drawCircle(64,47,47);
  u8g2.drawTriangle(x0,y0, x1,y1, x2,y2);
  drawTriangleLine(x3,y3, x1,y1, x2,y2);
  u8g2.sendBuffer();


}

void setup()
{
  int ret = 0;

  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  u8g2.begin();
  calibration();

}

float baseAngle=0.0;

void loop()
{
  int8_t   msgid;
  uint32_t msgdata;

  int ret = MP.Recv(&msgid, &msgdata);
  if (ret < 0) {
    errorLoop(3);
  }

  switch(msgid){
    case 10:
    baseAngle=(float)msgdata;
    printf("base angle = %F\n", baseAngle);
    break;

    case 100:
    display((float)msgdata);
    break;

    default:
    errorLoop(2);
    break;

  }

}

void errorLoop(int num)
{
  int i;

  while (1) {
    for (i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}