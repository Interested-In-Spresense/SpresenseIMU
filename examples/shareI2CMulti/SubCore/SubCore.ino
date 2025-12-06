/*
 *  SubCore.ino - Sample for sharing I2C with multi-core.
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
 
#include <MP.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface

#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  unsigned status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(3000);

}

// ------------------------------------------------------
// Loop (wait for main → measure → return)
// ------------------------------------------------------
void loop()
{
  int8_t msgid;
  uint32_t dummy;

  if (MP.Recv(&msgid, &dummy) >= 0 && msgid == MSG_REQ_BMP) {

    up_enable_irq(CXD56_IRQ_SCU_I2C0);

    Serial.print(F("[Sub] Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("[Sub] Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("[Sub] Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    Serial.flush();

    float hpa = bmp.readTemperature();

    up_disable_irq(CXD56_IRQ_SCU_I2C0);
    MP.Send(MSG_RET_BMP, hpa);

  } else {
    Serial.println("[Sub] recv timeout");
  }

}
