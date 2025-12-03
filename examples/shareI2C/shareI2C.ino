/*
 *  shareI2C.ino - Sample for Multi-IMU and sensors simultaneously over I2C.
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

#include "SpresenseIMU.h"
#include "BMI160Gen.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU設定
#define SAMPLINGRATE (60)    // Hz
#define ADRANGE       (4)    // G
#define GDRANGE (    500)    // dps
#define FIFO_DEPTH    (1)    // FIFO

#define RESET_INTERVAL 20    // restart every N samples

/****************************************************************************
 * Setup
 ****************************************************************************/
void setup(void)
{
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);      // wait for the serial port to open

  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0)
    {
      printf("Spresense IMU begin.\n");
      return;
    }

  BMI160.begin();

  // Set the accelerometer range to 2G
  BMI160.setAccelerometerRange(2);

}


/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  pwbImuData imuData;

  int ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret)
    {
      SpresenseIMU.end();
      return;
    }

  ret = SpresenseIMU.start();
  if (!ret)
    {
      SpresenseIMU.finalize();
      SpresenseIMU.end();
      return;
    }

  if (!ret)
    {
      SpresenseIMU.end();
      return;
    }

  int count = 0;
  while (count < RESET_INTERVAL)
    {
      if (SpresenseIMU.get(imuData))
        {
          imuData.print();
          count++;
        }
      else
        {
          Serial.println("[ERROR] get() failed, restarting IMU...");
          break;
        }
    }
  ret = SpresenseIMU.stop();

  float ax, ay, az;   //Accelerometer values for BMI160
  // read accelerometer measurements from BMI160
  BMI160.readAccelerometerScaled(ax, ay, az);

  // display tab-separated accelerometer x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();

}

