/*
 *  sample.ino - Raw date read sample.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU設定
#define SAMPLINGRATE (60)    // Hz
#define ADRANGE       (4)    // G
#define GDRANGE (    500)    // dps
#define FIFO_DEPTH    (1)    // FIFO

/****************************************************************************
 * Setup
 ****************************************************************************/
void setup(void)
{
  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0)
    {
      printf("Spresense IMU begin.\n");
      return;
    }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
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
}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  pwbImuData imuData;
  if (SpresenseIMU.get(imuData)) {
     imuData.print();
/*    float timestamp = imuData.data.timestamp / 19200000.0f;
    printf("%4.2F,%4.2F,%F,%F,%F,%F,%F,%F\n", timestamp, imuData.data.temp, imuData.data.ax, 
            imuData.data.ay, imuData.data.az, imuData.data.gx, imuData.data.gy, imuData.data.gz);*/
  }
}

