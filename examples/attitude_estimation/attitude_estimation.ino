/*
 *  attitude_estimation.ino - Attitude estimation with AHRS.
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
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

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

  MadgwickFilter.begin(SAMPLINGRATE);
  
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
    cxd5602pwbimu_data_t data;
    if (SpresenseIMU.get(data)) {
      float timestamp = data.timestamp / 19200000.0f;
      MadgwickFilter.updateIMU(data.gx*180/PI, data.gy*180/PI, data.gz*180/PI, data.ax, data.ay, data.az);
      float roll  = MadgwickFilter.getRoll();
      float pitch = MadgwickFilter.getPitch();
      float yaw   = MadgwickFilter.getYaw();      
      printf("%4.2F,%4.2F,%F,%F,%F\n", timestamp, data.temp, roll, pitch, yaw);
  
    }
}

