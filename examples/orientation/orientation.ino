/*
 *  orientation.ino - Orientation with AHRS.
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

#define USE_QUATERNION

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU setting
#define SAMPLINGRATE (60)   // Hz
#define ADRANGE      (4)    // G
#define GDRANGE      (500)  // dps
#define FIFO_DEPTH   (1)    // FIFO

/****************************************************************************
 * Calibrate
 ****************************************************************************/
float gyroBias[3] = {0, 0, 0};
float trueRate = 0;

void calibrateGyroBias(int ms = 2000) {
  printf("Please Stop for Caribration(%d ms)...\n", ms);

  cxd5602pwbimu_data_t raw;
  double sum[3] = {0, 0, 0};
  int count = 0;
  float first_ts = 0;
  float last_ts = 0;
  unsigned long start = millis();

  while (millis() - start < (unsigned)ms) {
    if (SpresenseIMU.get(raw)) {
      sum[0] += raw.gx * 180 / PI;
      sum[1] += raw.gy * 180 / PI;
      sum[2] += raw.gz * 180 / PI;
      if(count==0) {
        first_ts = (raw.timestamp  / 19200000.0f);
      }else{
        last_ts = (raw.timestamp  / 19200000.0f);
      }
      count++;
    }
  }

  if (count > 0) {
    gyroBias[0] = sum[0] / count;
    gyroBias[1] = sum[1] / count;
    gyroBias[2] = sum[2] / count;
    trueRate = 1.0f / ( (last_ts - first_ts) / count);
  }

  printf("Gyro Bias (deg/s): %f, %f, %f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
  printf("Sampling Rate: %f\n", trueRate);
}


/****************************************************************************
 * Setup
 ****************************************************************************/
void setup(void)
{
  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0)
    {
      printf("Spresense IMU begin error!.\n");
      return;
    }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) {
    SpresenseIMU.end();
    return;
  }

  ret = SpresenseIMU.start();
  if (!ret) {
    SpresenseIMU.finalize();
    SpresenseIMU.end();
    return;
  }

  sleep(1);
  
  ledOn(LED0); ledOn(LED1); ledOn(LED2); ledOn(LED3);
  calibrateGyroBias(2000);
  ledOff(LED0); ledOff(LED1); ledOff(LED2); ledOff(LED3);

  MadgwickFilter.begin(trueRate);

}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  cxd5602pwbimu_data_t data;
  if (SpresenseIMU.get(data)) {

    float timestamp = data.timestamp / 19200000.0f;

    // Update Madgwick
    MadgwickFilter.updateIMU(
      data.gx * 180/PI,
      data.gy * 180/PI,
      data.gz * 180/PI,
      data.ax, data.ay, data.az
    );

#ifdef USE_QUATERNION
    // --- Quaternion ---
    float q0 = MadgwickFilter.getQ0();
    float q1 = MadgwickFilter.getQ1();
    float q2 = MadgwickFilter.getQ2();
    float q3 = MadgwickFilter.getQ3();

    printf("%4.2F,%4.2F,%F,%F,%F,%F\n",
           timestamp, data.temp, q0, q1, q2, q3);

#else
    // --- Euler ---
    float roll  = MadgwickFilter.getRoll();
    float pitch = MadgwickFilter.getPitch();
    float yaw   = MadgwickFilter.getYaw();

    printf("%4.2F,%4.2F,%F,%F,%F\n",
           timestamp, data.temp, roll, pitch, yaw);
#endif
  }
}
