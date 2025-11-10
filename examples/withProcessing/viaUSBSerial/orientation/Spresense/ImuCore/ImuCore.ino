/*
 *  ImuCore.ino - AHRS date read sample for multi core.
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
#include "SpresenseIMU.h"

#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

//#define SUBCORE_PRINT

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU setting
#define SAMPLINGRATE   (60)    // Hz
#define ADRANGE         (4)    // G
#define GDRANGE       (500)    // dps
#define FIFO_DEPTH      (1)    // FIFO

#define SENDDECIMATION (SAMPLINGRATE/30)    // Hz

#ifdef SUBCORE
USER_HEAP_SIZE(64 * 1024); 
#endif

enum error_no {
  BEGIN_ERROR = 0,
  INIT_ERROR,
  STRAT_ERROR,
  SEND_ERROR
};

/* For Caribration */
float gyroBias[3] = {0, 0, 0};
float trueRate = 0;

/****************************************************************************
 * calibrate for GyroBias
 ****************************************************************************/
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
  MP.begin(); 

  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0)
    {
      printf("Spresense IMU begin.\n");
      errorLoop(BEGIN_ERROR);
    }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret)
    {
      SpresenseIMU.end();
      errorLoop(INIT_ERROR);
    }

  ret = SpresenseIMU.start();
  if (!ret)
    {
      SpresenseIMU.finalize();
      SpresenseIMU.end();
      errorLoop(STRAT_ERROR);
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
  cxd5602pwbimu_data_t raw;
  static pwbQuaternionData data;
  static int count = 0;

  if (SpresenseIMU.get(raw)) {

    float gx = raw.gx * 180/PI - gyroBias[0];
    float gy = raw.gy * 180/PI - gyroBias[1];
    float gz = raw.gz * 180/PI - gyroBias[2];

    MadgwickFilter.updateIMU(gx, gy, gz, raw.ax, raw.ay, raw.az);

    if(count>0) {
      count--;
      return;
    }

    data.timestamp = raw.timestamp / 19200000.0f;
    data.temp = raw.temp;

    data.q0 = MadgwickFilter.getQ0();
    data.q1 = MadgwickFilter.getQ1();
    data.q2 = MadgwickFilter.getQ2();
    data.q3 = MadgwickFilter.getQ3();
    
    int8_t msgid = 10;
    int ret = MP.Send(msgid, (void*)MP.Virt2Phys(&data));
    if (ret < 0) {
      errorLoop(SEND_ERROR);
    }

#ifdef SUBCORE_PRINT
    printf("%4.2F,%2.2F,%F,%F,%F,%F\n", data.timestamp, data.temp, data.q0, data.q1, data.q2, data.q3);
#endif

    count = SENDDECIMATION;

  }
}

/****************************************************************************
 * Error
 ****************************************************************************/
void errorLoop(int num)
{
  int i;

  printf("Subcore error %d\n",num);

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
