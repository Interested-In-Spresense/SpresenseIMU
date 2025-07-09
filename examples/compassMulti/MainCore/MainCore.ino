/*
 *  MainCore.ino - Gyro compass sample with multi core.
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

#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include "SpresenseIMU.h"
#include <MadgwickAHRS.h>

#include <MP.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU設定
#define SAMPLINGRATE (60)    // Hz
#define ADRANGE       (4)    // G
#define GDRANGE (    500)    // dps
#define FIFO_DEPTH    (1)    // FIFO

#define MAX_CALIBRATION  (20)
#define MIM_CALIBRATION  (3)
#define CALIBRATION_TIME (10)

pwbGyroData dataArray[MAX_CALIBRATION];
pwbGyroData bias_vec;

Madgwick MadgwickFilter;

const int disp_core = 1;

/****************************************************************************
 * calibration
 ****************************************************************************/
int calibration()
{

  Serial.println("Press 'c' key with the Spresense board stationary.");
  ledOn(LED0);

  for(int i=0;i<MAX_CALIBRATION;){

    while(Serial.available() <= 0);

    switch (Serial.read()) {
      case 'c': {
        Serial.println("Calibration No."+i);
        Serial.print("Count Down to start T :");
        for(int count_down_time = 3;count_down_time>=0;count_down_time--){
          Serial.print(count_down_time);
          Serial.print(". ");
          sleep(1);
        }
        Serial.print("\nStart Calibration for ");
        Serial.print(CALIBRATION_TIME);
        Serial.println(" seconds");
        ledOff(LED0);
        ledOn(LED1);
        pwbImuData data;
        if (SpresenseIMU.getAverage(data,(CALIBRATION_TIME*SAMPLINGRATE))) {
          Serial.println("Calibration done!");
          dataArray[i] = data;
          i++;
          ledOff(LED1);
        } else {
          Serial.println("Device read error!");
        }
        break;
      }

      case 'q': {
        if(i<MIM_CALIBRATION){
          Serial.println("Now, Posture data is less than 3.");
          Serial.println("Please continue calibration.");
          break;
        }
        ledOff(LED0);
        ledOn(LED2);
        Serial.print("Calibration completed ");
        Serial.print(i);
        Serial.println(" times.");
        return i;
      }

      case '\n': {
        Serial.println("Press 'c' key with the Spresense board stationary.");
        if(i>=MIM_CALIBRATION){
          Serial.println("Do you want to quit this app? Press 'q' key then : ");
        }
        ledOn(LED0);
        break;
      }

      default:
        break;
    }
  }  
  
  Serial.print("Calibration completed ");
  Serial.print(MAX_CALIBRATION);
  Serial.println(" times.");

  return MAX_CALIBRATION;

}

/****************************************************************************
 * Setup
 ****************************************************************************/
float baseAngle;
void setup(void)
{
  Serial.begin(115200);
 
  int ret = MP.begin(disp_core);
  if (ret < 0) {
    MPLog("MP.begin(%d) error = %d\n", disp_core, ret);
  }

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

  int num = calibration();
  SpresenseIMU.calcEarthsRotation(dataArray, num, &bias_vec);
  baseAngle  = SpresenseIMU.calcAngleFrX(dataArray[num],bias_vec);

  pwbImuData imuData;
  if (SpresenseIMU.get(imuData)) {
    MadgwickFilter.updateIMU(imuData.data.gx*180/PI, imuData.data.gy*180/PI, imuData.data.gz*180/PI, imuData.data.ax, imuData.data.ay, imuData.data.az);
    baseAngle -= MadgwickFilter.getYaw();
    printf("baseAngle = %F\n",baseAngle);
  }

  ledOff(LED2);
  ledOn(LED3);

  ret = MP.Send(10,(uint32_t)baseAngle, disp_core);
  if (ret < 0) {
    printf("MP.Send error = %d\n", ret);
  }

}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  static int i=0;
  pwbImuData imuData;
  if (SpresenseIMU.get(imuData)) {
    MadgwickFilter.updateIMU(imuData.data.gx*180/PI, imuData.data.gy*180/PI, imuData.data.gz*180/PI, imuData.data.ax, imuData.data.ay, imuData.data.az);
    float angle  = MadgwickFilter.getYaw() - baseAngle;
    if(angle<0) angle += 360.0;
//    printf("angle = %F\n", angle);

    if(i==0){
      int ret = MP.Send(100,(uint32_t)angle, disp_core);
      if (ret < 0) {
        printf("MP.Send error = %d\n", ret);
        return;
      }
    }
    i=(i+1)%(SAMPLINGRATE+30);
  }

}

