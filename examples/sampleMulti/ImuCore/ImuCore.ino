/*
 *  ImuCore.ino - Raw date read sample for multi core.
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

//#define SUBCORE_PRINT

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU設定
#define SAMPLINGRATE (60)    // Hz
#define ADRANGE       (4)    // G
#define GDRANGE (    500)    // dps
#define FIFO_DEPTH    (1)    // FIFO

#ifdef SUBCORE
USER_HEAP_SIZE(64 * 1024); 
#endif

enum error_no {
  BEGIN_ERROR = 0,
  INIT_ERROR,
  STRAT_ERROR,
  SEND_ERROR
};

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
}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  cxd5602pwbimu_data_t data;
  if (SpresenseIMU.get(data)) {

#ifndef SUBCORE_PRINT
    int8_t msgid = 10;
    int ret = MP.Send(msgid, (void*)MP.Virt2Phys(&data));
    if (ret < 0) {
      errorLoop(SEND_ERROR);
    }
    usleep(10*1000);
#else
    float timestamp = data.timestamp / 19200000.0f;
    printf("%4.2F,%4.2F,%F,%F,%F,%F,%F,%F\n", timestamp, data.temp, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
#endif

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
