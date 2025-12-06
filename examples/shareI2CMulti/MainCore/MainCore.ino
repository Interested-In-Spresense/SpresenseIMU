/*
 *  MainCore.ino - Sample for sharing I2C with multi-core.
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
#include <MP.h>

#include <nuttx/arch.h>
#include <arch/cxd56xx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// IMU setting
#define SAMPLINGRATE (60)    // Hz
#define ADRANGE       (4)    // G
#define GDRANGE (    500)    // dps
#define FIFO_DEPTH    (1)    // FIFO

#define RESET_INTERVAL 20   // restart every N samples

// Multi-Core setting
#define SUBCORE_ID    1
#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

/****************************************************************************
 * Setup
 ****************************************************************************/
void setup(void)
{

  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0) {
    printf("Spresense IMU begin.\n");
    return;
  }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) {
    SpresenseIMU.end();
    return;
  }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  ret = MP.begin(SUBCORE_ID);
  if (ret < 0) {
    MPLog("MP.begin(%d) error = %d\n", SUBCORE_ID, ret);
  }
 MP.RecvTimeout(3000);

}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{
  int8_t msgid;
  uint32_t bmp_raw = 0;
  pwbImuData imuData;

  Serial.println("\n[Main] → BMP request sent");
  int ret = MP.Send(MSG_REQ_BMP, (uint32_t)&bmp_raw, SUBCORE_ID);

  if (MP.Recv(&msgid, &bmp_raw, SUBCORE_ID) >= 0 && msgid == MSG_RET_BMP) {
    up_enable_irq(CXD56_IRQ_SCU_I2C0);

    ret = SpresenseIMU.start();
    if (!ret){
      SpresenseIMU.finalize();
      SpresenseIMU.end();
      return;
    }

    int count = 0;
      while (count < RESET_INTERVAL) {
        if (SpresenseIMU.get(imuData)){
            imuData.print();
              count++;
          } else {
              Serial.println("[ERROR] get() failed, restarting IMU...");
              break;
          }
      }

    ret = SpresenseIMU.stop();
    up_disable_irq(CXD56_IRQ_SCU_I2C0);
    Serial.flush();

  }
}

