/*
 *  MainCore.ino - Subcore boot and USB Serial output sample.
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

#include <MP.h>
#include "SpresenseIMU.h"
#include <USBSerial.h>

USBSerial UsbSerial;
const int usbserial_baurate = 921600;

const int imu_core = 1;

void setup()
{
  int ret = 0;

  Serial.begin(115200);
  while (!Serial);

  UsbSerial.begin(usbserial_baurate);
  Serial.println("Done!");

  ret = MP.begin(imu_core);
  if (ret < 0) {
    MPLog("MP.begin(%d) error = %d\n", imu_core, ret);
  }
}

void loop()
{
  int8_t   msgid = 0;
  pwbQuaternionData* data;

  int ret = MP.Recv(&msgid, &data, imu_core);
  if (ret >= 0) {
    printf("%4.2F,%2.2F,%F,%F,%F,%F\n", data->timestamp, data->temp, data->q0, data->q1, data->q2, data->q3);
    UsbSerial.print(data->timestamp, 2);
    UsbSerial.print(",");
    UsbSerial.print(data->temp, 2);
    UsbSerial.print(",");
    UsbSerial.print(data->q0);
    UsbSerial.print(",");
    UsbSerial.print(data->q1);
    UsbSerial.print(",");
    UsbSerial.print(data->q2);
    UsbSerial.print(",");
    UsbSerial.println(data->q3);
  }
}

