/*
 *  MainCore.ino - Raw data stored sample.
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
#include <SDHCI.h>
#include <MP.h>

SDClass SD;  /**< SDClass object */ 

const int imu_core = 1;

enum error_no {
  SETUP_ERROR = 0,
  SAVE_ERROR
};

/****************************************************************************
 * Setup
 ****************************************************************************/
void setup()
{
  int ret = 0;

  Serial.begin(115200);
  while (!Serial);

  /* Initialize SD */
  while (!SD.begin()) {
    Serial.println("Insert SD card.");
  }

  ret = MP.begin(imu_core);
  if (ret < 0) {
    MPLog("MP.begin(%d) error = %d\n", imu_core, ret);
    errorLoop(SETUP_ERROR);
  }

  MP.RecvTimeout(MP_RECV_BLOCKING);

}

/****************************************************************************
 * Save
 ****************************************************************************/
bool fsave(File fp)
{
  int8_t msgid = 0;
  char* data;
  const int data_number = 1920/4;
  const int max_fsize = 1*1000*1000;
  int fsize = 0;

  do{
    int ret = MP.Recv(&msgid, &data, imu_core);
    if (ret < 0) {puts("koko?");return false;}
    ret = fp.write(data, data_number*sizeof(cxd5602pwbimu_data_t));
    if (ret != data_number*sizeof(cxd5602pwbimu_data_t)) {puts("write error!");return false;}
    fsize += data_number*sizeof(cxd5602pwbimu_data_t);
  } while (fsize < max_fsize);

  return true;
}

/****************************************************************************
 * Loop
 ****************************************************************************/
void loop()
{

  char fname[16];
  static char fnumber = 0;

  sprintf(fname, "imu%03d.dat", fnumber);
  printf("%s\n", fname);
  if (SD.exists(fname)) SD.remove(fname);

  File fp = SD.open(fname, FILE_WRITE);
  if (!fp) {
    Serial.println(String(fname) + ": Open error!" );
    return;
  }
  
  if (!fsave(fp)){
    fp.close();
    errorLoop(SAVE_ERROR);
  } 

  fp.close();
  fnumber++;
  
}

/****************************************************************************
 * Error
 ****************************************************************************/
void errorLoop(int num)
{
  printf("Subcore error %d\n",num);

  while (1) {
    delay(1000);
  }
}
