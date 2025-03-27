/*
 *  SpresenseIMU.cpp - Spresense Multi IMU board wrapper library.
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

#include <arch/board/board.h> 

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <time.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH      "/dev/imu0"

#define itemsof(a) (sizeof(a)/sizeof(a[0]))


/****************************************************************************
 * begin
 ****************************************************************************/
int SpresenseImuClass::begin()
{
  int ret;
  ret = board_cxd5602pwbimu_initialize(5); 
  if (ret < 0)
    {
      printf("ERROR: Failed to initialize CXD5602PWBIMU.\n");
      return ret;
    }

  fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: Device %s open failure. %d\n", CXD5602PWBIMU_DEVPATH, errno);
      return errno;
    }

  return true;

}

/****************************************************************************
 * end
 ****************************************************************************/
void SpresenseImuClass::end()
{
  close(fd);
}

/****************************************************************************
 * initialize
 ****************************************************************************/
bool SpresenseImuClass::initialize(int rate, int adrange, int gdrange, int nfifos)
{
  int ret;
  cxd5602pwbimu_range_t range;

  outbuf = (cxd5602pwbimu_data_t *)malloc(sizeof(cxd5602pwbimu_data_t) * rate);
  if (outbuf == NULL)
    {
      printf("ERROR: Output buffer allocation failed.\n");
      return false;
    }

  /*
   * Set sampling rate. Available values (Hz) are below.
   *
   * 15 (default), 30, 60, 120, 240, 480, 960, 1920
   */

  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return false;
    }

  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return false;
    }

  /*
   * Set hardware FIFO threshold.
   * Increasing this value will reduce the frequency with which data is
   * received.
   */

  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return false;
    }

  return true;

}

/****************************************************************************
 * finalize
 ****************************************************************************/
void SpresenseImuClass::finalize()
{
  free(outbuf);
}

/****************************************************************************
 * start
 ****************************************************************************/
bool SpresenseImuClass::start()
{

  memset(outbuf, 0, sizeof(1));

  /*
   * Start sensing, user can not change the all of configurations.
   */

  int ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return false;
    }

  return true;

}

/****************************************************************************
 * stop
 ****************************************************************************/
bool SpresenseImuClass::stop()
{
  int ret = ioctl(fd, SNIOC_ENABLE, 0);
  if (ret)
    {
      printf("ERROR: Disable failed. %d\n", errno);
      return true;
    }

  return false;

}

/****************************************************************************
 * get
 ****************************************************************************/
bool SpresenseImuClass::get(cxd5602pwbimu_data_t& data)
{
  int ret = read(fd, &data, sizeof(data));
  if (ret != sizeof(data)) return false;
  return true;
}

/****************************************************************************
 * get samples
 ****************************************************************************/
bool SpresenseImuClass::get(cxd5602pwbimu_data_t* ptr, int size)
{
  for(;size>0;size--,ptr++){
    if(!get(*ptr)) return false;
  }

  return true;

}


/****************************************************************************
 * Instance
 ****************************************************************************/

class SpresenseImuClass SpresenseIMU;
