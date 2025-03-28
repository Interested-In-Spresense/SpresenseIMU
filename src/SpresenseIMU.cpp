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

const int device_timeout = 1000;

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
  fifo_depth = nfifos;
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
}

/****************************************************************************
 * start
 ****************************************************************************/
bool SpresenseImuClass::start()
{

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
  struct pollfd fds;
  fds.fd     = fd;
  fds.events = POLLIN;

  int ret = poll(&fds, 1, device_timeout);
  if (ret <= 0)
    {
      puts("Device timeout\n");
      return false;
    }

  ret = read(fd,&data, sizeof(data)*fifo_depth);
  if (ret != (int)sizeof(data)*fifo_depth) { return false; }
  return true;
}

/****************************************************************************
 * get samples
 ****************************************************************************/
bool SpresenseImuClass::get(cxd5602pwbimu_data_t* ptr, int size)
{
  for(;size>0;size=size-fifo_depth,ptr++){
    if(!get(*ptr)) return false;
  }

  return true;

}


/****************************************************************************
 * Instance
 ****************************************************************************/

class SpresenseImuClass SpresenseIMU;
