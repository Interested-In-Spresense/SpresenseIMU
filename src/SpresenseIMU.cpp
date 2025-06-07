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
 * get one sample
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
 * get one sample
 ****************************************************************************/
bool SpresenseImuClass::get(pwbImuData& data)
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

  ret = read(fd,&data.data, sizeof(data)*fifo_depth);
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
 * get samples
 ****************************************************************************/
bool SpresenseImuClass::get(pwbImuData* ptr, int size)
{
  for(;size>0;size=size-fifo_depth,ptr++){
    if(!get(*ptr)) return false;
  }

  return true;

}

/****************************************************************************
 * get verage
 ****************************************************************************/
#define MAX_AVERAGE_COUNT 1000
bool SpresenseImuClass::getAverage(pwbImuData& data, int count)
{
  if(count > MAX_AVERAGE_COUNT) return false;

  cxd5602pwbimu_data_t tmp;

  for(int i=0;i<count;i++){
    if(!get(tmp)) return false;
    data += tmp;
  }

  data /= count;

  return true;

}

/****************************************************************************
 * Calculate Earth`s rotation
 ****************************************************************************/
#define EARTH_RATE   (2.0 * M_PI / 86164.098903691)
#define RAD2DEG      (180.0 / M_PI)

#define SQUARED(v)  ((v) * (v))
#define CUBE(v)     ((v) * (v) * (v))
#define DET_MATRIX(m) m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - \
                      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + \
                      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

int SpresenseImuClass::calcEarthsRotation(pwbGyroData* gavgs, int num, pwbGyroData *bias_out)
{
  int i, r, c;
  double f_mat[3][3] = {0};
  double g_vec[3] = {0};
  double t_vec[3];
  double tmp_mat[3][3];
  double workA, workB;
  double x, y;

  for (i = 0; i < num; i++)
    {
      x = (double)gavgs[i].x;
      y = (double)gavgs[i].y;

      f_mat[0][2] += x;
      f_mat[1][2] += y;
      f_mat[0][0] += SQUARED(x);
      f_mat[1][1] += SQUARED(y);
      f_mat[0][1] += x * y;

      g_vec[0] += CUBE(x) + x * SQUARED(y);
      g_vec[1] += CUBE(y) + y * SQUARED(x);
      g_vec[2] += SQUARED(x) + SQUARED(y);
    }

  f_mat[1][0] = f_mat[0][1];
  f_mat[2][0] = f_mat[0][2];
  f_mat[2][1] = f_mat[1][2];
  f_mat[2][2] = (double)num;

  g_vec[0] = -g_vec[0];
  g_vec[1] = -g_vec[1];
  g_vec[2] = -g_vec[2];

  workB = DET_MATRIX(f_mat);

  for (i = 0; i < 3; i++)
    {
      for (r = 0; r < 3; r++)
        {
          for (c = 0; c < 3; c++)
            {
              tmp_mat[r][c] = (c == i) ? g_vec[r] : f_mat[r][c];
            }
        }

      workA = DET_MATRIX(tmp_mat);
      t_vec[i] = workA / workB;
    }

  workA = -t_vec[0] / 2;
  workB = -t_vec[1] / 2;

  bias_out->x = workA;
  bias_out->y = workB;

  double residual = SQUARED(workA) + SQUARED(workB) - t_vec[2];
  double zsum = 0.0;

  for (i = 0; i < num; i++)
    {
      zsum += gavgs[i].z;
    }

  bias_out->z = zsum / num;

  return (residual > 0.0) ? 0 : -1;
}

/****************************************************************************
 * Calculate the angle to Notth from X axis
 ****************************************************************************/
double SpresenseImuClass::calcAngleFrX(pwbGyroData& data, pwbGyroData& bias)
{
  float x = data.x - bias.x;
  float y = data.y - bias.y;
                                      
  double deg = atan2(-x, y) * RAD2DEG;

  if (deg < 0)
    {
      deg += 360.0;
    }

  return deg;
}


/****************************************************************************
 * Instance
 ****************************************************************************/

class SpresenseImuClass SpresenseIMU;
