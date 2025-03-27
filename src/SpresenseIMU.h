/*
 *  SpresenseIMU.h - Spresense Multi IMU board wrapper library header.
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

#ifndef _SPRESENSE_IMU_H_
#define _SPRESENSE_IMU_H

#include <Arduino.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

/**************************************************************************
 * Class
 **************************************************************************/

class SpresenseImuClass {

public:
  SpresenseImuClass(){}
  ~SpresenseImuClass(){}

  int begin();
  void end();

  bool initialize(int, int, int, int);
  void finalize();

  bool start();
  bool stop();

  bool get(cxd5602pwbimu_data_t&);
  bool get(cxd5602pwbimu_data_t*, int);

private:
  int fd;
  cxd5602pwbimu_data_t* outbuf;

};

/****************************************************************************
 * Instance
 ****************************************************************************/

extern class SpresenseImuClass SpresenseIMU;

#endif // _SPRESENSE_IMU_H
