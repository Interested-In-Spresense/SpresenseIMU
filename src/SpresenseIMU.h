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
#include <math.h>


/**************************************************************************
 * Structures
 **************************************************************************/

struct pwbImuData {
  cxd5602pwbimu_data_t data;

  pwbImuData()
    : data{0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {}

  pwbImuData& operator+=(const pwbImuData& other) {
    data.timestamp = other.data.timestamp;
    data.temp += other.data.temp;
    data.gx   += other.data.gx;
    data.gy   += other.data.gy;
    data.gz   += other.data.gz;
    data.ax   += other.data.ax;
    data.ay   += other.data.ay;
    data.az   += other.data.az;
    return *this;
  }

  pwbImuData& operator+=(const cxd5602pwbimu_data_t& other) {
    data.timestamp = other.timestamp;
    data.temp += other.temp;
    data.gx   += other.gx;
    data.gy   += other.gy;
    data.gz   += other.gz;
    data.ax   += other.ax;
    data.ay   += other.ay;
    data.az   += other.az;
    return *this;
  }

  pwbImuData& operator/=(int i) {
    data.temp /= i;
    data.gx   /= i;
    data.gy   /= i;
    data.gz   /= i;
    data.ax   /= i;
    data.ay   /= i;
    data.az   /= i;
    return *this;  
  }
  
  void print(){
    printf("%4.2F,%4.2F,%F,%F,%F,%F,%F,%F\n", (data.timestamp / 19200000.0f), data.temp, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
  }

  void printImu(){
    printf("%F,%F,%F,%F,%F,%F\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
  }

  void printAccelerometer(){
    printf("%F,%F,%F\n", data.ax, data.ay, data.az);
  }

  void printGyro(){
    printf("%F,%F,%F\n", data.gx, data.gy, data.gz);
  }
};

struct pwbGyroData {
  double x;
  double y;
  double z;

  pwbGyroData():x(0),y(0),z(0){}
  
  pwbGyroData& operator=(const pwbImuData& other){
    x = other.data.gx;
    y = other.data.gy;
    z = other.data.gz;
    return *this; 
  }

  pwbGyroData& operator+=(const pwbGyroData& other){
    x = other.x;
    y = other.y;
    z = other.z;
    return *this; 
  }

  void print(){
    printf("%F,%F,%F\n", x, y, z);
  }

};

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

  bool get(pwbImuData&);
  bool get(pwbImuData*, int);
  bool getAverage(pwbImuData&, int);

  int calcEarthsRotation(pwbGyroData* gavgs, int num, pwbGyroData *bias_out);
  double calcAngleFrX(pwbGyroData&, pwbGyroData&);

private:

  int fd;
  int fifo_depth;
  cxd5602pwbimu_data_t* outbuf;

};

/****************************************************************************
 * Instance
 ****************************************************************************/

extern class SpresenseImuClass SpresenseIMU;

#endif // _SPRESENSE_IMU_H
