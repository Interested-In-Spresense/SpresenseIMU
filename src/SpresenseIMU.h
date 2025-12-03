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

  pwbImuData() { memset(&data, 0, sizeof(data)); }

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

  void printIum(){
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

  pwbGyroData() { memset(this, 0, sizeof(*this)); }

  pwbGyroData& operator=(const pwbImuData& other){
    x = other.data.gx;
    y = other.data.gy;
    z = other.data.gz;
    return *this;
  }

  pwbGyroData& operator+=(const pwbGyroData& other){
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  void print(){
    printf("%F,%F,%F\n", x, y, z);
  }

};

struct pwbEulerData {
  float roll;
  float pitch;
  float yaw;

  pwbEulerData() { memset(this, 0, sizeof(*this)); }

  pwbEulerData(float r, float p, float y)
    : roll(r), pitch(p), yaw(y) {}

  pwbEulerData& operator+=(const pwbEulerData& other) {
    roll  += other.roll;
    pitch += other.pitch;
    yaw   += other.yaw;
    return *this;
  }

  void print() const {
    printf("%f,%f,%f\n", roll, pitch, yaw);
  }
};

struct pwbQuaternionData {
  float timestamp;
  float temp;

  float q0;
  float q1;
  float q2;
  float q3;

  // Identity quaternion by default
  pwbQuaternionData() {
    memset(this, 0, sizeof(*this));
    q0 = 1.0f;  // Identity quaternion exception
  }

  // Init with values
  pwbQuaternionData(float _q0, float _q1, float _q2, float _q3)
    : timestamp(0), temp(0), q0(_q0), q1(_q1), q2(_q2), q3(_q3) {}

  // Normalize quaternion
  void normalize() {
    float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (n == 0) { q0 = 1; q1 = q2 = q3 = 0; return; }
    q0 /= n; q1 /= n; q2 /= n; q3 /= n;
  }

  // Quaternion multiplication (Hamilton product)
  pwbQuaternionData operator*(const pwbQuaternionData& q) const {

    pwbQuaternionData result(
      q0*q.q0 - q1*q.q1 - q2*q.q2 - q3*q.q3,
      q0*q.q1 + q1*q.q0 + q2*q.q3 - q3*q.q2,
      q0*q.q2 - q1*q.q3 + q2*q.q0 + q3*q.q1,
      q0*q.q3 + q1*q.q2 - q2*q.q1 + q3*q.q0
    );

    result.timestamp = q.timestamp;
    result.temp      = q.temp;

    return result;
  }

  // Convert to Euler angles (deg)
  pwbEulerData toEuler() const {
    pwbEulerData e;

    float sinr = 2.0f * (q0 * q1 + q2 * q3);
    float cosr = 1.0f - 2.0f * (q1*q1 + q2*q2);
    e.roll = atan2f(sinr, cosr);

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    e.pitch = fabsf(sinp) >= 1 ? copysignf(M_PI/2, sinp) : asinf(sinp);

    float siny = 2.0f * (q0 * q3 + q1 * q2);
    float cosy = 1.0f - 2.0f * (q2*q2 + q3*q3);
    e.yaw = atan2f(siny, cosy);

    e.roll  *= 180.0f/M_PI;
    e.pitch *= 180.0f/M_PI;
    e.yaw   *= 180.0f/M_PI;
    return e;
  }

  void fromEuler(float roll, float pitch, float yaw) {

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    normalize(); // ensure unit quaternion
  }

  void print() const {
    printf("%f,%f,%f,%f\n", q0, q1, q2, q3);
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
  bool end();

  bool initialize(int, int, int, int);
  void finalize();

  bool start();
  bool stop();

  bool get(cxd5602pwbimu_data_t&);
  bool get(cxd5602pwbimu_data_t*, int);

  bool get(pwbImuData&);
  bool get(pwbImuData*, int);
  bool getAverage(pwbImuData&, int);

  void convQuaternion(pwbQuaternionData& data, const cxd5602pwbimu_data_t& raw, float prevTimestamp);

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
