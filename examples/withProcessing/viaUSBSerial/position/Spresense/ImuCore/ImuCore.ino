/*
 *  ImuCore.ino - Sample for estimating orientation from Multi-IMU data.
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
#include "InternalData.h"

//#define SUBCORE_PRINT

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

const int pos_core = 2;

#define SAMPLINGRATE  (960)    // Hz
#define ADRANGE         (4)    // G
#define GDRANGE       (500)    // dps
#define FIFO_DEPTH      (1)    // FIFO

#ifdef SUBCORE
USER_HEAP_SIZE(64 * 1024); 
#endif

enum error_no {
  BEGIN_ERROR = 0,
  INIT_ERROR,
  STRAT_ERROR,
  SEND_ERROR
};

/* For Caribration */
float gyroBias[3] = {0, 0, 0};
float trueGravity = 0.0;

static pwbQuaternionData data;

/****************************************************************************
 * calibrate for GyroBias
 ****************************************************************************/
/****************************************************************************
 * Combined Calibration + Initial Orientation
 ****************************************************************************/
void calibrateAndInitOrientation(int ms = 2000)
{
  printf("Keep still for calibration + initial alignment (%d ms)...\n", ms);

  cxd5602pwbimu_data_t raw;
  double sum_gx = 0, sum_gy = 0, sum_gz = 0;
  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  int count = 0;

  unsigned long start = millis();
  while (millis() - start < (unsigned)ms)
  {
    if (SpresenseIMU.get(raw))
    {
      // Gyro raw accumulation
      sum_gx += raw.gx;
      sum_gy += raw.gy;
      sum_gz += raw.gz;

      // Accel raw accumulation
      sum_ax += raw.ax;
      sum_ay += raw.ay;
      sum_az += raw.az;

      count++;
    }
  }

  if (count > 0)
  {
    // Gyro bias
    gyroBias[0] = sum_gx / count;
    gyroBias[1] = sum_gy / count;
    gyroBias[2] = sum_gz / count;

    // Gravity magnitude
    float ax = sum_ax / count;
    float ay = sum_ay / count;
    float az = sum_az / count;

    trueGravity = sqrt(ax * ax + ay * ay + az * az);

    // Normalize accel to unit gravity vector
    ax /= trueGravity;
    ay /= trueGravity;
    az /= trueGravity;

    // ---- Gravity → Roll/Pitch ----
    float roll  = atan2(ay, az);
    float pitch = -atan2(ax, sqrt(ay * ay + az * az));
    float yaw   = 0.0f;  // Cannot measure yaw from gravity alone

    // ---- Set initial quaternion ----
    pwbQuaternionData initQ;
    initQ.fromEuler(roll, pitch, yaw);
    initQ.normalize();
    data = initQ;

    printf("[Calibration Done]\n");
    printf("Gyro Bias: %f %f %f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
    printf("Init Roll=%.3f Pitch=%.3f\n", roll, pitch);
  }

  // Send gravity magnitude to pos-core
  int8_t msgid = 20;
  int ret = MP.Send(msgid, &trueGravity, pos_core);
  if (ret < 0) errorLoop(SEND_ERROR);
}

/****************************************************************************
 * Setup
 ****************************************************************************/
void setup(void)
{
  MP.begin(); 

  int ret;
  ret = SpresenseIMU.begin();
  if (ret < 0) errorLoop(BEGIN_ERROR);

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) errorLoop(INIT_ERROR);

  ret = SpresenseIMU.start();
  if (!ret) errorLoop(STRAT_ERROR);

  sleep(1);

  ledOn(LED0); ledOn(LED1); ledOn(LED2); ledOn(LED3);

  calibrateAndInitOrientation(2000);

  ledOff(LED0); ledOff(LED1); ledOff(LED2); ledOff(LED3);
}

/****************************************************************************
 * Loop
 ****************************************************************************/
#define BUFFER_SIZE 4

void loop()
{
  cxd5602pwbimu_data_t raw;
  static OrientationData_t OrientationDataBlock[BUFFER_SIZE][BLOCK_SIZE];
  static int block_idx = 0;
  static int buffer_idx = 0;
  static float last_timestamp = 0;

  // ---- Static detection ----
  static int static_counter = 0;
  const float GYRO_THRESH = 0.05f;
  const float ACC_THRESH  = 0.05f;
  const int STATIC_CONFIRM_FRAMES = 16; 

  if (SpresenseIMU.get(raw)) {

    if (last_timestamp == 0) {
      last_timestamp = raw.timestamp / 19200000.0f;
      return;
    }

    // ---- Gyro bias compensation ----
    raw.gx = raw.gx - gyroBias[0];
    raw.gy = raw.gy - gyroBias[1];
    raw.gz = raw.gz - gyroBias[2];

    float t = raw.timestamp / 19200000.0f;
    OrientationDataBlock[buffer_idx][block_idx].timestamp = t;
    OrientationDataBlock[buffer_idx][block_idx].ax = raw.ax;
    OrientationDataBlock[buffer_idx][block_idx].ay = raw.ay;
    OrientationDataBlock[buffer_idx][block_idx].az = raw.az;

    pwbQuaternionData result;
    SpresenseIMU.convQuaternion(result,raw,last_timestamp);
    data = data * result;   // ← ここでちゃんと identity から積分が始まる
    last_timestamp = t;
    
    // ---- Static 判定 ----
    float gyro_norm = sqrtf(raw.gx*raw.gx + raw.gy*raw.gy + raw.gz*raw.gz);
    float acc_norm = sqrtf(raw.ax*raw.ax + raw.ay*raw.ay + raw.az*raw.az);
    bool isStaticFrame = (gyro_norm < GYRO_THRESH) && (fabsf(acc_norm - trueGravity) < ACC_THRESH);

    if (isStaticFrame) static_counter++; else static_counter = 0;
    OrientationDataBlock[buffer_idx][block_idx].isStatic = (static_counter >= STATIC_CONFIRM_FRAMES);

    OrientationDataBlock[buffer_idx][block_idx].q0 = data.q0;
    OrientationDataBlock[buffer_idx][block_idx].q1 = data.q1;
    OrientationDataBlock[buffer_idx][block_idx].q2 = data.q2;
    OrientationDataBlock[buffer_idx][block_idx].q3 = data.q3;

#ifdef SUBCORE_PRINT
    printf("%4.2f,%f,%f,%f,%f,%d\n",
      t,
      data.q0, data.q1, data.q2, data.q3,
      OrientationDataBlock[buffer_idx][block_idx].isStatic ? 1 : 0
    );
#endif

    if (++block_idx >= BLOCK_SIZE) {
      int8_t msgid = 10;
      int ret = MP.Send(msgid, MP.Virt2Phys(OrientationDataBlock[buffer_idx]), pos_core);
      if (ret < 0) errorLoop(SEND_ERROR);
      buffer_idx = (buffer_idx + 1) % BUFFER_SIZE;
      block_idx = 0;
    }
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

