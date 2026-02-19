/*
  * evalSample.ino - Board evaluation sample
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

// ====== Settings ======
#define SAMPLINGRATE   (1920)   // Hz
#define ADRANGE        (4)      // [G]
#define GDRANGE        (500)    // [dps]
#define FIFO_DEPTH     (1)      // FIFO depth

#define RECORD_SEC     (3)
#define MAX_SAMPLES    (SAMPLINGRATE * RECORD_SEC)

// ====== Buffer ======
static pwbImuData g_buf[MAX_SAMPLES];
static int g_count = 0;
static bool g_done = false;

void setup(void)
{
  int ret;

  ret = SpresenseIMU.begin();
  if (ret < 0) {
    printf("[FATAL] SpresenseIMU.begin() failed\n");
    return;
  }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) {
    printf("[FATAL] SpresenseIMU.initialize() failed (rate=%d)\n", SAMPLINGRATE);
    SpresenseIMU.end();
    return;
  }

  ret = SpresenseIMU.start();
  if (!ret) {
    printf("[FATAL] SpresenseIMU.start() failed\n");
    SpresenseIMU.finalize();
    SpresenseIMU.end();
    return;
  }

  printf("=== eval_sample ===\n");
  printf("Capture: %d Hz x %d sec => %d samples\n", SAMPLINGRATE, RECORD_SEC, MAX_SAMPLES);
  printf("Start capture...\n");
}

static void dump_buffer(void)
{
  for (int i = 0; i < g_count; i++) {
    g_buf[i].print();
  }
}

void loop(void)
{
  if (g_done) {
    delay(1000);
    return;
  }

  pwbImuData imuData;

  if (SpresenseIMU.get(imuData)) {
    if (g_count < MAX_SAMPLES) {
      g_buf[g_count] = imuData;
      g_count++;
    }

    if (g_count >= MAX_SAMPLES) {

      SpresenseIMU.stop();

      printf("Capture done: %d samples\n", g_count);
      printf("Dump start...\n");
      dump_buffer();
      printf("Dump end.\n");

      SpresenseIMU.finalize();
      SpresenseIMU.end();

      g_done = true;
    }
  }
}

