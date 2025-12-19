/*
 *  PosCore.ino - Sample for position estimation from orientation data.
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
#include "InternalData.h"

//#define SUBCORE_PRINT

#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

enum error_no {
  BEGIN_ERROR = 0,
  INIT_ERROR,
  STRAT_ERROR,
  SEND_ERROR
};

// ---- 状態 ----
float px=0, py=0, pz=0;
float vx=0, vy=0, vz=0;
float last_ts = -1;

// ---- 重力（Core1 から受信） ----
float trueGravity = 9.80665f;  // fallback、その後 msgid=20 で上書き

// ---- マウント補正 ----
bool mountReady = false;
float mount_q[4] = {1,0,0,0}; 
float g_sum[3] = {0,0,0};
int g_count = 0;

// ---- クォータニオン演算 ----
struct Q { float w,x,y,z; };

Q qMul(Q a, Q b){
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Q qConj(Q q){ return {q.w,-q.x,-q.y,-q.z}; }

void rotateVector(Q q, float &x, float &y, float &z){
  Q v = {0,x,y,z};
  Q r = qMul(q, qMul(v, qConj(q)));
  x=r.x; y=r.y; z=r.z;
}

// ---- 初期姿勢補正 (gravity→world Z+) ----
void computeMountCorrection(){
  float gx = g_sum[0]/g_count;
  float gy = g_sum[1]/g_count;
  float gz = g_sum[2]/g_count;

  float norm = sqrt(gx*gx+gy*gy+gz*gz);
  gx/=norm; gy/=norm; gz/=norm;

  // world gravity vector (0,0,-1)
  float dot = -(gx*0 + gy*0 + gz*1);
  float angle = acos(dot);

  float rx = gy;
  float ry = -gx;
  float rz = 0;

  float rnorm = sqrt(rx*rx + ry*ry + rz*rz);
  if(rnorm < 1e-6){
    mount_q[0]=1; mount_q[1]=mount_q[2]=mount_q[3]=0;
    Serial.println("[Mount correction skipped: nearly aligned]");
    return;
  }

  rx/=rnorm; ry/=rnorm; rz/=rnorm;

  mount_q[0] = cos(angle/2);
  mount_q[1] = rx*sin(angle/2);
  mount_q[2] = ry*sin(angle/2);
  mount_q[3] = rz*sin(angle/2);

  Serial.printf("[Mount correction applied] angle=%.4f rad\n", angle);
  mountReady = true;
}

void setup(){
  MP.begin();
  Serial.begin(115200);
  Serial.println("[RAW INTEGRATION MODE]");
  Serial.println("Waiting for gravity calibration (msgid=20)...");
}

#define BUFFER_SIZE 4
void loop(){

  int8_t msgid; uint32_t addr;
  static PosePacket_t PoseData[BUFFER_SIZE];
  static int buffer_idx = 0;

  if(MP.Recv(&msgid,&addr,1)<=0) return;

  // ---- Gravity calibration reception ----
  if(msgid == 20){
      float* gptr = (float*)addr;
      trueGravity = *gptr;
      Serial.printf("[Gravity Received] G = %.6f\n", trueGravity);
      Serial.println("Now waiting for IMU stream...");
      return;
  }

  if(msgid != 10) return;

  OrientationData_t* block = (OrientationData_t*)addr;

  for(int i=0;i<BLOCK_SIZE;i++){
    auto &d = block[i];

    // ---- dt ----
    float dt = (last_ts<0)?(1.0/120.0):(d.timestamp-last_ts);
    last_ts = d.timestamp;

    // ---- 初期2秒間 → 重力平均 ----
    if(!mountReady && d.timestamp < 2.0f){
      g_sum[0]+=d.ax; g_sum[1]+=d.ay; g_sum[2]+=d.az;
      g_count++;
      if(d.timestamp >= 2.0f) computeMountCorrection();
      continue;
    }

    // ---- 世界座標へ変換 ----
    float ax=d.ax, ay=d.ay, az=d.az;
    Q qs={d.q0,d.q1,d.q2,d.q3};
    rotateVector(qs,ax,ay,az);

    Q qm={mount_q[0],mount_q[1],mount_q[2],mount_q[3]};
    rotateVector(qm, ax,ay,az);

    // ---- 重力除去（補正された値を使用）----
    az -= trueGravity;

    // ---- ZUPT（確定静止）----
    if (d.isStatic) {
       vx = 0.0f;
       vy = 0.0f;
       vz = 0.0f;

      // 静止中は線形加速度を積分しない
      ax = 0.0f;
      ay = 0.0f;
      az = 0.0f;
    }

    // ---- 積分 ----
    vx += ax*dt;
    vy += ay*dt;
    vz += az*dt;

    px += vx*dt;
    py += vy*dt;
    pz += vz*dt;

#ifdef SUBCORE_PRINT
    Serial.printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
      d.timestamp,px,py,pz,vx,vy,vz,d.isStatic);
#endif
    PoseData[buffer_idx].timestamp = d.timestamp;
    PoseData[buffer_idx].x = px;
    PoseData[buffer_idx].y = py;
    PoseData[buffer_idx].z = pz;
  }

  #define BUFFER_SIZE 4

  msgid = 10;
  int ret = MP.Send(msgid, MP.Virt2Phys(&PoseData[buffer_idx]));
  if (ret < 0) errorLoop(SEND_ERROR);
  buffer_idx = (buffer_idx + 1) % BUFFER_SIZE;    

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


