/*
 *  InternalData.h - Inter-Core Communication Data Structure.
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

#ifndef INTERNAL_DATA_H
#define INTERNAL_DATA_H

#define BLOCK_SIZE 80  // Number of IMU frames per inter-core transfer block

//-----------------------------------------------------------------------------
// Orientation + acceleration data structure for inter-core sensor transfer
// Used for attitude estimation and inertial navigation (INS)
//-----------------------------------------------------------------------------
struct OrientationData_t {

  float timestamp;        // IMU timestamp [seconds]
  float q0, q1, q2, q3;   // Attitude quaternion 
  float ax, ay, az;       // Acceleration
  bool  isStatic;         // Static state flag (for ZUPT or motion detection)

  // Constructor (default initializer)
  OrientationData_t()
    : timestamp(0.0f),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), // Identity quaternion
      ax(0.0f), ay(0.0f), az(0.0f),           // Clear acceleration
      isStatic(false)                         // Default: not static
  {}
};

//-----------------------------------------------------------------------------
// Position data structure.
//-----------------------------------------------------------------------------
struct PosePacket_t {
  float timestamp;
  float x;
  float y;
  float z;
} __attribute__((packed));

//-----------------------------------------------------------------------------
// Shared IMU data block (allocated in producer core)
//-----------------------------------------------------------------------------
extern OrientationData_t OrientationDataBlock[BLOCK_SIZE];

#endif // INTERNAL_DATA_H

