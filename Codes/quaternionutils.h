//#include "Arduino.h"
#include "string.h"
#include <math.h>

void quatmultiplication(float *P_, float *A, float *B);
void quatinverse(float *Q, float *Q_);
//void quatoffset(float *P_, float *A, float *B);
void quatrotation(float *Q_P, float *dQ, float *Q_v, float *dQ_);

float Q_c[4];//Latest Unit Quaternion from Sensor
float Q_d[4];//Offset Unit Quaternion from sensor
float Q_d_[4];//Inverse of Offset Unit Quaternion from sensor
float Q_v[4];//Quaternion Assigned to Position Vector (first element always 0)
float dQ[4];//Delta Q to calculate Unit Quaternion From Offset
float dQ_[4];//Inverse of dQ
float Q_P[4];//New X, Y, Z calculated from the rotated Position Vector

void quatmultiplication(float P_[], float A[], float B[]) {
  P_[0] = A[0] * B[0] - A[1] * B[1] - A[2] * B[2] - A[3] * B[3];
  P_[1] = A[0] * B[1] + A[1] * B[0] + A[2] * B[3] - A[3] * B[2];
  P_[2] = A[0] * B[2] - A[1] * B[3] + A[2] * B[0] + A[3] * B[1];
  P_[3] = A[0] * B[3] + A[1] * B[2] - A[2] * B[1] + A[3] * B[0];
}

void quatinverse(float *Q, float *Q_){
  memcpy(&Q_, &Q, sizeof(Q));
  for (uint8_t i = 1; i < 4; i++) {
    Q_[i] = Q_[i] * (-1);
  }
}

void quatrotation(float *Q_P, float *dQ, float *Q_v, float *dQ_){
  

}


// float rotation(float *A, float *B) {
//   float Q[4];  // Unit Quaternion used for rotation
//   float P[4];  // 4-D vector position for
//   float P_[4]; // Rotated position
//   float Q_[4]; // Inverse of Unit Quaternion
//   Q[0] = A[0];
//   Q[1] = A[1];
//   Q[2] = A[2];
//   Q[3] = A[3];
//   P[0] = 0;
//   P[1] = B[0];
//   P[2] = B[1];
//   P[3] = B[2];
//   memcpy(&Q_, &Q, sizeof(Q));
//   for (uint8_t i = 1; i < 4; i++) {
//     Q_[i] = Q_[i] * (-1);
//   }
//   quatmultiplication(Q, P);
//   quatmultiplication(P_, Q_);
//   return P_;
// }


// float rotationoffset(float *RotationQuaternion, float *CoordFrOffset) {
//   float Q_c[4];     // Raw Unit Quaternion from the Sensor (End Coordinate)
//   float Q_d[4];     // Offset Unit Quaternion (Initial Coordinate)
//   float Delta_P[4]; // Relative Coordinate Frame
//   Q_c[0] = RotationQuaternion[0];
//   Q_c[1] = RotationQuaternion[1];
//   Q_c[2] = RotationQuaternion[2];
//   Q_c[3] = RotationQuaternion[3];
//   Q_d[0] = CoordFrOffset[0];
//   Q_d[1] = CoordFrOffset[1];
//   Q_d[2] = CoordFrOffset[2];
//   Q_d[3] = CoordFrOffset[3];
//   Delta_P = quatmultiplication(Q, P);
//   return Delta_P;
// }

/*w1 = A(1);
x1 = A(2);
y1 = A(3);
z1 = A(4);
w2 = 0;
x2 = u(1);
y2 = u(2);
z2 = u(3);
P(1) = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
P(2) = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
P(3) = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
P(4) = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
w1 = P(1);
x1 = P(2);
y1 = P(3);
z1 = P(4);
w2 = B(1);
x2 = B(2);
y2 = B(3);
z2 = B(4);
P(1) = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
P(2) = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
P(3) = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
P(4) = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;*/
