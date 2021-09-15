#ifndef VECTOR3D_H
#define VECTOR3D_H

typedef struct {
  float x;
  float y;
  float z;
} vector3d;

vector3d vectorMake(float x, float y, float z) {
  vector3d v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

vector3d zeroVector() {
  return vectorMake(.0, .0, .0);
}

vector3d vectorSum(vector3d a, vector3d b) {
  vector3d sum;
  sum.x = a.x + b.x;
  sum.y = a.y + b.y;
  sum.z = a.z + b.z;
  return sum;
}

vector3d vectorDiff(vector3d a, vector3d b) {
  vector3d sum;
  sum.x = a.x - b.x;
  sum.y = a.y - b.y;
  sum.z = a.z - b.z;
  return sum;
}

vector3d vectorMul(vector3d a, float b) {
  vector3d sum;
  sum.x = a.x*b;
  sum.y = a.y*b;
  sum.z = a.z*b;
  return sum;
}

float vectorScalarMul(vector3d a, vector3d b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

#endif