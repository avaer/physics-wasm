#include "vector.h"

Matrix::Matrix(const Vec &position, const Quat &quaternion, const Vec &scale) {
  float *te = this->elements;

  const float x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
  const float x2 = x + x, y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;

  const float sx = scale.x, sy = scale.y, sz = scale.z;

  te[ 0 ] = ( 1.0f - ( yy + zz ) ) * sx;
  te[ 1 ] = ( xy + wz ) * sx;
  te[ 2 ] = ( xz - wy ) * sx;
  te[ 3 ] = 0.0f;

  te[ 4 ] = ( xy - wz ) * sy;
  te[ 5 ] = ( 1 - ( xx + zz ) ) * sy;
  te[ 6 ] = ( yz + wx ) * sy;
  te[ 7 ] = 0.0f;

  te[ 8 ] = ( xz + wy ) * sz;
  te[ 9 ] = ( yz - wx ) * sz;
  te[ 10 ] = ( 1 - ( xx + yy ) ) * sz;
  te[ 11 ] = 0.0f;

  te[ 12 ] = position.x;
  te[ 13 ] = position.y;
  te[ 14 ] = position.z;
  te[ 15 ] = 1.0f;
}