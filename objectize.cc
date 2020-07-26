#include <emscripten.h>
#include "collide.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
  free(ptr);
}

EMSCRIPTEN_KEEPALIVE void initPhysx() {
  doInitPhysx();
}
EMSCRIPTEN_KEEPALIVE void registerGeometry(float *positions, unsigned int *indices, unsigned int numPositions, unsigned int numIndices, float *meshPosition, float *meshQuaternion, uintptr_t *result) {
  *result = doRegisterGeometry(positions, indices, numPositions, numIndices, meshPosition, meshQuaternion);
}
EMSCRIPTEN_KEEPALIVE void unregisterGeometry(uintptr_t geometrySpecPtr) {
  doUnregisterGeometry(geometrySpecPtr);
}
EMSCRIPTEN_KEEPALIVE void raycast(float *origin, float *direction, unsigned int *hit, float *position, float *normal, float *distance) {
  doRaycast(origin, direction, *hit, position, normal, *distance);
}
EMSCRIPTEN_KEEPALIVE void collide(float radius, float halfHeight, float *position, float *quaternion, unsigned int maxIter, unsigned int *hit, float *direction, float *depth) {
  doCollide(radius, halfHeight, position, quaternion, maxIter, *hit, direction, *depth);
}

}
