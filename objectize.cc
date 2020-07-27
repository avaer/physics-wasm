#include <emscripten.h>
#include "collide.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
  return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
  free(ptr);
}

EMSCRIPTEN_KEEPALIVE void initPhysx() {
  doInitPhysx();
}
EMSCRIPTEN_KEEPALIVE void registerBakedGeometry(unsigned int meshId, uintptr_t data, size_t size, float *meshPosition, float *meshQuaternion, uintptr_t *result) {
  *result = doRegisterBakedGeometry(meshId, data, size, meshPosition, meshQuaternion);
}
EMSCRIPTEN_KEEPALIVE void registerBoxGeometry(unsigned int meshId, float *position, float *quaternion, float w, float h, float d, uintptr_t *result) {
  *result = doRegisterBoxGeometry(meshId, position, quaternion, w, h, d);
}
EMSCRIPTEN_KEEPALIVE void registerCapsuleGeometry(unsigned int meshId, float *position, float *quaternion, float radius, float halfHeight, uintptr_t *result) {
  *result = doRegisterCapsuleGeometry(meshId, position, quaternion, radius, halfHeight);
}
EMSCRIPTEN_KEEPALIVE void bakeGeometry(float *positions, unsigned int *indices, unsigned int numPositions, unsigned int numIndices, uintptr_t *ptr, uintptr_t *data, size_t *size) {
  doBakeGeometry(positions, indices, numPositions, numIndices, *ptr, *data, *size);
}
EMSCRIPTEN_KEEPALIVE void releaseBakedGeometry(uintptr_t ptr) {
  doReleaseBakedGeometry(ptr);
}
EMSCRIPTEN_KEEPALIVE void unregisterGeometry(uintptr_t geometrySpecPtr) {
  doUnregisterGeometry(geometrySpecPtr);
}
EMSCRIPTEN_KEEPALIVE void raycast(float *origin, float *direction, float *meshPosition, float *meshQuaternion, unsigned int *hit, float *position, float *normal, float *distance, unsigned int *meshId, unsigned int *faceIndex) {
  doRaycast(origin, direction, meshPosition, meshQuaternion, *hit, position, normal, *distance, *meshId, *faceIndex);
}
EMSCRIPTEN_KEEPALIVE void collide(float radius, float halfHeight, float *position, float *quaternion, float *meshPosition, float *meshQuaternion, unsigned int maxIter, unsigned int *hit, float *direction) {
  doCollide(radius, halfHeight, position, quaternion, meshPosition, meshQuaternion, maxIter, *hit, direction);
}

}
