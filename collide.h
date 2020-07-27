// #include "DirectXMath.h"
/* #include "PhysX/physx/include/geometry/PxMeshQuery.h"
#include "geometry/PxCapsuleGeometry.h"
#include "foundation/PxTransform.h"
 */
#include "vector.h"
#include "PxPhysicsVersion.h"
#include "PxPhysics.h"
#include "extensions/PxDefaultStreams.h"
#include "extensions/PxDefaultAllocator.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "cooking/PxCooking.h"
#include "extensions/PxTriangleMeshExt.h"
#include "PxQueryReport.h"
#include "geometry/PxGeometryQuery.h"
#include <set>
#include <algorithm>
#include <iostream>

using namespace physx;

constexpr float subparcelSize = 10;
const float subparcelRadius = std::sqrt((subparcelSize/2.0f)*(subparcelSize/2.0f)*3.0f);

class GeometrySpec {
public:
  GeometrySpec(unsigned int meshId, PxTriangleMesh *triangleMesh, PxGeometry *meshGeom, Vec position, Quat quaternion, Sphere boundingSphere) :
    meshId(meshId), triangleMesh(triangleMesh), meshGeom(meshGeom), position(position), quaternion(quaternion), boundingSphere(boundingSphere) {}
  ~GeometrySpec() {
    delete meshGeom;
    triangleMesh->release();
  }

  unsigned int meshId;
  PxTriangleMesh *triangleMesh;
  PxGeometry *meshGeom;
  Vec position;
  Quat quaternion;
  Sphere boundingSphere;
};

PxDefaultAllocator *gAllocator = nullptr;
PxDefaultErrorCallback *gErrorCallback = nullptr;
PxFoundation *gFoundation = nullptr;
PxPhysics *physics = nullptr;
PxCooking *cooking = nullptr;
std::set<GeometrySpec *> geometrySpecs;

void doInitPhysx() {
  gAllocator = new PxDefaultAllocator();
  gErrorCallback = new PxDefaultErrorCallback();
  gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gAllocator, *gErrorCallback);
  PxTolerancesScale tolerancesScale;
  physics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, tolerancesScale);
  PxCookingParams cookingParams(tolerancesScale);
  // cookingParams.midphaseDesc = PxMeshMidPhase::eBVH34;
  cooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, cookingParams);
}

uintptr_t doRegisterBakedGeometry(unsigned int meshId, uintptr_t data, size_t size, float *meshPosition, float *meshQuaternion) {
  PxDefaultMemoryInputData readBuffer((PxU8 *)data, size);
  PxTriangleMesh *triangleMesh = physics->createTriangleMesh(readBuffer);
  PxTriangleMeshGeometry *meshGeom = new PxTriangleMeshGeometry(triangleMesh);
  Sphere boundingSphere(meshPosition[0], meshPosition[1], meshPosition[2], subparcelRadius);
  GeometrySpec *geometrySpec = new GeometrySpec(meshId, triangleMesh, meshGeom, Vec(), Quat(), boundingSphere);
  geometrySpecs.insert(geometrySpec);
  return (uintptr_t)geometrySpec;
}

uintptr_t doRegisterBoxGeometry(unsigned int meshId, float *position, float *quaternion, float w, float h, float d) {
  Vec p(position[0], position[1], position[2]);
  Quat q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  Vec halfScale(w/2.0f, h/2.0f, d/2.0f);
  PxBoxGeometry *meshGeom = new PxBoxGeometry(halfScale.x, halfScale.y, halfScale.z);
  Sphere boundingSphere(0, 0, 0, halfScale.magnitude());
  GeometrySpec *geometrySpec = new GeometrySpec(meshId, nullptr, meshGeom, p, q, boundingSphere);
  geometrySpecs.insert(geometrySpec);
  return (uintptr_t)geometrySpec;
}

uintptr_t doRegisterCapsuleGeometry(unsigned int meshId, float *position, float *quaternion, float radius, float halfHeight) {
  Vec p(position[0], position[1], position[2]);
  Quat q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  PxCapsuleGeometry *meshGeom = new PxCapsuleGeometry(radius, halfHeight);
  Sphere boundingSphere(0, 0, 0, radius + halfHeight);
  GeometrySpec *geometrySpec = new GeometrySpec(meshId, nullptr, meshGeom, p, q, boundingSphere);
  geometrySpecs.insert(geometrySpec);
  return (uintptr_t)geometrySpec;
}

void doBakeGeometry(float *positions, unsigned int *indices, unsigned int numPositions, unsigned int numIndices, uintptr_t &ptr, uintptr_t &data, size_t &size) {
  PxVec3 *verts = (PxVec3 *)positions;
  PxU32 nbVerts = numPositions/3;
  PxU32 *indices32 = (PxU32 *)indices;
  PxU32 triCount = numIndices/3;

  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count           = nbVerts;
  meshDesc.points.stride          = sizeof(PxVec3);
  meshDesc.points.data            = verts;

  meshDesc.triangles.count        = triCount;
  meshDesc.triangles.stride       = 3*sizeof(PxU32);
  meshDesc.triangles.data         = indices32;

  PxDefaultMemoryOutputStream *writeBuffer = new PxDefaultMemoryOutputStream();
  bool status = cooking->cookTriangleMesh(meshDesc, *writeBuffer);
  if (status) {
    ptr = (uintptr_t)writeBuffer;
    data = (uintptr_t)writeBuffer->getData();
    size = writeBuffer->getSize();
  } else {
    delete writeBuffer;
    ptr = 0;
    data = 0;
    size = 0;
  }
}

/* void doBakeConvexGeometry(float *positions, unsigned int *indices, unsigned int numPositions, unsigned int numIndices, uintptr_t &ptr, uintptr_t &data, size_t &size) {
  PxVec3 *verts = (PxVec3 *)positions;
  PxU32 nbVerts = numPositions/3;
  PxU32 *indices32 = (PxU32 *)indices;
  PxU32 triCount = numIndices/3;

  PxConvexMeshDesc meshDesc;
  meshDesc.points.count           = nbVerts;
  meshDesc.points.stride          = sizeof(PxVec3);
  meshDesc.points.data            = verts;

  meshDesc.triangles.count        = triCount;
  meshDesc.triangles.stride       = 3*sizeof(PxU32);
  meshDesc.triangles.data         = indices32;

  PxDefaultMemoryOutputStream *writeBuffer = new PxDefaultMemoryOutputStream();
  bool status = cooking->cookConvexMesh(meshDesc, *writeBuffer);
  if (status) {
    ptr = (uintptr_t)writeBuffer;
    data = (uintptr_t)writeBuffer->getData();
    size = writeBuffer->getSize();
  } else {
    delete writeBuffer;
    ptr = 0;
    data = 0;
    size = 0;
  }
} */

void doReleaseBakedGeometry(uintptr_t ptr) {
  PxDefaultMemoryOutputStream *writeBuffer = (PxDefaultMemoryOutputStream *)ptr;
  delete writeBuffer;
}

void doUnregisterGeometry(uintptr_t geometrySpecPtr) {
  GeometrySpec *geometrySpec = (GeometrySpec *)geometrySpecPtr;
  delete geometrySpec;
  geometrySpecs.erase(geometrySpec);
}

void doRaycast(float *origin, float *direction, float *meshPosition, float *meshQuaternion, unsigned int &hit, float *position, float *normal, float &distance, unsigned int &meshId, unsigned int &faceIndex) {
  PxVec3 originVec{origin[0], origin[1], origin[2]};
  PxVec3 directionVec{direction[0], direction[1], direction[2]};
  Ray ray(Vec{origin[0], origin[1], origin[2]}, Vec{direction[0], direction[1], direction[2]});
  PxTransform meshPose(
    PxVec3{meshPosition[0], meshPosition[1], meshPosition[2]},
    PxQuat{meshQuaternion[0], meshQuaternion[1], meshQuaternion[2], meshQuaternion[3]}
  );
  Vec p(meshPosition[0], meshPosition[1], meshPosition[2]);
  Quat q(meshQuaternion[0], meshQuaternion[1], meshQuaternion[2], meshQuaternion[3]);

  PxRaycastHit hitInfo;
  constexpr float maxDist = 1000.0;
  const PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
  constexpr PxU32 maxHits = 1;

  std::vector<std::pair<float, GeometrySpec *>> sortedGeometrySpecs;
  sortedGeometrySpecs.reserve(geometrySpecs.size());
  for (GeometrySpec *geometrySpec : geometrySpecs) {
    Sphere sphere(
      (geometrySpec->boundingSphere.center.clone().applyQuaternion(geometrySpec->quaternion) + geometrySpec->position)
        .applyQuaternion(q) + p,
      geometrySpec->boundingSphere.radius
    );
    if (ray.intersectsSphere(sphere)) {
      sortedGeometrySpecs.push_back(std::pair<float, GeometrySpec *>(sphere.center.distanceTo(ray.origin), geometrySpec));
    }
  }
  std::sort(sortedGeometrySpecs.begin(), sortedGeometrySpecs.end(), [](const std::pair<float, GeometrySpec *> &a, const std::pair<float, GeometrySpec *> &b) -> bool {
    return a.first < b.first;
  });
  for (std::pair<float, GeometrySpec *> &p : sortedGeometrySpecs) {
    GeometrySpec *geometrySpec = p.second;
    const unsigned int &meshIdData = geometrySpec->meshId;
    PxGeometry *meshGeom = geometrySpec->meshGeom;
    PxTransform meshPose2{
      PxVec3{geometrySpec->position.x, geometrySpec->position.y, geometrySpec->position.z},
      PxQuat{geometrySpec->quaternion.x, geometrySpec->quaternion.x, geometrySpec->quaternion.z, geometrySpec->quaternion.w}
    };
    PxTransform meshPose3 = meshPose * meshPose2;

    PxU32 hitCount = PxGeometryQuery::raycast(originVec, directionVec,
                                              *meshGeom,
                                              meshPose3,
                                              maxDist,
                                              hitFlags,
                                              maxHits, &hitInfo);

    if (hitCount > 0) {
      hit = 1;
      position[0] = hitInfo.position.x;
      position[1] = hitInfo.position.y;
      position[2] = hitInfo.position.z;
      normal[0] = hitInfo.normal.x;
      normal[1] = hitInfo.normal.y;
      normal[2] = hitInfo.normal.z;
      distance = hitInfo.distance;
      meshId = meshIdData;
      faceIndex = hitInfo.faceIndex;
      return;
    }
  }
  hit = 0;
}

void doCollide(float radius, float halfHeight, float *position, float *quaternion, float *meshPosition, float *meshQuaternion, unsigned int maxIter, unsigned int &hit, float *direction) {
  PxCapsuleGeometry geom(radius, halfHeight);
  PxTransform geomPose(
    PxVec3{position[0], position[1], position[2]},
    PxQuat{quaternion[0], quaternion[1], quaternion[2], quaternion[3]}
  );
  PxTransform meshPose{
    PxVec3{meshPosition[0], meshPosition[1], meshPosition[2]},
    PxQuat{meshQuaternion[0], meshQuaternion[1], meshQuaternion[2], meshQuaternion[3]}
  };

  PxVec3 directionVec;
  PxReal depthFloat;

  Vec capsulePosition(position[0], position[1], position[2]);
  Vec p(meshPosition[0], meshPosition[1], meshPosition[2]);
  Quat q(meshQuaternion[0], meshQuaternion[1], meshQuaternion[2], meshQuaternion[3]);

  std::vector<std::pair<float, GeometrySpec *>> sortedGeometrySpecs;
  sortedGeometrySpecs.reserve(geometrySpecs.size());
  const float maxDistance = subparcelRadius + halfHeight + radius;
  for (GeometrySpec *geometrySpec : geometrySpecs) {
    Vec spherePosition = (geometrySpec->boundingSphere.center.clone().applyQuaternion(geometrySpec->quaternion) + geometrySpec->position)
      .applyQuaternion(q) + p;
    float distance = spherePosition.distanceTo(capsulePosition);
    if (distance < maxDistance) {
      sortedGeometrySpecs.push_back(std::pair<float, GeometrySpec *>(distance, geometrySpec));
    }
  }
  std::sort(sortedGeometrySpecs.begin(), sortedGeometrySpecs.end(), [](const std::pair<float, GeometrySpec *> &a, const std::pair<float, GeometrySpec *> &b) -> bool {
    return a.first < b.first;
  });
  Vec offset(0, 0, 0);
  bool anyHadHit = false;
  for (unsigned int i = 0; i < maxIter; i++) {
    bool hadHit = false;
    for (const std::pair<float, GeometrySpec *> &p : sortedGeometrySpecs) {
      GeometrySpec *geometrySpec = p.second;
      PxGeometry *meshGeom = geometrySpec->meshGeom;
      PxTransform meshPose2{
        PxVec3{geometrySpec->position.x, geometrySpec->position.y, geometrySpec->position.z},
        PxQuat{geometrySpec->quaternion.x, geometrySpec->quaternion.x, geometrySpec->quaternion.z, geometrySpec->quaternion.w}
      };
      PxTransform meshPose3 = meshPose * meshPose2;

      bool result = PxGeometryQuery::computePenetration(directionVec, depthFloat, geom, geomPose, *meshGeom, meshPose3);
      /* bool result = PxComputeTriangleMeshPenetration(
        directionVec,
        depthFloat,
        geom,
        geomPose,
        *meshGeom,
        meshPose,
        1,
        nullptr
      ); */
      if (result) {
        hadHit = true;
        offset += Vec(directionVec.x, directionVec.y, directionVec.z)*depthFloat;
        geomPose.p.x += directionVec.x*depthFloat;
        geomPose.p.y += directionVec.y*depthFloat;
        geomPose.p.z += directionVec.z*depthFloat;
        break;
      }
    }
    if (hadHit) {
      anyHadHit = true;
      continue;
    } else {
      break;
    }
  }
  if (anyHadHit) {
    hit = 1;
    direction[0] = offset.x;
    direction[1] = offset.y;
    direction[2] = offset.z;
  } else {
    hit = 0;
  }
}