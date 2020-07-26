// #include "DirectXMath.h"
/* #include "PhysX/physx/include/geometry/PxMeshQuery.h"
#include "geometry/PxCapsuleGeometry.h"
#include "foundation/PxTransform.h"
 */
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
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "PxRigidStatic.h"
#include <set>
#include <iostream>

using namespace physx;

class GeometrySpec {
public:
  GeometrySpec(unsigned int meshId, PxTriangleMeshGeometry *meshGeom, PxRigidStatic *actor, PxShape *shape) :
    meshId(meshId), meshGeom(meshGeom), actor(actor), shape(shape) {}
  ~GeometrySpec() {
    PxTriangleMesh *triangleMesh = meshGeom->triangleMesh;
    delete meshGeom;
    triangleMesh->release();
    actor->release();
    shape->release();
  }

  unsigned int meshId;
  PxTriangleMeshGeometry *meshGeom;
  PxRigidStatic *actor;
  PxShape *shape;
};
std::set<GeometrySpec *> geometrySpecs;

PxDefaultAllocator *gAllocator = nullptr;
PxDefaultErrorCallback *gErrorCallback = nullptr;
PxFoundation *gFoundation = nullptr;
PxPhysics *physics = nullptr;
PxCooking *cooking = nullptr;
PxScene *gScene = nullptr;
PxMaterial *gMaterial = nullptr;

void doInitPhysx() {
  gAllocator = new PxDefaultAllocator();
  gErrorCallback = new PxDefaultErrorCallback();
  gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gAllocator, *gErrorCallback);
  PxTolerancesScale tolerancesScale;
  physics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, tolerancesScale);
  PxCookingParams cookingParams(tolerancesScale);
  // cookingParams.midphaseDesc = PxMeshMidPhase::eBVH34;
  cooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, cookingParams);
  gScene = physics->createScene(PxSceneDesc(tolerancesScale));
  gMaterial = physics->createMaterial(1, 1, 1);
}

uintptr_t doRegisterGeometry(unsigned int meshId, float *positions, unsigned int *indices, unsigned int numPositions, unsigned int numIndices, float *meshPosition, float *meshQuaternion) {
  PxVec3 *verts = (PxVec3 *)positions;
  PxU32 nbVerts = numPositions/3;
  PxU32 *indices32 = (PxU32 *)indices;
  PxU32 triCount = numIndices/3;

  /* std::vector<PxU32> indicesCache;
  if (indices32 == nullptr) {
    numIndices = nbVerts/3;
    indicesCache.resize(numIndices);
    for (unsigned int i = 0; i < numIndices; i++) {
      indicesCache[i] = i;
    }
    indices32 = indicesCache.data();
    triCount = numIndices/3;
  } */

  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count           = nbVerts;
  meshDesc.points.stride          = sizeof(PxVec3);
  meshDesc.points.data            = verts;

  meshDesc.triangles.count        = triCount;
  meshDesc.triangles.stride       = 3*sizeof(PxU32);
  meshDesc.triangles.data         = indices32;

  PxDefaultMemoryOutputStream writeBuffer;
  bool status = cooking->cookTriangleMesh(meshDesc, writeBuffer);
  if (!status) {
    return 0;
  }

  PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
  PxTriangleMesh *triangleMesh = physics->createTriangleMesh(readBuffer);
  PxTriangleMeshGeometry *meshGeom = new PxTriangleMeshGeometry(triangleMesh);
  /* PxTransform meshPose(
    PxVec3{meshPosition[0], meshPosition[1], meshPosition[2]},
    PxQuat{meshQuaternion[0], meshQuaternion[1], meshQuaternion[2], meshQuaternion[3]}
  ); */
  PxTransform meshPose;
  PxRigidStatic *actor = physics->createRigidStatic(meshPose);
  PxShape *shape = physics->createShape(*meshGeom, *gMaterial);
  actor->attachShape(*shape);
  gScene->addActor(*actor);
  gScene->flushQueryUpdates();
  gScene->forceDynamicTreeRebuild(true, true);

  GeometrySpec *geometrySpec = new GeometrySpec(meshId, meshGeom, actor, shape);
  geometrySpecs.insert(geometrySpec);
  return (uintptr_t)geometrySpec;
}

void doUnregisterGeometry(uintptr_t geometrySpecPtr) {
  GeometrySpec *geometrySpec = (GeometrySpec *)geometrySpecPtr;
  gScene->removeActor(*geometrySpec->actor);
  delete geometrySpec;
  geometrySpecs.erase(geometrySpec);
}

void doRaycast(float *origin, float *direction, unsigned int &hit, float *position, float *normal, float &distance, unsigned int &meshId, unsigned int &faceIndex) {
  PxVec3 originVec{origin[0], origin[1], origin[2]};
  PxVec3 directionVec{direction[0], direction[1], direction[2]};
  float maxDist = 1000.0;
  PxRaycastBuffer hitResult;
  bool result = gScene->raycast(originVec, directionVec, maxDist, hitResult);
  if (result) {
    std::cout << "got hit " << hitResult.block.position.x << " " << hitResult.block.position.y << " " << hitResult.block.position.z << std::endl;
    hit = 1;
    position[0] = hitResult.block.position.x;
    position[1] = hitResult.block.position.y;
    position[2] = hitResult.block.position.z;
    normal[0] = hitResult.block.normal.x;
    normal[1] = hitResult.block.normal.y;
    normal[2] = hitResult.block.normal.z;
    distance = hitResult.block.distance;
    meshId = 0;
    for (GeometrySpec *geometrySpec : geometrySpecs) {
      if (geometrySpec->actor == hitResult.block.actor) {
        meshId = geometrySpec->meshId;
        break;
      }
    }
    faceIndex = hitResult.block.faceIndex;
  } else {
    hit = 0;
  }
}

void doCollide(float radius, float halfHeight, float *position, float *quaternion, unsigned int maxIter, unsigned int &hit, float *direction, float &depth) {
  for (GeometrySpec *geometrySpec : geometrySpecs) {
    PxTriangleMeshGeometry *meshGeom = geometrySpec->meshGeom;
    // const PxTransform &meshPose = geometrySpec->meshPose;
    PxTransform meshPose;

    PxCapsuleGeometry geom(radius, halfHeight);
    PxTransform geomPose(
      PxVec3{position[0], position[1], position[2]},
      PxQuat{quaternion[0], quaternion[1], quaternion[2], quaternion[3]}
    );

    // PxU32 maxIter = 4;
    /* PxU32 nb;
    PxVec3 depenetrationVector = PxComputeMeshPenetration(
      maxIter,
      geom,
      geomPose,
      meshGeom,
      meshPose,
      nb
    ); */
    PxVec3 directionVec{0, 0, 0};
    PxReal depthFloat = 0;
    // bool result = PxGeometryQuery::computePenetration(direction, depth, geom, geomPose, meshGeom, meshPose);
    bool result = PxComputeTriangleMeshPenetration(
      directionVec,
      depthFloat,
      geom,
      geomPose,
      *meshGeom,
      meshPose,
      maxIter,
      nullptr
    );
    if (result) {
      hit = 1;
      direction[0] = directionVec.x;
      direction[1] = directionVec.y;
      direction[2] = directionVec.z;
      depth = depthFloat;
      return;
    }
  }
  hit = 0;
}

/* void doFindOverlap() {
  std::cout << "overlap 1" << std::endl;
  PxCapsuleGeometry geom(
    1, // radius
    1 // halfHeight
  );
  PxTransform geomPose;

  std::cout << "overlap 2" << std::endl;

  PxTriangleMesh *triangleMesh;
  // {
    PxVec3 verts[] = {
      {-1, -0.5, -1},
      {1, -0.5, -1},
      {0, -0.5, 1},
    };
    PxU32 nbVerts = sizeof(verts)/sizeof(verts[0]);
    PxU32 indices32[] = {0, 2, 1};
    PxU32 triCount = sizeof(indices32)/sizeof(indices32[0])/3;

    std::cout << "overlap 3 " << nbVerts << " " << triCount << std::endl;

    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count           = nbVerts;
    meshDesc.points.stride          = sizeof(PxVec3);
    meshDesc.points.data            = verts;

    meshDesc.triangles.count        = triCount;
    meshDesc.triangles.stride       = 3*sizeof(PxU32);
    meshDesc.triangles.data         = indices32;

    PxDefaultMemoryOutputStream writeBuffer;
    bool status = cooking->cookTriangleMesh(meshDesc, writeBuffer);
    if (!status) {
      std::cerr << "cooking failed" << std::endl;
      return;
    }

    PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
    triangleMesh = physics->createTriangleMesh(readBuffer);
  // }

  std::cout << "overlap 4 " << (void *)triangleMesh << " " << writeBuffer.getSize() << std::endl;

  PxTriangleMeshGeometry meshGeom(triangleMesh);
  PxTransform meshPose;
  PxU32 maxIter = 4;
  std::cout << "overlap 5" << std::endl;
  PxVec3 direction{0, 0, 0};
  PxReal depth = 0;
  // bool result = PxGeometryQuery::computePenetration(direction, depth, geom, geomPose, meshGeom, meshPose);
  bool result = PxComputeTriangleMeshPenetration(
    direction,
    depth,
    geom,
    geomPose,
    meshGeom,
    meshPose,
    maxIter,
    nullptr
  );
  std::cout << "got result " << result << " " << direction.x << " " << direction.y << " " << direction.z << " " << depth << std::endl;

  PxVec3 origin{0, 2, 0};
  PxVec3 unitDir{0, -1, 0};
  PxRaycastHit hitInfo;
  float maxDist = 1000.0;
  PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
  PxU32 maxHits = 1;
  PxU32 hitCount = PxGeometryQuery::raycast(origin, unitDir,
                                            meshGeom,
                                            meshPose,
                                            maxDist,
                                            hitFlags,
                                            maxHits, &hitInfo);
  std::cout << "raycast result 1 " << hitCount << " " << hitInfo.position.x << " " << hitInfo.position.y << " " << hitInfo.position.z << " " << hitInfo.distance << std::endl;

  origin = PxVec3{0, -2, 0};
  unitDir = PxVec3{0, 1, 0};
  hitCount = PxGeometryQuery::raycast(origin, unitDir,
                                            meshGeom,
                                            meshPose,
                                            maxDist,
                                            hitFlags,
                                            maxHits, &hitInfo);
  std::cout << "raycast result 2 " << hitCount << " " << hitInfo.position.x << " " << hitInfo.position.y << " " << hitInfo.position.z << " " << hitInfo.distance << std::endl;

  std::cout << "overlap 6" << std::endl;
} */