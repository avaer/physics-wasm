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
#include <iostream>

using namespace physx;

PxDefaultAllocator *gAllocator = nullptr;
PxDefaultErrorCallback *gErrorCallback = nullptr;
PxFoundation *gFoundation = nullptr;
PxPhysics *physics = nullptr;
PxCooking *cooking = nullptr;

void doInitOverlap() {
  gAllocator = new PxDefaultAllocator();
  gErrorCallback = new PxDefaultErrorCallback();
  gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gAllocator, *gErrorCallback);
  physics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());
  cooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
}

void doFindOverlap() {
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
  /* PxU32 nb;
  PxVec3 depenetrationVector = PxComputeMeshPenetration(
    maxIter,
    geom,
    geomPose,
    meshGeom,
    meshPose,
    nb   
  ); */
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
  /* PxU32 results[16];
  PxU32 maxResults = sizeof(results)/sizeof(results[0]);
  PxU32 startIndex = 0;
  bool overflow = false;
  PxU32 numResults = PxMeshQuery::findOverlapTriangleMesh(geom, geomPose,
                                meshGeom, meshPose,
                                results, maxResults, startIndex, overflow);
  for (PxU32 i = 0; i < numResults; i++) {
    PxU32 index = results[i];
    PxTriangle triangle;
    PxMeshQuery::getTriangle(
      meshGeom,
      geomPose,
      index,
      triangle
    );
  } */

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
}