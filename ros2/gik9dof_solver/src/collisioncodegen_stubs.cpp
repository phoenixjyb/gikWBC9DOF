/**
 * @file collisioncodegen_stubs.cpp
 * @brief Stub implementations for MATLAB collisioncodegen functions
 * 
 * These are minimal implementations to satisfy linker requirements.
 * For full collision detection, you would need the actual MATLAB
 * Robotics System Toolbox collision library or implement using libccd.
 * 
 * For IK solving without active collision avoidance, these stubs are sufficient.
 */

#include "collisioncodegen_api.hpp"
#include <cstdlib>
#include <cstring>

// Missing typedef from collision headers
typedef void *CollisionResultVoidPtr;

// Simple stub that returns null/does nothing
// This allows the code to link and run, but collision checking will be disabled

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeBox(real64_T x, real64_T y, real64_T z)
{
    // Return nullptr - no collision geometry created
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeSphere(real64_T r)
{
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeCylinder(real64_T r, real64_T h)
{
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeCapsule(real64_T r, real64_T h)
{
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeMesh(real64_T *vertices, real64_T numVertices)
{
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_destructGeometry(const CollisionGeometryVoidPtr *objPtr)
{
    // Nothing to free since we return nullptr
}

EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_copyGeometry(CollisionGeometryVoidPtr obj)
{
    // Return nullptr copy
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getVertices(CollisionGeometryVoidPtr obj, real64_T *vertices)
{
    // No vertices to return
}

EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getNumVertices(CollisionGeometryVoidPtr obj)
{
    return 0.0;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getFaces(CollisionGeometryVoidPtr obj, int32_T *faces)
{
    // No faces to return
}

EXTERN_C COLLISIONCODEGEN_API uint32_T
collisioncodegen_getNumFaces(CollisionGeometryVoidPtr obj)
{
    return 0;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getPose(CollisionGeometryVoidPtr obj, real64_T *pose)
{
    // Return identity matrix (4x4)
    if (pose) {
        memset(pose, 0, 16 * sizeof(real64_T));
        pose[0] = 1.0; // pose[0,0]
        pose[5] = 1.0; // pose[1,1]
        pose[10] = 1.0; // pose[2,2]
        pose[15] = 1.0; // pose[3,3]
    }
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_setPose(CollisionGeometryVoidPtr obj, const real64_T *pose)
{
    // No-op
}

EXTERN_C COLLISIONCODEGEN_API int32_T
collisioncodegen_getPrimitiveType(CollisionGeometryVoidPtr obj)
{
    return 0; // Return "none" type
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getPrimitiveParameters(CollisionGeometryVoidPtr obj,
                                        real64_T *params)
{
    // No parameters
}

EXTERN_C COLLISIONCODEGEN_API CollisionResultVoidPtr
collisioncodegen_checkCollision(const CollisionGeometryVoidPtr obj1,
                                const real64_T *pose1,
                                const CollisionGeometryVoidPtr obj2,
                                const real64_T *pose2,
                                real64_T exhaustiveCheck,
                                real64_T maxNumContacts)
{
    // Return nullptr - no collision result
    return nullptr;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_destructCollisionResult(const CollisionResultVoidPtr *resPtr)
{
    // Nothing to free
}

EXTERN_C COLLISIONCODEGEN_API boolean_T
collisioncodegen_getCollisionStatus(CollisionResultVoidPtr res)
{
    // Always return no collision
    return false;
}

EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getMinimumDistance(CollisionResultVoidPtr res)
{
    // Return large distance (no collision)
    return 1000.0;
}

EXTERN_C COLLISIONCODEGEN_API uint32_T
collisioncodegen_getNumContacts(CollisionResultVoidPtr res)
{
    return 0;
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getContactPositions(CollisionResultVoidPtr res,
                                      real64_T *positions)
{
    // No contact positions
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getContactNormals(CollisionResultVoidPtr res,
                                    real64_T *normals)
{
    // No contact normals
}

EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getContactDepths(CollisionResultVoidPtr res,
                                   real64_T *depths)
{
    // No contact depths
}
