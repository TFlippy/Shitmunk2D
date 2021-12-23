/* Copyright (c) 2013 Scott Lembcke and Howling Moon Software
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 // All of the struct definitions for Chipmunk should be considered part of the private API.
 // However, it is very valuable to know the struct sizes for preallocating memory.

#ifndef CHIPMUNK_STRUCTS_H
#define CHIPMUNK_STRUCTS_H

#include "chipmunk/chipmunk.h"

struct cpArray
{
	int num, max;
	void** arr;
};

struct cpImpact
{
	cpVect p;
	cpVect n;

	cpFloat size;
	cpFloat ke;
	cpFloat bounce;
	cpFloat bounce_rigid;

	int count;
	cpTimestamp stamp;

	uint8_t dirty;
	uint8_t material_type_a;
	uint8_t material_type_b;
	uint8_t unused;
};

struct cpBody
{
	cpVect cog;
	cpVect v_bias;
	cpVect p;
	cpVect v;
	cpVect f;
	cpVect s;

	cpBB bb;

	cpEntity owner_entity;
	cpEntity parent_entity;

	cpSpace* space;

	cpShape* shapeList;
	cpArbiter* arbiterList;
	cpConstraint* constraintList;

	struct
	{
		cpBody* root;
		cpBody* next;
		cpFloat idleTime;
	} sleeping;

	cpFloat a;
	cpFloat w;
	cpFloat t;

	cpFloat m;
	cpFloat m_inv;

	cpFloat i;
	cpFloat i_inv;

	cpFloat w_bias;
	cpFloat gravity;
	cpFloat max_velocity;

	cpBodyType type;

	uint8_t faction_id;
	uint8_t player_id;
	uint8_t unused_00;
	uint8_t unused_01;

	cpTransform transform;
	cpTransform transform_unscaled;

	cpImpact impact;
};

enum cpArbiterState
{
	// Arbiter is active and its the first collision.
	CP_ARBITER_STATE_FIRST_COLLISION,
	// Arbiter is active and its not the first collision.
	CP_ARBITER_STATE_NORMAL,
	// Collision has been explicitly ignored.
	// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
	CP_ARBITER_STATE_IGNORE,
	// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
	CP_ARBITER_STATE_CACHED,
	// Collison arbiter is invalid because one of the shapes was removed.
	CP_ARBITER_STATE_INVALIDATED,
};

struct cpArbiterThread
{
	struct cpArbiter* next, * prev;
};

struct cpContact
{
	cpVect r1, r2;

	cpFloat nMass, tMass;
	cpFloat bounce_rigid;
	cpFloat bounce; // TODO: look for an alternate bounce solution.

	cpFloat jnAcc, jtAcc, jBias;
	cpFloat bias;

	cpHashValue hash;
};

struct cpCollisionInfo
{
	const cpShape* a, * b;
	cpCollisionID id;

	cpVect n;

	int count;
	// TODO Should this be a unique struct type?
	struct cpContact* arr;
};

struct cpArbiter
{
	cpFloat e;
	cpFloat u;
	cpVect surface_vr;

	cpDataPointer data;

	const cpShape* a, * b;
	cpBody* body_a, * body_b;
	struct cpArbiterThread thread_a, thread_b;

	int count;
	int offset;
	struct cpContact* contacts;
	cpVect n;

	// Regular, wildcard A and wildcard B collision handlers.
	cpCollisionHandler* handler, * handlerA, * handlerB;
	cpBool swapped;
	cpBool dirty;

	cpTimestamp stamp;
	enum cpArbiterState state;
};

struct cpShapeMassInfo
{
	cpFloat m;
	cpFloat i;
	cpVect cog;
	cpFloat area;
};

typedef enum cpShapeType
{
	CP_CIRCLE_SHAPE,
	CP_SEGMENT_SHAPE,
	CP_POLY_SHAPE,
	//CP_BOX_SHAPE,
	CP_NUM_SHAPES
} cpShapeType;

typedef cpBB(*cpShapeCacheDataImpl)(cpShape* shape, cpTransform transform);
typedef void (*cpShapeDestroyImpl)(cpShape* shape);
typedef void (*cpShapePointQueryImpl)(const cpShape* shape, cpVect p, cpPointQueryInfo* info);
typedef void (*cpShapeSegmentQueryImpl)(const cpShape* shape, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo* info);

typedef struct cpShapeClass cpShapeClass;

struct cpShapeClass
{
	cpShapeType type;

	cpShapeCacheDataImpl cacheData;
	cpShapeDestroyImpl destroy;
	cpShapePointQueryImpl pointQuery;
	cpShapeSegmentQueryImpl segmentQuery;
};

struct cpShape
{
	const cpShapeClass* klass;

	cpSpace* space;
	cpBody* body;
	struct cpShapeMassInfo massInfo;
	cpBB bb;

	cpBool sensor;

	cpFloat e;
	cpFloat u;
	cpVect surfaceV;

	cpDataPointer userData;

	uint8_t unused_00;
	uint8_t unused_01;
	uint8_t block_id;
	uint8_t material_type;

	uint64_t attached_component_id;

	cpCollisionType type;
	cpShapeFilter filter;

	cpShape* next;
	cpShape* prev;

	cpHashValue hashid;
};

struct cpCircleShape
{
	cpShape shape;

	cpVect c, tc;
	cpFloat r;
};

struct cpSegmentShape
{
	cpShape shape;

	cpVect a, b, n;
	cpVect ta, tb, tn;
	cpFloat r;

	//cpVect a_tangent, b_tangent;
};

struct cpBoxShape
{
	cpShape shape;
	cpVect a, b;
};

struct cpSplittingPlane
{
	cpVect v0, n;
};

#define CP_POLY_SHAPE_INLINE_ALLOC 6

struct cpPolyShape
{
	cpShape shape;

	cpFloat r;

	int count;
	// The untransformed planes are appended at the end of the transformed planes.
	struct cpSplittingPlane* planes;

	// Allocate a small number of splitting planes internally for simple poly.
	struct cpSplittingPlane _planes[2 * CP_POLY_SHAPE_INLINE_ALLOC];
};

typedef void (*cpConstraintPreStepImpl)(cpConstraint* constraint, cpFloat dt);
typedef void (*cpConstraintApplyCachedImpulseImpl)(cpConstraint* constraint, cpFloat dt_coef);
typedef void (*cpConstraintApplyImpulseImpl)(cpConstraint* constraint, cpFloat dt);
typedef cpFloat(*cpConstraintGetImpulseImpl)(cpConstraint* constraint);

typedef struct cpConstraintClass
{
	cpConstraintPreStepImpl preStep;
	cpConstraintApplyCachedImpulseImpl applyCachedImpulse;
	cpConstraintApplyImpulseImpl applyImpulse;
	cpConstraintGetImpulseImpl getImpulse;
} cpConstraintClass;

struct cpConstraint
{
	const cpConstraintClass* klass;

	cpSpace* space;

	cpBody* a, * b;
	cpConstraint* next_a, * next_b;

	cpFloat maxForce;
	cpFloat errorBias;
	cpFloat maxBias;

	cpBool collideBodies;

	cpConstraintPreSolveFunc preSolve;
	cpConstraintPostSolveFunc postSolve;

	cpDataPointer userData;
};

struct cpPinJoint
{
	cpConstraint constraint;
	cpVect anchorA, anchorB;
	cpFloat dist;

	cpVect r1, r2;
	cpVect n;
	cpFloat nMass;

	cpFloat jnAcc;
	cpFloat bias;
};

struct cpSlideJoint
{
	cpConstraint constraint;
	cpVect anchorA, anchorB;
	cpFloat min, max;

	cpVect r1, r2;
	cpVect n;
	cpFloat nMass;

	cpFloat jnAcc;
	cpFloat bias;
};

struct cpPivotJoint
{
	cpConstraint constraint;
	cpVect anchorA, anchorB;

	cpVect r1, r2;
	cpMat2x2 k;

	cpVect jAcc;
	cpVect jAcc_raw;

	cpVect bias;

	cpVect delta;
};

struct cpGrooveJoint
{
	cpConstraint constraint;
	cpVect grv_n, grv_a, grv_b;
	cpVect  anchorB;

	cpVect grv_tn;
	cpFloat clamp;
	cpVect r1, r2;
	cpMat2x2 k;

	cpVect jAcc;
	cpVect bias;
};

struct cpDampedSpring
{
	cpConstraint constraint;
	cpVect anchorA, anchorB;
	cpFloat restLength;
	cpFloat stiffness;
	cpFloat damping;
	cpDampedSpringForceFunc springForceFunc;

	cpFloat target_vrn;
	cpFloat v_coef;

	cpVect r1, r2;
	cpFloat nMass;
	cpVect n;

	cpFloat jAcc;
};

struct cpDampedRotarySpring
{
	cpConstraint constraint;
	cpFloat restAngle;
	cpFloat stiffness;
	cpFloat damping;
	cpDampedRotarySpringTorqueFunc springTorqueFunc;

	cpFloat target_wrn;
	cpFloat w_coef;

	cpFloat iSum;
	cpFloat jAcc;
};

struct cpRotaryLimitJoint
{
	cpConstraint constraint;
	cpFloat min, max;

	cpFloat iSum;

	cpFloat bias;
	cpFloat jAcc;
};

struct cpRatchetJoint
{
	cpConstraint constraint;
	cpFloat angle, phase, ratchet;

	cpFloat iSum;

	cpFloat bias;
	cpFloat jAcc;
};

struct cpGearJoint
{
	cpConstraint constraint;
	cpFloat phase, ratio;
	cpFloat ratio_inv;

	cpFloat iSum;

	cpFloat bias;
	cpFloat jAcc;
};

struct cpSimpleMotor
{
	cpConstraint constraint;
	cpFloat rate;

	cpFloat iSum;

	cpFloat jAcc;
};

typedef struct cpContactBufferHeader cpContactBufferHeader;
typedef void (*cpSpaceArbiterApplyImpulseFunc)(cpArbiter* arb);

struct cpSpace
{
	int iterations;

	cpVect gravity;
	cpFloat damping;
	cpFloat damping_w;

	cpFloat idleSpeedThreshold;
	cpFloat sleepTimeThreshold;

	cpFloat collisionSlop;
	cpFloat collisionBias;
	cpTimestamp collisionPersistence;

	cpDataPointer userData;

	cpTimestamp stamp;
	cpFloat curr_dt;

	cpArray* dynamicBodies;
	cpArray* staticBodies;
	cpArray* rousedBodies;
	cpArray* sleepingComponents;

	cpHashValue shapeIDCounter;
	cpSpatialIndex* staticShapes;
	cpSpatialIndex* dynamicShapes;

	cpArray* constraints;

	cpArray* arbiters;
	cpContactBufferHeader* contactBuffersHead;
	cpHashSet* cachedArbiters;
	cpArray* pooledArbiters;

	cpArray* allocatedBuffers;
	long locked;

	cpBool usesWildcards;
	cpHashSet* collisionHandlers;
	cpCollisionHandler defaultHandler;

	cpImpactFunc impactFunc;

	cpBool skipPostStep;
	cpArray* postStepCallbacks;

	cpBody* staticBody;
	cpBody _staticBody;
};

typedef struct cpPostStepCallback
{
	cpPostStepFunc func;
	void* key;
	void* data;
} cpPostStepCallback;

#endif
