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

#include "chipmunk/chipmunk_private.h"

 //MARK: Post Step Callback Functions

cpPostStepCallback*
cpSpaceGetPostStepCallback(cpSpace* space, void* key)
{
	cpArray* arr = space->postStepCallbacks;
	for (int i = 0; i < arr->num; i++)
	{
		cpPostStepCallback* callback = (cpPostStepCallback*)arr->arr[i];
		if (callback && callback->key == key) return callback;
	}

	return NULL;
}

static void PostStepDoNothing(cpSpace* space, void* obj, void* data)
{
}

cpBool
cpSpaceAddPostStepCallback(cpSpace* space, cpPostStepFunc func, void* key, void* data)
{
	cpAssertWarn(space->locked,
		"Adding a post-step callback when the space is not locked is unnecessary. "
		"Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");

	if (!cpSpaceGetPostStepCallback(space, key))
	{
		cpPostStepCallback* callback = (cpPostStepCallback*)cpcalloc(1, sizeof(cpPostStepCallback));
		callback->func = (func ? func : PostStepDoNothing);
		callback->key = key;
		callback->data = data;

		cpArrayPush(space->postStepCallbacks, callback);
		return cpTrue;
	}
	else
	{
		return cpFalse;
	}
}

//MARK: Locking Functions

void
cpSpaceLock(cpSpace* space)
{
	space->locked++;
}

void
cpSpaceUnlock(cpSpace* space, cpBool runPostStep)
{
	space->locked--;
	cpAssertHard(space->locked >= 0, "Internal Error: Space lock underflow.");

	if (space->locked == 0)
	{
		cpArray* waking = space->rousedBodies;

		for (int i = 0, count = waking->num; i < count; i++)
		{
			cpSpaceActivateBody(space, (cpBody*)waking->arr[i]);
			waking->arr[i] = NULL;
		}

		waking->num = 0;

		if (space->locked == 0 && runPostStep && !space->skipPostStep)
		{
			space->skipPostStep = cpTrue;

			cpArray* arr = space->postStepCallbacks;
			for (int i = 0; i < arr->num; i++)
			{
				cpPostStepCallback* callback = (cpPostStepCallback*)arr->arr[i];
				cpPostStepFunc func = callback->func;

				// Mark the func as NULL in case calling it calls cpSpaceRunPostStepCallbacks() again.
				// TODO: need more tests around this case I think.
				callback->func = NULL;
				if (func) func(space, callback->key, callback->data);

				arr->arr[i] = NULL;
				cpfree(callback);
			}

			arr->num = 0;
			space->skipPostStep = cpFalse;
		}
	}
}

//MARK: Contact Buffer Functions

struct cpContactBufferHeader
{
	cpTimestamp stamp;
	cpContactBufferHeader* next;
	unsigned int numContacts;
};

#define CP_CONTACTS_BUFFER_SIZE ((CP_BUFFER_BYTES - sizeof(cpContactBufferHeader))/sizeof(struct cpContact))
typedef struct cpContactBuffer
{
	cpContactBufferHeader header;
	struct cpContact contacts[CP_CONTACTS_BUFFER_SIZE];
} cpContactBuffer;

static cpContactBufferHeader*
cpSpaceAllocContactBuffer(cpSpace* space)
{
	cpContactBuffer* buffer = (cpContactBuffer*)cpcalloc(1, sizeof(cpContactBuffer));
	cpArrayPush(space->allocatedBuffers, buffer);
	return (cpContactBufferHeader*)buffer;
}

static cpContactBufferHeader*
cpContactBufferHeaderInit(cpContactBufferHeader* header, cpTimestamp stamp, cpContactBufferHeader* splice)
{
	header->stamp = stamp;
	header->next = (splice ? splice->next : header);
	header->numContacts = 0;

	return header;
}

void
cpSpacePushFreshContactBuffer(cpSpace* space)
{
	cpTimestamp stamp = space->stamp;

	cpContactBufferHeader* head = space->contactBuffersHead;

	if (!head)
	{
		// No buffers have been allocated, make one
		space->contactBuffersHead = cpContactBufferHeaderInit(cpSpaceAllocContactBuffer(space), stamp, NULL);
	}
	else if (stamp - head->next->stamp > space->collisionPersistence)
	{
		// The tail buffer is available, rotate the ring
		cpContactBufferHeader* tail = head->next;
		space->contactBuffersHead = cpContactBufferHeaderInit(tail, stamp, tail);
	}
	else
	{
		// Allocate a new buffer and push it into the ring
		cpContactBufferHeader* buffer = cpContactBufferHeaderInit(cpSpaceAllocContactBuffer(space), stamp, head);
		space->contactBuffersHead = head->next = buffer;
	}
}


struct cpContact*
	cpContactBufferGetArray(cpSpace* space)
{
	if (space->contactBuffersHead->numContacts + CP_MAX_CONTACTS_PER_ARBITER > CP_CONTACTS_BUFFER_SIZE)
	{
		// contact buffer could overflow on the next collision, push a fresh one.
		cpSpacePushFreshContactBuffer(space);
	}

	cpContactBufferHeader* head = space->contactBuffersHead;
	return ((cpContactBuffer*)head)->contacts + head->numContacts;
}

void
cpSpacePushContacts(cpSpace* space, int count)
{
	cpAssertHard(count <= CP_MAX_CONTACTS_PER_ARBITER, "Internal Error: Contact buffer overflow!");
	space->contactBuffersHead->numContacts += count;
}

static void
cpSpacePopContacts(cpSpace* space, int count)
{
	space->contactBuffersHead->numContacts -= count;
}

//MARK: Collision Detection Functions

static void*
cpSpaceArbiterSetTrans(cpShape** shapes, cpSpace* space)
{
	if (space->pooledArbiters->num == 0)
	{
		// arbiter pool is exhausted, make more
		int count = CP_BUFFER_BYTES / sizeof(cpArbiter);
		cpAssertHard(count, "Internal Error: Buffer size too small.");

		cpArbiter* buffer = (cpArbiter*)cpcalloc(1, CP_BUFFER_BYTES);
		cpArrayPush(space->allocatedBuffers, buffer);

		for (int i = 0; i < count; i++) cpArrayPush(space->pooledArbiters, buffer + i);
	}

	return cpArbiterInit((cpArbiter*)cpArrayPop(space->pooledArbiters), shapes[0], shapes[1]);
}

static inline cpBool
QueryRejectConstraint(cpBody* a, cpBody* b)
{
	CP_BODY_FOREACH_CONSTRAINT(a, constraint)
	{
		if (
			!constraint->collideBodies && (
				(constraint->a == a && constraint->b == b) ||
				(constraint->a == b && constraint->b == a)
				)
			) return cpTrue;
	}

	return cpFalse;
}

static inline cpBool
QueryReject(cpShape* a, cpShape* b)
{
	return (
		// BBoxes must overlap
		!cpBBIntersects(a->bb, b->bb)
		// Don't collide shapes attached to the same body.
		|| a->body == b->body
		|| (a->body->parent_entity != 0 && a->body->parent_entity == b->body->parent_entity)
		// Don't collide shapes that are filtered.
		|| cpShapeFilterReject(a->filter, b->filter)
		// Don't collide bodies if they have a constraint with collideBodies == cpFalse.
		|| QueryRejectConstraint(a->body, b->body)
		);
}

// Callback from the spatial hash.
cpCollisionID
cpSpaceCollideShapes(cpShape* a, cpShape* b, cpCollisionID id, cpSpace* space)
{
	// Reject any of the simple cases
	if (QueryReject(a, b)) return id;

	// Narrow-phase collision detection.
	struct cpCollisionInfo info = cpCollide(a, b, id, cpContactBufferGetArray(space));

	if (info.count == 0) return info.id; // Shapes are not colliding.
	cpSpacePushContacts(space, info.count);

	// Get an arbiter from space->arbiterSet for the two shapes.
	// This is where the persistant contact magic comes from.
	const cpShape* shape_pair[] = { info.a, info.b };
	cpHashValue arbHashID = CP_HASH_PAIR((cpHashValue)info.a, (cpHashValue)info.b);
	cpArbiter* arb = (cpArbiter*)cpHashSetInsert(space->cachedArbiters, arbHashID, shape_pair, (cpHashSetTransFunc)cpSpaceArbiterSetTrans, space);
	cpArbiterUpdate(arb, &info, space);

	cpCollisionHandler* handler = arb->handler;

	// Call the begin function first if it's the first step
	if (arb->state == CP_ARBITER_STATE_FIRST_COLLISION && !handler->beginFunc(arb, space, handler->userData))
	{
		cpArbiterIgnore(arb); // permanently ignore the collision until separation
	}

	if (
		// Ignore the arbiter if it has been flagged
		(arb->state != CP_ARBITER_STATE_IGNORE) &&
		// Call preSolve
		handler->preSolveFunc(arb, space, handler->userData) &&
		// Check (again) in case the pre-solve() callback called cpArbiterIgnored().
		arb->state != CP_ARBITER_STATE_IGNORE &&
		// Process, but don't add collisions for sensors.
		!(a->sensor || b->sensor) &&
		// Don't process collisions between two infinite mass bodies.
		// This includes collisions between two kinematic bodies, or a kinematic body and a static body.
		!(a->body->m == INFINITY && b->body->m == INFINITY)
		)
	{
		cpArrayPush(space->arbiters, arb);
	}
	else
	{
		cpSpacePopContacts(space, info.count);

		arb->contacts = NULL;
		arb->count = 0;

		// Normally arbiters are set as used after calling the post-solve callback.
		// However, post-solve() callbacks are not called for sensors or arbiters rejected from pre-solve.
		if (arb->state != CP_ARBITER_STATE_IGNORE) arb->state = CP_ARBITER_STATE_NORMAL;
	}

	// Time stamp the arbiter so we know it was used recently.
	arb->stamp = space->stamp;
	return info.id;
}

// Hashset filter func to throw away old arbiters.
cpBool
cpSpaceArbiterSetFilter(cpArbiter* arb, cpSpace* space)
{
	cpTimestamp ticks = space->stamp - arb->stamp;

	cpBody* a = arb->body_a, * b = arb->body_b;

	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happenening.
	if (
		(cpBodyGetType(a) == CP_BODY_TYPE_STATIC || cpBodyIsSleeping(a)) &&
		(cpBodyGetType(b) == CP_BODY_TYPE_STATIC || cpBodyIsSleeping(b))
		)
	{
		return cpTrue;
	}

	// Arbiter was used last frame, but not this one
	if (ticks >= 1 && arb->state != CP_ARBITER_STATE_CACHED)
	{
		arb->state = CP_ARBITER_STATE_CACHED;
		cpCollisionHandler* handler = arb->handler;
		handler->separateFunc(arb, space, handler->userData);
	}

	if (ticks >= space->collisionPersistence)
	{
		arb->contacts = NULL;
		arb->count = 0;

		cpArrayPush(space->pooledArbiters, arb);
		return cpFalse;
	}

	return cpTrue;
}

//MARK: All Important cpSpaceStep() Function

void
cpShapeUpdateFunc(cpShape* shape, void* unused)
{
	cpShapeCacheBB(shape);
}

void
cpSpaceStep(cpSpace* space, cpFloat dt)
{
	// don't step if the timestep is 0!
	if (dt == 0.0f) return;

	space->stamp++;
	cpTimestamp stamp = space->stamp;

	cpFloat prev_dt = space->curr_dt;
	space->curr_dt = dt;

	cpArray* bodies = space->dynamicBodies;
	cpArray* constraints = space->constraints;
	cpArray* arbiters = space->arbiters;

	// Reset and empty the arbiter lists.
	for (int i = 0; i < arbiters->num; i++)
	{
		cpArbiter* arb = (cpArbiter*)arbiters->arr[i];
		arb->state = CP_ARBITER_STATE_NORMAL;

		// If both bodies are awake, unthread the arbiter from the contact graph.
		if (!cpBodyIsSleeping(arb->body_a) && !cpBodyIsSleeping(arb->body_b))
		{
			cpArbiterUnthread(arb);
		}
	}
	arbiters->num = 0;

	cpSpaceLock(space);
	{
		// Integrate positions
		for (int i = 0; i < bodies->num; i++)
		{
			cpBody* body = (cpBody*)bodies->arr[i];
			cpBodyUpdatePosition(body, dt);
		}

		// Find colliding pairs.
		cpSpacePushFreshContactBuffer(space);
		cpSpatialIndexEach(space->dynamicShapes, (cpSpatialIndexIteratorFunc)cpShapeUpdateFunc, NULL);
		cpSpatialIndexReindexQuery(space->dynamicShapes, (cpSpatialIndexQueryFunc)cpSpaceCollideShapes, space);
	}
	cpSpaceUnlock(space, cpFalse);

	// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
	cpSpaceProcessComponents(space, dt);

	cpSpaceLock(space);
	{
		// Clear out old cached arbiters and call separate callbacks
		cpHashSetFilter(space->cachedArbiters, (cpHashSetFilterFunc)cpSpaceArbiterSetFilter, space);

		// Prestep the arbiters and constraints.
		cpFloat slop = space->collisionSlop;
		cpFloat biasCoef = 1.0f - cpfpow(space->collisionBias, dt);
		for (int i = 0; i < arbiters->num; i++)
		{
			cpArbiterPreStep((cpArbiter*)arbiters->arr[i], dt, slop, biasCoef);
		}

		for (int i = 0; i < constraints->num; i++)
		{
			cpConstraint* constraint = (cpConstraint*)constraints->arr[i];

			cpConstraintPreSolveFunc preSolve = constraint->preSolve;
			if (preSolve) preSolve(constraint, space);

			constraint->klass->preStep(constraint, dt);
		}

		// Integrate velocities.
		cpFloat damping = cpfpow(space->damping, dt);
		cpFloat damping_w = cpfpow(space->damping_w, dt);

		cpVect gravity = space->gravity;
		for (int i = 0; i < bodies->num; i++)
		{
			cpBody* body = (cpBody*)bodies->arr[i];
			cpBodyUpdateVelocity(body, gravity, damping, damping_w, dt);

			if (body->impact.dirty)
			{
				if (space->impactFunc != NULL) space->impactFunc(body, space, space->userData);

				body->impact.p = cpvzero;
				body->impact.n = cpvzero;

				body->impact.size = 0.00f;
				body->impact.ke = 0.00f;
				body->impact.bounce = 0.00f;
				body->impact.bounce_rigid = 0.00f;
				body->impact.count = 0;

				body->impact.dirty = 0;
			}
		}

		// Apply cached impulses
		cpFloat dt_coef = (prev_dt == 0.0f ? 0.0f : dt / prev_dt);
		for (int i = 0; i < arbiters->num; i++)
		{
			cpArbiterApplyCachedImpulse((cpArbiter*)arbiters->arr[i], dt_coef);
		}

		for (int i = 0; i < constraints->num; i++)
		{
			cpConstraint* constraint = (cpConstraint*)constraints->arr[i];
			constraint->klass->applyCachedImpulse(constraint, dt_coef);
		}

		// Run the impulse solver.
		for (int i = 0; i < space->iterations; i++)
		{
			for (int j = 0; j < arbiters->num; j++)
			{
				cpArbiterApplyImpulse((cpArbiter*)arbiters->arr[j]);
			}

			for (int j = 0; j < constraints->num; j++)
			{
				cpConstraint* constraint = (cpConstraint*)constraints->arr[j];
				constraint->klass->applyImpulse(constraint, dt);
			}
		}

		// Run the constraint post-solve callbacks
		for (int i = 0; i < constraints->num; i++)
		{
			cpConstraint* constraint = (cpConstraint*)constraints->arr[i];

			cpConstraintPostSolveFunc postSolve = constraint->postSolve;
			if (postSolve) postSolve(constraint, space);
		}

		// run the post-solve callbacks
		for (int i = 0; i < arbiters->num; i++)
		{
			cpArbiter* arb = (cpArbiter*)arbiters->arr[i];

			cpCollisionHandler* handler = arb->handler;
			handler->postSolveFunc(arb, space, handler->userData);

			if (arb->dirty)
			{
				int offset = arb->offset;
				int total_count = arb->count;
				int count = total_count - offset;

				if (count > 0)
				{
					cpFloat eCoef = (1 - arb->e) / (1 + arb->e);
					cpFloat sum = 0.00f;
					cpFloat bounce = 0.00f;
					cpFloat bounce_rigid = 0.00f;

					cpVect pos = cpvzero;

					cpBool swapped = arb->swapped;
					cpVect n = swapped ? cpvneg(arb->n) : arb->n;

					struct cpContact* contacts = arb->contacts;
					for (int i = offset; i < total_count; i++)
					{
						struct cpContact* con = &contacts[i];
						cpFloat jnAcc = con->jnAcc;
						cpFloat jtAcc = con->jtAcc;
						cpVect p1 = cpvadd(arb->body_a->p, arb->contacts[i].r1);
						cpVect p2 = cpvadd(arb->body_b->p, arb->contacts[i].r2);

						sum += eCoef * jnAcc * jnAcc / con->nMass + jtAcc * jtAcc / con->tMass;
						bounce += con->bounce;
						bounce_rigid += cpfabs(con->bounce_rigid);
						pos = cpvadd(pos, cpvmult(cpvadd(p1, p2), 0.50f));
					}

					//cpVect rv = cpvsub(arb->body_a->v, arb->body_b->v);
					pos = cpvmult(pos, 1.00f / count);

					if (arb->body_a->type == CP_BODY_TYPE_DYNAMIC)
					{
						cpImpact* imp = &arb->body_a->impact;
						imp->p = cpvadd(imp->p, pos);
						imp->n = cpvadd(imp->n, n);
						imp->bounce_rigid += bounce_rigid;
						imp->count += count;

						if (imp->dirty)
						{
							imp->p = cpvmult(imp->p, 0.50f);
							imp->n = cpvmult(imp->n, 0.50f);
							imp->bounce_rigid *= 0.50f;
						}
						else
						{
							imp->material_type_a = arb->a->material_type;
							imp->material_type_b = arb->b->material_type;

							imp->body_type_a = arb->body_a->type;
							imp->body_type_b = arb->body_b->type;
						}

						imp->bounce += bounce;
						imp->ke += sum;
						imp->dirty = 1;
						imp->stamp = stamp;
					}

					if (arb->body_b->type == CP_BODY_TYPE_DYNAMIC)
					{
						cpImpact* imp = &arb->body_b->impact;
						imp->p = cpvadd(imp->p, pos);
						imp->n = cpvadd(imp->n, cpvneg(n));
						imp->bounce_rigid += bounce_rigid;
						imp->count += count;

						if (imp->dirty)
						{
							imp->p = cpvmult(imp->p, 0.50f);
							imp->n = cpvmult(imp->n, 0.50f);
							imp->bounce_rigid *= 0.50f;
						}
						else
						{
							imp->material_type_a = arb->b->material_type;
							imp->material_type_b = arb->a->material_type;

							imp->body_type_a = arb->body_b->type;
							imp->body_type_b = arb->body_a->type;
						}

						imp->bounce += bounce;
						imp->ke += sum;
						imp->dirty = 1;
						imp->stamp = stamp;
					}
				}
			}
		}
	}
	cpSpaceUnlock(space, cpTrue);
}
