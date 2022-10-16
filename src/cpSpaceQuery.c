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

 //MARK: Nearest Point Query Functions

typedef struct PointQueryMeta PointQueryMeta;
typedef struct SegmentQueryMeta SegmentQueryMeta;
typedef struct ShapeQueryMeta ShapeQueryMeta;
typedef struct BBQueryMeta BBQueryMeta;


struct PointQueryContext
{
	cpVect point;
	cpFloat maxDistance;
	cpShapeFilter filter;
	cpSpacePointQueryFunc func;
};

static cpCollisionID NearestPointQuery(struct PointQueryContext* context, cpShape* shape, cpCollisionID id, void* data)
{
	if (!cpShapeFilterReject(shape->filter, context->filter))
	{
		cpPointQueryInfo info;
		cpShapePointQuery(shape, context->point, &info);

		if (info.shape && info.distance < context->maxDistance) context->func(shape, info.point, info.distance, info.gradient, data);
	}

	return id;
}

void cpSpacePointQuery(cpSpace* space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpSpacePointQueryFunc func, void* data)
{
	struct PointQueryContext context = { point, maxDistance, filter, func };
	cpBB bb = cpBBNewForCircle(point, cpfmax(maxDistance, 0.0f));

	cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQuery, data);
	cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQuery, data);
}

static cpCollisionID NearestPointQueryNearest(struct PointQueryContext* context, cpShape* shape, cpCollisionID id, cpPointQueryInfo* out)
{
	if (!cpShapeFilterReject(shape->filter, context->filter) && !shape->sensor)
	{
		cpPointQueryInfo info;
		cpShapePointQuery(shape, context->point, &info);

		if (info.distance < out->distance) (*out) = info;
	}

	return id;
}

cpShape*
cpSpacePointQueryNearest(cpSpace* space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo* out)
{
	cpPointQueryInfo info = { NULL, cpvzero, maxDistance, cpvzero };
	if (out)
	{
		(*out) = info;
	}
	else
	{
		out = &info;
	}

	struct PointQueryContext context = {
		point, maxDistance,
		filter,
		NULL
	};

	cpBB bb = cpBBNewForCircle(point, cpfmax(maxDistance, 0.0f));
	cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQueryNearest, out);
	cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQueryNearest, out);

	return (cpShape*)out->shape;
}

//MARK: Segment Query Functions

struct SegmentQueryContext
{
	cpVect start, end;
	cpFloat radius;
	cpShapeFilter filter;
	cpSpaceSegmentQueryFunc func;
};

static cpFloat
SegmentQuery(struct SegmentQueryContext* context, cpShape* shape, void* data)
{
	cpSegmentQueryInfo info;

	if (!cpShapeFilterReject(shape->filter, context->filter) && cpShapeSegmentQuery(shape, context->start, context->end, context->radius, &info))
	{
		context->func(shape, info.point, info.normal, info.alpha, data);
	}

	return 1.0f;
}

void
cpSpaceSegmentQuery(cpSpace* space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSpaceSegmentQueryFunc func, void* data)
{
	struct SegmentQueryContext context =
	{
		start,
		end,
		radius,
		filter,
		func,
	};

	//cpSpaceLock(space);
	//{
	cpSpatialIndexSegmentQuery(space->staticShapes, &context, start, end, radius, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery, data);
	cpSpatialIndexSegmentQuery(space->dynamicShapes, &context, start, end, radius, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery, data);
	//} 
	//cpSpaceUnlock(space, cpTrue);
}

static cpFloat
SegmentQueryFirst(struct SegmentQueryContext* context, cpShape* shape, cpSegmentQueryInfo* out)
{
	cpSegmentQueryInfo info;

	if (!cpShapeFilterReject(shape->filter, context->filter) && !shape->sensor && cpShapeSegmentQuery(shape, context->start, context->end, context->radius, &info) && info.alpha < out->alpha)
	{
		(*out) = info;
	}

	return out->alpha;
}

cpShape*
cpSpaceSegmentQueryFirst(cpSpace* space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSegmentQueryInfo* out)
{
	cpSegmentQueryInfo info = { NULL, end, cpvzero, 1.0f };
	if (out)
	{
		(*out) = info;
	}
	else
	{
		out = &info;
	}

	struct SegmentQueryContext context =
	{
		start,
		end,
		radius,
		filter,
		NULL
	};

	cpSpatialIndexSegmentQuery(space->staticShapes, &context, start, end, radius, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQueryFirst, out);
	cpSpatialIndexSegmentQuery(space->dynamicShapes, &context, start, end, radius, out->alpha, (cpSpatialIndexSegmentQueryFunc)SegmentQueryFirst, out);

	return (cpShape*)out->shape;
}

//MARK: BB Query Functions

struct BBQueryContext
{
	cpBB bb;
	cpShapeFilter filter;
	cpSpaceBBQueryFunc func;
};

static cpCollisionID
BBQuery(struct BBQueryContext* context, cpShape* shape, cpCollisionID id, void* data)
{
	if (!cpShapeFilterReject(shape->filter, context->filter) && cpBBIntersects(context->bb, shape->bb))
	{
		context->func(shape, data);
	}

	return id;
}

void
cpSpaceBBQuery(cpSpace* space, cpBB bb, cpShapeFilter filter, cpSpaceBBQueryFunc func, void* data)
{
	struct BBQueryContext context = { bb, filter, func };

	//cpSpaceLock(space);
	//{
	cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
	cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
	//} 
	//cpSpaceUnlock(space, cpTrue);
}

//MARK: Shape Query Functions

struct ShapeQueryContext
{
	cpSpaceShapeQueryFunc func;
	void* data;
	cpBool anyCollision;
};

struct ShapeQueryContext2
{
	cpShape* shape;
	cpShapeFilter filter;
};

// Callback from the spatial hash.
static cpCollisionID
ShapeQuery(cpShape* a, cpShape* b, cpCollisionID id, struct ShapeQueryContext* context)
{
	if (cpShapeFilterReject(a->filter, b->filter) || a == b) return id;

	cpContactPointSet set = cpShapesCollide(a, b);
	if (set.count)
	{
		if (context->func) context->func(b, &set, context->data);
		context->anyCollision = !(a->sensor || b->sensor);
	}

	return id;
}

cpBool
cpSpaceShapeQuery(cpSpace* space, cpShape* shape, cpSpaceShapeQueryFunc func, void* data)
{
	cpBody* body = shape->body;
	cpBB bb = (body ? cpShapeUpdate(shape, body->transform) : shape->bb);
	struct ShapeQueryContext context = { func, data, cpFalse };

	//cpSpaceLock(space);
	//{
	cpSpatialIndexQuery(space->dynamicShapes, shape, bb, (cpSpatialIndexQueryFunc)ShapeQuery, &context);
	cpSpatialIndexQuery(space->staticShapes, shape, bb, (cpSpatialIndexQueryFunc)ShapeQuery, &context);
	//} 
	//cpSpaceUnlock(space, cpTrue);

	return context.anyCollision;
}

struct PointQueryMeta
{
	cpPointQueryInfo* results;
	int count;
	int max_count;
};

static cpCollisionID
PointQuery2(struct PointQueryContext* context, cpShape* shape, cpCollisionID id, PointQueryMeta* meta)
{
	if (meta->count < meta->max_count && !cpShapeFilterReject(shape->filter, context->filter))
	{
		cpPointQueryInfo info;
		cpShapePointQuery(shape, context->point, &info);

		if (info.shape && info.distance < context->maxDistance)
		{
			meta->results[meta->count] = info;
			meta->count++;
		}
	}

	return id;
}

size_t
cpSpacePointQuery2(cpSpace* space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo* results, enum cpQueryFlags flags, int max_count)
{
	struct PointQueryContext context =
	{
		point,
		maxDistance,
		filter,
		NULL
	};

	struct PointQueryMeta meta =
	{
		results,
		0,
		max_count
	};

	cpBB bb = cpBBNewForCircle(point, cpfmax(maxDistance, 0.0f));

	if ((flags & QUERY_DYNAMIC) && meta.count < meta.max_count) cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)PointQuery2, &meta);
	if ((flags & QUERY_STATIC) && meta.count < meta.max_count) cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)PointQuery2, &meta);

	return meta.count;
}

struct SegmentQueryMeta
{
	cpSegmentQueryInfo* results;
	int count;
	int max_count;
};

static cpFloat
SegmentQuery2(struct SegmentQueryContext* context, cpShape* shape, SegmentQueryMeta* meta)
{
	cpSegmentQueryInfo info;

	if (meta->count < meta->max_count && !cpShapeFilterReject(shape->filter, context->filter) && cpShapeSegmentQuery(shape, context->start, context->end, context->radius, &info))
	{
		meta->results[meta->count] = info;
		meta->count++;
	}

	return 1.0f;
}

size_t
cpSpaceSegmentQuery2(cpSpace* space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSegmentQueryInfo* results, enum cpQueryFlags flags, int max_count)
{
	struct SegmentQueryContext context =
	{
		start,
		end,
		radius,
		filter,
		NULL,
	};

	struct SegmentQueryMeta meta =
	{
		results,
		0,
		max_count
	};

	if ((flags & QUERY_DYNAMIC) && meta.count < meta.max_count) cpSpatialIndexSegmentQuery(space->dynamicShapes, &context, start, end, radius, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery2, &meta);
	if ((flags & QUERY_STATIC) && meta.count < meta.max_count) cpSpatialIndexSegmentQuery(space->staticShapes, &context, start, end, radius, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery2, &meta);

	return meta.count;
}

struct ShapeQueryMeta
{
	cpShapeQueryInfo* results;
	int count;
	int max_count;
};

// Callback from the spatial hash.
static cpCollisionID
ShapeQuery2(struct ShapeQueryContext2* context, cpShape* shape, cpCollisionID id, ShapeQueryMeta* meta)
{
	if (meta->count < meta->max_count && shape != context->shape && !cpShapeFilterReject(shape->filter, context->filter))
	{
		cpContactPointSet set = cpShapesCollide(shape, context->shape);
		if (set.count)
		{
			meta->results[meta->count] = cpShapeQueryInfo
			{
				shape
			};
			meta->count++;
		}
	}

	return id;

	//if (cpShapeFilterReject(a->filter, b->filter) || a == b) return id;

	//cpContactPointSet set = cpShapesCollide(a, b);
	//if (set.count)
	//{
	//	if (context->func) context->func(b, &set, context->data);
	//	context->anyCollision = !(a->sensor || b->sensor);
	//}

	//return id;
}

size_t
cpSpaceShapeQuery2(cpSpace* space, cpShape* shape, cpShapeFilter filter, cpShapeQueryInfo* results, enum cpQueryFlags flags, int max_count)
{
	cpBody* body = shape->body;
	cpBB bb = (body ? cpShapeUpdate(shape, body->transform) : shape->bb);

	struct ShapeQueryContext2 context = 
	{ 
		shape,
		filter
	};

	struct ShapeQueryMeta meta =
	{
		results,
		0,
		max_count
	};

	if ((flags & QUERY_DYNAMIC) && meta.count < meta.max_count) cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)ShapeQuery2, &meta);
	if ((flags & QUERY_STATIC) && meta.count < meta.max_count) cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)ShapeQuery2, &meta);

	return meta.count;
}


struct BBQueryMeta
{
	cpBBQueryInfo* results;
	int count;
	int max_count;
};

static cpBool
BBQuery2(struct BBQueryContext* context, cpShape* shape, BBQueryMeta* meta)
{
	//if (cpBBIntersects(context->bb, shape->bb) && !cpShapeFilterReject(shape->filter, context->filter))
	if (meta->count < meta->max_count && cpBBIntersects(context->bb, shape->bb) && !cpShapeFilterReject(shape->filter, context->filter))
	{
		meta->results[meta->count++] = cpBBQueryInfo 
		{ 
			shape 
		};
	}

	return meta->count < meta->max_count;
}

size_t
cpSpaceBBQuery2(cpSpace* space, cpBB bb, cpShapeFilter filter, cpBBQueryInfo* results, enum cpQueryFlags flags, int max_count)
{
	struct BBQueryContext context =
	{
		bb,
		filter,
		NULL,
	};

	struct BBQueryMeta meta =
	{
		results,
		0,
		max_count
	};

	if ((flags & QUERY_DYNAMIC) && meta.count < meta.max_count) cpSpatialIndexBBQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexBBQueryFunc)BBQuery2, &meta);
	if ((flags & QUERY_STATIC) && meta.count < meta.max_count) cpSpatialIndexBBQuery(space->staticShapes, &context, bb, (cpSpatialIndexBBQueryFunc)BBQuery2, &meta);

	return meta.count;
}

