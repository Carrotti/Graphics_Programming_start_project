//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#ifndef INTERSECTIONS_HEADER
#define INTERSECTIONS_HEADER

//--- Standard Includes ---
#include <cassert>

//--- Framework Includes ---
#include <primitives.h>
#include <matrix.h>

namespace gfx
{
	//--- Intersection Tests ---
	[[maybe_unused]]
	static bool HitTestSphere(const Sphere& sphere, const Ray& ray,
		RayHitRecord& hit_record, const bool ignore_hit_record = false)
	{

		//using geometric solution
		const Vector3 L{ sphere.origin - ray.origin };
		const float t_ca{ Vector3::Dot(L, ray.direction) };
		const float od2{ Vector3::Dot(L,L) - t_ca * t_ca };
		const float t_hc{ sqrt(sphere.radius * sphere.radius - od2) };
		const float t0{ t_ca - t_hc };
		const float t1{ t_ca + t_hc };
		const Vector3 p{ ray.origin + t0 * ray.direction }; //p is intersection point

		if (ignore_hit_record) return false;//this is probably wrong

		if (t0 > ray.min && t0 < ray.max)
		{
			hit_record.t = t0;
			hit_record.ray = ray;
			return true;
		}
		else if (t1 > ray.min && t1 < ray.max)
		{
			hit_record.t = t1;
			hit_record.ray = ray;
			return true;
		}
		return false;
	}

	[[maybe_unused]]
	static bool HitTestPlane(const Plane& plane, const Ray& ray,
		RayHitRecord& hit_record, const bool ignore_hit_record = false)
	{
		(void)ignore_hit_record;
		
		const float t{ Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal) };
		

		if (t < ray.min || t > ray.max) return false; // if the plane is behind
		if (!plane.half_extent.has_value()) // if we have an infinite plane
		{
			hit_record.ray = ray;
			hit_record.t = t;
			return true;
		}
		const Vector2& half_extent{ plane.half_extent.value() };
		const Vector3 p{ ray.origin + ray.direction * t }; //intersection point
		const Vector3 op{ p - plane.origin }; //origin to intersect
		const Vector3 b{ Vector3::Cross(plane.normal, plane.tangent) };//accounting for rotation
		const float t2{ Vector3::Dot(op,plane.tangent) }; //horizontal distance intersection to origin
		const float b2{ Vector3::Dot(op,b) }; //vertical distance intersection to origin

		
		if (abs(t2) <= half_extent.x && abs(b2) <= half_extent.y)
		{
			hit_record.ray = ray;
			hit_record.t = t;
			return true;
		}
		return false;

	}

	[[maybe_unused]]
	static bool HitTestTriangle(const Triangle& triangle, const Ray& ray,
		RayHitRecord& hit_record, const bool ignore_hit_record = false)
	{
		//TODO
		assert(false && "Not Implemented");
		(void)triangle; (void)ray; (void)hit_record; (void)ignore_hit_record;
		return false;
	}

	[[maybe_unused]]
	static bool HitTestAABB(const AABB& aabb, const Ray& ray)
	{
		//TODO
		assert(false && "Not Implemented");
		(void)aabb; (void)ray;
		return false;
	}
}
#endif //INTERSECTIONS_HEADER