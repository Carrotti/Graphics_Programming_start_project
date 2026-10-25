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
		if (od2 > sphere.radius * sphere.radius)
			return false;
		const float t_hc{ sqrt(sphere.radius * sphere.radius - od2) };
		const float t{ t_ca - t_hc };
		const Vector3 p{ ray.origin + t * ray.direction }; //p is intersection point

		if (t < ray.min || t > ray.max) return false;
		if (ignore_hit_record) return true;

		hit_record.t = t;
		hit_record.ray = ray;
		return true;
	}

	[[maybe_unused]]
	static bool HitTestPlane(const Plane& plane, const Ray& ray,
		RayHitRecord& hit_record, const bool ignore_hit_record = false)
	{
		const float n_dot_r{ Vector3::Dot(ray.direction, plane.normal) };
		if (std::abs(n_dot_r) < 1e-6f) return false; // ray is parallel to plane

		if (!plane.double_sided && n_dot_r >= 0.f) return false; // backface culling

		const float t{ Vector3::Dot(plane.origin - ray.origin, plane.normal) / n_dot_r };

		if (t < ray.min || t > ray.max) return false; // if the plane is behind

		if (!plane.half_extent.has_value()) // if we have an infinite plane
		{
			if (!ignore_hit_record)
			{
				hit_record.ray = ray;
				hit_record.t = t;
			}
			return true;
		}
		else //finite plane bounds check
		{
			const Vector3 p{ ray.origin + ray.direction * t }; //intersection point
			const Vector3 op{ p - plane.origin }; //origin to intersect
			const Vector3 b{ Vector3::Cross(plane.normal, plane.tangent) };//accounting for rotation
			const float t2{ Vector3::Dot(op,plane.tangent) }; //horizontal distance intersection to origin
			const float b2{ Vector3::Dot(op,b) }; //vertical distance intersection to origin

			const Vector2& half_extent{ plane.half_extent.value() };
			if (abs(t2) > half_extent.x || abs(b2) > half_extent.y) return false;

			if (!ignore_hit_record)
			{
				hit_record.ray = ray;
				hit_record.t = t;
			}
			return true;
		}
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