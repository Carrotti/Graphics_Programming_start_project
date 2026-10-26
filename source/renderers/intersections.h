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
		const float n_dot_v{ Vector3::Dot(triangle.normal, ray.direction) };

		if (std::abs(n_dot_v) < std::numeric_limits<float>::epsilon()) return false; //parallel
		if (triangle.cull_mode == CullMode::kBackFaceCulling && n_dot_v > 0) return false;
		if (triangle.cull_mode == CullMode::kFrontFaceCulling && n_dot_v < 0) return false;


		//möller trumbone
		const Vector3 e1{ triangle.v1 - triangle.v0 };
		const Vector3 e2{ triangle.v2 - triangle.v0 };

		const Vector3 h{ Vector3::Cross(ray.direction , e2) };
		const float det{ Vector3::Dot(h, e1) };
		if (std::abs(det) < std::numeric_limits<float>::epsilon()) return false;
		const float inv_det{ 1.f / det };
		const Vector3 s{ ray.origin - triangle.v0 };

		const float u{ Vector3::Dot(s, h) * inv_det };
		if (u < 0 || u > 1) return false;

		const Vector3 q{ Vector3::Cross(s, e1) };
		const float v{ Vector3::Dot(ray.direction, q) * inv_det};
		if (v < 0 || (u + v) > 1) return false;

		const float t{ Vector3::Dot(e2,q) * inv_det };
		if (t < ray.min || t > ray.max) return false;

		if (!ignore_hit_record)
		{
			hit_record.t = t;
			hit_record.ray = ray;
			hit_record.barycentric_coordinates = { u,v };
		}
		return true;


		//first method
/*		
		const float t{ Vector3::Dot(triangle.v0 - ray.origin, triangle.normal) / n_dot_v };

		if (t < ray.min || t > ray.max) return false;
		const Vector3 p{ ray.origin + ray.direction * t };

		const std::array<const Vector3*, 3> vertices{ &triangle.v0, &triangle.v1, &triangle.v2 };
		std::array<float, 3> signed_areas_triangle{};
		for (size_t idx{ 0 }; idx < vertices.size(); ++idx)
		{
			const int next_idx{ static_cast<int>((idx + 1) % vertices.size()) };
			const Vector3 e{ *vertices[next_idx] - *vertices[idx] };
			const Vector3 vp{ p - *vertices[idx] };
			const Vector3 e_cross_p{ Vector3::Cross(e, vp) };
			if (Vector3::Dot(e_cross_p, triangle.normal) < 0) return false;
			if (!ignore_hit_record) signed_areas_triangle[static_cast<int>((idx + 2) % vertices.size())] = (e_cross_p.Magnitude() / 2);
		}

		if (!ignore_hit_record)
		{
			hit_record.ray = ray;
			hit_record.t = t;
			//for barycentric coords, storing u (belong to v1), and v (belong to v2)
			const float area{ Vector3::Cross(triangle.v0, triangle.v1).Magnitude() / 2 };
			hit_record.barycentric_coordinates = Vector2{ signed_areas_triangle[1] / area, signed_areas_triangle[2] / area };
		}
		return true*/
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