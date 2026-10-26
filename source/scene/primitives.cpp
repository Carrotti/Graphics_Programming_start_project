//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#include <primitives.h>
#include <vector3.h>
using namespace gfx;

//--- Primitives AABB Updates ---
void Sphere::UpdateAABB()
{
	if (!bounding_box.has_value()) bounding_box.emplace();
	const Vector3 box_half_extent{ radius,radius,radius };
	bounding_box.value().ExpandToInclude(origin + box_half_extent);
	bounding_box.value().ExpandToInclude(origin - box_half_extent);
}
/*
void Plane::UpdateAABB()
{

}
*/

void Triangle::UpdateAABB()
{
	if (!bounding_box.has_value()) bounding_box.emplace();
	bounding_box.value().ExpandToInclude(v0);
	bounding_box.value().ExpandToInclude(v1);
	bounding_box.value().ExpandToInclude(v2);
}

void TriangleMesh::UpdateAABB()
{
	if (!bounding_box.has_value()) bounding_box.emplace();
	for (const Vertex& vertex : vertices)
	{
		bounding_box.value().ExpandToInclude(vertex.position);
	}
}

//--- AABB ---
void AABB::ExpandToInclude(const Vector3& point)
{
	min = Vector3::Min(min, point);
	max = Vector3::Max(max, point);
}

void AABB::ExpandToInclude(const AABB& other)
{
	min = Vector3::Min(min, other.min);
	max = Vector3::Max(max, other.max);
}

bool AABB::Contains(const Vector3& point) const
{
	return point.x >= min.x && point.x <= max.x &&
		point.y >= min.y && point.y <= max.y &&
		point.z >= min.z && point.z <= max.z;
}

bool AABB::Overlaps(const AABB& other) const
{
	return min.x <= other.max.x && max.x >= other.min.x &&
		min.y <= other.max.y && max.y >= other.min.y &&
		min.z <= other.max.z && max.z >= other.min.z;
}

Vector3 AABB::GetCenter() const
{
	return { (min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f, (min.z + max.z) * 0.5f };
}

Vector3 AABB::GetSize() const
{
	return { max.x - min.x, max.y - min.y, max.z - min.z };
}

bool AABB::IsValid() const
{
	return min.x <= max.x && min.y <= max.y && min.z <= max.z;
}