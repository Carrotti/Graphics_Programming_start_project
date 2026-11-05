//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#include <primitives.h>
#include <vector3.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <ranges>
#include <numeric>
#include <utility>

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
	if (primitive_topology == PrimitiveTopology::kTriangleStrip)
	{
		std::cout << "idk this is a strip in TriangleMesh::UpdateAABB\n";
		return;
	}

	const uint32_t nr_triangles{ static_cast<uint32_t>(indices.size()) / 3 };
	//building arrays
	bvh_tri_indices.clear();
	bvh_tri_indices.resize(nr_triangles);
	std::iota(bvh_tri_indices.begin(), bvh_tri_indices.end(), 0);
	std::vector<Vector3> centroid_array(nr_triangles);
	std::vector<AABB> AABB_array(nr_triangles);
	//computing AABBs and centroids
	for (uint32_t idx{}; idx < nr_triangles; ++idx)
	{
		const Vector3 v0{ vertices[indices[3 * idx]].position };
		const Vector3 v1{ vertices[indices[3 * idx + 1]].position };
		const Vector3 v2{ vertices[indices[3 * idx + 2]].position };
		centroid_array.at(idx) = (v0 + v1 + v2) / 3.f;
		AABB_array.at(idx).ExpandToInclude(v0);
		AABB_array.at(idx).ExpandToInclude(v1);
		AABB_array.at(idx).ExpandToInclude(v2);
		bounding_box.value().ExpandToInclude(AABB_array.at(idx));
	}
	const uint32_t leaf_count = (nr_triangles + 3) / 4;
	bvh_nodes.clear();
	bvh_nodes.reserve(leaf_count * 2 - 1);
	bvh_nodes.emplace_back();
	CreateBVHNode(0, 0, nr_triangles, AABB_array, centroid_array);
	return;
}

void TriangleMesh::CreateBVHNode(
	uint32_t node_idx, uint32_t first_tri, uint32_t tri_count,
	const std::vector<AABB>& AABB_array,
	const std::vector<Vector3>& centroid_array)
{
	BVHNode node{};

	for (uint32_t bvh_idx{ first_tri }; bvh_idx < first_tri + tri_count; ++bvh_idx)
	{
		node.bounds.ExpandToInclude(AABB_array[bvh_tri_indices[bvh_idx]]);
	}

	if (tri_count <= 4)
	{
		//Create Leaf node
		node.first_tri = first_tri;
		node.tri_count = tri_count;

		bvh_nodes[node_idx] = node;
	}
	else
	{
		//Create internal node
		const uint32_t left_child_idx{ static_cast<uint32_t>(bvh_nodes.size()) };
		node.left_child = left_child_idx;
		bvh_nodes[node_idx] = node;

		const Vector3 AABB_size{ node.bounds.GetSize() };
		uint8_t axis{};

		//x-axis greatest
		if (AABB_size.x > AABB_size.y && AABB_size.x > AABB_size.z)
			axis = 0;
		//y-axis greatest
		else if (AABB_size.y > AABB_size.z)
			axis = 1;
		//z-axis greatest
		else
			axis = 2;

		//get midpoint
		float midpoint{};
		for (uint32_t bvh_idx{ first_tri }; bvh_idx < first_tri + tri_count; ++bvh_idx)
		{
			uint32_t tri_idx{ bvh_tri_indices[bvh_idx] };
			midpoint += centroid_array[tri_idx][axis];
		}
		midpoint /= tri_count;



		uint32_t left_count{};

		for (uint32_t bvh_idx{ first_tri }; bvh_idx < first_tri + tri_count; ++bvh_idx)
		{
			uint32_t tri_idx{ bvh_tri_indices[bvh_idx] };
			if (centroid_array[tri_idx][axis] < midpoint)
			{
				std::swap(bvh_tri_indices[bvh_idx], bvh_tri_indices[first_tri + left_count]);
				++left_count;
			}
		}
		bvh_nodes.emplace_back();
		bvh_nodes.emplace_back();

		CreateBVHNode(left_child_idx, first_tri, left_count, AABB_array, centroid_array);
		CreateBVHNode(left_child_idx + 1, first_tri + left_count, tri_count - left_count, AABB_array, centroid_array);
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