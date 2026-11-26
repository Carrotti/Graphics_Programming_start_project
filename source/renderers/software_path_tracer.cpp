//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#include <software_path_tracer.h>
#include <scenes.h>
#include <algorithm>
#include <execution>
#include <ranges>
#include <intersections.h>
#include <limits>
#include <cstdint>
#include <cstdlib>
#include <numbers>
#include <vector2.h>
#include <samplers.h>

using namespace gfx;

// =============================================================================
// Private Helper Functions
// =============================================================================

namespace 
{
	constexpr float kWireframeLineWidthPx{ 1.5f };

	// Returns true when the hit point lies within half_width (local space) of
	// any triangle edge. Works for Triangle and TriangleMesh primitives.
	bool IsOnTriangleEdge(const Scene* scene, const SceneObject& object,
		const RayHitRecord& hit, const float half_width)
	{
		if (!hit.barycentric_coordinates.has_value()) return false;

		const Primitive* prim{ scene->primitives_factory.Get(object.primitive_index) };
		Vector3 p0{}, p1{}, p2{};
		if (prim->type == PrimitiveType::kTriangleMesh && hit.vertex_indices.has_value())
		{
			const TriangleMesh* mesh{ static_cast<const TriangleMesh*>(prim) };
			const auto& idx{ hit.vertex_indices.value() };
			p0 = mesh->vertices[idx[0]].position;
			p1 = mesh->vertices[idx[1]].position;
			p2 = mesh->vertices[idx[2]].position;
		}
		else if (prim->type == PrimitiveType::kTriangle)
		{
			const Triangle* tri{ static_cast<const Triangle*>(prim) };
			p0 = tri->v0; p1 = tri->v1; p2 = tri->v2;
		}
		else
			return false; // Spheres/planes have no edges.

		const float u{ hit.barycentric_coordinates.value().x }; // weight of v1
		const float v{ hit.barycentric_coordinates.value().y }; // weight of v2
		const float w{ 1.f - u - v };                           // weight of v0

		// Distance from the hit point to the edge opposite a vertex is
		// (barycentric weight) * (triangle height over that edge),
		// and height = 2 * area / edge_length.
		const float two_area{ Vector3::Cross(p1 - p0, p2 - p0).Magnitude() };
		const float len_12{ (p2 - p1).Magnitude() }; // opposite v0
		const float len_20{ (p0 - p2).Magnitude() }; // opposite v1
		const float len_01{ (p1 - p0).Magnitude() }; // opposite v2
		if (len_12 <= 0.f || len_20 <= 0.f || len_01 <= 0.f) return false;

		const float d0{ w * two_area / len_12 };
		const float d1{ u * two_area / len_20 };
		const float d2{ v * two_area / len_01 };
		return std::min({ d0, d1, d2 }) < half_width;
	}

	constexpr float kBoxLineWidthPx{ 1.5f };

	// Unclipped slab test: entry and exit parameters of the ray through the box.
	bool RayBoxInterval(const AABB& box, const Ray& ray, float& t_enter, float& t_exit)
	{
		t_enter = std::numeric_limits<float>::lowest();
		t_exit = std::numeric_limits<float>::max();
		for (uint8_t a = 0; a < 3; ++a)
		{
			if (ray.direction[a] == 0.f)
			{
				if (ray.origin[a] < box.min[a] || ray.origin[a] > box.max[a]) return false;
				continue;
			}
			const float inv{ 1.f / ray.direction[a] };
			float t0{ (box.min[a] - ray.origin[a]) * inv };
			float t1{ (box.max[a] - ray.origin[a]) * inv };
			if (t0 > t1) std::swap(t0, t1);
			t_enter = std::max(t_enter, t0);
			t_exit = std::min(t_exit, t1);
			if (t_enter > t_exit) return false;
		}
		return t_exit > 0.f;
	}

	// A point on the box surface is on an edge when it is within h of the
	// face planes on at least two axes.
	bool IsBoxEdge(const AABB& box, const Vector3& p, const float h)
	{
		int near_faces{ 0 };
		for (uint8_t a = 0; a < 3; ++a)
		{
			if (std::abs(p[a] - box.min[a]) < h || std::abs(p[a] - box.max[a]) < h)
				++near_faces;
		}
		return near_faces >= 2;
	}

	// Tests the front and back crossing of the box; only crossings in front of
	// max_t (the current nearest hit) count. Ray is in the box's own space.
	bool BoxEdgeHit(const AABB& box, const Ray& local_ray, const float max_t,
		const float footprint_per_t, float& hit_t)
	{
		if (!box.IsValid()) return false;
		float t_enter{}, t_exit{};
		if (!RayBoxInterval(box, local_ray, t_enter, t_exit)) return false;

		// The ray param t is shared between spaces; the local direction length
		// converts the world-space pixel footprint to local space.
		const float dir_len{ local_ray.direction.Magnitude() };
		for (const float t : { t_enter, t_exit })
		{
			if (t < local_ray.min || t >= max_t) continue;
			const Vector3 p{ local_ray.origin + local_ray.direction * t };
			const float h{ 0.5f * kBoxLineWidthPx * footprint_per_t * t * dir_len };
			if (IsBoxEdge(box, p, h))
			{
				hit_t = t;
				return true;
			}
		}
		return false;
	}

	ColorRgba BvhDepthColor(const uint32_t depth)
	{
		static const std::array<ColorRgba, 6> palette{
			ColorRgba(1.f, 0.2f, 0.2f), ColorRgba(1.f, 0.6f, 0.1f),
			ColorRgba(0.9f, 0.9f, 0.1f), ColorRgba(0.2f, 0.9f, 0.3f),
			ColorRgba(0.2f, 0.7f, 1.f), ColorRgba(0.8f, 0.3f, 1.f) };
		return palette[depth % palette.size()];
	}

	// Returns true (and the line color) if this pixel's ray crosses a box edge
	// in front of scene_t.
	bool BoundingBoxOverlay(const Scene* scene, const Ray& view_ray, const float scene_t,
		const float footprint_per_t, const BoundingBoxMode mode, ColorRgba& out_color)
	{
		float best_t{ scene_t };
		bool found{ false };

		for (const SceneObject& object : scene->objects)
		{
			const Primitive* prim{ scene->primitives_factory.Get(object.primitive_index) };
			if (!prim || !prim->bounding_box.has_value()) continue;

			const Ray local_ray{ object.instance_transformation.has_value() ?
				object.instance_transformation->TransformRay(view_ray) : view_ray };

			float t{};
			if (BoxEdgeHit(prim->bounding_box.value(), local_ray, best_t, footprint_per_t, t))
			{
				best_t = t;
				out_color = ColorRgba(1.f, 0.85f, 0.1f);
				found = true;
			}

			if (mode == BoundingBoxMode::kObjects || prim->type != PrimitiveType::kTriangleMesh)
				continue;
			const TriangleMesh* mesh{ static_cast<const TriangleMesh*>(prim) };
			if (mesh->bvh_nodes.empty()) continue;

			// Iterative BVH traversal (node index, depth), pruned by the current nearest hit.
			std::array<std::pair<uint32_t, uint32_t>, 64> stack{};
			int sp{ 0 };
			stack[sp++] = { 0u, 0u };
			while (sp > 0)
			{
				const auto [node_idx, depth] = stack[--sp];
				const BVHNode& node{ mesh->bvh_nodes[node_idx] };

				Ray cull_ray{ local_ray };
				cull_ray.max = best_t;
				if (!HitTestAABB(node.bounds, cull_ray)) continue;

				const bool leaf{ node.IsLeaf() };
				if (leaf || mode == BoundingBoxMode::kBvhAll)
				{
					if (BoxEdgeHit(node.bounds, local_ray, best_t, footprint_per_t, t))
					{
						best_t = t;
						out_color = leaf ? ColorRgba(0.1f, 0.9f, 1.f) : BvhDepthColor(depth);
						found = true;
					}
				}
				if (!leaf && sp + 2 <= static_cast<int>(stack.size()))
				{
					stack[sp++] = { node.left_child, depth + 1 };
					stack[sp++] = { node.left_child + 1, depth + 1 };
				}
			}
		}
		return found;
	}
}

float SoftwarePathTracer::CalcCosine(const Vector3& normal, const Vector3& light_dir)
{
	return std::max(0.f, Vector3::Dot(normal, light_dir));
}

ColorRgba SoftwarePathTracer::CalcBrdf(const Material* material, const ShadingInput& input,
	const SurfaceProperties& surface)
{
	return material->Shade(input, surface);
}

ColorRgba SoftwarePathTracer::CalcRadiance(const Light* light, const ShadingInput& shading_input)
{
	//radiance - calculate using the light parameters directly
	switch (light->type)
	{
	case LightType::kPoint:
		return light->color * light->intensity / (light->origin - shading_input.world_position).SqrMagnitude();
	case LightType::kDirectional:
		return light->color * light->intensity;
	default:
		return ColorRgba{};
	}
}

ShadingInput SoftwarePathTracer::ConstructShadingInput(const RayHitRecord& hit) const
{
	const Scene* scene{ context_->scene_manager->GetActiveScene() };
	ShadingInput shading_input{};
	const SceneObject& object{ scene->objects[hit.object_index] };
	const Ray& ray{ hit.ray };
	shading_input.view_direction = -ray.direction;
	shading_input.world_position = ray.origin + ray.direction * hit.t;


	const Primitive* prim{ scene->primitives_factory.Get(object.primitive_index) };
	switch (prim->type)
	{
	case PrimitiveType::kSphere:
	{
		const Sphere* sphere{ scene->primitives_factory.GetAs<Sphere>(object.primitive_index) };
		shading_input.world_normal = (shading_input.world_position - sphere->origin).Normalized();
		shading_input.shadow_origin = shading_input.world_position;
		break;
	}
	case PrimitiveType::kPlane:
	{
		const Plane* plane{ scene->primitives_factory.GetAs<Plane>(object.primitive_index) };
		shading_input.world_normal = plane->normal;
		shading_input.shadow_origin = shading_input.world_position;
		break;
	}
	case PrimitiveType::kTriangle:
	{
		const Triangle* triangle{ scene->primitives_factory.GetAs<Triangle>(object.primitive_index) };
		shading_input.world_normal = triangle->normal;
		shading_input.shadow_origin = shading_input.world_position;/*
		if (Vector3::Dot(shading_input.world_normal, -ray.direction) < 0.f)
			shading_input.world_normal = -shading_input.world_normal;*/
		break;
	}
	case PrimitiveType::kTriangleMesh:
	{
		const TriangleMesh* triangle_mesh{ scene->primitives_factory.GetAs<TriangleMesh>(object.primitive_index) };
		
		const Vertex v0{ triangle_mesh->vertices[hit.vertex_indices.value()[0]] };
		const Vertex v1{ triangle_mesh->vertices[hit.vertex_indices.value()[1]] };
		const Vertex v2{ triangle_mesh->vertices[hit.vertex_indices.value()[2]] };
		
		Vector3 face_normal{ Vector3::Cross(v1.position - v0.position,v2.position - v0.position).Normalized() };
		if (Vector3::Dot(face_normal, -ray.direction) < 0.f) face_normal = -face_normal;
		
		const Vector3 n0{ v0.normal.value() };
		const Vector3 n1{ v1.normal.value() };
		const Vector3 n2{ v2.normal.value() };
		const float u{ hit.barycentric_coordinates.value()[0] };
		const float v{ hit.barycentric_coordinates.value()[1] };
		const float w{ 1 - u - v };

		shading_input.world_normal = (w * n0 + u * n1 + v * n2).Normalized();
		if (Vector3::Dot(shading_input.world_normal, face_normal) < 0.f)
			shading_input.world_normal = -shading_input.world_normal;

		if (v0.bitangent.has_value() && v1.bitangent.has_value() && v2.bitangent.has_value()
			&& v0.tangent.has_value() && v1.tangent.has_value() && v2.tangent.has_value())
		{
			shading_input.world_bitangent = (w * v0.bitangent.value() + u * v1.bitangent.value() + v * v2.bitangent.value()).Normalized();
			shading_input.world_tangent = (w * v0.tangent.value() + u * v1.tangent.value() + v * v2.tangent.value()).Normalized();
		}

		if (v0.texture_coordinates.has_value() && v1.texture_coordinates.has_value() && v2.texture_coordinates.has_value())
		{
			const Vector2 t0{ v0.texture_coordinates.value() };
			const Vector2 t1{ v1.texture_coordinates.value() };
			const Vector2 t2{ v2.texture_coordinates.value() };
			shading_input.texture_coordinates.emplace(t0.x * w + t1.x * u + t2.x * v, t0.y * w + t1.y * u + t2.y * v);
		}

		//shadow
		Vector3 tmpu{ shading_input.world_position - v0.position };
		Vector3 tmpv{ shading_input.world_position - v1.position };
		Vector3 tmpw{ shading_input.world_position - v2.position };
		const float dotu{ std::min(0.f, Vector3::Dot(tmpu, n0)) };
		const float dotv{ std::min(0.f, Vector3::Dot(tmpv, n1)) };
		const float dotw{ std::min(0.f, Vector3::Dot(tmpw, n2)) };
		tmpu -= dotu * n0;
		tmpv -= dotv * n1;
		tmpw -= dotw * n2;
		shading_input.shadow_origin = shading_input.world_position + w * tmpu + u * tmpv + v * tmpw + face_normal * 0.001f;
		break;
	}
	default:
		//unhandled case
		break;
	}

	if (object.instance_transformation.has_value())
	{
		const Matrix& object_transform{ object.instance_transformation.value() };
		shading_input.world_position = object_transform.TransformPoint(shading_input.world_position);
		shading_input.world_normal = object_transform.TransformNormal(shading_input.world_normal);
		shading_input.shadow_origin = object_transform.TransformPoint(shading_input.shadow_origin);
		shading_input.view_direction = object_transform.TransformVector(shading_input.view_direction).Normalized();
	}
	return shading_input;
}

ColorRgba gfx::SoftwarePathTracer::DirectIllumination(const ShadingInput& shading_input,
	const SurfaceProperties& surface, const Material* material,
	const Scene* scene, uint32_t& hash_state) const
{
	ColorRgba result{ ColorRgba::Black() };
	ShadingInput input{ shading_input };

	for (const Light* light : scene->lights_factory.GetAll())
	{
		Vector3 light_dir{ light->type == LightType::kDirectional ?
			-light->direction :
			(light->origin - input.shadow_origin) };
		float light_dist{ light_dir.Normalize() };
		input.light_direction = light_dir;

		const Ray shadow_ray{
			input.shadow_origin,
			light_dir,
			1e-3f,
			light->type == LightType::kDirectional ? std::numeric_limits<float>::max() : light_dist
		};

		const ColorRgba shadow_transmittance{ ShadowTransmittance(shadow_ray, scene, hash_state) };
		if (shadow_transmittance.Luminance() < 0.001f) continue;
		const float cosine{ CalcCosine(surface.shading_normal, light_dir) };
		if (cosine <= 0.f) continue;
		const ColorRgba radiance{ CalcRadiance(light, input) };
		const ColorRgba brdf{ CalcBrdf(material, input, surface) };


		result += shadow_transmittance * radiance * brdf * cosine;
	}

	return result;
}

ColorRgba gfx::SoftwarePathTracer::TracePath(const Ray& ray, const Scene* scene, uint32_t& hash_state, int max_bounces) const
{
	ColorRgba accumulated_color{ ColorRgba::Black() };
	ColorRgba throughput{ ColorRgba::White() };
	Ray current_ray{ ray };
	MediumState medium{};
	int bounce{};
	while (bounce < max_bounces)
	{
		RayHitRecord hit{};
		if (!scene->SceneClosestHitTest(current_ray, hit, false, medium.inside))
		{
			accumulated_color += throughput * EnvironmentRadiance(scene, current_ray.direction);
			break;
		}

		ShadingInput shading_input{ ConstructShadingInput(hit) };
		const uint32_t obj_idx{ hit.object_index };
		const SceneObject& obj{ scene->objects.at(obj_idx) };
		const Material* material{ scene->materials_factory.Get(obj.material_index) };
		SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, scene->textures_factory) };
		shading_input.world_normal = surface.shading_normal;
		surface.roughness = std::max(surface.roughness, 0.01f);

		if (IsAlphaSkipped(surface, hash_state))
		{
			current_ray.origin += current_ray.direction * (hit.t + 0.001f);
			--bounce;
			continue;
		}

		if (medium.inside && medium.attenuation_distance < std::numeric_limits<float>::max())
		{
			const float exp{ hit.t / medium.attenuation_distance };
			throughput *= ColorRgba(
				std::pow(medium.attenuation_color.r, exp),
				std::pow(medium.attenuation_color.g, exp),
				std::pow(medium.attenuation_color.b, exp));
		}

		if (surface.transmission <= 0.f)
			accumulated_color += throughput * DirectIllumination(shading_input, surface, material, scene, hash_state);

		if (bounce >= 3)
		{
			const float survival_probability{ std::clamp(throughput.Luminance(), 0.05f, 0.95f) };
			if (NextFloat(hash_state) > survival_probability)
				break;
			throughput *= 1 / survival_probability;
		}

		BounceSample bounce_sample{};
		if (surface.transmission > 0.f)
		{
			bounce_sample = SampleReflectOrRefract(shading_input, surface, medium, hash_state);
		}
		else
		{
			HemisphereSample sample{};
			switch (context_->debug_params.sampling_strategy)
			{
			case SamplingStrategy::kGGX:
				sample = SampleSpecularOrDiffuse(shading_input, surface, hash_state);
				break;
			case SamplingStrategy::kUniform:
				sample = SampleUniformHemisphere(shading_input.world_normal, hash_state);
				break;
			case SamplingStrategy::kCosine:
			default:
				sample = SampleCosineHemisphere(shading_input.world_normal, hash_state);
				break;
			}

			const float n_dot_l{ Vector3::Dot(shading_input.world_normal, sample.direction) };
			if (n_dot_l <= 0.f || sample.pdf < 1e-6f) break;

			shading_input.light_direction = sample.direction;
			shading_input.is_indirect = true;
			bounce_sample.direction = sample.direction;
			bounce_sample.weight = CalcBrdf(material, shading_input, surface) * n_dot_l / sample.pdf;
		}

		throughput *= bounce_sample.weight;
		throughput *= std::clamp(surface.occlusion, 0.f, 1.f);

		const bool same_side_as_normal{ Vector3::Dot(bounce_sample.direction, shading_input.world_normal) > 0.f };

		current_ray = Ray
		{
			same_side_as_normal
				? shading_input.shadow_origin
				: shading_input.world_position - shading_input.world_normal * 0.001f,
			bounce_sample.direction,
			0.001f,
			std::numeric_limits<float>::max()
		};

		++bounce;
	}

	if (context_->debug_params.visualization_mode == VisualizationMode::kBounceCount)
	{
		constexpr float kHeatmapScale{ 5.f };
		const float t{ std::clamp(static_cast<float>(bounce) / kHeatmapScale, 0.f, 1.f) };
		const float r{ std::clamp(2.f * t - 1.f, 0.f, 1.f) };
		const float g{ t < 0.5f ? 2.f * t : 2.f * (1.f - t) };
		const float b{ std::clamp(1.f - 2.f * t, 0.f, 1.f) };
		return ColorRgba(r, g, b);
	}
	return accumulated_color;
}

ColorRgba gfx::SoftwarePathTracer::EnvironmentRadiance(const Scene* scene, const Vector3& direction)
{
	if (!scene->environment_map.has_value())
		return scene->background_color;
	const EnvironmentMap& env{ scene->environment_map.value() };

	Vector3 dir{ direction };
	if (env.flip_horizontal)
		dir.x = -dir.x;
	if (env.rotation_degrees != 0.f)
	{
		const float rot_rad{ env.rotation_degrees * std::numbers::pi_v<float> / 180.f };
		const float cos_r{ std::cos(rot_rad) };
		const float sin_r{ std::sin(rot_rad) };
		dir = Vector3(
			cos_r * dir.x + sin_r * dir.z,
			dir.y,
			-sin_r * dir.x + cos_r * dir.z);
	}
	return env.texture.SampleDirection(dir) * env.intensity;
}

Vector2 gfx::SoftwarePathTracer::GenerateSubpixelJitter([[maybe_unused]] uint32_t pixel_index, uint32_t& hash_state) const
{
	const float jx{ NextFloat(hash_state) };
	const float jy{ NextFloat(hash_state) };
	return { jx,jy };
}

bool gfx::SoftwarePathTracer::IsThinWalled(const SurfaceProperties& surface)
{
	return surface.thickness <= 0.f && surface.attenuation_distance >= std::numeric_limits<float>::max();
}

BounceSample gfx::SoftwarePathTracer::SampleReflectOrRefract(const ShadingInput& input, const SurfaceProperties& surface, MediumState& medium, uint32_t& hash_state)
{
	const Vector3 incoming{ -input.view_direction };
	Vector3 normal{ input.world_normal };
	if (Vector3::Dot(incoming, normal) > 0.f)
		normal = -normal;
	if (IsThinWalled(surface))
	{
		if (NextFloat(hash_state) < FresnelFunction_Schlick(normal, input.view_direction, surface.f0).r)
			return { Vector3::Reflect(incoming, normal), ColorRgba::White() };
		else
			return { incoming, surface.albedo };
	}

	const float eta{ medium.inside ? surface.ior : 1.f / surface.ior };
	const Vector3 refracted{ Vector3::Refract(incoming, normal, eta) };
	const bool tir{ refracted.SqrMagnitude() < 1e-8f };
	const Vector3 schlick_dir{ medium.inside ? -refracted : input.view_direction };
	const float fresnel{ tir ? 1.f : FresnelFunction_Schlick(normal, schlick_dir, surface.f0).r };

	if (tir || NextFloat(hash_state) < fresnel)
		return { Vector3::Reflect(incoming, normal), ColorRgba::White() };

	medium.inside = !medium.inside;
	medium.attenuation_color = medium.inside ? surface.attenuation_color : ColorRgba::White();
	medium.attenuation_distance = medium.inside ? surface.attenuation_distance : std::numeric_limits<float>::max();
	return { refracted, surface.albedo };
}

bool gfx::SoftwarePathTracer::IsAlphaSkipped(const SurfaceProperties& surface, uint32_t& hash_state)
{
	if (surface.transmission > 0.f) return false;
	switch (surface.alpha_params.mode)
	{
	case AlphaMode::kMask:
		return surface.albedo.a < surface.alpha_params.cutoff;
	case AlphaMode::kBlend:
		return surface.albedo.a < NextFloat(hash_state);
	case AlphaMode::kOpaque:
	default:
		return false;
	}
}

ColorRgba gfx::SoftwarePathTracer::ShadowTransmittance(const Ray& shadow_ray, const Scene* scene, uint32_t& hash_state) const
{
	ColorRgba transmittance{ ColorRgba::White() };
	Ray ray{ shadow_ray };
	RayHitRecord hit{};
	while (scene->SceneClosestHitTest(ray, hit, false, true))
	{
		ShadingInput shading_input{ ConstructShadingInput(hit) };
		const uint32_t obj_idx{ hit.object_index };
		const SceneObject& obj{ scene->objects.at(obj_idx) };
		const Material* material{ scene->materials_factory.Get(obj.material_index) };
		const SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, scene->textures_factory) };
		if (!IsAlphaSkipped(surface, hash_state))
		{
			if (surface.transmission > 0.f && IsThinWalled(surface))
			{
				const float fresnel{ FresnelFunction_Schlick(shading_input.world_normal, -ray.direction, surface.f0).r };
				transmittance *= surface.albedo * (1.f - fresnel);
				if (transmittance.Luminance() < 0.001f)
					return ColorRgba::Black();
			}
			else
				return ColorRgba::Black();
		}
		const float traveled{ hit.t + 0.001f };
		ray.origin += ray.direction * traveled;
		ray.max -= traveled;
		hit = {};
	}
	return transmittance;
}

// =============================================================================
// Construction / Destruction
// =============================================================================
SoftwarePathTracer::SoftwarePathTracer(Context* const context)
	: Renderer(context)
{}

SoftwarePathTracer::~SoftwarePathTracer() = default;

// =============================================================================
// Public Functions
// =============================================================================
void SoftwarePathTracer::Render()
{
	assert(context_ && "Context not available!");

	Scene* const active_scene{ context_->scene_manager->GetActiveScene() };
	if (!active_scene) return;

	const SurfaceInfo& surface_info = context_->surface_info;

	const float aspect_ratio{ static_cast<float>(surface_info.width) / static_cast<float>(surface_info.height) };
	const float alpha{ active_scene->camera.GetFovAngle() * std::numbers::pi_v<float> / 180.f };
	const float FOV{ std::tan(alpha / 2.f) };

	const Matrix& camera_view{ active_scene->camera.GetView() };

	uint32_t total_indices{ surface_info.height * surface_info.width };
	auto pixel_indices{ std::views::iota(0u, total_indices) };

	const VisualizationMode current_vis_mode{ context_->debug_params.visualization_mode };
	const SamplingStrategy current_sampling{ context_->debug_params.sampling_strategy };
	const ToneMapOperator current_tone_map{ context_->debug_params.tone_map_operator };
	const BoundingBoxMode current_bounding_box_mode{ context_->debug_params.bounding_box_mode };
	if (active_scene->scene_changed || accumulation_buffer_.size() != total_indices ||
		current_vis_mode != last_vis_mode_ ||
		current_sampling != last_sampling_strategy_ ||
		current_tone_map != last_tone_map ||
		current_bounding_box_mode != last_bbox_mode_)
	{
		context_->debug_params.accumulated_samples = 0;
		accumulation_buffer_.assign(total_indices, ColorRgba::Black());
		last_vis_mode_ = current_vis_mode;
		last_sampling_strategy_ = current_sampling;
		last_tone_map = current_tone_map;
		last_bbox_mode_ = current_bounding_box_mode;
	}

	if (context_->debug_params.accumulated_samples >= context_->debug_params.max_samples ||
		context_->debug_params.sampling_paused && context_->debug_params.accumulated_samples >= 1)
		return;


	auto per_pixel_fnc = [&](const uint32_t pixel_idx) {

		uint32_t hash_state{ LowBias32(pixel_idx ^ (context_->debug_params.accumulated_samples * 2654435761u)) };

		const uint32_t px{ pixel_idx % surface_info.width };
		const uint32_t py{ pixel_idx / surface_info.width };

		//middle pixel breakpoint spot
#ifdef _DEBUG
		if (px == surface_info.width / 2 && py == surface_info.height / 2)
		{
			std::cout << "Middle pixel reached at (" << px << ", " << py << ")" << std::endl;
		}
#endif

		//calculate ray using NDC
		const Vector2 jitter{ GenerateSubpixelJitter(pixel_idx, hash_state) };
		float x_ndc{ (2.f * ((static_cast<float>(px) + jitter.x) / surface_info.width) - 1.f) * aspect_ratio * FOV };
		float y_ndc{ (1.f - 2.f * ((static_cast<float>(py) + jitter.y) / surface_info.height)) * FOV };
		Vector3 ray_direction{ x_ndc, y_ndc, 1 };
		Ray world_ray{ .direction = ray_direction.Normalized() };
		Ray view_ray{ camera_view.TransformRay(world_ray) };

		RayHitRecord closest_hit_record{};
		ShadingInput shading_input{};
		ColorRgba sample_color{ active_scene->background_color };

		if (current_vis_mode == VisualizationMode::kNone
			|| current_vis_mode == VisualizationMode::kBounceCount)
		{
			sample_color = TracePath(view_ray, active_scene, hash_state);
		}
		else if (active_scene->SceneClosestHitTest(view_ray, closest_hit_record))
		{
			shading_input = ConstructShadingInput(closest_hit_record);
			const uint32_t obj_idx{ closest_hit_record.object_index };
			const SceneObject& obj{ active_scene->objects.at(obj_idx) };
			const Material* material{ active_scene->materials_factory.Get(obj.material_index) };
			const SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, active_scene->textures_factory) };
			shading_input.world_normal = surface.shading_normal;

			switch (current_vis_mode)
			{
			case VisualizationMode::kNormals:
			{
				const Vector3& n = shading_input.world_normal;
				sample_color = {
					(n.x + 1.f) * 0.5f,
					(n.y + 1.f) * 0.5f,
					(n.z + 1.f) * 0.5f
				};
				break;
			}
			case VisualizationMode::kAlbedo:
				sample_color = surface.albedo;
				break;
			case VisualizationMode::kDepth:
			{
				const float max_depth{ 100.f };
				const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
				sample_color = { scaled_t, scaled_t, scaled_t };
				break;
			}
			case VisualizationMode::kUV:
				if (!shading_input.texture_coordinates.has_value()) break;
				sample_color = {
					shading_input.texture_coordinates.value().x,
					shading_input.texture_coordinates.value().y,
					0
				};
				break;
			case VisualizationMode::kMetallicRoughness:
				sample_color = {
					0.f,
					surface.roughness,
					surface.metallic
				};
				break;
			case VisualizationMode::kDirectOnly:
				sample_color = DirectIllumination(shading_input, surface, material, active_scene, hash_state);
				break;
			case VisualizationMode::kF0:
				sample_color = surface.f0;
				break;
			case VisualizationMode::kAttenuationColor:
				sample_color = surface.attenuation_color;
				break;
			case VisualizationMode::kReflectance:
				sample_color = surface.reflectance;
				break;
			case VisualizationMode::kTransmission:
				sample_color = surface.transmission;
				break;
			case VisualizationMode::kIor:
			{
				const float max{ 3.f }, min{ 1.f };
				sample_color = (surface.ior - min) / (max - min);
				break;
			}
			case VisualizationMode::kOcclusion:
				sample_color = surface.occlusion;
				break;
			case VisualizationMode::kAttenuationDistance:
				sample_color = surface.attenuation_distance;
				break;
			case VisualizationMode::kThickness:
				sample_color = surface.thickness;
				break;
			case VisualizationMode::kAlphaParams:
				if (surface.alpha_params.mode == AlphaMode::kMask)
					sample_color.r = surface.alpha_params.cutoff;
				else if (surface.alpha_params.mode == AlphaMode::kBlend)
					sample_color.g = surface.alpha_params.cutoff;
				else
					sample_color.b = surface.alpha_params.cutoff;
				break;
			case VisualizationMode::kWireframe:
			{
				// World-space size of one pixel at the hit distance.
				const float pixel_footprint{ 2.f * FOV * closest_hit_record.t
					/ static_cast<float>(surface_info.height) };
				// hit.ray is in object space; its direction length encodes the
				// instance scale, so this converts the footprint to local space.
				const float local_footprint{ pixel_footprint
					* closest_hit_record.ray.direction.Magnitude() };
				const float half_width{ 0.5f * kWireframeLineWidthPx * local_footprint };

				if (IsOnTriangleEdge(active_scene, obj, closest_hit_record, half_width))
					sample_color = ColorRgba(0.f, 1.f, 0.4f);
				else
				{
					// Dim facing-ratio fill so the silhouette/shape stays readable.
					const float facing{ std::abs(Vector3::Dot(
						shading_input.world_normal, shading_input.view_direction)) };
					sample_color = ColorRgba(0.05f + 0.15f * facing);
				}
				break;
			}
			case VisualizationMode::kMax:
				break;
			}

		}

		if (current_bounding_box_mode != BoundingBoxMode::kOff)
		{
			// Depth for occlusion: the path-traced modes don't fill closest_hit_record.
			float scene_t{ closest_hit_record.t };
			if (current_vis_mode == VisualizationMode::kNone
				|| current_vis_mode == VisualizationMode::kBounceCount)
			{
				RayHitRecord depth_hit{};
				if (active_scene->SceneClosestHitTest(view_ray, depth_hit))
					scene_t = depth_hit.t;
			}

			const float footprint_per_t{ 2.f * FOV / static_cast<float>(surface_info.height) };
			ColorRgba overlay_color{};
			if (BoundingBoxOverlay(active_scene, view_ray, scene_t, footprint_per_t,
				current_bounding_box_mode, overlay_color))
				sample_color = overlay_color;
		}

		if (!std::isfinite(sample_color.r) || !std::isfinite(sample_color.g) ||
			!std::isfinite(sample_color.b) || !std::isfinite(sample_color.a))
			sample_color = ColorRgba::Black();


		accumulation_buffer_[pixel_idx] += sample_color;

		const float inv_frames{ 1.f / static_cast<float>(context_->debug_params.accumulated_samples + 1.f) };
		ColorRgba final_color{ accumulation_buffer_[pixel_idx] * inv_frames };


		if (current_vis_mode == VisualizationMode::kNone ||
			current_vis_mode == VisualizationMode::kDirectOnly)
		{
			final_color = final_color.Tonemap(context_->debug_params.tone_map_operator);
			final_color = final_color.ToSrgb();
		}


		final_color.Clamp();
		// Write to surface
		surface_info.pixel_buffer[px + (py * surface_info.width)] = SDL_MapRGB(
			surface_info.pixel_format_details, nullptr,
			static_cast<uint8_t>(final_color.r * 255),
			static_cast<uint8_t>(final_color.g * 255),
			static_cast<uint8_t>(final_color.b * 255));

	};

#ifdef _DEBUG
	std::for_each(std::execution::seq, pixel_indices.begin(), pixel_indices.end(), per_pixel_fnc);
#else
	std::for_each(std::execution::par_unseq, pixel_indices.begin(), pixel_indices.end(), per_pixel_fnc);
#endif

	++context_->debug_params.accumulated_samples;

}