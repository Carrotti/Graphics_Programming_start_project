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
using namespace gfx;

// =============================================================================
// Private Helper Functions
// =============================================================================
namespace 
{

	uint32_t LowBias32(uint32_t h)
	{
		h ^= h >> 16; h *= 0x7feb352du;
		h ^= h >> 15; h *= 0x846ca68bu;
		h ^= h >> 16;
		return h;
	}

	float HashToFloat(const uint32_t h)
	{
		return static_cast<float>(h >> 8) * (1.f / 16777216.f);
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

ColorRgba SoftwarePathTracer::CalcDirect(const Light* light, const Vector3& light_dir,
	float light_dist, const ShadingInput& shading_input, const Scene* scene)
{
	//shadow
	const Ray shadow_ray{
		shading_input.shadow_origin,
		light_dir,
		1e-3f,
		light->type == LightType::kDirectional ? std::numeric_limits<float>::max() : light_dist
	};
	RayHitRecord hit_record{};
	if (scene->SceneClosestHitTest(shadow_ray, hit_record, true))
		return ColorRgba{};

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

ColorRgba gfx::SoftwarePathTracer::DirectIllumination(const ShadingInput& shading_input, const SurfaceProperties& surface, const Material* material, const Scene* scene) const
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

		result += CalcDirect(light, light_dir, light_dist, input, scene)
			* CalcBrdf(material, input, surface)
			* CalcCosine(surface.shading_normal, light_dir);
	}
	result = result.Tonemap(context_->debug_params.tone_map_operator);
	result = result.ToSrgb();

	return result;
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
	const float FOV{ tanf(alpha / 2.f) };

	const Matrix& camera_view{ active_scene->camera.GetView() };

	uint32_t total_indices{ surface_info.height * surface_info.width };
	auto pixel_indices{ std::views::iota(0u, total_indices) };

	auto per_pixel_fnc = [&](const uint32_t pixel_idx) {

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
		float x_ndc{ ((2.f * (px + 0.5f) / surface_info.width) - 1.f) * aspect_ratio * FOV };
		float y_ndc{ (1.f - (2.f * (py + 0.5f) / surface_info.height)) * FOV };
		Vector3 ray_direction{ x_ndc, y_ndc, 1 };
		Ray world_ray{ .direction = ray_direction.Normalized() };
		Ray view_ray{ camera_view.TransformRay(world_ray) };

		RayHitRecord closest_hit_record{};
		ShadingInput shading_input{};
		ColorRgba final_color{ active_scene->background_color };

		bool did_hit{ active_scene->SceneClosestHitTest(view_ray,closest_hit_record) };
		if (did_hit)
		{
			shading_input = active_scene->ConstructShadingInput(closest_hit_record);
			const uint32_t obj_idx{ closest_hit_record.object_index };
			const SceneObject& obj{ active_scene->objects.at(obj_idx) };
			const Material* material{ active_scene->materials_factory.Get(obj.material_index) };
			const SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, active_scene->textures_factory) };
			shading_input.world_normal = surface.shading_normal;

			switch (context_->debug_params.visualization_mode)
			{
			case VisualizationMode::kNone:
			{
				final_color = DirectIllumination(shading_input, surface, material, active_scene);
				break;
			}
			case VisualizationMode::kNormals:
			{
				const Vector3& n = shading_input.world_normal;
				final_color = {
					(n.x + 1.f) * 0.5f,
					(n.y + 1.f) * 0.5f,
					(n.z + 1.f) * 0.5f
				};
				break;
			}
			case VisualizationMode::kAlbedo:
			{
				final_color = surface.albedo;
				break;
			}
			case VisualizationMode::kDepth:
			{
				const float max_depth{ 100.f };
				const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
				final_color = { scaled_t, scaled_t, scaled_t };
				break;
			}
			case VisualizationMode::kUV:
			{
				if (!shading_input.texture_coordinates.has_value()) break;
				final_color = {
					shading_input.texture_coordinates.value().x,
					shading_input.texture_coordinates.value().y,
					0
				};
				break;
			}
			case VisualizationMode::kMetallicRoughness:
			{
				final_color = {
					0.f,
					surface.roughness,
					surface.metallic
				};
				break;
			}
			case VisualizationMode::kDirectOnly:
			{
				final_color = DirectIllumination(shading_input, surface, material, active_scene);
				break;
			}
			case VisualizationMode::kBounceCount:
			{
				break;
			}
			case VisualizationMode::kMax:
			{
				break;
			}
			}
		}


		//TEMP
		const float u{ HashToFloat(LowBias32(pixel_idx)) };
		final_color = { u,u,u };




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
	std::for_each(std::execution::par, pixel_indices.begin(), pixel_indices.end(), per_pixel_fnc);
#endif

}

ColorRgba gfx::SoftwarePathTracer::CalculateRadiance(const Light* light, const Vector3& world_position)
{
	switch (light->type)
	{
	case LightType::kPoint:
	{
		return light->color * light->intensity / (light->origin - world_position).SqrMagnitude();
	}
	case LightType::kDirectional:
	{
		return light->color * light->intensity;
	}
	default:
		return ColorRgba{};
	}
}
