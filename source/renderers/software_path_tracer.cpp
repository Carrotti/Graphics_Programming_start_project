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
using namespace gfx;

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

		auto calc_direct_fnc = [&](const Light* light, Vector3 light_dir) -> ColorRgba {

			const float light_dist{ light_dir.Normalize() };

			//angle
			const float cos_theta{ Vector3::Dot(shading_input.world_normal, light_dir) };
			if (cos_theta < 0.f) return ColorRgba{};

			//shadow
			Ray shadow_ray{
				shading_input.shadow_origin,
				light_dir,
				1e-3f,
				light->type == LightType::kDirectional ? std::numeric_limits<float>::max() : light_dist
			};
			RayHitRecord hit_record{};
			if (active_scene->SceneClosestHitTest(shadow_ray, hit_record, true)) return ColorRgba{};

			//radiance
			ColorRgba radiance = CalculateRadiance(light, shading_input.world_position);
			return radiance * cos_theta;
			};

		bool did_hit{ active_scene->SceneClosestHitTest(view_ray,closest_hit_record) };
		if (did_hit)
		{
			shading_input = active_scene->ConstructShadingInput(closest_hit_record);
			switch (context_->debug_params.visualization_mode)
			{
			case VisualizationMode::kNone:
			{
				const uint32_t obj_idx{ closest_hit_record.object_index };
				const SceneObject& obj{ active_scene->objects.at(obj_idx) };
				const Material* material{ active_scene->materials_factory.Get(obj.material_index) };
				const SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, active_scene->textures_factory) };



				for (const Light* light : active_scene->lights_factory.GetAll())
				{

					Vector3 light_dir{
						light->type == LightType::kDirectional ?
						-light->direction :
						(light->origin - shading_input.shadow_origin)
					};
					shading_input.light_direction = light_dir;
					const ColorRgba brdf{ material->Shade(shading_input,surface) };
					final_color += calc_direct_fnc(light, light_dir) * brdf;
				}
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
				break;
			}
			case VisualizationMode::kDepth:
			{
				const float max_depth{ 100.f };
				const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
				final_color = { scaled_t, scaled_t, scaled_t };
				break;
			}
			case VisualizationMode::kDirectOnly:
			{
				for (const Light* light : active_scene->lights_factory.GetAll())
				{
					Vector3 light_dir{
						light->type == LightType::kDirectional ?
						-light->direction :
						(light->origin - shading_input.shadow_origin)
					};
					final_color += calc_direct_fnc(light, light_dir);
				}
				break;
			}
			}
		}



		final_color = final_color.Tonemap(context_->debug_params.tone_map_operator);
		//everything is way too bright with srgb on
		//final_color = final_color.ToSrgb();
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
