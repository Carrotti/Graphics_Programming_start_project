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

	return result;
}

ColorRgba gfx::SoftwarePathTracer::TracePath(const Ray& ray, const Scene* scene, uint32_t& hash_state, int max_bounces) const
{
	ColorRgba accumulated_color{ ColorRgba::Black() };
	ColorRgba throughput{ ColorRgba::White() };
	Ray current_ray{ ray };
	int bounce{};
	while (bounce < max_bounces)
	{
		RayHitRecord hit{};
		if (!scene->SceneClosestHitTest(current_ray, hit))
		{
			accumulated_color += throughput * EnvironmentRadiance(scene, current_ray.direction);
			break;
		}

		ShadingInput shading_input{ scene->ConstructShadingInput(hit) };
		const uint32_t obj_idx{ hit.object_index };
		const SceneObject& obj{ scene->objects.at(obj_idx) };
		const Material* material{ scene->materials_factory.Get(obj.material_index) };
		const SurfaceProperties surface{ material->GetSurfaceProperties(shading_input, scene->textures_factory) };
		shading_input.world_normal = surface.shading_normal;

		accumulated_color += throughput * DirectIllumination(shading_input, surface, material, scene);

		HemisphereSample sample{};
		switch (context_->debug_params.sampling_strategy)
		{
		case SamplingStrategy::kUniform:
			sample = UniformSampleHemisphere(shading_input.world_normal, hash_state);
		default:
			sample = CosineSampleHemisphere(shading_input.world_normal, hash_state);
		}

		const float n_dot_l{ Vector3::Dot(shading_input.world_normal, sample.direction) };
		if (n_dot_l <= 0.f || sample.pdf < 1e-6f) break;

		shading_input.light_direction = sample.direction;

		ColorRgba brdf{ material->Shade(shading_input, surface) };
		BounceSample bounce_sample{
			sample.direction,
			CalcBrdf(material, shading_input, surface) * n_dot_l / sample.pdf
		};

		throughput *= bounce_sample.weight;

		current_ray = Ray
		{
			shading_input.shadow_origin,
			sample.direction,
			0.001f,
			std::numeric_limits<float>::max()
		};

		++bounce;
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

	const VisualizationMode current_vis_mode{ context_->debug_params.visualization_mode };
	const SamplingStrategy current_sampling{ context_->debug_params.sampling_strategy };
	const ToneMapOperator current_tone_map{ context_->debug_params.tone_map_operator };
	if (active_scene->scene_changed || accumulation_buffer_.size() != total_indices ||
		current_vis_mode != last_vis_mode_ ||
		current_sampling != last_sampling_strategy_ ||
		current_tone_map != last_tone_map)
	{
		context_->debug_params.accumulated_samples = 0;
		accumulation_buffer_.assign(total_indices, ColorRgba::Black());
		last_vis_mode_ = current_vis_mode;
		last_sampling_strategy_ = current_sampling;
		last_tone_map = current_tone_map;
	}

	if (context_->debug_params.accumulated_samples >= context_->debug_params.max_samples ||
		context_->debug_params.sampling_paused)
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

		if (context_->debug_params.visualization_mode == VisualizationMode::kNone)
		{
			sample_color = TracePath(view_ray, active_scene, hash_state);
		}
		else
		{
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
				{
					sample_color = surface.albedo;
					break;
				}
				case VisualizationMode::kDepth:
				{
					const float max_depth{ 100.f };
					const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
					sample_color = { scaled_t, scaled_t, scaled_t };
					break;
				}
				case VisualizationMode::kUV:
				{
					if (!shading_input.texture_coordinates.has_value()) break;
					sample_color = {
						shading_input.texture_coordinates.value().x,
						shading_input.texture_coordinates.value().y,
						0
					};
					break;
				}
				case VisualizationMode::kMetallicRoughness:
				{
					sample_color = {
						0.f,
						surface.roughness,
						surface.metallic
					};
					break;
				}
				case VisualizationMode::kDirectOnly:
				{
					sample_color = DirectIllumination(shading_input, surface, material, active_scene);
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
		}

		if (!std::isfinite(sample_color.r) || !std::isfinite(sample_color.g) ||
			!std::isfinite(sample_color.b) || !std::isfinite(sample_color.a))
			sample_color = ColorRgba::Black();


		accumulation_buffer_[pixel_idx] += sample_color;

		const float inv_frames{ 1.f / static_cast<float>(context_->debug_params.accumulated_samples + 1.f) };
		ColorRgba final_color{ accumulation_buffer_[pixel_idx] * inv_frames };


		if (context_->debug_params.visualization_mode == VisualizationMode::kNone ||
			context_->debug_params.visualization_mode == VisualizationMode::kDirectOnly)
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
	std::for_each(std::execution::par, pixel_indices.begin(), pixel_indices.end(), per_pixel_fnc);
#endif

	++context_->debug_params.accumulated_samples;

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
