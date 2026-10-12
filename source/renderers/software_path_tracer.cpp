//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#include <software_path_tracer.h>
#include <scenes.h>
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

	const SurfaceInfo& surface_info = context_->surface_info;

	/*
		const Sphere test_sphere{ {0.f,0.f,100.f }, 50.f };
		const Plane test_plane{
			{0.f,-50.f,0.f},
			{0.f,1.f,0.f},
			true,
			Vector2{100.f,100.f}
		};*/
	const float aspect_ratio{ static_cast<float>(surface_info.width) / surface_info.height };
	Scene* active_scene{ context_->scene_manager->GetActiveScene() };
	active_scene->camera.SetFovAngle(90);
	const float alpha{ active_scene->camera.GetFovAngle() * std::numbers::pi_v<float> / 180.f};
	const float FOV{ tanf(alpha / 2.f) };

	for (uint32_t py{ 0 }; py < surface_info.height; ++py)
	{
		for (uint32_t px{ 0 }; px < surface_info.width; ++px)
		{
			//middle pixel breakpoint spot
			//if (px == surface_info.width / 2 && py == surface_info.height / 2) continue;
			


			//calculate ray using NDC
			float x_ndc{ ((2.f * (px + 0.5f) / surface_info.width) - 1.f) * aspect_ratio * FOV };
			float y_ndc{ 1.f - (2.f * (py + 0.5f) / surface_info.height) * FOV };
			Vector3 ray_direction{ x_ndc, y_ndc, 1 };
			const Matrix& camera_view{ active_scene->camera.GetView() };
			const Ray view_ray{ active_scene->camera.GetPosition(), camera_view.TransformVector(ray_direction.Normalized())};


			RayHitRecord closest_hit_record{};
			ShadingInput shading_input{};
			ColorRgba final_color{};

			bool did_hit{ active_scene->SceneClosestHitTest(view_ray,closest_hit_record) };
			if (did_hit)
			{
				shading_input = active_scene->ConstructShadingInput(closest_hit_record);
				switch (context_->debug_params.visualization_mode)
				{
				case VisualizationMode::kDepth:
				{
					const float max_depth{ 100.f };
					const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
					final_color = {
						scaled_t, scaled_t, scaled_t
					};
					break;
				}
				case VisualizationMode::kNone:
				{
					const uint32_t idx{ closest_hit_record.object_index };
					final_color = {
						static_cast<float>(idx & 1), static_cast<float>((idx >> 1) & 1), static_cast<float>((idx >> 2) & 1)
					};
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
				}
			}
			else
			{
				final_color = active_scene->background_color;
			}

			final_color.MaxToOne();

			// Write to surface
			surface_info.pixel_buffer[px + (py * surface_info.width)] = SDL_MapRGB(
				surface_info.pixel_format_details, nullptr,
				static_cast<uint8_t>(final_color.r * 255),
				static_cast<uint8_t>(final_color.g * 255),
				static_cast<uint8_t>(final_color.b * 255));

		}
	}
}