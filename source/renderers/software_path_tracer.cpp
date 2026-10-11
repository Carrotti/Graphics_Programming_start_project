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


	const Sphere test_sphere{ {0.f,0.f,100.f }, 50.f };
	const Plane test_plane{ 
		{0.f,-50.f,0.f},
		{0.f,1.f,0.f},
		true,
		Vector2{100.f,100.f}
	};
	const float aspect_ratio{ static_cast<float>(surface_info.width) / surface_info.height };

	for (uint32_t py{ 0 }; py < surface_info.height; ++py)
	{
		for (uint32_t px{ 0 }; px < surface_info.width; ++px)
		{
			if (px == surface_info.width / 2 && py == surface_info.height / 2)
			{
				//middle pixel breakpoint spot
			}
			//calculate NDC
			float x_ndc{ ((2.f * (px + 0.5f) / surface_info.width) - 1.f) * aspect_ratio };
			float y_ndc{ 1.f - (2.f * (py + 0.5f) / surface_info.height) };
			Vector3 ray_direction{ x_ndc, y_ndc, 1 };
			ray_direction.Normalize();
			const Ray view_ray{ {0.f, 0.f,0.f}, ray_direction };


			RayHitRecord closest_hit_record{};
			
			bool did_hit{ HitTestPlane(test_plane,view_ray,closest_hit_record) };
			//bool did_hit{ HitTestSphere(test_sphere, view_ray, closest_hit_record) };

			ShadingInput shading_input{};

			Vector3 hit_position{ view_ray.origin + view_ray.direction * closest_hit_record.t };
			shading_input.world_position = hit_position;
			shading_input.world_normal = test_plane.normal.Normalized();


			ColorRgba final_color{};

			if (!did_hit)
			{
				final_color = { 0,0,0 };
			}
			else
			{
				switch (context_->debug_params.visualization_mode)
				{
				case VisualizationMode::kDepth:
				{
					const float max_depth{ 100.f };
					const float scaled_t{ 1.f - std::clamp(closest_hit_record.t / max_depth,0.f,1.f) };
					final_color = {
						scaled_t, scaled_t, scaled_t
					};
				}
				break;
				case VisualizationMode::kNone:
					final_color = {
						1.f,0.f,0.f
					};
					break;
				case VisualizationMode::kNormals:
					const Vector3& n = shading_input.world_normal;
					final_color = {
						(n.x + 1.f) * 0.5f,
						(n.y + 1.f) * 0.5f,
						(n.z + 1.f) * 0.5f
					};
					break;
				}
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