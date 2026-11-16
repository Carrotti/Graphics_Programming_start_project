//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#ifndef SOFTWARE_PATH_TRACER_HEADER
#define SOFTWARE_PATH_TRACER_HEADER

//--- Standard Includes ---

//--- Framework Includes ---
#include <renderer.h>

namespace gfx
{


	//--- Software Path Tracer ---
	class SoftwarePathTracer final : public Renderer
	{
	private:
		//--- Private Helper Functions ---
		static float CalcCosine(const Vector3& normal, const Vector3& light_dir);
		static ColorRgba CalcBrdf(const Material* material, const ShadingInput& input, 
			const SurfaceProperties& surface);
		static ColorRgba CalcDirect(const Light* light, const Vector3& light_dir, 
			float light_dist, const ShadingInput& shading_input, const Scene* scene);

		ColorRgba DirectIllumination(const ShadingInput& shading_input,
			const SurfaceProperties& surface, const Material* material,
			const Scene* scene) const;

		uint32_t ao_sample_count_{ 1 };
		float ao_radius_{ 10.f };

		//float TraceAO(const ShadingInput& shading_input, uint32_t pixel_index) const;

		VisualizationMode last_vis_mode_{};
		SamplingStrategy last_sampling_strategy_{ SamplingStrategy::kCosine };
		ToneMapOperator last_tone_map{};
		std::vector<ColorRgba> accumulation_buffer_{};

		Vector2 GenerateSubpixelJitter(uint32_t pixel_index, uint32_t& hash_state) const;

	public:
		//--- Construction / Destruction ---
		SoftwarePathTracer(Context* const context);
		~SoftwarePathTracer() override;

		SoftwarePathTracer(const SoftwarePathTracer&) = delete;
		SoftwarePathTracer& operator=(const SoftwarePathTracer&) = delete;
		SoftwarePathTracer(SoftwarePathTracer&&) = delete;
		SoftwarePathTracer& operator=(SoftwarePathTracer&&) = delete;

		//--- Public Functions ---
		void Render() override;
		ColorRgba CalculateRadiance(const Light* light, const Vector3& world_position);
	};
}
#endif //SOFTWARE_PATH_TRACER_HEADER
