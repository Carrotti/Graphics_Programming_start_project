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
	struct BounceSample
	{
		Vector3 direction{};
		ColorRgba weight{ ColorRgba::Black() };
	};

	struct MediumState
	{
		bool inside{ false };
		ColorRgba attenuation_color{ ColorRgba::White() };
		float attenuation_distance{ std::numeric_limits<float>::max() };
	};

	//--- Software Path Tracer ---
	class SoftwarePathTracer final : public Renderer
	{
	private:
		struct FrameData
		{
			uint32_t width{};
			uint32_t height{};
			float aspect_ratio{};
			float FOV{};
			const Scene* scene{};
			const Matrix& camera_view{};
		};

		//--- Private Helper Functions ---
		static float CalcCosine(const Vector3& normal, const Vector3& light_dir);
		static ColorRgba CalcBrdf(const Material* material, const ShadingInput& input, 
			const SurfaceProperties& surface);
		static ColorRgba CalcRadiance(const Light* light, const ShadingInput& shading_input);

		ShadingInput ConstructShadingInput(const RayHitRecord& hit) const;


		ColorRgba DirectIllumination(const ShadingInput& shading_input, 
			const SurfaceProperties& surface, const Material* material, 
			const Scene* scene, uint32_t& hash_state) const;

		ColorRgba TracePath(const Ray& ray, const Scene* scene,
			uint32_t& hash_state, int max_bounces = 8) const;

		static ColorRgba EnvironmentRadiance(const Scene* scene, const Vector3& direction);

		uint32_t ao_sample_count_{ 1 };


		VisualizationMode last_vis_mode_{};
		SamplingStrategy last_sampling_strategy_{ SamplingStrategy::kCosine };
		ToneMapOperator last_tone_map{};
		BoundingBoxMode last_bbox_mode_{};

		std::vector<ColorRgba> accumulation_buffer_{};

		Vector2 GenerateSubpixelJitter(uint32_t pixel_index, uint32_t& hash_state) const;

		static bool IsThinWalled(const SurfaceProperties& surface);
		static BounceSample SampleReflectOrRefract(const ShadingInput& input, const SurfaceProperties& surface, MediumState& medium, uint32_t& hash_state);

		static bool IsAlphaSkipped(const SurfaceProperties& surface, uint32_t& hash_state);
		ColorRgba ShadowTransmittance(const Ray& shadow_ray, const Scene* scene, uint32_t& hash_state) const;

		void PerPixelRender(uint32_t pixel_idx, const FrameData& frame_data);


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
	};
}
#endif //SOFTWARE_PATH_TRACER_HEADER
