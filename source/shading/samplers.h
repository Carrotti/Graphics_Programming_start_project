#include <cmath>
#include <algorithm>
#include "brdfs.h"
#ifndef SAMPLERS_HEADER
#define SAMPLERS_HEADER


namespace gfx
{


	struct HemisphereSample
	{
		Vector3 direction{};
		float pdf{};
	};

	[[maybe_unused]]
	static uint32_t LowBias32(uint32_t h)
	{
		h ^= h >> 16; h *= 0x7feb352du;
		h ^= h >> 15; h *= 0x846ca68bu;
		h ^= h >> 16;
		return h;
	}

	[[maybe_unused]]
	static float HashToFloat(const uint32_t h)
	{
		return static_cast<float>(h >> 8) * (1.f / 16777216.f);
	}

	[[maybe_unused]]
	static float NextFloat(uint32_t& h)
	{
		h = LowBias32(h);
		return HashToFloat(h);
	}

	[[maybe_unused]]
	static Vector3 LocalToWorld(const Vector3& local_dir, const Vector3& normal)
	{
		const Vector3 helper{ std::abs(normal.x) > 0.9 ? Vector3::UnitY() : Vector3::UnitX() };
		const Vector3 t{ Vector3::Cross(normal, helper).Normalized() };
		const Vector3 b{ Vector3::Cross(normal, t) };
		return t * local_dir.x + b * local_dir.y + normal * local_dir.z;
	}

	[[maybe_unused]]
	static float SpecularProbability(const ShadingInput& input, const SurfaceProperties& surface)
	{
		const float fresnel{ FresnelFunction_Schlick(input.world_normal, input.view_direction, surface.f0).r };
		const float probability{ std::clamp(fresnel, 0.05f, 0.95f) };
		return std::lerp(probability, 1.f, surface.metallic);
	}

	[[maybe_unused]]
	static float PdfUniformHemisphere(const Vector3& normal, const Vector3& direction)
	{
		if (Vector3::Dot(normal, direction) < 0.f)
			return 0;
		else
			return 1.f / (2 * std::numbers::pi_v<float>);
	}

	[[maybe_unused]]
	static float PdfCosineHemisphere(const Vector3& normal, const Vector3& direction)
	{
		const float cos_theta{ Vector3::Dot(normal, direction) }; //assuming n and d are normalized
		if (cos_theta < 0.f)
			return 0;
		else
			return cos_theta / std::numbers::pi_v<float>;
	}

	[[maybe_unused]]
	static float PdfGGX(const Vector3& normal, const Vector3& view_dir, const Vector3& light_dir, const float roughness)
	{
		const Vector3 h{ (view_dir + light_dir).Normalized() };
		const float n_dot_h{ std::max(0.0001f, Vector3::Dot(normal, h)) };
		const float v_dot_h{ std::max(0.0001f, Vector3::Dot(view_dir, h)) };
		return NormalDistribution_GGX(normal, h, roughness) * n_dot_h / (4.f * v_dot_h);
	}

	[[maybe_unused]]
	static float PdfSpecularOrDiffuse(const ShadingInput& input, const SurfaceProperties& surface, const Vector3& direction)
	{
		if (Vector3::Dot(input.world_normal, direction) <= 0.f)
			return 0.f;
		const float p{ SpecularProbability(input, surface) };
		return p * PdfGGX(input.world_normal, input.view_direction, direction, surface.roughness)
			+ (1.f - p) * PdfCosineHemisphere(input.world_normal, direction);
	}

	[[maybe_unused]]
	static HemisphereSample SampleUniformHemisphere(const Vector3& normal, uint32_t& hash_state)
	{

		const float u1{ NextFloat(hash_state) };
		const float u2{ NextFloat(hash_state) };

		const float cos_theta{ u2 };
		const float sin_theta{ std::sqrt(1 - u2 * u2) };
		const float phi{ 2 * std::numbers::pi_v<float> *u1 };

		const Vector3 local_dir{ std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, cos_theta };
		const Vector3 direction{ LocalToWorld(local_dir, normal) };
		return { direction, PdfUniformHemisphere(normal, direction) };
	}

	[[maybe_unused]]
	static HemisphereSample SampleCosineHemisphere(const Vector3& normal, uint32_t& hash_state)
	{
		const float u1{ NextFloat(hash_state) };
		const float u2{ NextFloat(hash_state) };

		const float cos_theta{ std::sqrt(1 - u2) };
		const float sin_theta{ std::sqrt(u2) };
		const float phi{ 2 * std::numbers::pi_v<float> *u1 };

		const Vector3 local_dir{ std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, cos_theta };
		const Vector3 direction{ LocalToWorld(local_dir, normal) };
		return { direction, PdfCosineHemisphere(normal, direction) };
	}

	[[maybe_unused]]
	static HemisphereSample SampleGGX(const Vector3& normal, const Vector3& view_dir, const float roughness, uint32_t& hash_state)
	{
		const float u1{ NextFloat(hash_state) };
		const float u2{ NextFloat(hash_state) };

		const float a{ roughness * roughness };
		const float a2{ a * a };
		const float cos_theta_h{ std::sqrt((1.f - u1) / (1.f + (a2 - 1.f) * u1)) };
		const float sin_theta_h{ std::sqrt(1.f - cos_theta_h * cos_theta_h) };
		const float phi{ 2.f * std::numbers::pi_v<float> *u2 };

		const Vector3 local_h{ sin_theta_h * std::cos(phi), sin_theta_h * std::sin(phi), cos_theta_h };
		const Vector3 h{ LocalToWorld(local_h, normal) };
		const Vector3 l{ Vector3::Reflect(-view_dir, h) };
		return { l, PdfGGX(normal, view_dir, l, roughness) };
	}

	[[maybe_unused]]
	static HemisphereSample SampleSpecularOrDiffuse(const ShadingInput& input, const SurfaceProperties& surface, uint32_t& hash_state)
	{
		const float p{ SpecularProbability(input, surface) };
		const float u{ NextFloat(hash_state) };
		const Vector3& direction{ u < p ? 
			SampleGGX(input.world_normal, input.view_direction, surface.roughness, hash_state).direction : 
			SampleCosineHemisphere(input.world_normal, hash_state).direction };
		return { direction, PdfSpecularOrDiffuse(input, surface, direction )};
	}
}





#endif //SAMPLERS_HEADER