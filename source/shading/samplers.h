#include <cmath>
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
	static HemisphereSample UniformSampleHemisphere(const Vector3& normal, uint32_t& hash_state)
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
	static HemisphereSample CosineSampleHemisphere(const Vector3& normal, uint32_t& hash_state)
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
}




#endif //SAMPLERS_HEADER