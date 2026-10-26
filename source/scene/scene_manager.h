//==============================================================
//  Graphics Programming 1 (2026-2027)
//  Authors : Matthieu Delaere
//  Copyright (c) 2026 Matthieu Delaere. All rights reserved.
//==============================================================
#ifndef SCENE_MANAGER_HEADER
#define SCENE_MANAGER_HEADER

//--- Framework Includes ---
#include <camera.h>
#include <primitives.h>
#include <materials.h>
#include <matrix.h>
#include <texture.h>
#include <factory.h>
#include <intersections.h>

namespace gfx
{
	struct SceneObject final
	{
		uint32_t primitive_index{ std::numeric_limits<uint32_t>::max() };
		uint32_t material_index{ std::numeric_limits<uint32_t>::max() };
		std::optional<Matrix> instance_transformation{ Matrix{} };
	};

	struct EnvironmentMap final
	{
		Texture texture;
		float intensity{ 1.f };
		float rotation_degrees{ 0.f };
		bool flip_horizontal{ false };
	};

	struct Scene
	{
		Factory<Primitive> primitives_factory{ Factory<Primitive>() };
		Factory<Material> materials_factory{ Factory<Material>() };
		Factory<Texture> textures_factory{ Factory<Texture>() };
		Factory<Light> lights_factory{ Factory<Light>() };
		std::vector<SceneObject> objects{ };
		Camera camera{ };
		ColorRgba background_color{ ColorRgba::Black() };
		std::optional<EnvironmentMap> environment_map{ };
		bool scene_changed{ true };

		Scene() = default;
		virtual ~Scene() = default;
		Scene(const Scene&) = delete;
		Scene& operator=(const Scene&) = delete;
		Scene(Scene&&) = delete;
		Scene& operator=(Scene&&) = delete;
		virtual void Setup() = 0;
		virtual bool Update(const double delta_time)
		{
			(void)delta_time; return false;
		}

		[[nodiscard]] uint32_t GetPrimitiveCount() const
		{
			return static_cast<uint32_t>(primitives_factory.GetAll().size());
		}

		[[nodiscard]] uint32_t GetTriangleCount() const
		{
			uint32_t count = 0;
			for (const Primitive* p : primitives_factory.GetAll())
			{
				if (p->type == PrimitiveType::kTriangle)
					++count;
				else if (p->type == PrimitiveType::kTriangleMesh)
					count += static_cast<uint32_t>(
						static_cast<const TriangleMesh*>(p)->indices.size() / 3);
			}
			return count;
		}

		bool SceneClosestHitTest(const Ray& ray,
			RayHitRecord& closest_hit, bool ignore_record = false) const
		{
			bool did_hit_anything{ false };
			for (int object_index{ 0 }; object_index < objects.size(); object_index++)
			{

				bool did_hit{ false };

				const SceneObject& object{ objects[object_index] };
				Primitive* prim{ primitives_factory.Get(object.primitive_index) };
				if (prim == nullptr) continue;
				Ray transformed_ray{
					object.instance_transformation.has_value() ?
					object.instance_transformation->TransformRay(ray) :
					ray
				};

				RayHitRecord candidate_closest_hit{};

				auto save_closest_hit = [&]()
					{
						if (did_hit) did_hit_anything = true;
						if (did_hit && candidate_closest_hit.t < closest_hit.t && !ignore_record)
						{
							candidate_closest_hit.object_index = object_index;
							closest_hit = candidate_closest_hit;
						}
					};

				switch (prim->type)
				{
				case PrimitiveType::kSphere:
				{
					const Sphere* sphere{ primitives_factory.GetAs<Sphere>(object.primitive_index) };
					did_hit = HitTestSphere(*sphere, transformed_ray, candidate_closest_hit, ignore_record);
					save_closest_hit();
					break;
				}
				case PrimitiveType::kPlane:
				{
					const Plane* plane{ primitives_factory.GetAs<Plane>(object.primitive_index) };
					did_hit = HitTestPlane(*plane, transformed_ray, candidate_closest_hit, ignore_record);
					save_closest_hit();
					break;
				}
				case PrimitiveType::kTriangle:
				{
					const Triangle* triangle{ primitives_factory.GetAs<Triangle>(object.primitive_index) };
					did_hit = HitTestTriangle(*triangle, transformed_ray, candidate_closest_hit, ignore_record);
					save_closest_hit();
					break;
				}
				case PrimitiveType::kTriangleMesh:
				{
					const TriangleMesh* triangle_mesh{ primitives_factory.GetAs<TriangleMesh>(object.primitive_index) };
					for (int triangle_idx{}; triangle_idx < triangle_mesh->indices.size(); triangle_idx += 3)
					{
						Triangle triangle{};

						triangle.v0 = triangle_mesh->vertices[triangle_mesh->indices[triangle_idx]].position;
						triangle.v1 = triangle_mesh->vertices[triangle_mesh->indices[triangle_idx + 1]].position;
						triangle.v2 = triangle_mesh->vertices[triangle_mesh->indices[triangle_idx + 2]].position; 
						triangle.normal = Vector3::Cross(
							triangle.v1 - triangle.v0,
							triangle.v2 - triangle.v0
						).Normalized();
						triangle.cull_mode = triangle_mesh->cull_mode;
						did_hit = HitTestTriangle(triangle, transformed_ray, candidate_closest_hit, ignore_record);
						if (did_hit && candidate_closest_hit.t < closest_hit.t && !ignore_record)
						{
							candidate_closest_hit.vertex_indices = {
								triangle_mesh->indices[triangle_idx],
								triangle_mesh->indices[triangle_idx + 1],
								triangle_mesh->indices[triangle_idx + 2]
							};
						}
						save_closest_hit();
					}
					break;
				}
				default:
					std::cerr << "unhandled case: SceneClosestHitTest\n";
					continue;
				}


			}
			return did_hit_anything;
		}

		ShadingInput ConstructShadingInput(const RayHitRecord& hit) const
		{
			ShadingInput shading_input{};
			const SceneObject& object{ objects[hit.object_index] };
			const Ray& ray{ hit.ray };
			shading_input.view_direction = -ray.direction;
			shading_input.world_position = ray.origin + ray.direction * hit.t;


			const Primitive* prim{ primitives_factory.Get(object.primitive_index) };
			switch (prim->type)
			{
			case PrimitiveType::kSphere:
			{
				const Sphere* sphere{ primitives_factory.GetAs<Sphere>(object.primitive_index) };
				shading_input.world_normal = (shading_input.world_position - sphere->origin).Normalized();
				break;
			}
			case PrimitiveType::kPlane:
			{
				const Plane* plane{ primitives_factory.GetAs<Plane>(object.primitive_index) };
				shading_input.world_normal = plane->normal;
				break;
			}
			case PrimitiveType::kTriangle:
			{
				const Triangle* triangle{ primitives_factory.GetAs<Triangle>(object.primitive_index) };
				shading_input.world_normal = triangle->normal;
				//TODO
				break;
			}
			case PrimitiveType::kTriangleMesh:
			{
				const TriangleMesh* triangle_mesh{ primitives_factory.GetAs<TriangleMesh>(object.primitive_index) };
				shading_input.world_normal = triangle_mesh->vertices[hit.vertex_indices.value()[0]].normal.value();
				break;
			}
			default:
				//unhandled case
				break;
			}

			if (object.instance_transformation.has_value())
			{
				shading_input.world_position = object.instance_transformation->TransformPoint(shading_input.world_position);
				shading_input.world_normal = object.instance_transformation->TransformNormal(shading_input.world_normal);
			}
			return shading_input;
		}
	};

	class SceneManager final
	{
		std::vector<std::unique_ptr<Scene>> scenes_{};
		uint32_t active_scene_index_{ std::numeric_limits<uint32_t>::max() };

	public:
		template<typename SceneType>
		bool CreateScene(const bool activate_scene = true)
		{
			static_assert(std::is_base_of_v<Scene, SceneType>, "Scene specialization given must derive from Scene");

			scenes_.emplace_back(std::make_unique<SceneType>());
			SceneType* scene = static_cast<SceneType*>(scenes_.back().get());
			scene->Setup();
			if (activate_scene)
				active_scene_index_ = static_cast<uint32_t>(scenes_.size() - 1);
			return true;
		}

		bool ActivateScene(const uint32_t index)
		{
			if (index >= static_cast<uint32_t>(scenes_.size()))
				return false;
			active_scene_index_ = index;
			scenes_[active_scene_index_]->scene_changed = true;
			return true;
		}

		void BeginFrame()
		{
			if (active_scene_index_ >= static_cast<uint32_t>(scenes_.size()))
				return;

			// Reset the dirty flag from the previous frame.
			// Input and Update during this frame will set it again if needed.
			scenes_[active_scene_index_]->scene_changed = false;
		}

		void UpdateActiveScene(const double delta_time, const bool update_scene = true)
		{
			if (active_scene_index_ >= static_cast<uint32_t>(scenes_.size()))
				return;

			Scene* scene = scenes_[active_scene_index_].get();

			// Propagate component-level changes into scene_changed.
			if (scene->camera.IsDirty())
			{
				scene->scene_changed = true;
				scene->camera.ClearDirty();
			}

			// Update the active scene. If it returns true, scene data changed.
			if (update_scene && scene->Update(delta_time))
				scene->scene_changed = true;

			// Afterward pre-cache the inverse matrices.
			for (SceneObject& object : scene->objects)
			{
				if (object.instance_transformation.has_value())
				{
					const Matrix& inverse = object.instance_transformation->GetInverse();
					(void)inverse;
				}
			}
		}

		[[nodiscard]] Scene* GetActiveScene() const
		{
			if (active_scene_index_ >= static_cast<uint32_t>(scenes_.size()))
				return nullptr;
			return scenes_[active_scene_index_].get();
		}

		void NextScene()
		{
			if (!scenes_.empty())
			{
				active_scene_index_ = (active_scene_index_ + 1) % static_cast<uint32_t>(scenes_.size());
				scenes_[active_scene_index_]->scene_changed = true;
			}
		}

		void PreviousScene()
		{
			if (!scenes_.empty())
			{
				active_scene_index_ = (active_scene_index_ + static_cast<uint32_t>(scenes_.size()) - 1)
					% static_cast<uint32_t>(scenes_.size());
				scenes_[active_scene_index_]->scene_changed = true;
			}
		}
	};
}
#endif //SCENE_MANAGER_HEADER