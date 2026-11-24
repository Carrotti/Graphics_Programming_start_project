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
#include <stack>

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
			RayHitRecord& closest_hit, bool ignore_record = false, bool reverse_culling = false) const
		{
			[[maybe_unused]] const bool culling_flag{ reverse_culling || ignore_record };

			bool did_hit_anything{ false };
			for (int object_index{ 0 }; object_index < objects.size(); object_index++)
			{
				const SceneObject& object{ objects[object_index] };
				const Primitive* prim{ primitives_factory.Get(object.primitive_index) };
				if (prim == nullptr) continue;
				const Ray& transformed_ray{
					object.instance_transformation.has_value() ?
					object.instance_transformation->TransformRay(ray) :
					ray
				};

				if (prim->bounding_box.has_value())
				{
					if (!HitTestAABB(prim->bounding_box.value(), transformed_ray)) continue;
				}

				RayHitRecord candidate_closest_hit{};
				bool did_hit{ false };
				if (prim->type != PrimitiveType::kTriangleMesh)
				{
					switch (prim->type)
					{
					case PrimitiveType::kSphere:
					{
						const Sphere* sphere{ primitives_factory.GetAs<Sphere>(object.primitive_index) };
						did_hit = HitTestSphere(*sphere, transformed_ray, candidate_closest_hit, ignore_record, culling_flag);
						break;
					}
					case PrimitiveType::kPlane:
					{
						const Plane* plane{ primitives_factory.GetAs<Plane>(object.primitive_index) };
						did_hit = HitTestPlane(*plane, transformed_ray, candidate_closest_hit, ignore_record, culling_flag);
						break;
					}
					case PrimitiveType::kTriangle:
					{
						const Triangle* triangle{ primitives_factory.GetAs<Triangle>(object.primitive_index) };
						did_hit = HitTestTriangle(*triangle, transformed_ray, candidate_closest_hit, ignore_record, culling_flag);
						break;
					}
					}
					if (did_hit) did_hit_anything = true;
					if (did_hit && candidate_closest_hit.t < closest_hit.t && !ignore_record)
					{
						candidate_closest_hit.object_index = object_index;
						closest_hit = candidate_closest_hit;
					}
				}
				else
				{
					const TriangleMesh* triangle_mesh{ primitives_factory.GetAs<TriangleMesh>(object.primitive_index) };
					if (triangle_mesh->bvh_nodes.empty()) continue;
					std::stack<uint32_t> idx_stack{};
					idx_stack.push(0);
					while (!idx_stack.empty())
					{
						const uint32_t node_idx{ idx_stack.top() };
						idx_stack.pop();
						const BVHNode& node{ triangle_mesh->bvh_nodes[node_idx] };
						if (!HitTestAABB(node.bounds, transformed_ray)) continue;
						if (node.IsLeaf())
						{
							//test triangles
							for (uint32_t bvh_idx{ node.first_tri }; bvh_idx < node.first_tri + node.tri_count; ++bvh_idx)
							{
								uint32_t tri_idx{ triangle_mesh->bvh_tri_indices[bvh_idx] };
								Triangle triangle(
									triangle_mesh->vertices[triangle_mesh->indices[3 * tri_idx]].position,
									triangle_mesh->vertices[triangle_mesh->indices[3 * tri_idx + 1]].position,
									triangle_mesh->vertices[triangle_mesh->indices[3 * tri_idx + 2]].position,
									triangle_mesh->cull_mode
								);
								did_hit = HitTestTriangle(triangle, transformed_ray, candidate_closest_hit, ignore_record, culling_flag);
								if (did_hit) did_hit_anything = true;
								if (did_hit && candidate_closest_hit.t < closest_hit.t && !ignore_record)
								{
									closest_hit = candidate_closest_hit;
									closest_hit.object_index = object_index;
									closest_hit.vertex_indices = {
										triangle_mesh->indices[3 * tri_idx],
										triangle_mesh->indices[3 * tri_idx + 1],
										triangle_mesh->indices[3 * tri_idx + 2]
									};
								}
							}
						}
						else
						{
							idx_stack.push(node.left_child);
							idx_stack.push(node.left_child + 1);
						}
					}
				}
			}
			return did_hit_anything;
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