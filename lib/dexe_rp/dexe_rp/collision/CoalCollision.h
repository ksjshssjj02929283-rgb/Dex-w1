/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision_object.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>

#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/collision_data.h"
#include "coal/math/transform.h"
#include "coal/mesh_loader/loader.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/shape/geometric_shapes.h"
#include "dexe_rp/common/CommonHeader.h"

using json = nlohmann::json;

namespace DexeRP {
namespace Collision {

struct Obstacle {
public:
    std::string name_;
    std::shared_ptr<coal::CollisionObject> collision_object_;
    std::shared_ptr<coal::CollisionGeometry> collision_geometry_;
    CVec3d position_;
    CVec4d quaternion_;
    std::string frame_;
    bool is_user_object_{false};
    bool is_enabled_{true};

    Obstacle(const std::string& name,
             const std::shared_ptr<coal::CollisionObject>& collision_object,
             const std::shared_ptr<coal::CollisionGeometry>& collision_geometry,
             const CVec3d& position,
             const CVec4d& quaternion,
             const std::string& frame,
             bool is_user_object = false,
             bool is_enabled = true)
        : name_(name),
          collision_object_(collision_object),
          collision_geometry_(collision_geometry),
          position_(position),
          quaternion_(quaternion),
          frame_(frame),
          is_user_object_(is_user_object),
          is_enabled_(is_enabled) {}
    ~Obstacle() = default;
};

struct RobotLink {
public:
    std::string name_;
    std::shared_ptr<coal::CollisionObject> collision_object_;
    std::shared_ptr<coal::CollisionGeometry> collision_geometry_;
    CVec3d position_;
    CVec4d quaternion_;

    RobotLink(const std::string& name,
              const std::shared_ptr<coal::CollisionObject>& collision_object,
              const std::shared_ptr<coal::CollisionGeometry>& collision_geometry,
              const CVec3d& position,
              const CVec4d& quaternion)
        : name_(name),
          collision_object_(collision_object),
          collision_geometry_(collision_geometry),
          position_(position),
          quaternion_(quaternion) {}
    ~RobotLink() = default;
};

struct RobotComponent {
public:
    std::string name_;
    std::vector<std::shared_ptr<RobotLink>> links_;
    bool object_attached_{false};
    Mat4d flange_T_object_{Mat4d::Identity()};
    std::string attached_object_name_{""};

    RobotComponent(const std::string& name, const std::vector<std::shared_ptr<RobotLink>>& links)
        : name_(name), links_(links) {}
    ~RobotComponent() = default;
};

struct CollisionResult {
public:
    bool has_collision{false};
    std::string object1_name{""};
    std::string object2_name{""};

    void Clear() {
        has_collision = false;
        object1_name = "";
        object2_name = "";
    }
};

class CoalCollision {
public:
    /**
     * @brief Convert CVec3d to coal::Vec3s
     */
    static inline coal::Vec3s ToCoalVec3(const CVec3d& vec) { return coal::Vec3s(vec(0), vec(1), vec(2)); }

    /**
     * @brief Convert CVec4d (wxyz) to coal::Quatf (w, x, y, z)
     */
    static inline coal::Quatf ToCoalQuatWxyz(const CVec4d& q_wxyz) {
        coal::Quatf q(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3));  // w, x, y, z
        // q.normalize(); // Ensure the quaternion is normalized
        return q;
    }

    /**
     * @brief Convert position and quaternion to coal::Transform3s
     */
    static inline coal::Transform3s ToCoalTransform(const CVec3d& position, const CVec4d& quaternion) {
        return coal::Transform3s(ToCoalQuatWxyz(quaternion), ToCoalVec3(position));
    }

    static constexpr std::array<std::string_view, 4> support_obstacle_types = {"box", "sphere", "mesh", "cylinder"};
    static const inline std::array<std::string_view, 4> GetSupportedObstacleTypes() { return support_obstacle_types; }

    static bool IsSupportedObstacleType(std::string_view type) {
        return std::find(support_obstacle_types.begin(), support_obstacle_types.end(), type) !=
               support_obstacle_types.end();
    }

    /**
     * @brief Construct a CoalCollision instance
     *
     * @param link_meshes A vector of tuples, where each tuple contains a component name and a mesh path for the robot
     *        links.
     *
     * @note The component name decides which robot component the link belongs to:
     * - A component can have multiple links
     * - Each component will construct an AABB tree manager used for broadphase collision checking.
     */
    CoalCollision(const std::vector<std::tuple<std::string, std::string, std::string>>& link_meshes,
                  const bool& verbose = true);

    /**
     * @brief Destructor for CoalCollision
     */
    ~CoalCollision() = default;

    /**
     * @brief Create an obstacle from a mesh file
     *
     * Include the coal collision geometry and collision object
     * @param is_user_object indicates whether the obstacle can be attached to robot component
     * @param is_enabled indicates whether the obstacle is enabled for collision checking.
     */
    inline Obstacle CreateObstacleFromMesh(const std::string& name,
                                           const std::string& mesh_path,
                                           const CVec3d& position,
                                           const CVec4d& quaternion,
                                           const std::string& frame,
                                           bool is_user_object = false,
                                           bool is_enabled = true) {
        coal::MeshLoader loader;
        coal::BVHModelPtr_t bvh = loader.load(mesh_path);
        auto collision_geometry = std::static_pointer_cast<coal::CollisionGeometry>(bvh);
        auto collision_object =
            std::make_shared<coal::CollisionObject>(collision_geometry, ToCoalTransform(position, quaternion));
        collision_object->computeAABB();
        return Obstacle(name, collision_object, collision_geometry, position, quaternion, frame, is_user_object,
                        is_enabled);
    }

    /**
     * @brief Create a robot link from a mesh file
     *
     * Include the coal collision geometry and collision object
     */
    inline RobotLink CreateRobotLinkFromMesh(const std::string& name,
                                             const std::string& mesh_path,
                                             const CVec3d& position,
                                             const CVec4d& quaternion) {
        coal::MeshLoader loader;
        coal::BVHModelPtr_t bvh = loader.load(mesh_path);
        auto collision_geometry = std::static_pointer_cast<coal::CollisionGeometry>(bvh);
        auto collision_object =
            std::make_shared<coal::CollisionObject>(collision_geometry, ToCoalTransform(position, quaternion));
        collision_object->computeAABB();
        object_name_map_.emplace(collision_object.get(), name);
        name_object_map_.emplace(name, collision_object.get());
        return RobotLink{name, collision_object, collision_geometry, position, quaternion};
    }

    /**
     * @brief Create robot links from a vector of (component_name, mesh_path) tuples
     *
     * @param link_meshes A vector of tuples, where each tuple contains a component name and a mesh path for the robot
     */
    inline void CreateRobotLinks(const std::vector<std::tuple<std::string, std::string, std::string>>&
                                     link_meshes  // vector of (component_name, mesh_path)
    ) {
        // TODO: 构建robot_link之前是否需要清空已有的links？
        // robot_components_.clear();

        std::unordered_map<std::string, std::size_t> component_index;

        for (const auto& [component_name, link_name, mesh_path] : link_meshes) {
            if (!std::filesystem::exists(mesh_path)) {
                DEXERP_ERROR("Mesh file '{}' does not exist for component '{}'. Please provide a valid path.",
                             mesh_path, component_name);
                continue;
            }
            CVec3d position(0.0, 0.0, 0.0);
            CVec4d quaternion(1.0, 0.0, 0.0, 0.0);  // Identity quaternion (wxyz)
            // std::string link_name = std::filesystem::path(mesh_path).stem().string();
            auto link =
                std::make_shared<RobotLink>(CreateRobotLinkFromMesh(link_name, mesh_path, position, quaternion));
            auto it = component_index.find(component_name);
            if (it == component_index.end()) {
                const std::size_t new_index = robot_components_.size();
                std::shared_ptr<RobotComponent> new_component =
                    std::make_shared<RobotComponent>(component_name, std::vector<std::shared_ptr<RobotLink>>{});
                robot_components_.push_back(new_component);
                robot_components_[new_index]->links_.push_back(link);
                component_index.emplace(component_name, new_index);
            } else {
                robot_components_[it->second]->links_.push_back(link);
            }
        }
    }

    /**
     * @brief Create a DynamicAABBTreeCollisionManager for each robot component, and register the component's links to
     * the manager
     */
    inline void CreateRobotLinkManagers() {
        robot_links_manager_.clear();
        for (const auto& component : robot_components_) {
            if (component->links_.empty()) {
                continue;
            }
            auto manager = std::make_shared<coal::DynamicAABBTreeCollisionManager>();
            for (const auto& link : component->links_) {
                manager->registerObject(link->collision_object_.get());
            }
            manager->setup();
            robot_links_manager_.emplace(component->name_, manager);
        }
    }

    void SetVerbose(bool verbose) { verbose_ = verbose; }

    /**
     * @brief Add an obstacle to the environment
     *
     * @param obstacle_type The type of the obstacle, can be "box", "sphere" or "mesh"
     * @param name The unique name of the obstacle
     * @param position The position of the obstacle in the world frame
     * @param quaternion The orientation of the obstacle in the world frame, represented as a quaternion in (w, x, y, z)
     * format
     * @param obstacle_mesh_path The path to the mesh file for the obstacle, required if the obstacle type is "mesh"
     * @param size The size of the obstacle, required if the obstacle type is "box" or "sphere". For "box", it should be
     * a 3D vector representing the dimensions (length, width, height). For "sphere", it should be a 1D vector
     * representing the radius.
     * @param frame The reference frame of the obstacle's position and orientation, default is "base_link" (not used
     * yet)
     * @param is_user_object Indicates whether the obstacle can be attached to robot components. If true, the obstacle
     * will be considered as a user object and can be attached to robot components. If false, the obstacle will be
     * considered as a static object in the environment and cannot be attached to robot components. Default is false.
     */
    bool AddObstacle(const std::string& obstacle_type,
                     const std::string& name,
                     const CVec3d& position,
                     const CVec4d& quaternion,
                     const std::string& obstacle_mesh_path = "",
                     const CVecXd& size = CVecXd(),
                     const std::string& frame = "base_link",
                     bool is_user_object = false);

    bool AddOctomapObstacle(const std::string& name,
                            const std::shared_ptr<octomap::OcTree>& octree,
                            const Mat4d& world_T_octomap,
                            const std::string& frame = "base_link",
                            bool is_user_object = false);

    /**
     * @brief Enable or disable an obstacle for collision checking
     */
    bool SetObstacleState(const std::string& name, bool enabled);

    /**
     * @brief Update the pose of an obstacle in the environment
     */
    bool TransformObstacle(const std::string& name, const CVec3d& position, const CVec4d& quaternion);

    /**
     * @brief Get the pose of an obstacle in the environment.
     *
     * @return The pose of the obstacle as a 4x4 homogeneous transformation matrix. If the obstacle does not exist,
     * return std::nullopt.
     */
    std::optional<Mat4d> GetObstaclePose(const std::string& name) const;

    /**
     * @brief Load the environment configuration from a JSON file.
     */
    bool LoadEnvironmentFromJson(const std::string& json_path);

    /**
     * @brief Load the ignored collision pairs from a JSON file.
     */
    bool LoadIgnoredCollisionPairsFromJson(const std::string& json_path);

    /**
     * @brief Update the poses of the links in a robot component.
     *
     * @param component_name The name of the robot component to update.
     * @param link_poses A vector of 4x4 homogeneous transformation matrices representing the poses of the links in the
     * robot component.
     *
     * @note The order of link poses should correspond to the order of links in the robot component, which is determined
     * by the order of mesh paths in CreateRobotLinks().
     */
    bool UpdateRobotComponentPose(const std::string& component_name, const std::vector<Mat4d>& link_poses);

    /**
     * @brief Attach an object to a robot component
     *
     * The attached object will be considered as part of the robot component for collision checking, and will move
     * together with the robot component when the component's pose is updated. The pose of the attached object relative
     * to the component's flange is defined by component_flange_T_object.
     *
     * @note The flange link of the robot component is defined as the last link in the component's links vector. The
     * attached object will be attached to the flange link. When the component's pose is updated, the attached object's
     * pose will be updated according to the component_flange_T_object transform.
     */
    bool AttachObjectToComponent(const std::string& component_name,
                                 const std::string& object_name,
                                 const Mat4d& component_flange_T_object);

    /**
     * @brief Detach an object from a robot component
     *
     * @param component_name The name of the robot component to detach from
     * @param component_flange_pose The current pose of the component's flange in the world frame
     */
    bool DetachObjectFromComponent(const std::string& component_name, const Mat4d& component_flange_pose);

    /**
     * @brief Ignore collision between a robot component and an obstacle
     */
    bool IgnoreObstacleWithComponent(const std::string& obstacle_name, const std::string& component_name);

    /**
     * @brief Ignore collision between a robot component and an object
     */
    bool IgnoreObjectWithComponent(const std::string& object_name, const std::string& component_name);

    /**
     * @brief Ignore collision between two collision objects, which can be either robot links or obstacles.
     */
    bool IgnoreObjectsCollision(const std::string& object_name1, const std::string& object_name2);

    /**
     * @brief Collision callback function for coal::CollisionManager. This function will be called during collision
     * checking
     */
    bool CollisionCallback(const coal::CollisionObject* obj1,
                           const coal::CollisionObject* obj2,
                           coal::CollisionRequest& request,
                           coal::CollisionResult& result,
                           bool& done);

    class CoalCollisionBroadphaseCallback : public coal::CollisionCallBackBase {
    public:
        CoalCollisionBroadphaseCallback(CoalCollision& collision,
                                        coal::CollisionRequest& request,
                                        coal::CollisionResult& result,
                                        bool& done)
            : collision_(collision), request_(request), result_(result), done_(done) {}

        bool collide(coal::CollisionObject* obj1, coal::CollisionObject* obj2) override {
            return collision_.CollisionCallback(obj1, obj2, request_, result_, done_);
        }

    private:
        CoalCollision& collision_;
        coal::CollisionRequest& request_;
        coal::CollisionResult& result_;
        bool& done_;
    };

    /**
     * @brief Check collision between a robot component and all obstacles in the environment
     */
    bool CheckComponentCollisionWithObstacles(const std::string& component_name);

    /**
     * @brief Check collision between robot components and obstacles for all components in the environment
     */
    bool CheckComponentCollisionWithObstacles();

    /**
     * @brief Check collision between two specific robot components
     */
    bool CheckComponentsCollision(const std::string& component_name1, const std::string& component_name2);

    /**
     * @brief Check collision between robot components for all components in the environment
     */
    bool CheckComponentsCollision();

    /**
     * @brief Check self-collision within a robot component
     */
    bool CheckComponentSelfCollision(const std::string& component_name);

    /**
     * @brief Check self-collision within robot components for all components in the environment
     */
    bool CheckComponentSelfCollision();

    /**
     * @brief Check collision between a specific robot link and an obstacle
     */
    bool CheckLinkCollisionWithObstacle(const std::string& component_name,
                                        const std::string& link_name,
                                        const std::string& obstacle_name);

    /**
     * @brief Check collision between two specific collision objects, which can be either robot links or obstacles
     */
    bool CheckObjectsCollision(const std::string& object_name1, const std::string& object_name2);

    /**
     * @brief Genral collision checking function that checks collision for the whole scene, including all robot
     * components and obstacles.
     */
    bool CheckCollision();

    /**
     * @brief Export the current scene, including all robot components and obstacles, to a PLY file for visualization.
     */
    bool ExportSceneToPly(const std::string& file_path);

    bool FindLinkInComponent(const std::string& link_name, std::string& component_name, size_t& link_index);

private:
    bool verbose_{true};
    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    std::shared_ptr<coal::DynamicAABBTreeCollisionManager> obstacles_manager_ =
        std::make_shared<coal::DynamicAABBTreeCollisionManager>();
    std::unordered_map<std::string, std::shared_ptr<coal::CollisionObject>> obstacle_name_to_object_;
    std::unordered_map<std::shared_ptr<coal::CollisionObject>, std::string> object_to_obstacle_name_;
    std::unordered_map<const coal::CollisionObject*, std::string> object_name_map_;
    std::unordered_map<std::string, const coal::CollisionObject*> name_object_map_;
    std::vector<std::shared_ptr<RobotComponent>> robot_components_;
    std::unordered_map<std::string, std::shared_ptr<coal::DynamicAABBTreeCollisionManager>> robot_links_manager_;
    std::set<std::pair<std::string, std::string>> ignored_collision_pairs_;
    // TODO: 给每个component都支持添加attach物体，attach的物体需要考虑是否是enabled的？
    // TODO:
    // 传给component的link顺序是否要按照一定的默认顺序，最后一个link就作为默认的法兰link，attached的物体也是attached在这个上面
    // 是否要支持指定attached到component的某个link上？如果支持的话，传给component的link顺序就不重要了，直接指定attached到哪个link上就行了
    // TODO: 碰撞检测的callback函数中不要日志打印，通过CollisionResult来记录碰撞的结果
};

}  // namespace Collision
}  // namespace DexeRP
