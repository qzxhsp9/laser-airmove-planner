#include "airmove/CollisionWorld.hpp"
#include "airmove/StlMeshLoader.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace airmove {

CollisionWorld::CollisionWorld(double head_radius, double safety_margin)
    : head_radius_(head_radius), safety_margin_(safety_margin) {}

void CollisionWorld::addBoxObstacle(const Vec3& center, const Vec3& size_xyz) {
    auto box = std::make_shared<fcl::Boxd>(size_xyz.x(), size_xyz.y(), size_xyz.z());
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = center;
    obstacles_.push_back(std::make_shared<fcl::CollisionObjectd>(box, tf));
}

void CollisionWorld::addSphereObstacle(const Vec3& center, double radius) {
    if (radius <= 0.0) {
        throw std::invalid_argument("Sphere obstacle radius must be positive.");
    }

    auto sphere = std::make_shared<fcl::Sphered>(radius);
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = center;
    obstacles_.push_back(std::make_shared<fcl::CollisionObjectd>(sphere, tf));
}

void CollisionWorld::addCylinderObstacle(const Vec3& center, double radius, double height) {
    if (radius <= 0.0 || height <= 0.0) {
        throw std::invalid_argument("Cylinder obstacle radius and height must be positive.");
    }

    auto cylinder = std::make_shared<fcl::Cylinderd>(radius, height);
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = center;
    obstacles_.push_back(std::make_shared<fcl::CollisionObjectd>(cylinder, tf));
}

void CollisionWorld::addMeshObstacle(const std::vector<Vec3>& vertices,
                                     const std::vector<Eigen::Vector3i>& triangles) {
    if (vertices.empty() || triangles.empty()) {
        throw std::invalid_argument("Mesh obstacle requires vertices and triangles.");
    }

    std::vector<fcl::Vector3d> fcl_vertices;
    fcl_vertices.reserve(vertices.size());
    for (const auto& v : vertices) {
        fcl_vertices.emplace_back(v.x(), v.y(), v.z());
    }

    std::vector<fcl::Triangle> fcl_triangles;
    fcl_triangles.reserve(triangles.size());
    for (const auto& t : triangles) {
        fcl_triangles.emplace_back(t.x(), t.y(), t.z());
    }

    auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    model->beginModel();
    model->addSubModel(fcl_vertices, fcl_triangles);
    model->endModel();

    obstacles_.push_back(std::make_shared<fcl::CollisionObjectd>(model));
}

void CollisionWorld::addAsciiStlObstacle(const std::string& path) {
    const auto mesh = loadAsciiStl(path);
    addMeshObstacle(mesh.vertices, mesh.triangles);
}

bool CollisionWorld::isStateValid(const Vec3& tcp_position) const {
    const double radius = head_radius_ + safety_margin_;
    auto head = std::make_shared<fcl::Sphered>(radius);
    fcl::Transform3d head_tf = fcl::Transform3d::Identity();
    head_tf.translation() = tcp_position;
    auto head_obj = std::make_shared<fcl::CollisionObjectd>(head, head_tf);

    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;

    for (const auto& obstacle : obstacles_) {
        result.clear();
        fcl::collide(head_obj.get(), obstacle.get(), request, result);
        if (result.isCollision()) {
            return false;
        }
    }
    return true;
}

bool CollisionWorld::isSegmentValid(const Vec3& start, const Vec3& goal, double resolution) const {
    if (resolution <= 0.0) {
        throw std::invalid_argument("Segment validity resolution must be positive.");
    }

    const double length = (goal - start).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / resolution)));
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        if (!isStateValid(start + t * (goal - start))) {
            return false;
        }
    }
    return true;
}

double CollisionWorld::distanceToNearestObstacle(const Vec3& tcp_position) const {
    if (obstacles_.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    auto head = std::make_shared<fcl::Sphered>(head_radius_);
    fcl::Transform3d head_tf = fcl::Transform3d::Identity();
    head_tf.translation() = tcp_position;
    auto head_obj = std::make_shared<fcl::CollisionObjectd>(head, head_tf);

    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    double min_distance = std::numeric_limits<double>::infinity();

    for (const auto& obstacle : obstacles_) {
        result.clear();
        const double distance = fcl::distance(head_obj.get(), obstacle.get(), request, result);
        min_distance = std::min(min_distance, distance);
    }

    return min_distance - safety_margin_;
}

} // namespace airmove
