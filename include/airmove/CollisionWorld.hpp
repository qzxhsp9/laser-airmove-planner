#pragma once

#include "airmove/Types.hpp"

#include <fcl/fcl.h>
#include <memory>
#include <string>
#include <vector>

namespace airmove {

class CollisionWorld {
public:
    CollisionWorld(double head_radius, double safety_margin);

    void addBoxObstacle(const Vec3& center, const Vec3& size_xyz);
    void addSphereObstacle(const Vec3& center, double radius);
    void addCylinderObstacle(const Vec3& center, double radius, double height);
    void addMeshObstacle(const std::vector<Vec3>& vertices,
                         const std::vector<Eigen::Vector3i>& triangles);
    void addAsciiStlObstacle(const std::string& path);

    bool isStateValid(const Vec3& tcp_position) const;
    bool isSegmentValid(const Vec3& start, const Vec3& goal, double resolution) const;
    double distanceToNearestObstacle(const Vec3& tcp_position) const;

private:
    using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;
    using CollisionObjectPtr = std::shared_ptr<fcl::CollisionObjectd>;

    double head_radius_{0.0};
    double safety_margin_{0.0};
    std::vector<CollisionObjectPtr> obstacles_;
};

} // namespace airmove
