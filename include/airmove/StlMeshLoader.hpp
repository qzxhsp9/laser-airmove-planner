#pragma once

#include "airmove/Types.hpp"

#include <string>
#include <vector>

namespace airmove {

struct TriangleMesh {
    std::vector<Vec3> vertices;
    std::vector<Eigen::Vector3i> triangles;
};

TriangleMesh loadAsciiStl(const std::string& path);

} // namespace airmove
