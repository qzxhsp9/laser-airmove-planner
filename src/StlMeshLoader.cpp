#include "airmove/StlMeshLoader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace airmove {

TriangleMesh loadAsciiStl(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open STL file: " + path);
    }

    TriangleMesh mesh;
    std::string line;
    std::vector<Vec3> current;

    while (std::getline(in, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "vertex") {
            double x = 0.0, y = 0.0, z = 0.0;
            iss >> x >> y >> z;
            current.emplace_back(x, y, z);
            if (current.size() == 3) {
                const int base = static_cast<int>(mesh.vertices.size());
                mesh.vertices.push_back(current[0]);
                mesh.vertices.push_back(current[1]);
                mesh.vertices.push_back(current[2]);
                mesh.triangles.emplace_back(base, base + 1, base + 2);
                current.clear();
            }
        }
    }

    if (mesh.triangles.empty()) {
        throw std::runtime_error("No triangles parsed. This loader supports ASCII STL only: " + path);
    }

    return mesh;
}

} // namespace airmove
