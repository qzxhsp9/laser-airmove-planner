#include "airmove/BSpline.hpp"

#include <algorithm>

namespace airmove {
namespace {

Vec3 catmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, double t) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    return 0.5 * ((2.0 * p1) +
                  (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

} // namespace

Path3 BSplineSmoother::smooth(const Path3& polyline, int samples) const {
    if (polyline.size() < 3 || samples <= static_cast<int>(polyline.size())) {
        return polyline;
    }

    Path3 result;
    result.reserve(static_cast<std::size_t>(samples));
    result.push_back(polyline.front());

    const int segments = static_cast<int>(polyline.size()) - 1;
    const int samples_per_segment = std::max(2, samples / segments);

    for (int i = 0; i < segments; ++i) {
        const Vec3& p0 = polyline[std::max(0, i - 1)];
        const Vec3& p1 = polyline[i];
        const Vec3& p2 = polyline[i + 1];
        const Vec3& p3 = polyline[std::min(static_cast<int>(polyline.size()) - 1, i + 2)];

        for (int k = 1; k <= samples_per_segment; ++k) {
            const double t = static_cast<double>(k) / samples_per_segment;
            result.push_back(catmullRom(p0, p1, p2, p3, t));
        }
    }

    if ((result.back() - polyline.back()).norm() > 1e-9) {
        result.push_back(polyline.back());
    }
    return result;
}

} // namespace airmove
