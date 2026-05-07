#include "airmove/AirMovePlanner.hpp"
#include "airmove/BSpline.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace airmove;

    CollisionWorld world(1.0, 0.5);
    world.addBoxObstacle(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 10.0, 10.0));

    assert(!world.isStateValid(Vec3(0.0, 0.0, 0.0)));
    assert(world.isStateValid(Vec3(20.0, 0.0, 0.0)));
    assert(!world.isSegmentValid(Vec3(-20.0, 0.0, 0.0), Vec3(20.0, 0.0, 0.0), 1.0));
    assert(world.isSegmentValid(Vec3(-20.0, 20.0, 0.0), Vec3(20.0, 20.0, 0.0), 1.0));

    Path3 line{Vec3(0.0, 0.0, 0.0), Vec3(3.0, 4.0, 0.0)};
    assert(std::abs(pathLength(line) - 5.0) < 1e-9);

    Path3 polyline{
        Vec3(0.0, 0.0, 0.0),
        Vec3(1.0, 1.0, 0.0),
        Vec3(2.0, 0.0, 0.0),
    };
    auto smooth = BSplineSmoother().smooth(polyline, 12);
    assert(smooth.size() > polyline.size());
    assert((smooth.front() - polyline.front()).norm() < 1e-9);
    assert((smooth.back() - polyline.back()).norm() < 1e-9);

    return 0;
}
