#pragma once

#include "airmove/Types.hpp"

namespace airmove {

// 用 Catmull-Rom 形式生成三次 B-spline 风格平滑路径。
// 工程上可替换为严格 clamped B-spline / NURBS 实现。
class BSplineSmoother {
public:
    Path3 smooth(const Path3& polyline, int samples) const;
};

} // namespace airmove
