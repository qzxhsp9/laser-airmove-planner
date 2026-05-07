#pragma once

#include "airmove/Types.hpp"

namespace airmove {

class BSplineSmoother {
public:
    Path3 smooth(const Path3& polyline, int samples) const;
};

} // namespace airmove
