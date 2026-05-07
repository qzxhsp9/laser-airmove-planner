#pragma once

#include "airmove/CollisionWorld.hpp"
#include "airmove/Types.hpp"

#include <filesystem>

namespace airmove {

PlanningProblem loadPlanningProblemJson(const std::filesystem::path& path);
CollisionWorld buildCollisionWorld(const PlanningProblem& problem);

void writePathCsv(const std::filesystem::path& path, const Path3& points);
void writeTrajectoryCsv(const std::filesystem::path& path, const std::vector<TrajectorySample>& trajectory);
void writeSummaryJson(const std::filesystem::path& path, const PlanningResult& result);
void writeGCodePrototype(const std::filesystem::path& path, const Path3& path_points);
void writePlanningOutputs(const std::filesystem::path& output_dir, const PlanningResult& result);

} // namespace airmove
