#pragma once

#include <Eigen/Eigen>

#include <string_view>
#include <vector>

namespace algorithms
{

std::vector<Eigen::Vector2d>
load_path_from_csv(std::string_view file_path);

} // namespace algorithms