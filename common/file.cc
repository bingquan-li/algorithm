#include "file.hh"

#include <fstream>


namespace algorithms
{

std::vector<Eigen::Vector2d>
load_path_from_csv(std::string_view file_path)
{
  std::string const path(file_path);
  std::ifstream input(path);
  if(!input.is_open())
  {
    throw std::runtime_error("Cannot open path CSV: " + path);
  }

  auto const trim = [](std::string const& value)
  {
    auto const first = value.find_first_not_of(" \t\r\n");
    if(first == std::string::npos)
    {
      return std::string{};
    }
    auto const last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  };

  std::vector<Eigen::Vector2d> points;
  std::string line;
  std::size_t line_number = 0;
  bool first_row = true;
  while(std::getline(input, line))
  {
    ++line_number;
    // Accept UTF-8 CSV files with a byte order mark.
    if(line_number == 1 && line.starts_with("\xEF\xBB\xBF"))
    {
      line.erase(0, 3);
    }
    if(trim(line).empty())
    {
      continue;
    }

    std::istringstream row(line);
    std::string x_field;
    std::string y_field;
    std::getline(row, x_field, ',');
    std::getline(row, y_field, ',');
    x_field = trim(x_field);
    y_field = trim(y_field);
    if(first_row && x_field == "x" && y_field == "y")
    {
      first_row = false;
      continue;
    }
    first_row = false;

    auto const parse_coordinate = [&](std::string const& field)
    {
      std::istringstream value(field);
      double coordinate = 0;
      if(!(value >> coordinate) || !(value >> std::ws).eof() ||
         !std::isfinite(coordinate))
      {
        throw std::runtime_error("Invalid x/y coordinate in " + path +
                                 " at line " + std::to_string(line_number));
      }
      return coordinate;
    };
    double const x = parse_coordinate(x_field);
    double const y = parse_coordinate(y_field);
    points.emplace_back(x, y);
  }
  if(input.bad())
  {
    throw std::runtime_error("Failed to read path CSV: " + path);
  }
  return points;
}
} // namespace algorithms