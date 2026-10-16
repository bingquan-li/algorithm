#include "grid_map.hh"

#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

namespace
{
void
check(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

template <typename Exception, typename Function>
void
expect_exception(Function function)
{
  try
  {
    function();
  }
  catch(Exception const&)
  {
    return;
  }
  throw std::runtime_error("Expected exception was not thrown");
}

struct TestFile
{
  std::filesystem::path path;
  ~TestFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }

  void
  write(std::string const& text) const
  {
    std::ofstream output(path);
    output << text;
    output.close();
    check(static_cast<bool>(output), "Failed to write test fixture");
  }
};
} // namespace

int
main(int argc, char** argv)
{
  try
  {
    check(argc == 3, "Expected data directory and temporary file path");
    TestFile file{argv[2]};
    auto const source = nlohmann::json::parse(R"({"local_map": {
      "width": 3, "height": 2, "map_scale": 0.25, "origin": [-2.0, 1.5],
      "data": [[0, 1, 2], [3, 254, 255]]
    }})");
    file.write(source.dump());
    auto grid = algorithms::load_grid_map_json(file.path);
    check(grid.geometry().width == 3 && grid.geometry().height == 2,
          "Incorrect dimensions");
    check(grid.geometry().resolution == 0.25 &&
              grid.geometry().origin_x == -2.0 &&
              grid.geometry().origin_y == 1.5,
          "Incorrect geometry");
    check(grid(1, 0) == 1 && grid.at(0, 1) == 3 && grid(2, 1) == 255 &&
              grid.cells()[4] == 254,
          "Incorrect row-major cell order");
    check(grid.contains(2, 1) && !grid.contains(3, 1) && !grid.contains(-1, 0),
          "Incorrect grid bounds");
    auto reversed = source;
    reversed["local_map"]["row_order"] = "top_to_bottom";
    file.write(reversed.dump());
    auto const flipped = algorithms::load_grid_map_json(file.path);
    check(flipped(2, 0) == 255 && flipped(1, 1) == 1, "Incorrect row reversal");

    auto reject = [&](auto mutate)
    {
      auto invalid = source;
      mutate(invalid["local_map"]);
      file.write(invalid.dump());
      expect_exception<std::invalid_argument>(
          [&] { (void)algorithms::load_grid_map_json(file.path); });
    };
    for(auto const& value : nlohmann::json::array({-1, 0, 1.5, true, "3"}))
    {
      reject([&](auto& map) { map["width"] = value; });
      reject([&](auto& map) { map["height"] = value; });
    }
    for(auto const& value :
        nlohmann::json::array({-1, 256, 1.5, true, nullptr}))
    {
      reject([&](auto& map) { map["data"][0][0] = value; });
    }
    reject([](auto& map) { map["data"].erase(0); });
    reject([](auto& map) { map["data"][0].erase(0); });
    reject([](auto& map) { map["origin"] = {0}; });
    reject([](auto& map) { map["map_scale"] = 0; });
    reject([](auto& map) { map["map_scale"] = -0.1; });
    reject([](auto& map) { map.erase("origin"); });
    reject([](auto& map) { map["row_order"] = "unknown"; });
    reject([](auto& map) { map["rotation_xyzw"] = {0, 0, 1, 0}; });
    file.write("{");
    expect_exception<std::invalid_argument>(
        [&] { (void)algorithms::load_grid_map_json(file.path); });
    std::filesystem::remove(file.path);
    expect_exception<std::runtime_error>(
        [&] { (void)algorithms::load_grid_map_json(file.path); });

    std::filesystem::path const data_dir(argv[1]);
    for(auto const* name : {"narrow_passage_reversing",
                            "narrow_passage_reverseing2",
                            "tight_u_turn"})
    {
      auto const path = data_dir / name / "frame.json";
      std::ifstream input(path);
      auto const document = nlohmann::json::parse(input);
      auto const& map = document.at("local_map");
      auto const loaded = algorithms::load_grid_map_json(path);
      check(loaded.geometry().width == 300 && loaded.geometry().height == 300,
            "Incorrect full map dimensions");
      check(loaded.geometry().resolution == map.at("map_scale").get<double>() &&
                loaded.geometry().origin_x ==
                    map.at("origin")[0].get<double>() &&
                loaded.geometry().origin_y == map.at("origin")[1].get<double>(),
            "Incorrect full map geometry");
      for(std::size_t y = 0; y < 300; ++y)
      {
        for(std::size_t x = 0; x < 300; ++x)
        {
          check(loaded(x, y) == map.at("data")[y][x].get<int>(),
                "Full map cell differs from source");
        }
      }
    }
    // 示例省略了数据，严格加载时应报告尺寸不匹配。
    expect_exception<std::invalid_argument>(
        [&]
        { (void)algorithms::load_grid_map_json(data_dir / "gridmap.json"); });
    algorithms::GridMap<double> numeric_grid({2, 3, 0.1, 0, 0}, 1.5);
    check(numeric_grid.at(1, 2) == 1.5, "Generic GridMap must remain usable");
    std::cout << "Grid map JSON tests passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
