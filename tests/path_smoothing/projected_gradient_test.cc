#include "path_smoothing.hh"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

using Points = std::vector<Eigen::Vector2d>;
using path_smoothing::Options;
using path_smoothing::ProjectedGradientOptions;
using path_smoothing::RegionShape;
using path_smoothing::Result;

void
check(bool condition, std::string const& message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

void
check_near(double actual,
           double expected,
           double tolerance,
           std::string const& message)
{
  if(!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
  {
    std::ostringstream error;
    error << message << ": expected " << std::setprecision(17) << expected
          << ", actual " << actual << ", tolerance " << tolerance;
    throw std::runtime_error(error.str());
  }
}

template <typename Exception, typename Function>
std::string
expect_exception(Function const& function, std::string const& context)
{
  try
  {
    function();
  }
  catch(Exception const& exception)
  {
    return exception.what();
  }
  catch(std::exception const& exception)
  {
    throw std::runtime_error(context +
                             ": unexpected exception: " + exception.what());
  }
  throw std::runtime_error(context + ": expected an exception");
}

ProjectedGradientOptions&
solver(Options& options)
{
  return std::get<ProjectedGradientOptions>(options.solver);
}

Points
three_points()
{
  return {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
}

Options
analytic_options()
{
  Options options;
  options.problem.reference_weight = 2.0;
  options.problem.smoothness_weight = 20.0;
  options.problem.tolerance = 1e-8;
  solver(options).max_displacement = 2.0;
  return options;
}

class ClogCapture
{
public:
  ClogCapture()
    : previous_(std::clog.rdbuf(buffer_.rdbuf()))
  {}
  ~ClogCapture()
  {
    std::clog.rdbuf(previous_);
  }
  ClogCapture(ClogCapture const&) = delete;
  ClogCapture&
  operator=(ClogCapture const&) = delete;
  std::string
  str() const
  {
    return buffer_.str();
  }

private:
  std::ostringstream buffer_;
  std::streambuf* previous_;
};

std::pair<Result, std::string>
run_captured(Points const& points, Options const& options)
{
  ClogCapture capture;
  auto result = path_smoothing::smooth(points, options);
  return {std::move(result), capture.str()};
}

struct LoggedRow
{
  std::size_t iteration;
  double objective;
  double max_step{};
  double pg_norm{};
  std::size_t clipped{};
  double violation{};
};

std::vector<LoggedRow>
read_rows(std::string const& log)
{
  std::vector<LoggedRow> rows;
  std::istringstream lines(log);
  std::string line;
  while(std::getline(lines, line))
  {
    auto const first = line.find_first_not_of(" \t");
    if(first == std::string::npos || line[first] < '0' || line[first] > '9')
    {
      continue;
    }
    std::istringstream fields(line);
    LoggedRow row{};
    check(static_cast<bool>(fields >> row.iteration >> row.objective),
          "cannot parse iteration row: " + line);
    check(std::isfinite(row.objective), "non-finite logged objective");
    double reference = 0.0;
    double smoothness = 0.0;
    std::string max_step;
    std::string pg_norm;
    std::string clipped;
    check(static_cast<bool>(fields >> reference >> smoothness >> max_step >>
                            pg_norm >> clipped >> row.violation),
          "cannot parse iteration diagnostics: " + line);
    if(row.iteration > 0)
    {
      row.max_step = std::stod(max_step);
      row.pg_norm = std::stod(pg_norm);
      row.clipped = std::stoull(clipped);
    }
    rows.push_back(row);
  }
  return rows;
}

double
objective(Points const& points, Points const& origin, Options const& options)
{
  double value = 0.0;
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    value += options.problem.reference_weight *
             (points[i] - origin[i]).squaredNorm();
  }
  for(std::size_t i = 1; i + 1 < points.size(); ++i)
  {
    value += options.problem.smoothness_weight *
             (points[i - 1] - 2.0 * points[i] + points[i + 1]).squaredNorm();
  }
  return value;
}

void
check_objective_log(std::string const& log,
                    Result const& result,
                    Points const& origin,
                    Options const& options)
{
  auto const rows = read_rows(log);
  check(!rows.empty(), "verbose log has no iteration rows");
  check(rows.back().iteration == result.iterations,
        "last logged iteration does not match result");
  double const expected = objective(result.points, origin, options);
  double const rounding_tolerance = 1e-6 * std::max(1.0, std::abs(expected));
  check_near(rows.back().objective,
             expected,
             rounding_tolerance,
             "last iteration objective does not match returned points");
  auto const position = log.rfind("objective:");
  check(position != std::string::npos, "missing summary objective");
  std::istringstream summary(
      log.substr(position + std::string("objective:").size()));
  double actual = 0.0;
  check(static_cast<bool>(summary >> actual), "cannot parse summary objective");
  check_near(actual,
             expected,
             rounding_tolerance,
             "summary objective does not match returned points");
}

void
analytic_solution()
{
  auto const points = three_points();
  for(double const weight : {0.0, 0.5, 2.0, 20.0})
  {
    auto options = analytic_options();
    options.problem.smoothness_weight = weight;
    options.problem.tolerance = 1e-11;
    auto const result = path_smoothing::smooth(points, options);
    check(result.converged, "three-point problem failed to converge");
    // With fixed endpoints, J(y) = w_ref * (y - 1)^2 + 4 * w_smooth * y^2.
    double const expected = 2.0 / (2.0 + 4.0 * weight);
    check_near(result.points[1].y(),
               expected,
               1e-9,
               "analytic solution, smoothness_weight=" +
                   std::to_string(weight));
    check_near(result.points[1].x(), 1.0, 1e-12, "middle x");
    if(weight == 0.0)
    {
      check(result.iterations == 1,
            "zero smoothness should converge in one update");
      check_near(result.max_step, 0.0, 0.0, "zero smoothness displacement");
    }
  }
}

void
finite_difference_gradient()
{
  Points const origin{{-2.0, 1.0},
                      {-0.5, -1.0},
                      {1.0, 2.0},
                      {2.0, -0.75},
                      {3.0, 0.25}};
  Options options;
  options.problem.reference_weight = 2.0;
  options.problem.smoothness_weight = 3.0;
  options.problem.fix_endpoints = false;
  options.problem.tolerance = 1e-14;
  solver(options).max_displacement = 100.0;
  solver(options).max_iterator = 1;
  auto const first = path_smoothing::smooth(origin, options);
  solver(options).max_iterator = 2;
  auto const second = path_smoothing::smooth(origin, options);
  check(!first.converged && !second.converged && second.iterations == 2,
        "gradient fixture must perform both updates");

  // With no active constraints, p(k+1) = p(k) - alpha * gradient(p(k)).
  // Recover the gradient from the second update to include a nonzero reference term.
  double const alpha = 0.01;
  double const epsilon = 1e-6;
  for(std::size_t i = 0; i < origin.size(); ++i)
  {
    check((first.points[i] - origin[i]).norm() < 50.0 &&
              (second.points[i] - origin[i]).norm() < 50.0,
          "gradient fixture must not activate projection");
    for(int axis = 0; axis < 2; ++axis)
    {
      auto plus = first.points;
      auto minus = first.points;
      plus[i][axis] += epsilon;
      minus[i][axis] -= epsilon;
      double const numeric = (objective(plus, origin, options) -
                              objective(minus, origin, options)) /
                             (2.0 * epsilon);
      double const analytic =
          (first.points[i][axis] - second.points[i][axis]) / alpha;
      check_near(analytic,
                 numeric,
                 1e-7 * std::max(1.0, std::abs(numeric)),
                 "finite difference at point " + std::to_string(i) + ", axis " +
                     std::to_string(axis));
    }
  }
}

void
projection_constraints()
{
  Points const points{{0.0, 0.0}, {1.0, 2.0}, {4.0, -1.0}, {5.0, 1.0}};
  for(auto const shape : {RegionShape::Circle, RegionShape::Square})
  {
    for(bool const fixed : {false, true})
    {
      Options options;
      options.problem.fix_endpoints = fixed;
      options.problem.tolerance = 1e-10;
      solver(options).shape = shape;
      solver(options).max_displacement = 0.1;
      solver(options).max_iterator = 4000;
      auto const result = path_smoothing::smooth(points, options);
      check(result.converged, "constrained problem did not converge");
      check(result.points.size() == points.size(), "point count changed");
      for(std::size_t i = 0; i < points.size(); ++i)
      {
        check(result.points[i].allFinite(), "non-finite constrained result");
        auto const offset = result.points[i] - points[i];
        double const distance = shape == RegionShape::Circle
                                    ? offset.norm()
                                    : offset.cwiseAbs().maxCoeff();
        check(distance <= 0.1 + 1e-12,
              "projection exceeded displacement bound");
      }
      if(fixed)
      {
        check((result.points.front() - points.front()).norm() == 0.0,
              "fixed first endpoint moved");
        check((result.points.back() - points.back()).norm() == 0.0,
              "fixed last endpoint moved");
      }
      else
      {
        check((result.points.front() - points.front()).norm() > 1e-6,
              "free endpoint unexpectedly remained fixed");
      }
    }
  }
}

void
degenerate_problem()
{
  auto const points = three_points();
  for(std::size_t const maximum : {std::size_t{0}, std::size_t{1000}})
  {
    auto options = analytic_options();
    options.problem.reference_weight = 0.0;
    options.problem.smoothness_weight = 0.0;
    solver(options).max_iterator = maximum;
    auto const result = path_smoothing::smooth(points, options);
    check(result.converged && result.iterations == 0,
          "zero objective should be solved without updates");
    check_near(result.max_step, 0.0, 0.0, "zero objective max_step");
    check(result.points == points, "zero objective changed the input");
  }
  auto options = analytic_options();
  solver(options).max_displacement = 0.0;
  auto const result = path_smoothing::smooth(points, options);
  check(result.converged && result.iterations == 1 && result.points == points,
        "zero-radius feasible set should preserve every point");
}

void
invalid_parameters()
{
  auto const points = three_points();
  double const nan = std::numeric_limits<double>::quiet_NaN();
  double const inf = std::numeric_limits<double>::infinity();
  for(double const value : {-1.0, nan, inf})
  {
    auto options = analytic_options();
    options.problem.reference_weight = value;
    expect_exception<std::invalid_argument>(
        [&] { path_smoothing::smooth(points, options); },
        "reference_weight");
    options = analytic_options();
    options.problem.smoothness_weight = value;
    expect_exception<std::invalid_argument>(
        [&] { path_smoothing::smooth(points, options); },
        "smoothness_weight");
    options = analytic_options();
    solver(options).max_displacement = value;
    expect_exception<std::invalid_argument>(
        [&] { path_smoothing::smooth(points, options); },
        "max_displacement");
  }
  for(double const value : {0.0, -1.0, nan, inf})
  {
    auto options = analytic_options();
    options.problem.tolerance = value;
    expect_exception<std::invalid_argument>(
        [&] { path_smoothing::smooth(points, options); },
        "tolerance");
  }
  auto options = analytic_options();
  solver(options).shape = static_cast<RegionShape>(255);
  expect_exception<std::invalid_argument>(
      [&] { path_smoothing::smooth(points, options); },
      "region shape");
  for(double const value : {nan, inf})
  {
    auto invalid_points = points;
    invalid_points[1].x() = value;
    expect_exception<std::invalid_argument>(
        [&] { path_smoothing::smooth(invalid_points); },
        "non-finite coordinates");
  }
  expect_exception<std::invalid_argument>([] { path_smoothing::smooth({}); },
                                          "empty path");
  expect_exception<std::invalid_argument>(
      [] { path_smoothing::smooth({{0, 0}, {1, 1}}); },
      "two-point path");
}

void
numeric_failure()
{
  auto options = analytic_options();
  Points const huge_points{{0.0, 0.0}, {1e308, 1e308}, {2.0, 0.0}};
  auto message = expect_exception<std::runtime_error>(
      [&] { path_smoothing::smooth(huge_points, options); },
      "overflowing iterate");
  check(message.find("iteration") != std::string::npos,
        "numeric failure does not identify iteration: " + message);
  check(message.find("point") != std::string::npos,
        "numeric failure does not identify point: " + message);

  solver(options).verbose = true;
  {
    ClogCapture capture;
    auto const verbose_message = expect_exception<std::runtime_error>(
        [&] { path_smoothing::smooth(huge_points, options); },
        "verbose overflow");
    check(verbose_message == message, "verbose changed the numerical failure");
    auto const log = capture.str();
    check(log.find("status:               numerical error") !=
                  std::string::npos &&
              log.find(message) != std::string::npos,
          "numeric failure log omitted status or reason");
  }
  solver(options).verbose = false;

  options.problem.reference_weight = std::numeric_limits<double>::denorm_min();
  options.problem.smoothness_weight = 0.0;
  expect_exception<std::runtime_error>(
      [&] { path_smoothing::smooth(three_points(), options); },
      "overflowing step");

  options = analytic_options();
  options.problem.reference_weight = 1e308;
  options.problem.smoothness_weight = 1e308;
  expect_exception<std::runtime_error>(
      [&] { path_smoothing::smooth(three_points(), options); },
      "overflowing Lipschitz bound");
}

void
verbose_equivalence()
{
  auto options = analytic_options();
  auto const points = three_points();
  auto const [quiet_result, quiet_log] = run_captured(points, options);
  check(quiet_log.empty(), "verbose=false emitted a log");
  solver(options).verbose = true;
  auto const [loud_result, loud_log] = run_captured(points, options);
  check(loud_result.points == quiet_result.points &&
            loud_result.iterations == quiet_result.iterations &&
            loud_result.converged == quiet_result.converged &&
            loud_result.max_step == quiet_result.max_step,
        "verbose changed the solution");
  for(auto const* const field :
      {"pg_norm", "clipped", "violation", "stop reason:"})
  {
    check(loud_log.find(field) != std::string::npos,
          std::string("missing diagnostic field: ") + field);
  }
  check_objective_log(loud_log, loud_result, points, options);
  auto const last = read_rows(loud_log).back();
  check_near(last.pg_norm,
             loud_result.max_step * 644.0,
             1e-6 * loud_result.max_step * 644.0,
             "projected gradient norm");
  check(last.clipped == 0 && last.violation <= 1e-12,
        "unconstrained fixture reported clipping or constraint violation");

  solver(options).max_displacement = 0.1;
  solver(options).max_iterator = 1;
  auto const [clipped_result, clipped_log] = run_captured(points, options);
  auto const clipped_last = read_rows(clipped_log).back();
  check(clipped_last.clipped == 1 && clipped_last.violation <= 1e-12,
        "single constrained point has incorrect clipping diagnostics");
  check_near(clipped_last.pg_norm,
             64.4,
             1e-5,
             "constrained projected gradient norm");
  check_objective_log(clipped_log, clipped_result, points, options);

  // A disabled smoothness term must not evaluate huge squared residuals.
  Points const huge_points{{0.0, 0.0}, {1.0, 1e308}, {2.0, 0.0}};
  options.problem.reference_weight = 1.0;
  options.problem.smoothness_weight = 0.0;
  solver(options).verbose = false;
  auto const [huge_quiet, huge_quiet_log] = run_captured(huge_points, options);
  solver(options).verbose = true;
  auto const [huge_loud, huge_log] = run_captured(huge_points, options);
  check(huge_quiet_log.empty() && huge_quiet.points == huge_points &&
            huge_loud.points == huge_quiet.points && huge_loud.converged &&
            huge_loud.iterations == huge_quiet.iterations,
        "disabled smoothness changed the huge-coordinate result");
  for(auto const& row : read_rows(huge_log))
  {
    check_near(row.objective,
               0.0,
               0.0,
               "disabled smoothness logged nonzero cost");
  }
}

void
check_iterations(std::string const& log,
                 std::vector<std::size_t> const& expected)
{
  auto const rows = read_rows(log);
  std::vector<std::size_t> actual;
  actual.reserve(rows.size());
  for(auto const& row : rows)
  {
    actual.push_back(row.iteration);
  }
  if(actual != expected)
  {
    std::ostringstream message;
    message << "wrong logged iterations; expected:";
    for(auto const value : expected)
    {
      message << ' ' << value;
    }
    message << "; actual:";
    for(auto const value : actual)
    {
      message << ' ' << value;
    }
    throw std::runtime_error(message.str());
  }
}

void
verbose_sampling()
{
  // A long wavelength decays slowly enough to reach either iteration budget.
  Points long_path;
  double const pi = std::acos(-1.0);
  for(std::size_t i = 0; i <= 100; ++i)
  {
    long_path.emplace_back(static_cast<double>(i), std::sin(pi * i / 100.0));
  }
  for(std::size_t const maximum : {std::size_t{1000}, std::size_t{1001}})
  {
    auto options = analytic_options();
    options.problem.reference_weight = 0.0;
    options.problem.smoothness_weight = 1.0;
    options.problem.tolerance = 1e-20;
    solver(options).verbose = true;
    solver(options).max_iterator = maximum;
    auto const [result, log] = run_captured(long_path, options);
    check(!result.converged && result.iterations == maximum,
          "sampling fixture should exhaust the iteration budget");
    std::size_t const interval = maximum == 1000 ? 50 : 51;
    std::vector<std::size_t> expected{0, 1};
    for(std::size_t i = interval; i < maximum; i += interval)
    {
      expected.push_back(i);
    }
    expected.push_back(maximum);
    check_iterations(log, expected);
    check_objective_log(log, result, long_path, options);
  }
  auto options = analytic_options();
  solver(options).verbose = true;
  auto const points = three_points();
  auto const [early_result, early_log] = run_captured(points, options);
  check(early_result.converged && early_result.iterations == 59,
        "three-point fixture should converge on iteration 59");
  check_iterations(early_log, {0, 1, 50, 59});
  check_objective_log(early_log, early_result, points, options);
  for(std::size_t const maximum : {std::size_t{0}, std::size_t{1}})
  {
    solver(options).max_iterator = maximum;
    auto const [result, log] = run_captured(points, options);
    check(!result.converged && result.iterations == maximum,
          "short budget should not report convergence");
    check_iterations(log,
                     maximum == 0 ? std::vector<std::size_t>{0}
                                  : std::vector<std::size_t>{0, 1});
    check_objective_log(log, result, points, options);
    if(maximum == 0)
    {
      check(log.find("N/A") != std::string::npos,
            "zero-iteration diagnostics should show N/A");
    }
  }
}

void
stream_format()
{
  auto const old_flags = std::clog.flags();
  auto const old_precision = std::clog.precision();
  auto const old_fill = std::clog.fill();
  struct Restore
  {
    std::ios_base::fmtflags flags;
    std::streamsize precision;
    char fill;
    ~Restore()
    {
      std::clog.flags(flags);
      std::clog.precision(precision);
      std::clog.fill(fill);
    }
  } restore{old_flags, old_precision, old_fill};
  std::clog << std::fixed << std::setprecision(3) << std::setfill('#');
  auto const flags = std::clog.flags();
  auto options = analytic_options();
  solver(options).verbose = true;
  auto const [result, log] = run_captured(three_points(), options);
  check(std::clog.flags() == flags && std::clog.precision() == 3 &&
            std::clog.fill() == '#',
        "verbose polluted std::clog formatting");

  std::ostringstream output;
  output << std::noboolalpha << std::fixed << std::setprecision(3)
         << std::setfill('#');
  auto const output_flags = output.flags();
  output << result;
  check(output.flags() == output_flags && output.precision() == 3 &&
            output.fill() == '#',
        "Result output polluted caller stream formatting");
}

void
unsupported_backend()
{
  Options options;
  options.solver = path_smoothing::OsqpOptions{};
  expect_exception<std::logic_error>(
      [&] { path_smoothing::smooth(three_points(), options); },
      "unimplemented OSQP backend");
}

} // namespace

int
main(int argc, char** argv)
{
  std::vector<std::pair<std::string_view, std::function<void()>>> const cases{
      {"analytic_solution", analytic_solution},
      {"finite_difference_gradient", finite_difference_gradient},
      {"projection_constraints", projection_constraints},
      {"degenerate_problem", degenerate_problem},
      {"invalid_parameters", invalid_parameters},
      {"numeric_failure", numeric_failure},
      {"verbose_equivalence", verbose_equivalence},
      {"verbose_sampling", verbose_sampling},
      {"stream_format", stream_format},
      {"unsupported_backend", unsupported_backend}};

  bool found = false;
  bool failed = false;
  for(auto const& [name, function] : cases)
  {
    if(argc > 1 && name != argv[1])
    {
      continue;
    }
    found = true;
    try
    {
      function();
      std::cout << "PASS " << name << '\n';
    }
    catch(std::exception const& exception)
    {
      failed = true;
      std::cerr << "FAIL " << name << ": " << exception.what() << '\n';
    }
  }

  if(!found)
  {
    std::cerr << "Unknown test case\n";
  }

  return !found || failed ? 1 : 0;
}
