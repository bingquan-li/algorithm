#include <mujoco/mujoco.h>

#include "viewer.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string_view>

namespace
{
void
step_simulation(mjModel const* model, mjData* data)
{
  // data->ctrl[0] = std::clamp(-8.0 * data->qpos[0] - 2.0 * data->qvel[0], -10.0, 10.0);
  data->ctrl[0] = 0;
  mj_step(model, data);
}
} // namespace

int
main(int argc, char** argv)
{
  bool headless = false;
  char const* model_path = nullptr;
  for(int i = 1; i < argc; ++i)
  {
    std::string_view const arg(argv[i]);
    if(arg == "--help")
    {
      std::cout << "Usage: " << argv[0] << " [--headless] [pendulum.xml]\n";
      return EXIT_SUCCESS;
    }
    if(arg == "--headless")
    {
      headless = true;
    }
    else if(!arg.starts_with("-") && (model_path == nullptr))
    {
      model_path = argv[i];
    }
    else
    {
      std::cerr << "Unexpected argument: " << arg << '\n';
      return EXIT_FAILURE;
    }
  }
  if(mj_version() != mjVERSION_HEADER)
  {
    std::cerr << "MuJoCo header/library version mismatch\n";
    return EXIT_FAILURE;
  }

  // RAII: data is destroyed before the model, including on early returns.
  if(model_path == nullptr)
  {
    model_path = MUJOCO_PENDULUM_MODEL;
  }
  char error[1024] = {};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(model_path, nullptr, error, sizeof(error)),
      mj_deleteModel);
  if(!model)
  {
    std::cerr << "Failed to load " << model_path << ": " << error << '\n';
    return EXIT_FAILURE;
  }
  // This controller expects the supplied single-joint, direct-drive model.
  int const hinge = mj_name2id(model.get(), mjOBJ_JOINT, "hinge");
  int const motor = mj_name2id(model.get(), mjOBJ_ACTUATOR, "motor");
  if(model->nq != 1 || model->nv != 1 || model->nu != 1 || hinge < 0 ||
     motor < 0 || model->jnt_type[hinge] != mjJNT_HINGE)
  {
    std::cerr << "Expected a single hinge joint named 'hinge' and actuator "
                 "named 'motor'\n";
    return EXIT_FAILURE;
  }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()),
      mj_deleteData);
  if(!data)
  {
    std::cerr << "Failed to allocate simulation data\n";
    return EXIT_FAILURE;
  }

  data->qpos[0] = 0.8; // Initial angle in radians; zero points downward.
  mj_forward(model.get(), data.get());
  if(!headless)
  {
    return run_viewer(model.get(), data.get(), step_simulation);
  }
  std::cout << "MuJoCo " << mj_versionString() << "\nModel: " << model_path
            << "\nPD control: target = 0 rad, initial angle = 0.8 rad"
            << "\ntime(s)  angle(rad)  velocity(rad/s)  torque(N*m)\n"
            << std::fixed << std::setprecision(4);

  constexpr int steps = 2500; // Five simulated seconds with the supplied XML.
  for(int step = 0; step <= steps; ++step)
  {
    // Apply PD torque to drive the pendulum toward its downward equilibrium.
    data->ctrl[motor] =
        std::clamp(-8.0 * data->qpos[0] - 2.0 * data->qvel[0], -10.0, 10.0);
    if(step % 250 == 0)
    {
      std::cout << data->time << "  " << data->qpos[0] << "  " << data->qvel[0]
                << "  " << data->ctrl[motor] << '\n';
    }
    if(step == steps)
    {
      break;
    }
    step_simulation(model.get(), data.get());
    if(!std::isfinite(data->qpos[0]) || !std::isfinite(data->qvel[0]))
    {
      std::cerr << "Simulation produced a non-finite state\n";
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
