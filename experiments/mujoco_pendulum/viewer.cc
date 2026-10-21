#include "viewer.hh"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>

namespace
{
struct Viewer
{
  mjModel const* model;
  mjData* data;
  mjvCamera camera{};
  mjvOption option{};
  mjvScene scene{};
  mjrContext context{};
  bool paused = false;
  double last_x = 0;
  double last_y = 0;

  Viewer(mjModel const* m, mjData* d)
    : model(m)
    , data(d)
  {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &scene, 1000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);
    camera.lookat[2] = 0.8;
    camera.distance = 3.5;
    camera.azimuth = 90;
    camera.elevation = -15;
  }
  ~Viewer()
  {
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
  }
};

Viewer&
state(GLFWwindow* window)
{
  return *static_cast<Viewer*>(glfwGetWindowUserPointer(window));
}

void
keyboard(GLFWwindow* window, int key, int, int action, int)
{
  if(action != GLFW_PRESS)
  {
    return;
  }
  auto& viewer = state(window);
  if(key == GLFW_KEY_ESCAPE)
  {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
  if(key == GLFW_KEY_SPACE)
  {
    viewer.paused = !viewer.paused;
  }
  if(key == GLFW_KEY_R)
  {
    mj_resetData(viewer.model, viewer.data);
    viewer.data->qpos[0] = 0.8;
    mj_forward(viewer.model, viewer.data);
  }
}

void
mouse_move(GLFWwindow* window, double x, double y)
{
  auto& viewer = state(window);
  double const dx = x - viewer.last_x;
  double const dy = y - viewer.last_y;
  viewer.last_x = x;
  viewer.last_y = y;
  bool const left =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  bool const right =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  bool const middle =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  if(!left && !right && !middle)
  {
    return;
  }
  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if(height <= 0)
    return;
  bool const shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse const action = right  ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                          : left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                                 : mjMOUSE_ZOOM;
  mjv_moveCamera(viewer.model,
                 action,
                 dx / height,
                 dy / height,
                 &viewer.camera);
}
} // namespace

int
run_viewer(mjModel const* model,
           mjData* data,
           void (*step)(mjModel const*, mjData*))
{
  glfwSetErrorCallback(
      [](int code, char const* description)
      { std::cerr << "GLFW error " << code << ": " << description << '\n'; });
  if(glfwInit() == 0)
  {
    std::cerr
        << "Cannot initialize graphics. Use --headless without a display.\n";
    return EXIT_FAILURE;
  }
  GLFWwindow* window =
      glfwCreateWindow(1200, 900, "MuJoCo Pendulum", nullptr, nullptr);
  if(window == nullptr)
  {
    glfwTerminate();
    return EXIT_FAILURE;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  {
    // Free rendering resources before destroying the OpenGL context.
    Viewer viewer(model, data);
    glfwSetWindowUserPointer(window, &viewer);
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window,
                               [](GLFWwindow* w, int, int, int)
                               {
                                 auto& v = state(w);
                                 glfwGetCursorPos(w, &v.last_x, &v.last_y);
                               });
    glfwSetScrollCallback(
        window,
        [](GLFWwindow* w, double, double dy)
        {
          auto& v = state(w);
          mjv_moveCamera(v.model, mjMOUSE_ZOOM, 0, -0.05 * dy, &v.camera);
        });
    double previous = glfwGetTime();
    double accumulated = 0;
    while(glfwWindowShouldClose(window) == 0)
    {
      glfwPollEvents();
      double const now = glfwGetTime();
      // Cap catch-up after a stalled frame to keep the window responsive.
      accumulated =
          viewer.paused ? 0 : accumulated + std::min(now - previous, 0.1);
      previous = now;
      while(accumulated >= model->opt.timestep)
      {
        step(model, data);
        accumulated -= model->opt.timestep;
      }
      mj_forward(model, data);
      mjrRect viewport{};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      if(viewport.width > 0 && viewport.height > 0)
      {
        mjv_updateScene(model,
                        data,
                        &viewer.option,
                        nullptr,
                        &viewer.camera,
                        mjCAT_ALL,
                        &viewer.scene);
        mjr_render(viewport, &viewer.scene, &viewer.context);
        char status[160];
        std::snprintf(status,
                      sizeof(status),
                      "%s | t = %.2f s | angle = %.3f rad",
                      viewer.paused ? "Paused" : "Running",
                      data->time,
                      data->qpos[0]);
        mjr_overlay(mjFONT_NORMAL,
                    mjGRID_TOPLEFT,
                    viewport,
                    "Space: pause | R: reset | Esc: exit\nDrag: rotate/pan | "
                    "Wheel: zoom",
                    status,
                    &viewer.context);
        glfwSwapBuffers(window);
      }
      glfwWaitEventsTimeout(0.001);
    }
  }
  glfwDestroyWindow(window);
  glfwTerminate();
  return EXIT_SUCCESS;
}
