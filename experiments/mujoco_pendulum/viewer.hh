#pragma once

#include <mujoco/mujoco.h>

// Runs until the user closes the window. Space pauses; R resets; Esc exits.
int
run_viewer(mjModel const* model,
           mjData* data,
           void (*step)(mjModel const*, mjData*));
