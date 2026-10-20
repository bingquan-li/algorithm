"""Run reproducible physics and default-controller regression checks without a GUI."""

import numpy as np

# MuJoCo exports these native functions without Python type stubs.
from mujoco import (
    mj_forward,  # pyright: ignore[reportAttributeAccessIssue]
    mj_step,  # pyright: ignore[reportAttributeAccessIssue]
)
from spring_cart import (
    HERE,
    is_single_motion,
    load_config,
    make_model,
    metrics,
    simulate,
)

c = load_config(HERE / "config.json")
m, d, _ = make_model(c)
assert m.nq == 3 and m.nu == 1, "Only cart x, load relative x/z and cart force"
assert np.max(abs(d.qacc)) < 1e-8
for _ in range(1000):
    mj_step(m, d)
assert np.max(abs(d.qvel)) < 1e-10, "Equilibrium must not drift"

m, d, _ = make_model(c)
d.qpos[1] = 0.05
mj_forward(m, d)
assert d.qacc[1] < -0.1, "Horizontal displacement must produce restoring acceleration"
for _ in range(500):
    mj_step(m, d)
assert abs(d.qpos[1] - 0.05) > 0.01, "Swing must remain free"

baseline = metrics(simulate(c), c)
shaped = metrics(simulate(c, 2.4), c)
assert shaped["constraints_ok"] and shaped["settling_time"] is not None
assert shaped["max_sway"] < baseline["max_sway"] * 0.5
shaped_residual = shaped["residual_sway"]
baseline_residual = baseline["residual_sway"]
assert shaped_residual is not None, "Anti-sway cart must stop to measure residual sway"
assert baseline_residual is not None, "Baseline cart must stop to measure residual sway"
assert shaped_residual < baseline_residual * 0.1
reverse = metrics(
    simulate({**c, "target": -c["target"]}, 2.4), c | {"target": -c["target"]}
)
assert reverse["constraints_ok"] and reverse["settling_time"] is not None
print(
    "PASS: static equilibrium, unconstrained swing, default anti-sway performance, reverse target"
)

assert not is_single_motion(1.2, c), "Old segmented motion must be rejected"
assert is_single_motion(2.4, c)
run = simulate(c, 2.4)
stop = metrics(run, c)["cart_stop_time"]
assert stop is not None
speed = run["state"][run["t"] <= stop, 3]
peak = int(np.argmax(speed))
assert np.all(speed >= -1e-9)
assert np.all(np.diff(speed[: peak + 1]) >= -1e-9)
assert np.all(np.diff(speed[peak:]) <= 1e-9)
print("PASS: single acceleration/deceleration in actual cart speed")
