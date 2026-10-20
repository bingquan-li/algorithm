"""Elastic load transport: step PD versus ZVD-shaped quintic tracking."""

import argparse
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TypedDict

import numpy as np

# MuJoCo 3.16 exports these from native extensions without Python type stubs.
# Keep the workaround at the import boundary; retain checks on our own code.
from mujoco import (
    MjData,  # pyright: ignore[reportAttributeAccessIssue]
    MjModel,  # pyright: ignore[reportAttributeAccessIssue]
    mj_forward,  # pyright: ignore[reportAttributeAccessIssue]
    mj_resetDataKeyframe,  # pyright: ignore[reportAttributeAccessIssue]
    mj_step,  # pyright: ignore[reportAttributeAccessIssue]
)
from numpy.typing import NDArray

HERE = Path(__file__).resolve().parent


class Metrics(TypedDict):
    settling_time: float | None
    cart_stop_time: float | None
    max_sway: float
    residual_sway: float | None
    max_speed: float
    max_force: float
    constraints_ok: bool


def required_element(root: ET.Element, path: str) -> ET.Element:
    """Return a required model element, with a useful error if XML is incomplete."""
    element = root.find(path)
    if element is None:
        raise ValueError(f"model.xml is missing required element: {path}")
    return element


def load_config(path):
    c = json.loads(Path(path).read_text())
    for key, value in c.items():
        if not np.isfinite(value):
            raise ValueError(f"{key} must be finite")
        if key in ("horizontal_damping", "radial_damping") and value == 0:
            continue
        if key != "target" and value <= 0:
            raise ValueError(f"{key} must be positive")
    if c["duration_max"] < c["duration_min"]:
        raise ValueError("duration_max must be >= duration_min")
    if c["horizon"] <= c["hold_time"]:
        raise ValueError("horizon must exceed hold_time")
    return c


def make_model(c):
    root = ET.parse(HERE / "model.xml").getroot()
    if root is None:
        raise ValueError("model.xml has no root element")
    option = required_element(root, "option")
    option.set("timestep", str(c["dt"]))
    option.set("gravity", f"0 0 {-c['gravity']}")
    for name, mass in [("cart_geom", c["cart_mass"]), ("load_geom", c["load_mass"])]:
        required_element(root, f".//geom[@name='{name}']").set("mass", str(mass))
    required_element(root, ".//body[@name='load']").set(
        "pos", f"0 0 {-c['spring_length']}"
    )
    required_element(root, ".//joint[@name='relative_x']").set(
        "damping", str(c["horizontal_damping"])
    )
    spring = required_element(root, ".//spatial")
    for attr, key in [
        ("stiffness", "stiffness"),
        ("damping", "radial_damping"),
        ("springlength", "spring_length"),
    ]:
        spring.set(attr, str(c[key]))
    required_element(root, ".//motor").set(
        "ctrlrange", f"{-c['force_limit']} {c['force_limit']}"
    )
    target = c["target"]
    required_element(root, ".//geom[@name='target']").set(
        "fromto", f"{target} -.18 .1 {target} -.18 2.25"
    )
    required_element(root, ".//geom[@name='rail']").set(
        "fromto", f"{min(0, target) - 1} 0 2 {max(0, target) + 1} 0 2"
    )
    extension = c["load_mass"] * c["gravity"] / c["stiffness"]
    required_element(root, ".//key").set("qpos", f"0 0 {-extension}")
    xml = ET.tostring(root, encoding="unicode")
    model = MjModel.from_xml_string(xml)
    data = MjData(model)
    mj_resetDataKeyframe(model, data, 0)
    mj_forward(model, data)
    return model, data, xml


def shaper(c):
    # Small-angle suspended-load mode with cart following its prescribed path.
    length = c["spring_length"] + c["load_mass"] * c["gravity"] / c["stiffness"]
    omega = np.sqrt(c["gravity"] / length)
    zeta = c["horizontal_damping"] / (2 * c["load_mass"] * omega)
    if zeta >= 1:
        raise ValueError("ZVD requires an underdamped lateral mode (zeta < 1)")
    half_period = np.pi / (omega * np.sqrt(1 - zeta**2))
    k = np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
    return np.array([0, half_period, 2 * half_period]), np.array([1, 2 * k, k * k]) / (
        1 + k
    ) ** 2


def reference(t, duration, c):
    delays, weights = shaper(c)
    s = np.clip((np.asarray(t)[..., None] - delays) / duration, 0, 1)
    x = (10 * s**3 - 15 * s**4 + 6 * s**5) @ weights
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) @ weights / duration
    a = (60 * s - 180 * s**2 + 120 * s**3) @ weights / duration**2
    return c["target"] * np.stack([x, v, a], axis=-1)


def is_single_motion(duration, c):
    """Require one speed peak over the entire shaped reference, with no restart."""
    end = duration + float(shaper(c)[0][-1])
    times = np.linspace(0, end, max(2001, int(np.ceil(end / c["dt"])) + 1))
    # Normalize direction and distance, including a zero-distance target.
    speed = reference(times, duration, {**c, "target": 1.0})[:, 1]
    peak = int(np.argmax(speed))
    return bool(
        np.all(np.diff(speed[: peak + 1]) >= -1e-9)
        and np.all(np.diff(speed[peak:]) <= 1e-9)
        and np.all(speed[1:-1] > 0)
    )


def simulate(c, duration=None):
    model, data, _ = make_model(c)
    times = np.arange(round(c["horizon"] / c["dt"]) + 1) * c["dt"]
    refs = (
        reference(times, duration, c)
        if duration is not None
        else np.tile([c["target"], 0, 0], (len(times), 1))
    )
    states = np.empty((len(times), 6))
    forces = np.empty(len(times))
    for i, ref in enumerate(refs):
        states[i] = np.r_[data.qpos, data.qvel]
        force = c["kp"] * (ref[0] - data.qpos[0]) + c["kd"] * (ref[1] - data.qvel[0])
        if duration is not None:
            force += (c["cart_mass"] + c["load_mass"]) * ref[2]
        # One-step velocity governor. For these Cartesian slide coordinates,
        # cart acceleration changes by delta_force / cart_mass exactly.
        # This limits predicted speed without imposing any joint lock/velocity reset.
        data.ctrl[0] = 0
        mj_forward(model, data)
        lo = c["cart_mass"] * (
            (-c["speed_limit"] - data.qvel[0]) / c["dt"] - data.qacc[0]
        )
        hi = c["cart_mass"] * (
            (c["speed_limit"] - data.qvel[0]) / c["dt"] - data.qacc[0]
        )
        force = np.clip(force, lo, hi)
        force = np.clip(force, -c["force_limit"], c["force_limit"])
        forces[i] = force
        if i < len(times) - 1:
            data.ctrl[0] = force
            mj_step(model, data)
    if not np.all(np.isfinite(states)):
        raise RuntimeError("Simulation produced non-finite states")
    return {"t": times, "state": states, "force": forces, "reference": refs}


def sustained_time(mask, t, hold):
    # Settled means all samples from this time to simulation end satisfy limits,
    # and at least hold seconds have been observed. No zero-crossing shortcut.
    bad = np.flatnonzero(~mask)
    start = int(bad[-1] + 1) if len(bad) else 0
    if start >= len(t) or t[-1] - t[start] < hold:
        return None
    return float(t[start])


def metrics(run, c) -> Metrics:
    t, q = run["t"], run["state"]
    cart_ok = (abs(q[:, 0] - c["target"]) <= c["position_tolerance"]) & (
        abs(q[:, 3]) <= c["speed_tolerance"]
    )
    extension = c["load_mass"] * c["gravity"] / c["stiffness"]
    all_ok = (
        cart_ok
        & (abs(q[:, 1]) <= c["sway_tolerance"])
        & (abs(q[:, 4]) <= c["relative_speed_tolerance"])
        & (abs(q[:, 2] + extension) <= c["vertical_tolerance"])
        & (abs(q[:, 5]) <= c["relative_speed_tolerance"])
    )
    stop = sustained_time(cart_ok, t, c["hold_time"])
    residual = float(np.max(abs(q[t >= stop, 1]))) if stop is not None else None
    peak_speed = float(np.max(abs(q[:, 3])))
    peak_sway = float(np.max(abs(q[:, 1])))
    peak_force = float(np.max(abs(run["force"])))
    return {
        "settling_time": sustained_time(all_ok, t, c["hold_time"]),
        "cart_stop_time": stop,
        "max_sway": peak_sway,
        "residual_sway": residual,
        "max_speed": peak_speed,
        "max_force": peak_force,
        "constraints_ok": bool(
            peak_speed <= c["speed_limit"] + 1e-4
            and peak_sway <= c["sway_limit"] + 1e-6
            and peak_force <= c["force_limit"] + 1e-8
        ),
    }


def plot(runs, c, path):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/spring-cart-matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for name, run in runs.items():
        axes[0].plot(run["t"], run["state"][:, 0], label=name)
        axes[1].plot(run["t"], run["state"][:, 1], label=name)
        axes[2].plot(run["t"], run["force"], label=name)
        axes[3].plot(run["t"], run["state"][:, 3], label=name)
        if name == "anti_sway":
            axes[3].plot(
                run["t"], run["reference"][:, 1], "--", label="anti_sway reference"
            )
    axes[0].axhline(c["target"], color="gray", ls="--", label="target")
    for sign in (-1, 1):
        axes[1].axhline(sign * c["sway_limit"], color="gray", ls=":")
        axes[2].axhline(sign * c["force_limit"], color="gray", ls=":")
        axes[3].axhline(sign * c["speed_limit"], color="gray", ls=":")
    for ax, label in zip(
        axes, ["Cart x [m]", "Load x - cart x [m]", "Force [N]", "Cart speed [m/s]"]
    ):
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        ax.legend()
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def playback(run, c):
    import mujoco.viewer

    model, data, _ = make_model(c)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        camera = viewer.cam
        if camera is None:
            raise RuntimeError("MuJoCo viewer closed before camera initialization")
        camera.lookat[:] = [c["target"] / 2, 0, 1.2]
        camera.distance = max(3.5, abs(c["target"]) + 2)
        camera.azimuth = 90
        camera.elevation = -12
        start = time.monotonic()
        while viewer.is_running():
            elapsed = time.monotonic() - start
            i = min(int(elapsed / c["dt"]), len(run["t"]) - 1)
            with viewer.lock():
                data.qpos[:] = run["state"][i, :3]
                data.qvel[:] = run["state"][i, 3:]
                data.ctrl[0] = run["force"][i]
                data.time = run["t"][i]
                mj_forward(model, data)
            viewer.sync()
            if elapsed >= run["t"][-1] + 1:
                break
            time.sleep(1 / 60)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--output", type=Path, default=HERE / "results")
    parser.add_argument(
        "--mode",
        choices=["pd", "anti_sway", "both"],
        default="both",
        help="Controller(s) to simulate (default: both)",
    )
    parser.add_argument(
        "--viewer",
        nargs="?",
        const="both",
        choices=["pd", "anti_sway", "both"],
        help="Play selected runs; optionally choose a controller to view",
    )
    parser.add_argument(
        "--duration", type=float, help="Skip search; use this base trajectory duration"
    )
    args = parser.parse_args()
    selected = ["pd", "anti_sway"] if args.mode == "both" else [args.mode]
    if args.viewer in ("pd", "anti_sway") and args.viewer not in selected:
        parser.error(f"--viewer {args.viewer} is not included in --mode {args.mode}")
    if args.mode == "pd" and args.duration is not None:
        parser.error("--duration applies only to anti_sway; omit it for --mode pd")
    c = load_config(args.config)
    if args.duration is not None and (
        not np.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be finite and positive")
    if args.duration is not None and not is_single_motion(args.duration, c):
        parser.error(
            "--duration produces multiple speed peaks; omit it to search automatically "
            "or increase it (default config: 2.4 s)"
        )
    args.output.mkdir(parents=True, exist_ok=True)
    _, data, xml = make_model(c)
    equilibrium_acceleration = float(np.max(abs(data.qacc)))
    if equilibrium_acceleration > 1e-8:
        raise RuntimeError(f"Invalid initial equilibrium: {data.qacc}")
    (args.output / "configured_model.xml").write_text(xml)
    runs = {}
    if "pd" in selected:
        runs["pd"] = simulate(c)
    best: tuple[float, float, dict[str, NDArray[np.float64]]] | None = None
    if "anti_sway" in selected:
        candidates = (
            [args.duration]
            if args.duration is not None
            else np.arange(
                c["duration_min"], c["duration_max"] + 1e-9, c["duration_step"]
            )
        )
        search = []
        for duration in candidates:
            if not is_single_motion(float(duration), c):
                search.append(
                    {
                        "duration": float(duration),
                        "single_motion": False,
                        "reason": "Reference speed has multiple peaks; skipped",
                    }
                )
                continue
            run = simulate(c, float(duration))
            stat = metrics(run, c)
            search.append({"duration": float(duration), "single_motion": True, **stat})
            settling_time = stat["settling_time"]
            if (
                stat["constraints_ok"]
                and settling_time is not None
                and (best is None or settling_time < best[0])
            ):
                best = (settling_time, float(duration), run)
        (args.output / "search.json").write_text(json.dumps(search, indent=2))
        if best is None:
            raise RuntimeError(
                "No feasible, settled trajectory found; see search.json. Increase horizon/duration range or relax limits."
            )
        runs["anti_sway"] = best[2]
    summary = {
        "config": c,
        "mode": args.mode,
        "equilibrium_acceleration": equilibrium_acceleration,
        "base_duration": best[1] if best is not None else None,
        "shaped_duration": best[1] + float(shaper(c)[0][-1])
        if best is not None
        else None,
        "metrics": {name: metrics(run, c) for name, run in runs.items()},
    }
    (args.output / "metrics.json").write_text(json.dumps(summary, indent=2))
    for name, run in runs.items():
        np.savetxt(
            args.output / f"{name}.csv",
            np.column_stack([run["t"], run["state"], run["force"], run["reference"]]),
            delimiter=",",
            header="t,cart_x,relative_x,relative_z,cart_v,relative_vx,relative_vz,force,ref_x,ref_v,ref_a",
            comments="",
        )
    plot(runs, c, args.output / "comparison.png")
    print(f"Equilibrium max acceleration: {equilibrium_acceleration:.3g} m/s²")
    if best is not None:
        print(
            f"Selected base duration: {best[1]:.2f} s; shaped duration: {summary['shaped_duration']:.2f} s"
        )
    print(json.dumps(summary["metrics"], indent=2))
    print(f"Results: {args.output.resolve()}")
    if args.viewer:
        for name in selected if args.viewer == "both" else [args.viewer]:
            print(f"Playing {name}; close window to continue.")
            playback(runs[name], c)


if __name__ == "__main__":
    main()
