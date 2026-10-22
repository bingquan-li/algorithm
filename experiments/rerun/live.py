"""Live, unbounded Rerun state demo. Ctrl+C to stop.

Install: python -m pip install -U rerun-sdk
Run:     python rerun_live_states.py
Optional: --hz 10 --seed 42 (seed makes event draws reproducible)
Synthetic events only; not a vehicle dynamics or production planner model.
"""

import argparse
import math
import random
import time

import rerun as rr
import rerun.blueprint as rrb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 0 < args.hz <= 100:
        parser.error("--hz must be in (0, 100]")

    if not all(hasattr(rr, name) for name in ("StateChange", "StateConfiguration")):
        raise SystemExit("Please upgrade: python -m pip install -U rerun-sdk")

    rng = random.Random(args.seed)
    rr.init("live_planning_states", spawn=True)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Vertical(
                rrb.StateTimelineView(name="Live behavior", origin="/planner"),
                rrb.Horizontal(
                    rrb.TextDocumentView(name="Current status", origin="/status"),
                    rrb.TextLogView(name="Transitions", origin="/events"),
                ),
            ),
        )
    )
    names = {"cruise": "巡航", "follow": "跟车", "yield": "让行"}
    rr.log(
        "planner/behavior",
        rr.StateConfiguration(
            values=list(names),
            labels=list(names.values()),
            colors=[[76, 175, 80], [33, 150, 243], [255, 152, 0]],
        ),
        static=True,
    )

    # Continuous-time event hazards: probability of an event during dt.
    # Lead vehicle toggles presence; crossing events have priority over following.
    lead_present = False
    crossing = False
    previous_state = None
    entered_at = 0.0
    transitions = 0
    start = previous_tick = time.monotonic()
    next_tick = start
    print("实时生成中，无时长限制。Viewer 选择 live_time，并保持跟随最新时间。")
    print("Ctrl+C 停止。交通事件为随机模拟数据。", flush=True)

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            dt = now - previous_tick
            previous_tick = now

            # Average arrival/departure intervals, not fixed schedules.
            lead_mean = 12.0 if lead_present else 8.0
            if rng.random() < -math.expm1(-dt / lead_mean):
                lead_present = not lead_present
            crossing_mean = 4.0 if crossing else 16.0
            if rng.random() < -math.expm1(-dt / crossing_mean):
                crossing = not crossing

            if crossing:
                state, reason = "yield", "检测到横穿交通参与者，让行优先"
            elif lead_present:
                state, reason = "follow", "前方存在慢车，进入跟车"
            else:
                state, reason = "cruise", "无横穿冲突且前方无慢车，保持巡航"

            # Real elapsed monotonic time: no future data is precomputed.
            rr.set_time("live_time", duration=elapsed)
            if state != previous_state:
                old = names.get(previous_state, "初始化")
                if previous_state is not None:
                    transitions += 1
                entered_at = elapsed
                rr.log("planner/behavior", rr.StateChange(state=state))
                message = f"{elapsed:8.2f}s | {old} → {names[state]} | {reason}"
                rr.log(
                    "events/transitions",
                    rr.TextLog(message, level=rr.TextLogLevel.INFO),
                )
                print(message, flush=True)
                previous_state = state

            # Heartbeat advances recording time even when state does not change.
            # StateChange itself is only emitted on transitions.
            rr.log(
                "status/current",
                rr.TextDocument(
                    f"# {names[state]}\n\n"
                    f"- 实时运行：**{elapsed:.1f} 秒**\n"
                    f"- 当前状态持续：**{elapsed - entered_at:.1f} 秒**\n"
                    f"- 状态切换次数：**{transitions}**\n"
                    f"- 前方慢车：{'有' if lead_present else '无'}\n"
                    f"- 横穿冲突：{'有' if crossing else '无'}\n\n"
                    f"## 决策原因\n{reason}\n\n"
                    "随机模拟交通事件；持续运行直到 Ctrl+C。",
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )

            # Pace against monotonic clock, avoiding an accumulated sleep drift.
            next_tick += 1.0 / args.hz
            if next_tick < time.monotonic():
                next_tick = time.monotonic()
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\n已停止生成，可在 Viewer 中回看已接收的数据。")


if __name__ == "__main__":
    main()
