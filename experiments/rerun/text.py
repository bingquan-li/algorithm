import rerun as rr

rr.init("planning_debug", spawn=True)
MODE_PATH = "planner/behavior"

# 静态配置：原始状态值、显示名称和颜色一一对应。
rr.log(
    MODE_PATH,
    rr.StateConfiguration(
        values=["cruise", "follow", "yield"],
        labels=["巡航", "跟车", "让行"],
        colors=[
            [76, 175, 80],  # 绿色：巡航
            [33, 150, 243],  # 蓝色：跟车
            [255, 152, 0],  # 橙色：让行
        ],
    ),
    static=True,
)


# 模拟规划器每帧输出的状态。
def get_behavior(frame: int) -> str:
    if frame < 30:
        return "cruise"
    if frame < 70:
        return "follow"
    if frame < 100:
        return "yield"
    return "cruise"


previous_behavior = None

for frame in range(121):
    rr.set_time("frame", sequence=frame)
    behavior = get_behavior(frame)

    # 状态变化时才记录；不需要每帧重复写入同一个状态。
    if behavior != previous_behavior:
        rr.log(MODE_PATH, rr.StateChange(state=behavior))
        previous_behavior = behavior


def log_path_decision(frame: int, candidates: list[dict]) -> None:
    rr.set_time("frame", sequence=frame)

    feasible = [c for c in candidates if c["feasible"]]
    selected = min(feasible, key=lambda c: c["cost"]) if feasible else None

    # 同时绘制路径，让文字解释能与几何形状对照。
    rr.log(
        "planner/candidates",
        rr.LineStrips2D(
            [c["points"] for c in candidates],
            labels=[f"{c['id']} | cost={c['cost']:.1f}" for c in candidates],
            colors=[
                [76, 175, 80]
                if c is selected
                else [244, 67, 54]
                if not c["feasible"]
                else [158, 158, 158]
                for c in candidates
            ],
            show_labels=True,
        ),
    )

    rows = "\n".join(
        f"| {c['id']} | {'是' if c['feasible'] else '否'} "
        f"| {c['cost']:.1f} | {c['reason']} |"
        for c in candidates
    )

    if selected is None:
        conclusion = "没有可行候选路径，本次未选出路径。"
    else:
        conclusion = (
            f"选择 **{selected['id']}**：通过可行性检查，"
            f"且在可行候选中总成本最低（{selected['cost']:.1f}）。"
        )

    markdown = f"""
# 路径选择解释

- 当前帧：**{frame}**
- 选择规则：先检查可行性，再比较总成本。

## 结论

{conclusion}

## 候选对比

| 路径 | 可行 | 总成本 | 检查说明 |
|---|---|---:|---|
{rows}

## 图中颜色

- 绿色：已选路径
- 红色：不可行路径
- 灰色：可行但未选中
"""

    rr.log(
        "planner/decision_explanation",
        rr.TextDocument(markdown, media_type=rr.MediaType.MARKDOWN),
    )


log_path_decision(
    frame=70,
    candidates=[
        {
            "id": "A",
            "points": [[0, 0], [5, 0], [10, 0]],
            "feasible": False,
            "cost": 8.0,
            "reason": "与预测障碍物轨迹发生碰撞",
        },
        {
            "id": "B",
            "points": [[0, 0], [5, 1], [10, 2]],
            "feasible": True,
            "cost": 12.4,
            "reason": "碰撞检查和运动学约束均通过",
        },
        {
            "id": "C",
            "points": [[0, 0], [5, -2], [10, -3]],
            "feasible": True,
            "cost": 18.7,
            "reason": "检查通过，但偏离参考线的成本较高",
        },
    ],
)
