import rerun.blueprint as rrb

blueprint = rrb.Blueprint(
    rrb.Spatial3DView(
        name="narrow_passage",          # 原先视图名称显示为 "/"
        origin="/",     # 以该实体作为视图参考原点
        contents=["/**"], # 只显示这个实体
    ),
    auto_layout=False,
    auto_views=False,
)

# 必须与 C++ RecordingStream 的 application_id 一致。
blueprint.save("narrow_passage", "narrow_passage.rbl")