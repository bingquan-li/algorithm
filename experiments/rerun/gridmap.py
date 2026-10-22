"""测试 GridMap 的 RGBA 透明显示。"""

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

width, height = 64, 64
cell_size = 0.1

# 本次测试的关键参数。
# 可以分别改为 0.5 和 1.0，对比显示效果。
map_opacity = 0.5

# -1：未知；0：空闲；100：占据。
grid = np.full((height, width), -1, dtype=np.int8)
grid[8:56, 8:56] = 0
grid[20:44, 20:44] = 100

# 默认全透明：未知和空闲区域都不显示。
rgba = np.zeros((height, width, 4), dtype=np.uint8)

# 占据区域：橙色，像素 Alpha 为 255。
rgba[grid == 100] = [255, 100, 40, 255]

rr.init("grid_map_rgba_opacity_test", spawn=True)

# 地图位于 z=0，覆盖 x、y ∈ [-3.2, 3.2]。
rr.log(
    "world/map",
    rr.GridMap.from_fields(
        clear_unset=True,
        data=rgba.tobytes(),
        format=rr.components.ImageFormat(
            width=width,
            height=height,
            color_model="RGBA",
            channel_datatype="U8",
        ),
        cell_size=cell_size,
        translation=[
            -width * cell_size / 2.0,
            -height * cell_size / 2.0,
            0.0,
        ],
        opacity=map_opacity,
        # 不设置 colormap。
    ),
)

# 地图下方的蓝色十字线。
# 两端超出地图边界，方便确认线本身被正常绘制。
rr.log(
    "world/test_lines",
    rr.LineStrips3D(
        [
            [[-4.0, 0.0, -0.1], [4.0, 0.0, -0.1]],
            [[0.0, -4.0, -0.1], [0.0, 4.0, -0.1]],
        ],
        colors=[[0, 180, 255]],
        radii=0.04,
    ),
)

rr.send_blueprint(
    rrb.Blueprint(
        rrb.Spatial3DView(
            name="GridMap Alpha Test",
            origin="/world",
            contents=[
                "/world/map",
                "/world/test_lines",
            ],
        ),
    )
)

print(f"SDK version: {rr.__version__}")
print(f"GridMap opacity: {map_opacity}")
print("请从 +Z 方向朝下观察地图。")
print("预期：外围完全透明，中央橙色区域半透明，蓝色十字线可透出。")