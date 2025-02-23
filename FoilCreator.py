import math
import traceback
import adsk.core
import adsk.fusion


class UiLogger:
    def __init__(self):
        app = adsk.core.Application.get()
        ui = app.userInterface
        palettes = ui.palettes
        self.text_palette = palettes.itemById("TextCommands")
        self.text_palette.isVisible = True

    def print(self, text):
        self.text_palette.writeText(text)
        adsk.doEvents()


logger = UiLogger()


def generate_naca_airfoil_coordinates(naca="2412", num_points=50):
    """
    NACA 4桁エアフォイルの座標と、揚力中心位置を生成する関数
    入力:
      naca      : 4桁のNACAコード（例: '2412'）
      num_points: 1/2断面の分割数（上面、下面各50点なら合計約100点）
    出力:
      airfoil_coords : 上面と下面を連結した閉じた輪郭の (x, y) 座標リスト（x, yは0～1の正規化座標）
      center_of_lift : 揚力中心の座標 (x, y) ※ここでは x=0.25 におけるキャンバー線上の値を採用
    """
    if len(naca) != 4:
        raise ValueError("NACAコードは4桁で指定してください。")

    # パラメータの抽出
    m = int(naca[0]) / 100.0  # 例: 2% → 0.02
    p = int(naca[1]) / 10.0  # 例: 4 → 0.4
    t = int(naca[2:]) / 100.0  # 例: 12% → 0.12

    # 揚力中心の計算
    # ここでは、薄翼理論に基づき x = 0.25 (コード長の25%) のキャンバー線上の値を揚力中心とする
    x_lift = 0.5
    if x_lift < p and p != 0:
        y_lift = (m / (p * p)) * (2 * p * x_lift - x_lift * x_lift)
    elif p != 1:
        y_lift = (m / ((1 - p) * (1 - p))) * (
            (1 - 2 * p) + 2 * p * x_lift - x_lift * x_lift
        )
    else:
        y_lift = 0.0

    x_coords = []
    for i in range(num_points):
        beta = math.pi * i / (num_points - 1)
        x = 0.5 * (1 - math.cos(beta))
        x_coords.append(x)

    upper_points = list[tuple[float, float]]()
    lower_points = list[tuple[float, float]]()

    for x in x_coords:
        # キャンバー線とその微分
        if x < p and p != 0:
            y_c = (m / (p * p)) * (2 * p * x - x * x)
            dyc_dx = (2 * m / (p * p)) * (p - x)
        elif p != 1:
            y_c = (m / ((1 - p) * (1 - p))) * ((1 - 2 * p) + 2 * p * x - x * x)
            dyc_dx = (2 * m / ((1 - p) * (1 - p))) * (p - x)
        else:
            y_c = 0.0
            dyc_dx = 0.0

        theta = math.atan(dyc_dx)
        # 厚さ分布
        y_t = (
            5
            * t
            * (
                0.2969 * math.sqrt(x)
                - 0.1260 * x
                - 0.3516 * x * x
                + 0.2843 * x**3
                - 0.1015 * x**4
            )
        )

        # 上面・下面の座標計算
        x_upper = x - y_t * math.sin(theta)
        y_upper = y_c + y_t * math.cos(theta)
        x_lower = x + y_t * math.sin(theta)
        y_lower = y_c - y_t * math.cos(theta)

        upper_points.append((x_upper - x_lift, y_upper - y_lift))
        lower_points.append((x_lower - x_lift, y_lower - y_lift))

    # 輪郭を閉じるため、上面座標順 + 下面座標を逆順で連結
    airfoil_coords = upper_points + lower_points[::-1]

    return airfoil_coords


def get_entities(sketch: adsk.fusion.Sketch):
    entities = adsk.core.ObjectCollection.create()
    for sc in sketch.sketchCurves:
        entities.add(sc)
    return entities


def rotate_sketch(
    sketch: adsk.fusion.Sketch,
    rotate_angle: float,
):
    transform = adsk.core.Matrix3D.create()
    transform.setToRotation(
        rotate_angle,
        adsk.core.Vector3D.create(0, 0, 1),
        adsk.core.Point3D.create(0, 0, 0),
    )
    sketch.move(get_entities(sketch), transform)


def skew_sketch(sketch: adsk.fusion.Sketch, skew_angle: float, r: float):
    transform = adsk.core.Matrix3D.create()
    transform.setToRotation(
        skew_angle,
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Point3D.create(0, r, 0),
    )
    sketch.move(get_entities(sketch), transform)


def move_sketch(sketch: adsk.fusion.Sketch, offset: float):
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(0, 0, offset)
    sketch.move(get_entities(sketch), transform)


def create_sketch(
    sketch: adsk.fusion.Sketch,
    airfoil_coords: list[tuple[float, float]],
    rotate_angle: float,
    scale: float,
    skew_angle: float,
    skew_r: float,
):
    points_collection = adsk.core.ObjectCollection.create()
    for x, y in airfoil_coords:
        pt = adsk.core.Point3D.create(x * scale, y * scale, 0)
        points_collection.add(pt)
    sketch.sketchCurves.sketchFittedSplines.add(points_collection)

    rotate_sketch(sketch, rotate_angle)
    skew_sketch(sketch, skew_angle, skew_r)


def get_rotate_angle(fraction: float):
    start = 1 / 6
    return math.pi * start + fraction * math.pi * (1 / 2 - start)


def get_scale(fraction: float):
    all_scale = 1.2
    max_scale_fraction = 0.6
    if fraction < max_scale_fraction:
        return all_scale * (-pow(fraction - max_scale_fraction, 2) + 1)
    return all_scale * math.sqrt(
        1 - pow(1 / (1 - max_scale_fraction) * (fraction - max_scale_fraction), 2)
    )


def get_skew_angle(fraction: float):
    return math.pi * 1 / 2 * fraction


def end_point(sketch: adsk.fusion.Sketch, skew_r: float):
    end_skew_angle = get_skew_angle(1)
    end_pt = adsk.core.Point3D.create(
        0,
        skew_r - math.cos(end_skew_angle) * skew_r,
        -math.sin(end_skew_angle) * skew_r,
    )
    return sketch.sketchPoints.add(end_pt)


def run(_context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root_comp = design.rootComponent

        base_plane = root_comp.xZConstructionPlane
        airfoil_coords = generate_naca_airfoil_coordinates("2412", 20)
        sketches = root_comp.sketches
        loft_profiles = list[adsk.fusion.Profile]()

        sketch_num = 50
        skew_r = 2

        for i in range(sketch_num):
            if i < 6:
                continue
            sketch = sketches.add(base_plane)
            sketch.name = f"Airfoil Section {i}"
            fraction = i / float(sketch_num)
            create_sketch(
                sketch,
                airfoil_coords,
                get_rotate_angle(fraction),
                get_scale(fraction),
                get_skew_angle(fraction),
                skew_r,
            )

            loft_profiles.append(sketch.profiles.item(0))

        end_sketch = sketches.add(base_plane)
        end_sketch.name = "Airfoil End"
        end_sketch_point = end_point(end_sketch, skew_r)

        loft_feats = root_comp.features.loftFeatures
        loft_input = loft_feats.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        loft_sections = loft_input.loftSections
        for prof in loft_profiles:
            loft_sections.add(prof)
        loft_section = loft_sections.add(end_sketch_point)
        loft_section.setPointTangentEndCondition(
            adsk.core.ValueInput.createByReal(sketch_num / 2)
        )

        loft_feats.add(loft_input)

    except Exception:
        ui.messageBox("失敗:\n{}".format(traceback.format_exc()))
