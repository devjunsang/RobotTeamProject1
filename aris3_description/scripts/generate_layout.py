#!/usr/bin/env python3

import csv
from pathlib import Path

# 모든 거리 단위는 미터(m)
CENTER_X = 0.0
CENTER_Y = 0.0
RADIUS = 0.45
BOX_SIZE = 0.08
GROUND_Z = 0.0

# 이름, 색상, X 오프셋, Y 오프셋, 각도
OBJECTS = [
    ("red_box",    "red",    0.0,  1.0,  90),
    ("blue_box",   "blue",   1.0,  0.0,   0),
    ("yellow_box", "yellow", 0.0, -1.0, 270),
    ("green_box",  "green", -1.0, 0.0, 180),
]

output = (
    Path.home()
    / "WorkPlace/ARIS/ARIS3/config/object_layout.csv"
)

rows = []

for name, color, dx, dy, angle in OBJECTS:
    rows.append({
        "name": name,
        "color": color,
        "shape": "box",
        "x": round(CENTER_X + RADIUS * dx, 4),
        "y": round(CENTER_Y + RADIUS * dy, 4),
        "z": GROUND_Z + BOX_SIZE / 2,
        "width": BOX_SIZE,
        "height": BOX_SIZE,
        "angle_deg": angle,
    })

output.parent.mkdir(parents=True, exist_ok=True)

with output.open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

print("===== ARIS3 PHASE 3 / STEP 01 =====")

for row in rows:
    print(
        f"{row['name']:12s} "
        f"x={row['x']:.3f} "
        f"y={row['y']:.3f} "
        f"z={row['z']:.3f}"
    )

print(f"\n[PASS] 총 {len(rows)}개 객체 좌표 생성")
print(f"[SAVED] {output}")
