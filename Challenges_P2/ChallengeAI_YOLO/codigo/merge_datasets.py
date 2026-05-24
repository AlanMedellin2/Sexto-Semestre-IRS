import shutil
from pathlib import Path

SRC1 = Path("gtsrb_det")
SRC2 = Path("real_extra")
OUT = Path("gtsrb_det_plus_real")

for split in ["train", "val", "test"]:
    (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
    (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)

def copy_dataset(src):
    for split in ["train", "val", "test"]:
        for img in (src / "images" / split).glob("*"):
            dst = OUT / "images" / split / img.name
            if dst.exists():
                dst = OUT / "images" / split / f"{src.name}_{img.name}"
            shutil.copy2(img, dst)

        for lbl in (src / "labels" / split).glob("*.txt"):
            dst = OUT / "labels" / split / lbl.name
            if dst.exists():
                dst = OUT / "labels" / split / f"{src.name}_{lbl.name}"
            shutil.copy2(lbl, dst)

copy_dataset(SRC1)
copy_dataset(SRC2)

yaml_text = f"""path: {OUT.resolve()}
train: images/train
val: images/val
test: images/test

names:
  0: stop
  1: straight
  2: turn_right
  3: turn_left
  4: speed_limit_30
"""

with open(OUT / "data.yaml", "w") as f:
    f.write(yaml_text)

print("Dataset combinado creado en", OUT)
