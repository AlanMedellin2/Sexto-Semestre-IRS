import random
import shutil
from pathlib import Path

random.seed(42)

BASE = Path("real_extra")
IMG_ALL = BASE / "images" / "all"
LBL_ALL = BASE / "labels" / "all"

for split in ["train", "val", "test"]:
    (BASE / "images" / split).mkdir(parents=True, exist_ok=True)
    (BASE / "labels" / split).mkdir(parents=True, exist_ok=True)

image_exts = {".jpg", ".jpeg", ".png"}
images = [p for p in IMG_ALL.iterdir() if p.suffix.lower() in image_exts]
images.sort()
random.shuffle(images)

n = len(images)
n_train = int(0.7 * n)
n_val = int(0.15 * n)

train_imgs = images[:n_train]
val_imgs = images[n_train:n_train+n_val]
test_imgs = images[n_train+n_val:]

splits = {
    "train": train_imgs,
    "val": val_imgs,
    "test": test_imgs
}

for split, items in splits.items():
    for img_path in items:
        lbl_path = LBL_ALL / (img_path.stem + ".txt")
        shutil.copy2(img_path, BASE / "images" / split / img_path.name)

        dst_lbl = BASE / "labels" / split / (img_path.stem + ".txt")
        if lbl_path.exists():
            shutil.copy2(lbl_path, dst_lbl)
        else:
            dst_lbl.touch()

print("Split terminado.")
for split, items in splits.items():
    print(split, len(items))
