import shutil
from pathlib import Path
import pandas as pd
from sklearn.model_selection import train_test_split


BASE_DIR = Path(".")
TRAIN_CSV = BASE_DIR / "Train.csv"
TEST_CSV = BASE_DIR / "Test.csv"

OUT_DIR = BASE_DIR / "gtsrb_det"



# Usa las mismas 5 clases entrenadas before
CLASS_MAP = {
    14: "stop",
    35: "straight",
    33: "turn_right",
    34: "turn_left",
    1: "speed_limit_30",
}



# Mapeo a índices consecutivos para YOLO
YOLO_CLASS_ID = {k: i for i, k in enumerate(CLASS_MAP.keys())}

VAL_SIZE = 0.2
RANDOM_STATE = 42






def make_dirs():
    for split in ["train", "val", "test"]:
        (OUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)




def row_to_yolo_label(row):
    img_w = float(row["Width"])
    img_h = float(row["Height"])
    x1 = float(row["Roi.X1"])
    y1 = float(row["Roi.Y1"])
    x2 = float(row["Roi.X2"])
    y2 = float(row["Roi.Y2"])

    x_center = ((x1 + x2) / 2.0) / img_w
    y_center = ((y1 + y2) / 2.0) / img_h
    width = (x2 - x1) / img_w
    height = (y2 - y1) / img_h

    return x_center, y_center, width, height




def export_subset(df, split_name):
    copied = 0
    missing = 0

    for _, row in df.iterrows():
        original_class = int(row["ClassId"])
        if original_class not in CLASS_MAP:
            continue

        yolo_class = YOLO_CLASS_ID[original_class]
        rel_img_path = str(row["Path"])
        src_img = BASE_DIR / rel_img_path

        if not src_img.exists():
            print(f"No existe: {src_img}")
            missing += 1
            continue

        # Unique name
        safe_name = rel_img_path.replace("/", "_")
        dst_img = OUT_DIR / "images" / split_name / safe_name
        dst_lbl = OUT_DIR / "labels" / split_name / (Path(safe_name).stem + ".txt")


        shutil.copy2(src_img, dst_img)

        x_center, y_center, width, height = row_to_yolo_label(row)

        with open(dst_lbl, "w") as f:
            f.write(f"{yolo_class} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")


        copied += 1

    print(f"{split_name}: copiadas={copied}, faltantes={missing}")






def write_yaml():
    yaml_text = f"""path: {OUT_DIR.resolve()}
train: images/train
val: images/val
test: images/test

names:
"""
    for original_class, yolo_idx in YOLO_CLASS_ID.items():
        yaml_text += f"  {yolo_idx}: {CLASS_MAP[original_class]}\n"

    with open(OUT_DIR / "data.yaml", "w") as f:
        f.write(yaml_text)


def main():
    make_dirs()

    train_df = pd.read_csv(TRAIN_CSV)
    test_df = pd.read_csv(TEST_CSV)

    train_df = train_df[train_df["ClassId"].isin(CLASS_MAP.keys())].copy()
    test_df = test_df[test_df["ClassId"].isin(CLASS_MAP.keys())].copy()

    train_split, val_split = train_test_split(
        train_df,
        test_size=VAL_SIZE,
        random_state=RANDOM_STATE,
        stratify=train_df["ClassId"]
    )

    export_subset(train_split, "train")
    export_subset(val_split, "val")
    export_subset(test_df, "test")
    write_yaml()

    print("\nDataset YOLO detect creado en:", OUT_DIR)
    print("Clases YOLO:")
    for original_class, yolo_idx in YOLO_CLASS_ID.items():
        print(f"  {yolo_idx}: {CLASS_MAP[original_class]} (ClassId original {original_class})")






if __name__ == "__main__":
    main()
