import shutil
from pathlib import Path
import pandas as pd
from sklearn.model_selection import train_test_split

BASE_DIR = Path(".")
TRAIN_CSV = BASE_DIR / "Train.csv"
TEST_CSV = BASE_DIR / "Test.csv"
OUT_DIR = BASE_DIR / "gtsrb_cls"

CLASS_MAP = {
    14: "stop",
    35: "straight",
    33: "turn_right",
    34: "turn_left",
    1: "speed_limit_30",
}

VAL_SIZE = 0.2
RANDOM_STATE = 42

def make_dirs():
    for split in ["train", "val", "test"]:
        for class_name in CLASS_MAP.values():
            (OUT_DIR / split / class_name).mkdir(parents=True, exist_ok=True)


def copy_subset(df, split_name):
    copied = 0
    missing = 0

    for _, row in df.iterrows():
        class_id = int(row["ClassId"])

        if class_id not in CLASS_MAP:
            continue

        class_name = CLASS_MAP[class_id]
        rel_img_path = str(row["Path"])
        src = BASE_DIR / rel_img_path


        if not src.exists():
            print(f"No existe: {src}")
            missing += 1
            continue



        safe_name = rel_img_path.replace("/", "_")
        dst = OUT_DIR / split_name / class_name / safe_name

        shutil.copy2(src, dst)
        copied += 1

    print(f"{split_name}: copiadas={copied}, faltantes={missing}")







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


    copy_subset(train_split, "train")
    copy_subset(val_split, "val")
    copy_subset(test_df, "test")

    print("\nDataset creado correctamente en:", OUT_DIR)
    print("Clases usadas:")
    for k, v in CLASS_MAP.items():
        print(f"  {k} -> {v}")

if __name__ == "__main__":
    main()
