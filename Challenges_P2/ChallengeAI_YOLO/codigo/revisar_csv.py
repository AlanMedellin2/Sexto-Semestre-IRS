import pandas as pd

train_df = pd.read_csv("Train.csv")
test_df = pd.read_csv("Test.csv")
meta_df = pd.read_csv("Meta.csv")

print("=== Train.csv ===")
print(train_df.columns.tolist())
print(train_df.head())

print("\n=== Test.csv ===")
print(test_df.columns.tolist())
print(test_df.head())

print("\n=== Meta.csv ===")
print(meta_df.columns.tolist())
print(meta_df.head())
