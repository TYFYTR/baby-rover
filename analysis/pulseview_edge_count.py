import sys
import pandas as pd

df = pd.read_csv(sys.argv[1], skiprows=1, header=None,
    names=["enA_trA","enB_trA"])

for col in ["enA_trA", "enB_trA"]:
    count = ((df[col].shift(1) == 0) & (df[col] == 1)).sum()
    print(f"{col}: {count}")