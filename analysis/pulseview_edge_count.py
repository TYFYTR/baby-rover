import sys
import pandas as pd

df = pd.read_csv(sys.argv[1], skiprows=1, header=None,
                 names=["enA_trA", "enA_trB", "enB_trA", "enB_trB"])

rising_edges = ((df["enA_trA"].shift(1) == 0) & (df["enA_trA"] == 1)).sum()

print(rising_edges)
