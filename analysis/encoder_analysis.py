import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv('/home/mantis/01_projects/analysis/baby-rover/encoder_data/2603101620.csv', names=['enA_trA', 'enA_trB', 'enB_trA', 'enB_trB'],)


df['time'] = df.index * (1/100000)

fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)

axes[0].plot(df['time'], df['enA_trA'])
axes[0].set_ylabel('enA_trA')

axes[1].plot(df['time'], df['enA_trB'])
axes[1].set_ylabel('enA_trB')

axes[2].plot(df['time'], df['enB_trA'])
axes[2].set_ylabel('enB_trA')

axes[3].plot(df['time'], df['enB_trB'])
axes[3].set_ylabel('enB_trB')

axes[3].set_xlabel('Time (s)')
plt.tight_layout()
plt.show()