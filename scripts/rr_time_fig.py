import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os
form_file = "form_2023-09-13_07-06-54.csv"
seq_file = "seq_1_col_2023-09-13_02-45-29.csv"
nocol_file = "seq_1_nocol_2023-09-13_03-19-45.csv"
rr_file = "seq_5_col_2023-09-13_06-57-48.csv" #"seq_5_col_2023-09-13_03-07-02.csv"

form = pd.read_csv(f"../data/merge/{form_file}", header=1)['view_reward'].values[0:-1]

folder_path = f"../data/merge"
files_in_folder = os.listdir(folder_path)
seq_files = [f for f in files_in_folder if f.endswith('.csv') and 'seq_1_col' in f]
nocol_files = [f for f in files_in_folder if f.endswith('.csv') and 'seq_1_nocol' in f]
rr_files = [f for f in files_in_folder if f.endswith('.csv') and 'seq_5_col' in f]

seq = np.zeros((0,len(form)))
for file in seq_files:
    seq = np.vstack((seq, pd.read_csv(f"{folder_path}/{file}", header=1)['view_reward'].to_numpy()[1:-1]))
seq_std = seq.std(axis=0)
seq = seq.mean(axis=0)

nocol = np.zeros((0,len(form)))
for file in nocol_files:
    nocol = np.vstack((seq, pd.read_csv(f"{folder_path}/{file}", header=1)['view_reward'].to_numpy()[1:-1]))
nocol_std = nocol.std(axis=0)
nocol = nocol.mean(axis=0)

rr = np.zeros((0,len(form)))
for file in rr_files:
    rr = np.vstack((seq, pd.read_csv(f"{folder_path}/{file}", header=1)['view_reward'].to_numpy()[1:-1]))
rr_std = rr.std(axis=0)
rr = rr.mean(axis=0)

# form_file = "form_2023-09-13_05-11-00.csv"
# seq_file = "seq_1_col_2023-09-12_23-26-40.csv"
# nocol_file = "seq_1_nocol_2023-09-13_05-52-21.csv"
# rr_file = "seq_5_col_2023-09-12_23-48-23.csv" #"seq_5_col_2023-09-13_03-07-02.csv"

# form = pd.read_csv(f"../data/corridor/{form_file}", header=1)['view_reward'].values[0:-1]
# seq = pd.read_csv(f"../data/corridor/{seq_file}", header=1)['view_reward'].values[1:-1]
# nocol = pd.read_csv(f"../data/corridor/{nocol_file}", header=1)['view_reward'].values[1:-1]
# rr = pd.read_csv(f"../data/corridor/{rr_file}", header=1)['view_reward'].values[1:-1]

print(form)
print(seq)
print(nocol)
print(rr)
t = np.arange(rr.size)

t_smooth=np.linspace(t.min(), t.max(), 500)

form_cub = interp1d(t, form, kind = "cubic")
form_smooth = form_cub(t_smooth)

seq_cub = interp1d(t, seq, kind = "cubic")
seq_smooth = seq_cub(t_smooth)

nocol_cub = interp1d(t, nocol, kind = "cubic")
nocol_smooth = nocol_cub(t_smooth)

rr_cub = interp1d(t, rr, kind = "cubic")
rr_smooth = rr_cub(t_smooth)
lw = 3
al = 0.3
cmap = plt.cm.Paired
colors = cmap(np.linspace(0, 1, 12))
plt.plot(t, form, label='Formation', color=colors[1], linewidth=lw)
plt.plot(t, seq, label='Sequential',color=colors[5], linewidth=lw)
plt.fill_between(t, seq - seq_std, seq + seq_std, color=colors[5], alpha=al)

plt.plot(t, nocol, label='Seq w/o Constraint',color=colors[7], linewidth=lw)
plt.fill_between(t, nocol - nocol_std, nocol + nocol_std, alpha=al,color=colors[7])

plt.plot(t, rr, label='Round Robin',color=colors[3], linewidth=lw)
plt.fill_between(t, rr - rr_std, rr + rr_std, alpha=al,color=colors[3])
plt.grid(True)
# plt.plot(t_smooth, form_smooth, label='Formation')
# plt.plot(t_smooth, seq_smooth, label='Sequential')
# plt.plot(t_smooth, nocol_smooth, label='Seq w/o Constraint')
# plt.plot(t_smooth, rr_smooth, label='Round Robin')
plt.legend()

plt.xlim(0, 14) 

plt.xlabel('Timesteps')
plt.ylabel(r'Total View Reward $\sqrt{\frac{px}{m^2}}$')
plt.savefig("rr")
plt.show()