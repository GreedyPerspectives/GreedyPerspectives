import matplotlib.pyplot as plt
import os
import pandas as pd
import numpy as np

tests = [
    "split",
    "large",
    "merge",
    "corridor",
    "forest",
]
actCount = [4, 18, 4, 2, 2]
# actCount = [2, 2, 6, 3, 2]
data = {
    "Formation": [0, 0, 0, 0, 0],
    "Sequential": [[], [], [], [], []],
    "Sequential (no inter-robot constraint)": [[], [], [], [], []],
    # 'Round Robin': [0,0,0,0,0],
}

for test, numAc in zip(tests, actCount):
    print(test)
    folder_path = f"../data/{test}"
    files_in_folder = os.listdir(folder_path)
    csv_files = [f for f in files_in_folder if f.endswith(".csv") and "_" in f]
    for csv_file in csv_files:
        f_n = None
        if "form" in csv_file:
            df = pd.read_csv(f"{folder_path}/{csv_file}", header=1)
            avg_view_reward = df["view_reward"].mean() / numAc
            data["Formation"][tests.index(test)] = avg_view_reward
            continue
        elif "nocol" in csv_file:
            f_n = "Sequential (no inter-robot constraint)"
        elif "seq_1" in csv_file:
            f_n = "Sequential"
        # elif "seq_5" in csv_file and test in ["corridor", "split"]:
        #     f_n = f"Round Robin"
        if not f_n:
            continue
        # print(f"{folder_path}/{csv_file}")
        df = pd.read_csv(f"{folder_path}/{csv_file}", header=1)
        avg_view_reward = df["view_reward"].mean() / numAc
        data[f_n][tests.index(test)].append(avg_view_reward)


x = np.arange(len(tests)) * 5  # the label locations
cmap = plt.cm.tab20
colors = ["#FDB515", "#00964780", "#04367380"]

fig, ax = plt.subplots()
# form_dot = ax.scatter(
#     x, data["Formation"], label="Formation", color=colors[0]
# )
seq_box = ax.boxplot(
    [s/f for s,f in zip(data["Sequential"], data["Formation"])],
    positions=x+1,
    patch_artist=True,
    widths=0.75,
    flierprops=dict(
        marker="o",
        markerfacecolor=colors[1],
        markersize=6,
        linestyle="none",
        markeredgecolor="none",
    ),
    medianprops=dict(linewidth=0),
    showmeans=True,
    meanline=True,
    meanprops=dict(linestyle='-', color="black"),
)
nocol_box = ax.boxplot(
    [s/f for s,f in zip(data["Sequential (no inter-robot constraint)"], data["Formation"])],
    positions=x+2,
    patch_artist=True,
    widths=0.75,
    flierprops=dict(
        marker="o",
        markerfacecolor=colors[2],
        markersize=6,
        linestyle="none",
        markeredgecolor="none",
    ),
    medianprops=dict(linewidth=0),
    showmeans=True,
    meanline=True,
    meanprops=dict(linestyle='-', color="black"),
)

for patch in seq_box["boxes"]:
    patch.set_facecolor(colors[1])
for patch in nocol_box["boxes"]:
    patch.set_facecolor(colors[2])

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel(r"Normalized View Reward")
ax.set_xticks(x+1.5, [t.capitalize() for t in tests])
# ax.legend(loc='upper left', ncol=1)
l = ax.axhline(y=1, color='#EF3A47', linestyle='--', label='Formation')
ax.legend(
    [seq_box["boxes"][0], nocol_box["boxes"][0], l],
    ["Sequential ", "Sequential (no inter-robot constraint)","Formation Baseline"],
    loc="upper left",
)

# ax.grid(True)
# ax.set_ylim(1000, 2000)

plt.savefig("seq_results.png")
plt.show()
