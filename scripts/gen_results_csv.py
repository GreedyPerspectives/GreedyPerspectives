
import os 
import pandas as pd
import numpy as np
import statistics

tests = ["split", "large","merge", "corridor", "forest",]
actCount = [4, 18, 4, 2, 2]
# actCount = [2, 2, 6, 3, 2]
methods = ["form", "seq_1_nocol", "seq_1_col"]
out = pd.DataFrame(columns=["test","form_avg","form_std", "nocol_avg","nocol_std","seq_avg","seq_std"])

for test, numAc in zip(tests, actCount):
    print(test)
    row = [test.capitalize()]
    for method in methods:
        folder_path = f"../data/{test}"
        files_in_folder = os.listdir(folder_path)
        csv_files = [f for f in files_in_folder if f.endswith('.csv') and '_' in f]
        avgs = []
        for csv_file in csv_files:
            if method in csv_file:
                df = pd.read_csv(f"{folder_path}/{csv_file}", header=1)
                avg_view_reward = df['view_reward'].mean()/numAc
                avgs.append(avg_view_reward)
        row.append(round(statistics.mean(avgs)))
        row.append(round(statistics.pstdev(avgs)))
    out.loc[len(out.index)] = row 
out.set_index('test', inplace=True)

out.to_csv("viewreward.csv")
out.to_latex("viewreward.tex")
print(out)

