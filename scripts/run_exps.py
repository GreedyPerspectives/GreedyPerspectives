import subprocess
import csv
import yaml
tests = ["corridor"]#"forest","corridor","merge","large","split"]
methods = ["seq","nocol"]


for test in tests:
    file = "experiments/"+test+"/seq.yaml"
    with open(file, 'r') as f:
        agCt = len(yaml.load(f)['planner']['world']['agents'])

    rows = []
    with open("experiments/"+test+"/starts.csv", 'r') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i==0: 
                continue
            rows.append([float(value) for value in row])
    for j in range(10):
        print("_________________________________" + str(j))
        for method in methods:
            file = "experiments/"+test+"/"+method+".yaml"
            if method != "form":
                with open(file, 'r') as f:
                    data = yaml.load(f)
                    for i, agentData in enumerate(data['planner']['world']['agents']):
                        data['planner']['world']['agents'][i]['agent']['startPose'] = rows[agCt*j + i]
                with open(file, 'w') as f:
                    # data['planner']['world']['time']['horizon'] = 1
                    yaml.dump(data, f)
            subprocess.call(['./coverage-planner', file])

