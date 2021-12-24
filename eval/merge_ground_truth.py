import json

session = "OC6_1D2C480pB"

f0 = open('../data/log/'+ session +'_true_0.json')
data0 = json.load(f0)
f1 = open('../data/log/'+ session +'_true_1.json')
data1 = json.load(f1)

for frame_no in range (len(data0)):
    data0[frame_no]["Cam 1"] = data1[frame_no]["Cam 1"]

with open('../data/log/'+ session +'_true.json', 'w') as outfile:
    json.dump(data0, outfile, indent=4)