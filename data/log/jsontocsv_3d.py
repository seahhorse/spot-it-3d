#!/usr/bin/python

# Convert output of 3D json file to CSV for each camera
# Inputs: input json file
# Outputs: CSV files "output_<cam_number>.csv", one CSV file per camera
# CSV file header format: [Frame Number] [ID]
#                         Cam 1: [centroid X] [Centroid Y] 
#                                [bounding box width] [bounding box height] [target area]
#                         Cam 2: [centroid X] [Centroid Y] 
#                                [bounding box width] [bounding box height] [target area]

import json
import csv
import sys

if len(sys.argv) != 2:
    print("Usage: python jsontocsv.py <path to json file>")
    sys,exit()
filename = sys.argv[1]

# Open json file
with open(filename) as json_file:
    data = json.load(json_file)

# create csv file
csv_filename = 'output_3d.csv'
data_file = open(csv_filename,'w')
csv_writer = csv.writer(data_file)

# add headers
header = list()
header.append('Frame Number')
header.append('Target ID')
header.append('Cam 1: x')
header.append('Cam 1: y')
header.append('Cam 1: bounding box width')
header.append('Cam 1: bounding box height')
header.append('Cam 1: target area')
header.append('Cam 2: x')
header.append('Cam 2: y')
header.append('Cam 2: bounding box width')
header.append('Cam 2: bounding box height')
header.append('Cam 2: target area')
csv_writer.writerow(header)

# write data
for frame in data['Detections']:
    # loop through re-id targets in each frame
    rowdata = list()
    rowdata.append(frame['Frame Number'])
    rowdata.append(frame['ID'])
    rowdata.append(frame['Cam 1 X'])
    rowdata.append(frame['Cam 1 Y'])
    rowdata.append(frame['Cam 1 Bounding Box Size'])
    rowdata.append(frame['Cam 1 Bounding Box Size'])
    rowdata.append(frame['Cam 1 Target Size'])
    rowdata.append(frame['Frame Number'])
    rowdata.append(frame['Cam 2 X'])
    rowdata.append(frame['Cam 2 Y'])
    rowdata.append(frame['Cam 2 Bounding Box Size'])
    rowdata.append(frame['Cam 2 Bounding Box Size'])
    rowdata.append(frame['Cam 2 Target Size'])
    csv_writer.writerow(rowdata)

data_file.close()