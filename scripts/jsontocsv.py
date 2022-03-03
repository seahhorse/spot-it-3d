#!/usr/bin/python

# Convert output of 2D json file to CSV for each camera
# Inputs: input json file
# Outputs: CSV files "output_<cam_number>.csv", one CSV file per camera
# CSV file header format: [Frame Number] [ID] [centroid X] [Centroid Y] [width] [height]

import json
import csv
import sys

NUM_OF_CAMERAS = 2

# Calculate XY centroid from bottom_right and top_left coordinates
def get_centroid(bottom_right,top_left):
    centroid_x = (bottom_right[0] + top_left[0])/2
    centroid_y = (bottom_right[1] + top_left[1])/2
    return centroid_x,centroid_y

# Calculate detection width/height from bottom_right and top_left coordinates
def get_dimensions(bottom_right,top_left):
    width = abs(bottom_right[0] - top_left[0])
    height = abs(top_left[1] - bottom_right[1])
    return width,height

###########
# Main
###########

if len(sys.argv) != 2:
    print("Usage: python jsontocsv.py <path to json file>")
    sys,exit()
filename = sys.argv[1]

# Open json file
with open(filename) as json_file:
    data = json.load(json_file)

# create csv files, one for each camera
data_file = [None]*NUM_OF_CAMERAS
csv_writer = [None]*NUM_OF_CAMERAS
for i in range(0,NUM_OF_CAMERAS):
    csv_filename = 'output_' + str(i) + '.csv'
    data_file[i] = open(csv_filename,'w')
    csv_writer[i] = csv.writer(data_file[i])

# add headers
header = list()
header.append('Frame Number')
header.append('ID')
header.append('x')
header.append('y')
header.append('width')
header.append('height')
for i in range(0,NUM_OF_CAMERAS):
    csv_writer[i].writerow(header)

# write data
for frame in data:
    for i in range(0,NUM_OF_CAMERAS):
        # loop through detections of the specified camera in each frame
        idx = 0
        cam = "Cam " + str(i)
        while idx < len(frame[cam]):
            # for each detection, obtain centroid and ID number
            bottom_right = eval(frame[cam][idx]['Bottom-Right']) # eval as tuple
            top_left = eval(frame[cam][idx]['Top-Left'])
            cen_x,cen_y = get_centroid(bottom_right,top_left)
            width,height = get_dimensions(bottom_right,top_left)
            # rowdata contains the data for a given detection at a given frame
            rowdata = list()
            rowdata.append(frame['Frame Number'])
            rowdata.append(frame[cam][idx]['ID'])
            rowdata.append(cen_x)
            rowdata.append(cen_y)
            rowdata.append(width)
            rowdata.append(height)
            csv_writer[i].writerow(rowdata)
            idx += 1

for i in range(0,NUM_OF_CAMERAS):
    data_file[i].close()