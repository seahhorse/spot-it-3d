#!/usr/bin/python

# Convert output of 2D json file to CSV for each camera
# Inputs: input json file + camera number (e.g. 0  or 1)
# Outputs: CSV file called "output.csv" in the same folder as this script
# CSV file header format: [Frame Number] [ID] [centroid X] [Centroid Y] [width] [height]

import json
import csv
import sys

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

if len(sys.argv) != 3:
    print("Usage: python jsontocsv.py <path to json file> <cam_number>")
    sys,exit()
filename = sys.argv[1]
cam = 'Cam ' + sys.argv[2]

# Open json file
with open(filename) as json_file:
    data = json.load(json_file)

# create csv file
data_file = open('output.csv','w')
csv_writer = csv.writer(data_file)

# add headers
header = list()
header.append('Frame Number')
header.append('ID')
header.append('x')
header.append('y')
header.append('width')
header.append('height')
csv_writer.writerow(header)

# write data
for frame in data:
    # loop through detections of the specified camera in each frame
    idx = 0
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
        csv_writer.writerow(rowdata)
        idx += 1

data_file.close()