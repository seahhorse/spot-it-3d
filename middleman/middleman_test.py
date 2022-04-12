import requests
import json

BASE = "http://127.0.0.1:5000/"

file = open("example.json")
#file2 = open("empty_example.json")
coordinate_file = open("coordinates.json")

data = json.load(file)
#data2 = json.load(file2)
coordinate_data = json.load(coordinate_file)

# FRAMES
# API call test and examples
# GET method ("/list")
# Gets the latest frame from the spot it middleman
# response = requests.get(BASE + "latestframe")

# POST method ("/list")
# Posts the data to the server
response = requests.post(BASE + "latestframe", json = data)

# LIST method ("/list")
# List all the elements in the list
# response = requests.get(BASE + "listframes/3", data)
# print(response.json())

# CONVERT method ("/convert")
#response = requests.get(BASE + "convert", coordinate_data)
#print (response.json())

# IMAGE method ("/receiveimg")
#response = requests.post(BASE + "image") # Request to post
#response = requests.get(BASE + "image")

print(response.json())