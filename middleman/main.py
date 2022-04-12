from flask import Flask, request, jsonify
from flask_restful import Api, Resource, reqparse, abort
import base64
import math
import json

app = Flask(__name__)
api = Api(app)

# constants
FRAME_BUFFER_SIZE = 5
METER_TO_ARC_SECONDS_CONST = 30.8
IMAGE_PATH = "image.jpg"

# variable that contains the frames. 
# the lastest frame will be appended in the first element.
# older frames will be shifted to the higher index.

frames = [
]

image = [
]

# Frame class to obtain the frames from spot-it-3d.
class LatestFrame(Resource):
    # method to get the frame of a given frame id
    def get(self):
        # checks if the frame exists.
        check_frame_buffer()
        # returns the first element in the frame.
        frame = get_latest_frame()
        return frame, 200

    # method to post frame to the backend
    def post(self):
        # parses the argument sent by the post request
        try:
            args = frames_put_args.parse_args()
        except Exception as e: 
            print(e)
        # shifts the buffer to accomodate the latest frame detection
        shift_frame_index(get_list_size())
        append_frames(args)
        return "Latest frame sucessfully added", 201

#checks if there are already frames in the buffer
def check_frame_buffer():
    if (len(frames) == 0):
        return abort(404, error = '404 Not Found: Frame not found. There are no frame(s) added to the frame buffer.')

# returns the first element of the buffter
def get_latest_frame():
    return frames[0]

# returns the current list size
def get_list_size():
    return len(frames)

# shifts the elements of the list by one index
def shift_frame_index(list_size):
    # initialize the list if the list is empty.
    if(list_size == 0):
        return
    else:
        # appends the list until the maxiumum frame size.
        if (list_size < FRAME_BUFFER_SIZE):
            frames.append(frames[list_size - 1])
        # shifts the frames by one index place.
        for i in reversed(range(0,list_size - 1)):
            frames[i+1] = frames[i]

def append_frames(args):
    if(get_list_size() == 0):
        print("Adding the first element to the list")
        frames.append(args)
    else:
        frames[0] =  args

# JSON parsesr for frames
frames_put_args = reqparse.RequestParser(bundle_errors=True)
frames_put_args.add_argument("Detections", type=str, action= 'append', help="Please provide detection data in the request", required=True)
#frames_put_args.add_argument("Detections 2", type=str, action= 'append', help="Fake geolocation", required=True)

# Converter class to handle conversion from relative to absolute position
class Converter(Resource):
    def get(self):
        # posts the latitude and longitude of the camera position
        camera_location = lat_long_args.parse_args()
        data = convert_relative_to_absolute(camera_location)
        return data, 200

# performs calcualtion on the lat long and the base psoe from spot it.
# returns the list of the lat and long.
def convert_relative_to_absolute(coordinates):
    
    validate_camera_coordinates(coordinates)

    base_lat = coordinates["base_lat"]
    base_long = coordinates["base_long"]
    heading = coordinates["heading"]
    object_x = coordinates["object_x"]
    object_z = coordinates["object_z"]

    # calcualtes the absolute lat long
    delta_lat_in_meters = object_z * math.cos(math.radians(heading)) + object_x * math.sin(math.radians(heading))
    delta_long_in_meters = -object_z * math.sin(math.radians(heading)) + object_x * math.cos(math.radians(heading))

    # converts the delta lat long to arc seconds
    delta_lat_in_arc_seconds = delta_lat_in_meters / METER_TO_ARC_SECONDS_CONST
    delta_long_in_arc_seconds = delta_long_in_meters / METER_TO_ARC_SECONDS_CONST
    base_lat_in_arc_seconds = base_lat * 3600
    base_long_in_arc_seconds = base_long * 3600

    # combines the absolute lat and absolute_long into json file
    absolute_pose = [base_lat_in_arc_seconds + delta_lat_in_arc_seconds, base_long_in_arc_seconds + delta_long_in_arc_seconds]
    base_coordinates_lat = convert_to_dms(absolute_pose[0])
    base_coordinates_long = convert_to_dms(absolute_pose[1])
    
    absolute_pose_json = json.dumps({"latitude":base_coordinates_lat, "longitude":base_coordinates_long})

    return absolute_pose_json

def validate_camera_coordinates(coordinates):
    print(coordinates)
    # validates camera longitude
    validate_base_long(float(coordinates["base_long"]))
    validate_base_lat(float(coordinates["base_lat"]))
    validate_heading(float(coordinates["heading"]))
    validate_object_z(float(coordinates["object_z"]))


def validate_base_long(longitude):
    if (longitude < -180.0):
        abort(406, error = '406 Not Acceptable: The base longitude must be more than -180 degrees, please check your input.')
    if (longitude > 180.0):
        abort(406, error = '406 Not Acceptable: The base longitude must be less than 180 degrees, please check your input.')

def validate_base_lat(latitude):
    if (latitude < -90.0):
        abort(406, error = '406 Not Acceptable: The base latitude must be more than -90 degrees, please check your input.')
    if (latitude > 90.0):
        abort(406, error = '406 Not Acceptable: The base latitude must be less than 90 degrees, please check your input.')
    
def validate_heading(heading):
    if (heading < 0.0):
        abort(406, error = '406 Not Acceptable: The base heading must be between 0.0 and 360.0 degrees, please check your input.')

def validate_object_z(obj_z):
    if (obj_z < 0.0):
        abort(406, error = '406 Not Acceptable: The base heading must be between 0.0 and infinity meters, please check your input.')

# returns coordinates in a dictionary 
# format of dictionary:
# {
#   "degreees" : <int:degrees>,
#   "minutes"  : <int:minutes>,
#   "seconds"  : <int:seconds>
# }

def convert_to_dms(degrees):
    
    # converts the degrees to degree seconds
    d = int(degrees / 3600)
    m = int((degrees - d*3600) / 60)
    s = degrees - d*3600 - m*60

    coordinate_in_dms = {
        "degreees" : d,
        "minutes"  : m,
        "seconds"  : s
   }

    return coordinate_in_dms

#JSON parser for lat long
lat_long_args = reqparse.RequestParser(bundle_errors=True)
lat_long_args.add_argument("base_lat", type=float, help =  "Please provide the latitude", required = True)
lat_long_args.add_argument("base_long", type=float, help =  "Please provide the longitude", required = True)
lat_long_args.add_argument("heading", type=float, help = "Please provide the heading", required = True)
lat_long_args.add_argument("object_x", type=float, help = "Please provide the object x coordinate", required = True)
lat_long_args.add_argument("object_z", type=float, help = "Please provide the object z coordinate", required = True)

class ListFrames(Resource):
    def get(self):
        check_frame_buffer()
        frames_json = json.dumps(frames)
        return frames_json, 200

class Image(Resource):
    # get method to retreive the latest image byte array.
    def get(self):
        print(image)
        check_existing_image()
        image_json = json.dumps(str(image[0]))
        return image_json, 200

    # post method to save the latest image to the server
    def post(self):
        # reads the image from the folder.
        try:
            # opens the file where the image is located.
            with open(IMAGE_PATH,'rb') as img_obj:
                content = img_obj.read()
                img_str = base64.b64encode(content)
                image.append(img_str)
        except FileNotFoundError:
            msg = "Sorry, the file at " + IMAGE_PATH + "does not exist."
            print(msg)

def check_existing_image():
    if(image == None):
        abort (404, "Frame not found")

# api.add_resource(Frame, "/frame")
api.add_resource(LatestFrame, "/latestframe")
api.add_resource(Converter, "/convert")
api.add_resource(ListFrames, "/listframes")
api.add_resource(Image, "/image")

@app.errorhandler(404)
def resource_not_found(e):
    return jsonify(error=str(e)), 404

if __name__ == "__main__":
    app.run(debug = True)