# spot_middleman

Spot Middleman is a Python application that handles the information flow between SPOT-IT-3D and the client.

## Installation Guide
Run the installation script to install the dependencies.
```
pip install -r /path/to/requirements.txt
```

The main SPOT-IT-3D compile script will automatically run SPOT-IT-3D and the middleman. You can also run the server independently (without the SPOT-IT-3D code) using this command:
```
python3 main.py
```
# Documentation
SPOT-IT-3D will output frames with annotated information, which will be POST-ed onto the middleman. GET requests can be sent to the middleman to retrieve these frames. An added functionality is the conversion of relative coordinates to absolute coordinates in Latitude and Longitude, given a specific set of base coordinates. The following will document the various methods used.

The overall middleman process can be observed in the following flow chart:

<b>Latestframe & Listframe</b>

![Data Stream](https://user-images.githubusercontent.com/65325079/150501051-ad6cdf56-1408-4149-b82b-6d152202cd8e.png)

<b>Convert Coordinates</b>

![Convert](https://user-images.githubusercontent.com/65325079/150501118-ddc3aaab-2f31-4b90-b285-9cd859536a3c.png)

## Initial Configuration

To configure SPOT-IT-3D to post to the middleman, open the parameters file (/include/multi_cam_params.hpp) and set the following parameter to true:

```
const bool POST_TO_MIDDLEMAN_ = true;
```

By default, the middleman server will bind to localhost, port 500. To change the binding IP address and port, do the following steps (assuming the intended middleman IP is 192.168.1.23 and port is 5000):

1. In main.py, update the final line (which calls the Flask app) to the following:

```python
if __name__ == "__main__":
    app.run(debug = True, host="192.168.1.23")
```

2. Configure SPOT-IT-3D to post to the correct middleman URL by opening the parameters file and change the following parameter:

```cpptools
const std::string MIDDLEMAN_IP_ = "192.168.1.23:5000"
```


## Format of JSON Packet
````
{
	"Detections" :
	[
		{
			“Frame-Number” : <Frame No.>,
			"Objects" :
			[
				{
					“ID” : <ID No.>,
					“XYZ-Coordinates :
					[
						<X-Coordinate>
						<Y-Coordinate>
						<Z-Coordinate>
					]
				}
			,
				{
					“ID” : <ID No.>,
					“XYZ-Coordinates :
					[
						<X-Coordinate>
						<Y-Coordinate>
						<Z-Coordinate>
					]
				}
			],
			<etc.>
		},
		{
		<Subsequent data>
		}
	]
}
````		
**Example**
````
{
	"Detections" :
	[
		{
			"Frame-Number" : "1",
			"Objects" :
			[
				{
					"ID" : 0,
					"XYZ-Coordinates" : 
					[
						-5.5999999999999996,
						3.8100000000000001,
						10.17
					]
				}
			,
				{
					"ID" : 1,
					"XYZ-Coordinates" : 
					[
						-5.5999999999999996,
						3.8100000000000001,
						10.17
					]
				}
			]
		}
	]
}
````

## Latest Frame
````>> /latestframe ````
#### == GET ==
Returns JSON Object
Returns the latest frame available in the middleman.

\~Error Handling\~
- **Empty Buffer**
If there is no frame in the buffer, it will return:
> {'error': '404 Not Found: Frame not found. There is no frame(s) added to the frame buffer.'}
 - **Additional Parameters**
 Additional parameters in the URL after /latestframe
 > {'error': '404 Not Found: The requested URL was not found on the server. If you entered the URL manually please check your spelling and try again.'}
 - **JSON File In Request**
 There will not be any issues experienced by the user.
 
 - **Additional Parameters In Request**
 The additional parameters will be discarded. The request parser only recognizes "Detections" as the argument.
 
#### == POST ==
Adds JSON object to the buffer array.  
If the buffer array is full (based on buffer size), the oldest frame will be overridden.  
This method takes in 3 main arguments:
````id````
Refers to frame ID number. **This field is required.**
````cameras````
Number of cameras in one frame. **This field is required.**
````objects````
Number of objects in one frame. **This field is required.**

\~Error Handling\~
- **Empty Frame**
An empty frame is posted to the middleman. The JSON packet has an empty "Detections" field.
> {'error': {'Detections': 'Please provide detection data in the request'}}
- **Non-JSON File**
A file of filetype other than JSON is posted to the middleman.
> {'error': {'Detections': 'Please provide detection data in the request'}}
- **Additional Parameters**
 Additional parameters in the URL after /latestframe
 > {'error': '404 Not Found: The requested URL was not found on the server. If you entered the URL manually please check your spelling and try again.'}

## List Frames
````>> /listframes ````
#### == GET ==
Returns a JSON object.  
Returns the array of all frames in the buffer.  
Maximum size of array is the maximum size of the buffer.

\~Error Handling\~
- **Empty Buffer**
If there is no frame in the buffer, it will return:
> {'error': '404 Not Found: Frame not found. There is no frame(s) added to the frame buffer.'}
 - **Additional Parameters**
Additional parameters in the URL after /latestframe
> {'error': '404 Not Found: The requested URL was not found on the server. If you entered the URL manually please check your spelling and try again.'}
- **JSON File In Request**
There will not be any issues experienced by the user.

- **Additional Parameters In Request**
The additional parameters will be discarded. The request parser only recognizes "Detections" as the argument.
## Convert
````>> /convert````
#### == POST ==
Returns JSON object.  
Returns the absolute position based on the relative position and benchmark position of cameras.  
This method takes in five arguments:

````base_lat````
Ranges from -90 degrees to +90 degrees

````base_long````
Ranges from -180 degrees to +180 degrees

> Latitude and Longitude are in units of degrees, arc-minutes, and arc-seconds

> 1 degree = 60 arc-minutes

> 1 arc-minute = 60 arc-seconds

````object_x````
Ranges from -inf to +inf

````object_z````
Ranges from 0 to +inf

````heading````
Ranges from 0 to 360 degrees

\~Error Handling\~
- **Wrong Input Type**

If the input arguments are of the wrong type, it will return:

>>Wrong Longitude Range

> {'error': '406 Not Acceptable: The base longitude must be more than -180 degrees, please check your input.'}

> {'error': '406 Not Acceptable: The base longitude must be less than 180 degrees, please check your input.'}

>>Wrong Latitude Range

> {'error': '406 Not Acceptable: The base latitude must be more than -90 degrees, please check your input.'}

> {'error': '406 Not Acceptable: The base latitude must be less than 90 degrees, please check your input.'}

>>Wrong Heading Range

> {'error': '406 Not Acceptable: The base heading must be between 0.0 and 360.0 degrees, please check your input.'}

>>Wrong Z Coordinates Range

> {'error': '406 Not Acceptable: The base z coordinate must be between 0.0 and infinity meters, please check your input.'}


- **Additional Parameters**

Additional parameters in the URL after /convert:
> {'error': '404 Not Found: The requested URL was not found on the server. If you entered the URL manually please check your spelling and try again.'}

- **Non-JSON File**

A file of filetype other than JSON is posted to the middleman.
> {'error': {'Detections': 'Please provide detection data in the request'}}

- **Additional Parameters In Request**

The additional parameters will be discarded. The request parser only recognizes "Detections" as the argument.
