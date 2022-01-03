# Drone Classifier Service

Drone Classifier Service is a hosted model service capable of classifying flight path to determine if object is a `drone` or `other`.

## Components

### Prepare Service

Before starting the service, install the required dependencies with:

```
pip3 install -r requirements
```

You can also modify some of the `server_settings` and `model_settings`, under the `config.json` script. Every parameters that are modifiable can be found there.

### Initialize Service

Use the follow code block to initialize service:

```
python3 main.py
```

### Make a Prediction

To make a prediction, simply make an `POST API` Call to this route:

```
localhost:4000/api/classify
```

with the following sample payload:

```
{
  "flight_path": [
    0.65,
    0.496031746031746,
    0.653645833333333,
    0.491269841269841,
    0.6578125,
    0.486507936507937,
    0.6625,
    0.480952380952381,
    0.665625,
    0.476190476190476,
    0.66875,
    0.470634920634921,
    0.671354166666667,
    0.465079365079365,
    0.673958333333333,
    0.45952380952381
  ]
}
```

where the array `flight_path` is made up of 16 x & y `normalized` coordinates. Therefore, the schema of `flight_path` array is `[x, y, x, y, x, y ... up to 16 points]`.

### Sample Output Format

Here is a sample output from the prediction service:

```
{
    "class": "drone",
    "confidence": "0.9050773"
}
```
