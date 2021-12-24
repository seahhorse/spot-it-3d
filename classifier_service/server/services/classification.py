import numpy as np

from server import settings
from server.services import classifier


def predict(input_data):
    """Prediction function to classify if pathway is a
    drone or not.

    Input:
        input_data:
            Data Schema: [x, y, x, y, x, y, x, y......]
            Length: 16
            Normalized: True

    Output:
        output_prediction:
            Data Schema: {class: "drone", confidence: 0.8435}

    """
    classes = settings["model_settings"]["classes"]
    threshold = settings["model_settings"]["threshold"]

    results = classifier.predict_proba(np.array([input_data]))

    respond = [
        {"class": classes[str(each_class)], "confidence": str(each_conf)}
        for (each_class, each_conf) in enumerate(results[0])
        if each_conf >= threshold
    ]

    # print(respond)

    return respond[0]
