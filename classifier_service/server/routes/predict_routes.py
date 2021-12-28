import json
import logging

from flask import request
from server import app

from server.services.classification import predict


@app.route("/api/classify", methods=["POST"])
def classify():
    data = json.loads(request.data.decode("utf-8"))
    flight_path = data["flight_path"]
    return predict(flight_path)


@app.errorhandler(500)
def server_error(e):
    logging.exception("An error occurred during a request.")
    return (
        """
        An internal error occurred: <pre>{}</pre>
        See logs for full stacktrace.
        """.format(
            e
        ),
        500,
    )