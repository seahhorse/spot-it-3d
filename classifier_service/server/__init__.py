import json

from flask import Flask


## Initialize service settings
with open("./config.json") as settings_reader:
    settings = json.load(settings_reader)
settings_reader.close()

## Initialize flask server
app = Flask(__name__)
app.secret_key = "super secret NUS key"

# pylint: disable=wrong-import-position
from server.routes import predict_routes
