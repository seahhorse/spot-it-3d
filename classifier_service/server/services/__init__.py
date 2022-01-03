import os
import pickle

from server import settings


classifier = pickle.load(
    open(
        os.path.join(
            os.path.abspath(os.curdir),
            settings["server_settings"]["model_path"],
        ),
        "rb",
    )
)
