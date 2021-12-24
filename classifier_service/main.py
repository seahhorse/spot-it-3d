from server import app, settings

if __name__ == "__main__":
    app.run(
        host="0.0.0.0", port=settings["server_settings"]["port"], debug=True
    )
