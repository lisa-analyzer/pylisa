from flask import Flask
app = Flask(__name__)

@app.route("/")
async def hello_world():
    return "<p>Hello, World!</p>"