from flask import Blueprint
radar = Blueprint("radar", __name__)
@radar.route("/")
def home():
    return "Home"

@radar.route('/api/radar/distance', methods = ["GET"])
def distance():
    data = '2.55m'
    return data

