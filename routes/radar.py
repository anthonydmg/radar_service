from flask import Blueprint
from random import random
from flask import jsonify

radar = Blueprint("radar", __name__)
@radar.route("/")
def home():
    return "Home"

@radar.route('/api/radar/distance', methods = ["GET"])
def distance():
    distance = random() * 20.0
    return jsonify(distance = distance)

