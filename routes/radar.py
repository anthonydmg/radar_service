
from flask import Blueprint
from random import random
from flask import jsonify
from serial_comunication_radar import init_service_radar_distance, read_distance

radar = Blueprint("radar", __name__)

com = init_service_radar_distance()

# Wait for it to start
com.wait_start()
print('Sensor activated')

# Read out distance start
dist_start = com.register_read(0x81)
print(f'dist_start={dist_start / 1000} m')

dist_length = com.register_read(0x82)
print(f'dist_length={dist_length / 1000} m')

@radar.route("/")
def home():
    return "Home"

@radar.route('/api/radar/distance', methods = ["GET"])
def distance():
    distance = read_distance(com=com)
    #distance = random() * 20.0
    return jsonify(distance = distance)

