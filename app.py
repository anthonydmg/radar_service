from flask import Flask
from routes.radar import radar
app = Flask(__name__)
app.register_blueprint(radar)
