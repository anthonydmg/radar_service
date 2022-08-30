from flask import Blueprint, render_template

alerts  = Blueprint("alerts", __name__)

@alerts.route("/")
def home():
    return render_template('home.html')