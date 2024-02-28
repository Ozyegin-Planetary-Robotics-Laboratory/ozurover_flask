#! /usr/bin/env python
import rospy
import threading
from flask import Flask
from ozurover_messages.msg import GPS

rover_coordinates = [0.0, 0.0]

def ros_callback(msg):
    global rover_coordinates
    rover_coordinates = [msg.latitude, msg.longitude]

threading.Thread(target=lambda: rospy.init_node('gps_service', disable_signals=True)).start()
rospy.Subscriber('/ares/gps', GPS, ros_callback)

app = Flask(__name__)

@app.route('/gps/ares', methods=['GET'])
def serve_gps():
    return {'Coordinates': rover_coordinates}

if __name__ == '__main__':
    app.run(port=3000)