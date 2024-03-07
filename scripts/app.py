#! /usr/bin/env python
import rospy
import threading
from flask import Flask, request, jsonify
from sensor_msgs.msg import NavSatFix
from ozurover_messages.srv import Abort, AddMarker, AddMarkerRequest

rover_coordinates = [0.0, 0.0]

def ros_callback(msg):
    global rover_coordinates
    rover_coordinates = [msg.latitude, msg.longitude]

threading.Thread(target=lambda: rospy.init_node('gps_service', disable_signals=True)).start()
rospy.Subscriber('/ares/gps', NavSatFix, ros_callback)
enqueue = rospy.ServiceProxy('/ares/goal/enqueue', AddMarker)

app = Flask(__name__)

# ESP8266
@app.route('/gps/ares', methods=['GET'])
def serve_gps():
    return {'Coordinates': rover_coordinates}
# ESP8266

# Deimos GUI
@app.route('/goal/abort', methods=['POST'])
def abort_goal():
    rospy.wait_for_service('/ares/goal/abort')
    abort = rospy.ServiceProxy('/ares/goal/abort', Abort)
    return abort()

@app.route('/goal/enqueue', methods=['POST'])
def enqueue_goal():
    print("Enqueueing goal")
    data = request.json
    if 'gps' not in data or 'type' not in data:
        print("Invalid request data")
        return jsonify(success=False)

    requestMsg = AddMarkerRequest()
    requestMsg.gps = GPS(float(data['gps'][0]), float(data['gps'][1]))
    requestMsg.type = int(data['type'])
    
    # send ROS service request to /ares/goal/enqueue
    try:
        rospy.wait_for_service('/ares/goal/enqueue')
        goal_enqueue = rospy.ServiceProxy('/ares/goal/enqueue', AddMarker)
        response = goal_enqueue(requestMsg)
        print("Goal enqueued")
        return jsonify(success=True)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return jsonify(success=False)
    except TypeError as e:
        print("Type error occurred: %s" % e)
        return jsonify(success=False)
# Deimos GUI

if __name__ == '__main__':
    app.run(port=3000)