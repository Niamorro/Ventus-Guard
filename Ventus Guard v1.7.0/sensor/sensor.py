import rospy
import json
import websocket
import threading
from std_msgs.msg import Float32
from clover import srv
from sensor_msgs.msg import Range

rospy.init_node('rangefinder_control')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

ws = None
is_connected = False

def get_rangefinder_data():
    try:
        dist = rospy.wait_for_message('rangefinder/range', Range).range
        print(dist)
        return {
            "range": dist,
            "timestamp": rospy.Time.now().to_sec()
        }
    except rospy.ServiceException as e:
        rospy.logwarn_throttle(10, f"Error getting rangefinder data: {e}. Using default values.")
        return {
            "range": 0,
            "timestamp": rospy.Time.now().to_sec()
        }

def send_rangefinder_data(event):
    global ws
    if ws and ws.sock and ws.sock.connected:
        try:
            data = get_rangefinder_data()
            ws.send(json.dumps({"type": "rangefinder", "data": data}))
        except websocket.WebSocketConnectionClosedException:
            rospy.logwarn("WebSocket connection is closed. Reconnecting...")
            websocket_thread()
        except Exception as e:
            rospy.logerr(f"Error sending rangefinder data: {str(e)}")

def on_message(ws, message):
    rospy.loginfo(f"Received message: {message}")

def on_error(ws, error):
    rospy.logerr(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    global is_connected
    is_connected = False
    rospy.loginfo("WebSocket connection closed")

def on_open(ws):
    global is_connected
    rospy.loginfo("WebSocket connection opened")
    is_connected = True

def websocket_thread():
    global ws
    ws_url = "ws://localhost:8000/ws/rangefinder"
    while not rospy.is_shutdown():
        try:
            ws = websocket.WebSocketApp(ws_url,
                                        on_message=on_message,
                                        on_error=on_error,
                                        on_close=on_close,
                                        on_open=on_open)
            ws.run_forever()
        except Exception as e:
            rospy.logerr(f"WebSocket error: {str(e)}")
        rospy.sleep(5)  # Wait before attempting to reconnect

if __name__ == "__main__":
    rospy.Timer(rospy.Duration(0.1), send_rangefinder_data)  # Send data every 100ms

    ws_thread = threading.Thread(target=websocket_thread)
    ws_thread.daemon = True
    ws_thread.start()

    rospy.spin()
