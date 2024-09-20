import rospy
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from cv_bridge import CvBridge
import cv2
import json
import base64
import websocket
import threading
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState

rospy.init_node('drone_control')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs)

bridge = CvBridge()
ws = None
is_connected = False

def set_led_color(r, g, b):
    leds = [LEDState(i, r, g, b) for i in range(72)]
    set_leds(leds)

def blink_led(r, g, b, duration=0.5, times=2):
    for _ in range(times):
        set_led_color(r, g, b)
        rospy.sleep(duration)
        set_led_color(0, 0, 0)
        rospy.sleep(duration)

def yellow_waiting_animation():
    while not is_connected and not rospy.is_shutdown():
        set_led_color(255, 255, 0)  # Yellow
        rospy.sleep(0.5)
        set_led_color(0, 0, 0)
        rospy.sleep(0.5)

def get_drone_state():
    try:
        telemetry = get_telemetry(frame_id='map')
        battery = rospy.wait_for_message('mavros/battery', BatteryState, timeout=0.5)
        gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=0.5)
        return {
            "battery": battery.percentage * 100,
            "altitude": telemetry.z,
            "gps": {"lat": gps.latitude, "lon": gps.longitude},
            "status": "flying" if telemetry.armed else "landed",
            "x": telemetry.x,
            "y": telemetry.y,
            "yaw": telemetry.yaw
        }
    except rospy.exceptions.ROSException:
        rospy.logwarn_throttle(10, "Timeout while waiting for drone state. Using default values.")
        return {
            "battery": 0,
            "altitude": 0,
            "gps": {"lat": 0, "lon": 0},
            "status": "unknown",
            "x": 0,
            "y": 0,
            "yaw": 0
        }

def process_command(command):
    try:
        if command['command'] == 'takeoff':
            navigate(x=0, y=0, z=command.get('height', 1.5), frame_id='body', auto_arm=True)
        elif command['command'] == 'land':
            land()
        elif command['command'] == 'navigate':
            navigate(x=command['x'], y=command['y'], z=command['z'], yaw=command.get('yaw', float('nan')), frame_id=command.get('frame_id', 'aruco_map'))
        elif command['command'] == 'set_velocity':
            set_velocity(vx=command['vx'], vy=command['vy'], vz=command['vz'], yaw=command.get('yaw', float('nan')), frame_id=command.get('frame_id', 'body'))
        elif command['command'] == 'set_led':
            leds = [LEDState(i, led['r'], led['g'], led['b']) for i, led in enumerate(command['leds'])]
            set_leds(leds)
        else:
            rospy.logwarn(f"Unknown command: {command['command']}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def send_state(event):
    global ws
    if ws and ws.sock and ws.sock.connected:
        try:
            state = get_drone_state()
            ws.send(json.dumps({"type": "state", "data": state}))
        except websocket.WebSocketConnectionClosedException:
            rospy.logwarn("WebSocket connection is closed. Reconnecting...")
            websocket_thread()
        except Exception as e:
            rospy.logerr(f"Error sending state: {str(e)}")

def camera_callback(data, camera_name):
    global ws
    if ws and ws.sock and ws.sock.connected:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        _, buffer = cv2.imencode('.jpg', cv_image)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        ws.send(json.dumps({"type": "camera", "name": camera_name, "data": jpg_as_text}))

def on_message(ws, message):
    try:
        data = json.loads(message)
        if isinstance(data, dict) and data.get("type") == "command":
            process_command(data)
        else:
            rospy.logwarn(f"Received unexpected message format: {message}")
    except json.JSONDecodeError:
        rospy.logerr(f"Failed to decode JSON message: {message}")
    except Exception as e:
        rospy.logerr(f"Error processing message: {str(e)}")

def on_error(ws, error):
    rospy.logerr(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    global is_connected
    is_connected = False
    rospy.loginfo("WebSocket connection closed")
    yellow_waiting_animation()

def on_open(ws):
    global is_connected
    rospy.loginfo("WebSocket connection opened")
    is_connected = True
    blink_led(0, 255, 0)  # Blink green twice

def websocket_thread():
    global ws
    ws_url = "ws://localhost:8000/ws/drone"
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
    rospy.Subscriber('/main_camera/image_raw', Image, lambda msg: camera_callback(msg, 'main_camera'))
    
    # Поиск доступных USB-камер
    available_cameras = []
    for i in range(10):  # Проверяем первые 10 возможных индексов камер
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    
    print(f"Available USB cameras: {available_cameras}")
    
    if available_cameras:
        rospy.Subscriber(f'/usb_cam_{available_cameras[0]}/image_raw', Image, lambda msg: camera_callback(msg, 'usb_camera'))
    
    rospy.Timer(rospy.Duration(1), send_state)

    # Start the yellow waiting animation in a separate thread
    waiting_thread = threading.Thread(target=yellow_waiting_animation)
    waiting_thread.daemon = True
    waiting_thread.start()

    ws_thread = threading.Thread(target=websocket_thread)
    ws_thread.daemon = True
    ws_thread.start()

    rospy.spin()