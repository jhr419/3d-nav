import sys
import threading

from unitree_api.msg import Request
import rclpy
from rclpy.node import Node
import json
import termios
import tty

ROBOT_SPORT_API_IDS = {
    "DAMP":                 1001,
    "BALANCESTAND":         1002,
    "STOPMOVE":             1003,
    "STANDUP":              1004,
    "STANDDOWN":            1005,
    "RECOVERRYSTAND":       1006,
    "EULER":                1007,
    "MOVE":                 1008,
    "SIT":                  1009,
    "RISESIT":              1010,
    "SWITCHGAIT":           1011,
    "TRIGGER":              1012,
    "BODYHEIGHT":           1013,
    "FOOTRAISEHEIGHT":      1014,
    "SPEEDLEVEL":           1015,
    "HELLO":                1016,
    "STRETCH":              1017,
    "TRAJECTORYFOLLOW":     1018,
    "CONTINUOUSGAIT":       1019,
    "CONTENT":              1020,
    "WALLOW":               1021,
    "DANCE1":               1022,
    "DANCE2":               1023,
    "GETBODYHEIGET":        1024,
    "GETFOOTRATSEHEIGHT":   1025,
    "GETSPEDDLEVEL":        1026,
    "SWITCHJOYSTICK":       1027,
    "POSE":                 1028,
    "SCRAPE":               1029,
    "FRONTFLIP":            1030,
    "FRONTJUMP":            1031,
    "FRONTPOUNCE":          1032
}

sportModel = {
    'h':    ROBOT_SPORT_API_IDS["HELLO"],
    'j':    ROBOT_SPORT_API_IDS["FRONTJUMP"],
    'k':    ROBOT_SPORT_API_IDS["STRETCH"],
    'n':    ROBOT_SPORT_API_IDS["SIT"],
    'm':    ROBOT_SPORT_API_IDS["RISESIT"]
}

moveBindings = {
    'w': (1,0,0,0),
    'e': (1,0,0,-1),
    'a': (0,0,0,1),
    'd': (0,0,0,-1),
    'q': (1,0,0,1),
    's': (-1,0,0,0),
    'c': (-1,0,0,1),
    'z': (-1,0,0,-1),
    'E': (1,-1,0,0),
    'W': (1,0,0,0),
    'A': (0,1,0,0),
    'D': (0,-1,0,0),
    'Q': (1,1,0,0),
    'S': (-1,0,0,0),
    'C': (-1,-1,0,0),
    'Z': (-1,1,0,0),
}

class TeleoNode(Node):
    def __init__(self):
        super().__init__('telop_ctrl_keyboard')
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("angular",0.5)

        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value

    def publish(self, api_id, x=0.0, y=0.0, z=0.0):
        req = Request()
        req.header.identity.api_id = api_id
        js = {"x": x, "y": y, "z": z}
        req.parameter = json.dumps(js)
        self.pub.publish(req)

def getKey(settings):
    # read mode
    tty.setraw(sys.stdin.fileno)
    # get a key
    key = sys.stdin.read(1)
    # return original setting
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    
    teleoNode = TeleoNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleoNode, ))
    spinner.start()

    try:
        while True :
            key = getKey(settings)
            # 1. 结束终端 CTRL+C -->站立平衡状态
            if key == '\x03':
                break
            # 2. 运动模式切换：设置api_id
            elif key in sportModel.keys():
                teleoNode.publish(sportModel[key])
            # 3. 运动控制：设置api_id 和速度消息
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][3]
                teleoNode.publish(ROBOT_SPORT_API_IDS["MOVE"], 
                                  x = x_bind * teleoNode.speed,
                                  y = y_bind * teleoNode.speed,
                                  z = z_bind * teleoNode.angular)
            # 4. 速度调整：设置api_id为运动模式 设置速度消息 
    finally:
        rclpy.shutdown()
        teleoNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])


if __name__ == '__main__':
        main()