#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from panda_chess.srv import send_board
from sensor_msgs.msg import Image
from ultralytics import YOLO
import json
import cv2
from cv_bridge import CvBridge

####GLOBAL VARS####
global model, names, frame

frame = None
# change the path to the board detection model
model = YOLO('/home/kovan4/Downloads/chess.pt')
names = model.names


def imgCallback(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")


def detect_board_corners():
    global model, names, frame
    new_frame = frame.copy()
    image = cv2.resize(new_frame, (640,640), interpolation=cv2.INTER_AREA)
    detectedList = []
    
    while True: 
        if image is not None:
            results = model(image, conf=0.7)
            
            for r in results:
                for box_info in range(len(r.boxes)):
                    a = r.boxes[box_info].xyxy[0].tolist()
                    class_name = names[int(r.boxes.cls[box_info])]

                    # for board detection
                    x1 = round(int(a[0]),2)
                    x2 = round(int(a[2]),2)
                    y1 = round(int(a[1]),2)
                    y2 = round(int(a[3]),2)

                    detectedList.append(x1)
                    detectedList.append(x2)
                    detectedList.append(y1)
                    detectedList.append(y2)

                    print(detectedList)
                    return detectedList
            
        else:
            print("Couldn't capture frame")
            continue




########## SERVER FUNCTIONS ##########
def send_board_corners(req):
    if req.qry == "send_board_corners":
        board_corners = detect_board_corners()
        res = json.dumps(board_corners)

    return res


def send_chess_table():
    print("srv")
    print(send_board)
    service = rospy.Service('send_board', send_board, send_board_corners)


########## SUBSCRIBED TO TURN_CONTROLLER ##########
def listener():
    rospy.init_node('board_server', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, imgCallback)
    rospy.spin()


########## MAIN ##########
if __name__ == "__main__":
    try:
        send_chess_table()
        listener()
    except rospy.ROSInterruptException:
        pass
