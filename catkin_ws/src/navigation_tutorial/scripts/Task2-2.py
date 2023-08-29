#!/usr/bin/env python3
import actionlib
import tf
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

class ActionGoal():
    def __init__(self):
        rospy.init_node('action_goal')
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()  # action serverの準備ができるまで待つ
        print(333)

    def set_goal(self, x, y, yaw):
        self.goal = MoveBaseGoal()  # goalのメッセージの定義
        self.goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
        print(444)
        
        # ゴールの姿勢を指定
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        self.goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    def send_action(self, duration=30.0):
        self.action_client.send_goal(self.goal)  # ゴールを命令
        result = self.action_client.wait_for_result(rospy.Duration(duration))
        print(555)
        return result


import copy
from typing import List

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
from ultralytics import YOLO
from ultralytics.engine.results import Results

first = [-1, 10 ** 9, -1, 10 ** 9] # 一人目 左手首の最大、最小 右手首の最大、最小　（x座標)
second = [-1, 10 ** 9, -1, 10 ** 9] # 二人目 上に同じ
fisright = False 
direction = ''
ag = ActionGoal()
class ObjectDetection:
    def __init__(self):
        #rospy.init_node('object_detection', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_rgb)

        self.bridge = CvBridge()
        self.rgb_image = None

        self.model = YOLO('yolov8n-pose.pt')

    def callback_rgb(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.rgb_image = cv_array

    def process(self):
        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue
            
            try:
                results: List[Results] = self.model.predict(self.rgb_image)
                keypoint = results[0].keypoints
                #print(keypoint.data[:, 5:11])
                first[0] = max(first[0], keypoint.data[0][9][0]) # 左手首
                first[1] = min(first[1], keypoint.data[0][9][0])
                first[2] = max(first[2], keypoint.data[0][10][0]) # 右手首
                first[3] = min(first[3], keypoint.data[0][10][0])
                second[0] = max(second[0], keypoint.data[1][9][0])
                second[1] = min(second[1], keypoint.data[1][9][0])
                second[2] = max(second[2], keypoint.data[1][10][0])
                second[3] = min(second[3], keypoint.data[1][10][0])

                fdif = max(first[0] - first[1], first[2] - first[3]) # 一人目の手首の動き
                sdif = max(second[0] - second[1], second[2] - second[3]) # 二人目の手首の動き

                global fisright # 不要
                fisright = False # 一人目が右 (x座標が大きい) かどうか

                if keypoint.data[0][9][0] > keypoint.data[1][9][0]:
                    fisright = True
                    print(fisright,111111111111111111111111)
                t = 8
                if fdif > t:
                    #direction = 'right' if fisright else 'left' 
                    print(111)
                    if fisright:
                        ag.set_goal(1.7, 3.5, 160)
                        print('r')
                        res = ag.send_action()
                    else:
                        ag.set_goal(1.7, 2.6, 160)
                        res = ag.send_action()
                        print('l')

                    break
                elif sdif > t:
                    #direction = 'right' if not fisright else 'left'
                    print(222)
                    if not fisright:
                        ag.set_goal(1.7, 3.5, 160)
                        res = ag.send_action()
                        print('r')
                    else:
                        ag.set_goal(1.7, 2.6, 160)
                        res = ag.send_action()
                        print('l')
                    break
            except IndexError:
                continue
                


            # plot bounding box
            tmp_image = copy.deepcopy(self.rgb_image)
            for result in results:
                boxes = result.boxes.cpu().numpy()
                names = result.names

                for xyxy, conf, cls in zip(boxes.xyxy, boxes.conf, boxes.cls):
                    if conf < 0.5:
                        continue
                    x1, y1, x2, y2 = map(int, xyxy[:4])
                    cls_pred = cls
                    tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    tmp_image = cv2.putText(tmp_image, names[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # publish image
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)
            time.sleep(0.1)
        





if __name__ == '__main__':
    od = ObjectDetection()
    try:
        od.process()
    except rospy.ROSInitException:
        pass
    """ ag = ActionGoal()
    if direction == 'right':
        ag.set_goal(1.5, 3.7, 90.0)
    else:
        ag.set_goal(1.5, 2.8, 90.0)
    res = ag.send_action() """



#memo
# right (1.5,3.7)
# left (1.5,2.8)
