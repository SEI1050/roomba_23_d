#!/usr/bin/env python3

import copy
from typing import List

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
from ultralytics import YOLO
from ultralytics.engine.results import Results

first = [-1, 10 ** 9, -1, 10 ** 9]
second = [-1, 10 ** 9, -1, 10 ** 9]
fisright = False
class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)

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

            results: List[Results] = self.model.predict(self.rgb_image)
            keypoint = results[0].keypoints
            #print(keypoint.data[:, 5:11])
            first[0] = max(first[0], keypoint.data[0][9][0])
            first[1] = min(first[1], keypoint.data[0][9][0])
            first[2] = max(first[2], keypoint.data[0][10][0])
            first[3] = min(first[3], keypoint.data[0][10][0])
            second[0] = max(second[0], keypoint.data[1][9][0])
            second[1] = min(second[1], keypoint.data[1][9][0])
            second[2] = max(second[2], keypoint.data[1][10][0])
            second[3] = min(second[3], keypoint.data[1][10][0])

            fdif = max(first[0] - first[1], first[2] - first[3])
            sdif = max(second[0] - second[1], second[2] - second[3])

            if keypoint.data[0][9][0] > keypoint.data[1][9][0]:
                fisright = True

            t = 5
            if fdif > t:
                print('right' if fisright else 'left')
                exit()
            if sdif > t:
                print('right' if not fisright else 'left')
                exit()
            


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
