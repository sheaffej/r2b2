#!/usr/bin/env python3
import numpy as np
import sys
import threading
import traceback
from typing import List, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D


class PARAM_NAMES:
    LOG_LEVEL = 'log_level'
    LOOP_HZ = 'loop_hz'
    RGB_IMAGE_TOPIC = 'rgb_image_topic'
    ANNOTATED_IMAGE_TOPIC = 'anno_image_topic'
    OBJECT_BBOXES_TOPIC = 'object_bboxes_topic'


class PARAM_DEFAULTS:
    NODE_NAME = "object_detector_node"
    LOG_LEVEL = "info"
    LOOP_HZ = 4
    RGB_IMAGE_TOPIC = "/base/camera/color/image_raw"
    ANNOTATED_IMAGE_TOPIC = "/object/detection/image_annotated"
    OBJECT_BBOXES_TOPIC = "/object/detection/bboxes"


def detect_objects(img: np.ndarray) -> List[Tuple[int, int]]:

    # reading image
    # img = cv2.imread('/home/jsheaffe/r2b2_project/r2b2-obj-detect/samples/gz_ball.jpg')

    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # using a findContours() function
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # for contour in contours:
    #     print(contour.shape)

    i = 0
    bboxes = []
    for contour in contours:

        # here we are ignoring first counter because
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

        x, y, w, h = cv2.boundingRect(approx)

        p1 = (x, y)
        p2 = (x + w, y + h)
        bboxes.append((p1, p2))
    return bboxes


class ObjectDetector(Node):
    def __init__(self):
        super().__init__(PARAM_DEFAULTS.NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                (PARAM_NAMES.LOG_LEVEL, PARAM_DEFAULTS.LOG_LEVEL, ParameterDescriptor(description='Log verbosity, defaults to `info`')),
                (PARAM_NAMES.LOOP_HZ, PARAM_DEFAULTS.LOOP_HZ, ParameterDescriptor(description='Frequency of the main logic loop')),
                (PARAM_NAMES.RGB_IMAGE_TOPIC, PARAM_DEFAULTS.RGB_IMAGE_TOPIC, ParameterDescriptor(description='Topic to listen for RGB images')),
                (PARAM_NAMES.ANNOTATED_IMAGE_TOPIC, PARAM_DEFAULTS.ANNOTATED_IMAGE_TOPIC, ParameterDescriptor(description='Topic to publish annotated RGB images')),
                (PARAM_NAMES.OBJECT_BBOXES_TOPIC, PARAM_DEFAULTS.OBJECT_BBOXES_TOPIC, ParameterDescriptor(description='Topic to publish bounding boxes of detected objects')),
            ]
        )

        self.rgb_img_msg: Image = None
        self.cv_bridge = CvBridge()
        self.lock = threading.RLock()

        cb_group_subscriptions = ReentrantCallbackGroup()
        cb_group_timer = MutuallyExclusiveCallbackGroup()

        # Publishes
        self._pub_image_annotated = self.create_publisher(
            msg_type=Image,
            topic=self.get_parameter(PARAM_NAMES.ANNOTATED_IMAGE_TOPIC).value,
            qos_profile=1
        )
        self._pub_object_bboxes = self.create_publisher(
            msg_type=BoundingBox2DArray,
            topic=self.get_parameter(PARAM_NAMES.OBJECT_BBOXES_TOPIC).value,
            qos_profile=1
        )

        # Subscribes
        self.create_subscription(
            msg_type=Image,
            topic=self.get_parameter(PARAM_NAMES.RGB_IMAGE_TOPIC).value,
            callback=self._rgb_image_callback,
            callback_group=cb_group_subscriptions,
            qos_profile=1
        )

        # Main loop timer
        loop_secs = 1.0 / self.get_parameter(PARAM_NAMES.LOOP_HZ).value
        self.create_timer(
            timer_period_sec=loop_secs,
            callback=self._base_loop_callback,
            callback_group=cb_group_timer
        )

    def _rgb_image_callback(self, msg):
        with self.lock:
            self.rgb_img_msg = msg
        # print('rgb_image_callback')

    def _base_loop_callback(self):
        # print('base_loop_callback')
        with self.lock:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.rgb_img_msg, self.rgb_img_msg.encoding)

        bboxes = detect_objects(cv_image)
        # print(bboxes)
        bboxes_msg = BoundingBox2DArray()

        for p1, p2 in bboxes:
            # print('bbox')
            x1, y1 = p1
            x2, y2 = p2

            # Annotate the image
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Populate the BoundingBox2DArray
            abox_msg = BoundingBox2D()
            # print(dir(abox_msg))
            abox_msg.center.position.x = float(((x2 - x1) / 2) + x1)
            abox_msg.center.position.y = float(((y2 - y1) / 2) + y1)
            abox_msg.size_x = float(x2 - x1)
            abox_msg.size_y = float(y2 - y1)
            # print('hello')
            bboxes_msg.boxes.append(abox_msg)

        anno_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self._pub_image_annotated.publish(anno_msg)
        # print('Published annotated image')

        self._pub_object_bboxes.publish(bboxes_msg)
        # print('Published bboxes')


def main(args=None):

    # Setup the ROS node
    rclpy.init(args=args)
    node = ObjectDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().fatal("Unhandled exeption...printing stack trace then shutting down node")
        node.get_logger().fatal(traceback.format_exc())

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(args=sys.argv)
