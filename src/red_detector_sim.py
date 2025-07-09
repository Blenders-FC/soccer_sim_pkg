#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

# Function to calculate and display FPS
def calculate_fps(prev_frame_time, new_frame_time, frame, position=(7, 70), font_scale=3, color=(100, 255, 0), thickness=3):
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Calculate FPS
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    fps_text = str(int(fps))  # Convert FPS to an integer and then to string
    # Put FPS text on the frame
    cv2.putText(frame, fps_text, position, font, font_scale, color, thickness, cv2.LINE_AA)
    return prev_frame_time

# Pre-calculate zones to exclude from drawing lines
def calculate_nozone(shape, offset=50):
    y, x = shape[:2]
    y_nozone = list(range(y - offset, y + 1)) + list(range(0, offset + 1))
    x_nozone = list(range(x - offset, x + 1)) + list(range(0, offset + 1))
    return y_nozone, x_nozone

# Image callback function to process each frame
def imageCallback(img_msg):
    global prev_frame_time
    try:
        # Convert the ROS image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Start measuring time for each frame
    new_frame_time = time.time()

    # Calculate zones for the current frame
    yimg_nozone, ximg_nozone = calculate_nozone(frame.shape)

    # Convert to HSV for red color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color has two ranges in HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for red
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Optional: morphological operations to clean the mask
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw centers
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10:  # Filter out small detections
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)  # Draw green center
                cv2.drawContours(frame, [cnt], -1, (0, 0, 255), 2)  # Optional: outline red cube

    # Calculate and display FPS
    #prev_frame_time = calculate_fps(prev_frame_time, new_frame_time, overlay_image)
    center_msg = Point()
    center_msg.x = cx
    center_msg.y = cy
    center_msg.z = 0  # Puedes dejarlo en 0 si no usas profundidad

    # Convert the final OpenCV image back to ROS format and publish
    mask_msg = bridge.cv2_to_imgmsg(mask_red, encoding="mono8")
    final_img = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_img.publish(final_img)
    red_loc.publish(center_msg)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('image_processing_node')
    bridge = CvBridge()
    
    # Predefined kernels for morphological operations
    kernel = np.ones((5, 5), np.uint8)
    white_kernel = np.ones((5, 5), np.uint8)
    
    # Variables to track frame time for FPS calculation
    prev_frame_time = 0
    
    # Publishers
    robot_id = rospy.get_param('robot_id', 0)
    pub_img = rospy.Publisher('/robotis_' + str(robot_id) + '/ImgFinal', Image, queue_size=1)

    red_loc = rospy.Publisher('/robotis_'+ str(robot_id)+ '/red_cube_loc', Point, queue_size =1)

    # Subscribers
    rospy.Subscriber("/robotis_op3/camera/image_raw", Image, imageCallback)

    rospy.loginfo("Image processing node started")
    rospy.spin()
