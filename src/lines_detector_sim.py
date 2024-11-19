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

    # Blur and convert to grayscale
    blurred = cv2.GaussianBlur(frame, (9, 9), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

    # Thresholding for white mask
    _, white_msk = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    white_msk = cv2.dilate(white_msk, white_kernel, iterations=2)

    # Convert to LAB color space to detect field lines
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lower_green = np.array([0, 0, 100])  # Lower bound for green
    upper_green = np.array([185, 125, 180])  # Upper bound for green
    mask_green = cv2.inRange(lab_frame, lower_green, upper_green)

    # Morphological transformations to clean up the mask
    opening = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel, iterations=1)

    # Sure background
    sure_bg = cv2.erode(opening, kernel, iterations=2)
    sure_bg = cv2.dilate(sure_bg, kernel, iterations=2)

    # Adaptive thresholding to detect contours
    thresh = cv2.adaptiveThreshold(sure_bg, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, blockSize=5, C=0)

    # Find contours of the field markings
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw only large contours
    dummy_mask = np.zeros_like(mask_green)
    large_contours = [c for c in contours if cv2.contourArea(c) > 1000]
    cv2.drawContours(dummy_mask, large_contours, -1, (255, 255, 255), 1)

    # Detect lines using Hough transform
    lines = cv2.HoughLinesP(dummy_mask, rho=2, theta=np.pi / 180, threshold=250,
                            minLineLength=25, maxLineGap=50)

    # Create a copy to draw lines on
    image_lines_thrs = frame.copy()
    lines_thrs = np.zeros_like(dummy_mask)

    # Draw detected lines while avoiding no-line zones
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (y1 not in yimg_nozone or y2 not in yimg_nozone) and (x1 not in ximg_nozone or x2 not in ximg_nozone):
                #cv2.line(image_lines_thrs, (x1, y1), (x2, y2), (255, 0, 0), 10)  # Blue lines
                cv2.line(lines_thrs, (x1, y1), (x2, y2), (255, 0, 0), 10)

    # Mask lines with white areas
    lines_thrs = cv2.bitwise_and(lines_thrs, white_msk)

    # Create a colored mask for detected lines
    colored_mask = np.zeros_like(frame)
    colored_mask[lines_thrs == 255] = [255, 0, 0]  # Blue lines

    # Overlay the colored mask on the original frame
    overlay_image = cv2.addWeighted(frame, 1, colored_mask, 1, 0)

    # Calculate and display FPS
    prev_frame_time = calculate_fps(prev_frame_time, new_frame_time, overlay_image)

    # Convert the final OpenCV image back to ROS format and publish
    final_img = bridge.cv2_to_imgmsg(overlay_image, "bgr8")
    pub_img.publish(final_img)

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

    # Subscribers
    rospy.Subscriber("/robotis_op3/camera/image_raw", Image, imageCallback)

    rospy.loginfo("Image processing node started")
    rospy.spin()
