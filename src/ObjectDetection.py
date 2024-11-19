import cv2
import numpy as np

class ObjectDetection:

    def __init__(self, frame) -> None:
        self.frame = frame
        self.center = None
        self._filters_dict = { 'bgr2hsv': cv2.COLOR_BGR2HSV
                             }


    def segmentation(self, filter='bgr2hsv', lower_range=100, upper_range=255):

        if filter in self._filters_dict:
            filter = self._filters_dict[filter]
        else:
            raise Exception("Filter not available yet in flow. To include it please go to ObjectDetection class")
        
        segmented_frame = cv2.cvtColor(self.frame, filter)
        mask = cv2.inRange(segmented_frame, lower_range, upper_range)

        return mask

    def hsv_detection(self, lower_range, upper_range, min_area=10000 erode=None, dilate=None, dilate_first=False):
        mask = None

        hsv_mask = self.segmentation(filter='bgr2hsv', lower_range=lower_range, upper_range=upper_range)

        if erode and dilate and dilate_first:
            mask = cv2.dilate(hsv_mask, np.ones((dilate, dilate), np.uint8))
            mask = cv2.erode(mask, np.ones((erode, erode), np.uint8))
        else:
            if erode:
                mask = cv2.erode(hsv_mask, np.ones((erode, erode), np.uint8))
            if dilate:
                mask = cv2.dilate(mask or hsv_mask, np.ones((dilate, dilate), np.uint8))

        contours, _ = cv2.findContours(mask or hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        
            if area > min_area:
                #cv2.drawContours(video, [approx], 0, (0, 0, 255), 5)
                x,y,w,h = cv2.boundingRect(approx)
                #print(x,y,w,h)
                cv2.rectangle(self.frame, (x,y), (x+w,y+h), color=(255,0,255), thickness=8)
                cv2.circle(self.frame, (int(x+(w/2)), int(y+(h/2))),5, color=(255,255,255), thickness=3)
               
    def binary_detection(self, mask, binary_frame, threshold=125, erode=None, dilate=None):

        #blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)

        _, umbral = cv2.threshold(binary_frame,threshold,255,cv2.THRESH_BINARY)

        #mask = cv2.erode(umbral,np.ones((erode,erode), np.uint8))
        #mask = cv2.dilate(mask, np.ones((dilate,dilate), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def inv_binary_detection(self, mask, binary_frame, threshold=125, erode=None, dilate=None):

        #blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)

        _, umbral = cv2.threshold(binary_frame,threshold,255,cv2.THRESH_BINARY_INV)

        #mask = cv2.erode(umbral,np.ones((erode,erode), np.uint8))
        #mask = cv2.dilate(mask, np.ones((dilate,dilate), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)