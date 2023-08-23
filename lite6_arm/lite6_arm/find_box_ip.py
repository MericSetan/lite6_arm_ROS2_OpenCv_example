#!usr/bin/env python3
import cv2
import numpy as np
import math
from collections import defaultdict
# Aruco
class HomogeneousBgDetector():
    def __init__(self):
        pass

    def detect_objects(self, frame):
        # Convert Image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Create a Mask with adaptive threshold
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 19, 5)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # cv2.imshow("mask", mask)
        objects_contours = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:
                #cnt = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
                objects_contours.append(cnt)

        return objects_contours
    


class FindBoxIp():
    def __init__(self):
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.detector = HomogeneousBgDetector()
        self.boxes=[]
        
    def find_angle(self,px1, py1, px2, py2):
        h = py2 - py1
        w = px2 - px1
        tan = math.atan(h/w)
        tan_deg = math.degrees(tan)
        if tan_deg < 0:
            tan_deg = -tan_deg
        return tan_deg
    
    def oklid(self,x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def map_value(self,value, in_min, in_max, out_min, out_max):
        in_range = in_max - in_min
        out_range = out_max - out_min
        value_scaled = (value - in_min) / in_range
        mapped_value = out_min + (value_scaled * out_range)
        return mapped_value

    def image_to_robot_coordinates(self,h,w,x, y,pixel_cm_ratio):
        if x >=w/2:
            new_y = self.map_value(x,w/2,w,0,((w/2)/pixel_cm_ratio))*10-1
            new_x = (y/pixel_cm_ratio)*10 +85
        elif x <w/2:
            new_y = -self.map_value(x,0,w/2,((w/2)/pixel_cm_ratio),0)*10-1
            new_x = (y/pixel_cm_ratio)*10 +85
        return float(round(new_x)), float(round(new_y))
    
    def get_box(self):

        cap = cv2.VideoCapture(2)    
        _, img = cap.read()

        corners, _, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.parameters)   
        if corners:
            
            aruco_perimeter = cv2.arcLength(corners[0], True)
            pixel_cm_ratio = aruco_perimeter / 20

            contours = self.detector.detect_objects(img)
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower_green = (38, 100, 100)
            upper_green = (75, 255, 255)
            lower_blue = (100, 60, 85)
            upper_blue =(255, 255, 165)
            upper_red = (200,250,250)
            lower_red =(0,170,0)
                    
            red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
            green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
            blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
                    
            masks = [red_mask, green_mask, blue_mask]
            color_names = ['red', 'green', 'blue']
            colors =  [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

            for mask, color, color_name in zip(masks, colors, color_names):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    if cv2.contourArea(contour) > 300:  
                        rect = cv2.minAreaRect(contour) 
                        (x, y), (w, h), angle = rect

                        # Nesne genisligi ve yuksekligini cm cinsinden hesaplama
                        object_width = w / pixel_cm_ratio
                        object_height = h / pixel_cm_ratio
                            
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                            
                        for i in box:
                            cv2.circle(img, (i[0], i[1]), 5, color, -1)
                            
                            x1, y1 = box[0]   
                            x2, y2 = box[1]                 
                            x3, y3 = box[2]             
                            x4, y4 = box[3]
                
                            """ 
                            cv2.putText(img, ("0"),(int(x1-25),int(y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
                            cv2.putText(img, ("1"),(int(x2-25),int(y2-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
                            cv2.putText(img, ("2"),(int(x3-25),int(y3-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
                            cv2.putText(img, ("3"),(int(x4-25),int(y4-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
                            
                            cv2.circle(img, (x1, y1), 8, (0,255,255), -1) """
                        
                            
                            aci1 = self.find_angle(x2, y2, x1, y1)
                            aci2 = self.find_angle(x4, y4,  x3, y3)
                            average_angle = aci1 + aci2 / 2
                            average_angle = average_angle
                            
                            
                            center_x = int(rect[0][0])
                            center_y = int(rect[0][1])

                            oklid1 = self.oklid(x4,y4,x3,y3) # 3-2
                            oklid2 = self.oklid(x4,y4,x1,y1)  # 0-3

                            if oklid1 < oklid2 : 
                                direction = "clockwise" # cw
                                rotatiton_angle = aci1                        
                            else:
                                direction = "counterclockwise" # counter cw
                                rotatiton_angle = -(90 - aci1) 

                            h,w,_ = img.shape
                            c_x,c_y = self.image_to_robot_coordinates(h,w,center_x,center_y,pixel_cm_ratio)
                            a_box = {
                                'color': color_name,
                                'p_x': c_x,
                                'p_y': c_y,
                                'angle': rotatiton_angle,
                                'width': object_width,
                                'height': object_height, 
                            }

                            self.boxes.append(a_box)
                            
            grouped_data = defaultdict(list)
            for item in self.boxes:
                color = item['color']
                grouped_data[color].append(item)

            unique_data = []

            for color, group in grouped_data.items():
                unique_data.append(group[0])
            return unique_data,pixel_cm_ratio                    
                        
                        
                            
