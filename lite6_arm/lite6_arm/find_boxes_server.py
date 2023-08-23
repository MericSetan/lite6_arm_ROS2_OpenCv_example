#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from lite6_arm_interface.srv import FindBoxes
from lite6_arm_interface.msg import Box
from find_box_ip import FindBoxIp

""" 
#Request

---
#Response
Box[] boxes
	string color
	int64 p_x
	int64 p_y
	float64 angle
	float64 width
	float64 height
float64 pixel_to_ratio
"""

class FindBoxesServer(Node):
    def __init__(self):
        super().__init__("find_boxes_server")
        self.get_logger().info("Find Boxes Server has been started...")
        self.server = self.create_service(FindBoxes,"find_boxes",self.callback)
        self.fb_obj = FindBoxIp()

    def callback(self,request,response):

        response = FindBoxes.Response()
        
        boxes,pixel_to_ratio = self.fb_obj.get_box()
        response.boxes = self.create_box_msg(boxes)   
        response.pixel_to_ratio = pixel_to_ratio

        return response
    

    def create_box_msg(self,box_list):
        box_msg = []
        for i,b in enumerate(box_list):
            a_box = Box()
            
            a_box.color = b['color']
            a_box.x = b['p_x']
            a_box.y = b['p_y']
            a_box.angle = b['angle']
            a_box.width = b['width']
            a_box.height = b['height']
            box_msg.append(a_box)
        return box_msg      

def main(args = None):
    rclpy.init(args=args)
    node = FindBoxesServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()