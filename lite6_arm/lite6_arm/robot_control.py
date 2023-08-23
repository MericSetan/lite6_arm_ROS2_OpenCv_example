#!usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from xarm_msgs.srv import MoveCartesian,MoveJoint,MoveHome,Call,SetInt16,SetInt16ById
from lite6_arm_interface.srv import FindBoxes
from lite6_arm_interface.msg import Box
from xarm_msgs.msg import RobotMsg
import math
import time
""" firstly start robot driver : ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.169 """
""" initialize robot : ros2 run lite6_arm initialize_robot """


home_point = [-1.508487582206726, -0.3773644268512726, 0.41182687878608704,
                0.03171942010521889, 0.7248659133911133, -1.4126126766204834]
point_a = [200.0, 100.0, 95.0, math.pi, 0.0, 1.57]
point_b = [260.0, 100.0, 95.0, math.pi, 0.0, 1.57]
point_c = [310.0, 100.0, 95.0, math.pi, 0.0, 1.57]

point_au = [200.0, 100.0, 200.0, math.pi, 0.0, 1.57]
point_bu = [260.0, 100.0, 200.0, math.pi, 0.0, 1.57]
point_cu = [310.0, 100.0, 200.0, math.pi, 0.0, 1.57]



t_pose=[point_a,point_b,point_c]
t_pose_u=[point_au,point_bu,point_cu]

class RobotControl(Node):
    def __init__(self):
        super().__init__("robot_control")
        self.get_logger().info("robot control  has been started...")
        self.pose = []
        self.enable_motor_client = self.create_client(SetInt16ById,'ufactory/motion_enable')
        self.move_cartesian_client=self.create_client(MoveCartesian,"ufactory/set_position")
        self.move_joint_client=self.create_client(MoveJoint,"ufactory/set_servo_angle")
        self.go_home_client=self.create_client(MoveHome,"ufactory/move_gohome")
        self.clean_error_client=self.create_client(Call,"ufactory/clean_error")
        self.clean_warn_client=self.create_client(Call,"ufactory/clean_warn")
        self.set_mode_client = self.create_client(SetInt16,'ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16,'ufactory/set_state')
        self.open_gripper_client=self.create_client(Call,"ufactory/open_lite6_gripper")
        self.close_gripper_client=self.create_client(Call,"ufactory/close_lite6_gripper")
        self.stop_gripper_client=self.create_client(Call,"ufactory/stop_lite6_gripper")
        self.find_boxes_client=self.create_client(FindBoxes,"find_boxes")
      
     
    def open_gripper(self):

        while not self.open_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/open_lite6_gripper')
        request = Call.Request()

        future = self.open_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True
        
    def close_gripper(self):

        while not self.close_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/close_lite6_gripper')
        request = Call.Request()

        future = self.close_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
        
    def stop_gripper(self):

        while not self.stop_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/stop_lite6_gripper')
        request = Call.Request()

        future = self.stop_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True

    def call_set_mode_client(self,data):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: ufactory/set_mode')

        request = SetInt16.Request()
        request.data = data

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
          
    def call_set_state_client(self,data):
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: ufactory/set_state')

        request = SetInt16.Request()
        request.data = data

        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
        
    def clean_error(self):
        while not self.clean_error_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/clean_error')
        request = Call.Request()

        future = self.clean_error_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
         
    def clean_warn(self):
        while not self.clean_warn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/clean_warn')
        request = Call.Request()

        future = self.clean_warn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True
             
    def go_home(self,speed=15.0,acc=50.0,mvtime=0.0):
        while not self.go_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: /ufactory/move_gohome')

        request=MoveHome.Request()
        request.speed=speed
        request.acc=acc
        request.mvtime=mvtime

        future = self.go_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
        
    def move_cartesian(self,pose,speed=40.0,acc=300.0,mvtime=0.0):
        while not self.move_cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: ufactory/set_position')

        request=MoveCartesian.Request()
        request.pose=pose
        request.speed=speed
        request.acc=acc
        request.mvtime=mvtime
        request.wait = True
        future = self.move_cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            
            return True if response.ret == 0 else False
         
    def move_joint(self,angles,speed=30.0,acc=300.0,mvtime=0.0):
        while not self.move_joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: ufactory/set_servo_angle')

        request=MoveJoint.Request()
        request.angles=angles
        request.speed=speed
        request.acc=acc
        request.mvtime=mvtime
        request.wait = True
        future = self.move_joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
        
    def call_enable_motor_client(self,id,data):
        while not self.enable_motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: ufactory/motion_enable')
            self.get_logger().info('Did you run the driver?')
        request = SetInt16ById.Request()
        request.id = id
        request.data = data

        future = self.enable_motor_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'response.ret : {response.ret}')
            self.get_logger().info(f'response.message : {response.message}')
            return True if response.ret == 0 else False
             
    def find_boxes(self):
        while not self.find_boxes_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Server: find_boxes')
        
        request=FindBoxes.Request()

        future = self.find_boxes_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            #self.get_logger().info(f'response.boxes : {response.boxes}')
            #self.get_logger().info(f'response.pixel_to_ratio : {response.pixel_to_ratio}')
            return response.boxes,response.pixel_to_ratio
    


def main(args = None):
    rclpy.init(args=args)

    node = RobotControl()
    # init robot
    node.call_enable_motor_client(8,1)
    node.call_set_mode_client(0) # 0 for POSITION mode. 1 for SERVOJ mode. 2 for TEACHING_JOINT mode.
    node.call_set_state_client(0)
    node.stop_gripper()

    node.move_joint(home_point)
    boxes,pixel_to_ratio = node.find_boxes()
    target_list = ['red','green','blue']
    target_pose = []
    for i in target_list:
        for n,b in enumerate(boxes):
            if b.color == i:
                target_pose.append((b.x,b.y,b.angle))

    print(target_pose)
    
    for a,i in enumerate(target_pose):
        pose=  [i[0], i[1], 200.0, math.pi, 0.0, math.radians(i[2])]  
        node.move_cartesian(pose)
        node.open_gripper()
        time.sleep(2)
        pose2=  [i[0], i[1], 90.0, math.pi, 0.0, math.radians(i[2])]  
        node.move_cartesian(pose2)
        node.close_gripper()
        time.sleep(2)
        node.move_cartesian(pose)
        node.move_cartesian(t_pose_u[a])
        node.move_cartesian(t_pose[a])
        node.open_gripper()
        time.sleep(1)
        node.move_cartesian(t_pose_u[a])
        node.close_gripper()

    node.stop_gripper()
    node.move_joint(home_point) 
    
if __name__=="__main__":
    main(args=None)



# state of robot:
#	1: RUNNING, executing motion command.
#	2: SLEEPING, not in execution, but ready to move.
#	3: PAUSED, paused in the middle of unfinished motion.
#	4: STOPPED, not ready for any motion commands.
#	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.