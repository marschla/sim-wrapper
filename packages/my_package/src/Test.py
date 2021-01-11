#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, LanePose, Twist2DStamped, Pose2DStamped
import yaml

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, CameraInfo, Image

import gym_duckietown
from gym_duckietown.simulator import Simulator

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        #setting up ros publisher
        #Publisher for simulated camera image
        self.pub_img = rospy.Publisher("fakebot/camera_node/image/compressed",CompressedImage, queue_size=1)
        #Publisher for camera info
        self.pub_camera_info = rospy.Publisher("fakebot/camera_node/camera_info",CameraInfo,queue_size=1)
        #Publisher for global pose of Duckiebot
        self.pub_global_pose = rospy.Publisher("fakebot/sim_node/global_pose",Pose2DStamped,queue_size=1)
        #Publisher of real lane pose to bypass malfunctioning lane filter
        self.pub_lane_pose = rospy.Publisher("fakebot/sim_node/lane_pose",LanePose,queue_size=1)

        #setting up ros Subscriber
        #Subscriber to receive wheel commands from the controller / dt-core
        self.sub = rospy.Subscriber("fakebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback,queue_size=1)
        #Subscriber to receive estimated lane pose from the lane filter for debug purposes
        #self.sub = rospy.Subscriber("fakebot/lane_filter_node/lane_pose", LanePose, self.callback_pose,queue_size=1)


        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'

        self.v_left = 0
        self.v_right = 0

        self.pose_est = [0,0]

    #stores received wheel commands to be used for simulator input
    def callback(self,msg):
        self.v_left = msg.vel_left
        self.v_right = msg.vel_right

    #prints estimated lane pose for debug purposes
    def callback_pose(self,msg):
        self.pose_est = [msg.d, msg.phi]
        rospy.loginfo("pose estimate = %s" % self.pose_est)
        

    def run(self):
        
        #setting up simulator environment
        env = Simulator(
            seed=123, # random seed
            map_name="marco_1",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=3, # start close to straight
            full_transparency=True,
            distortion=True,
        )

        self.bridge = CvBridge()

        image_msg = CompressedImage()

        #calibration file to get camera info parameters 
        cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        filename = cali_file_folder + rospy.get_namespace().strip("/") + "default.yaml"

        #loading camera info and storing it in a sensor_msg
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)

        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']

        #main loop that runs the simulator
        rate = rospy.Rate(20) # 1Hz
        while not rospy.is_shutdown():
            
            #feed actuator commands to simulator and simulate for one step -> receive camera image
            action = [self.v_left,self.v_right]
            #rospy.loginfo(action)
            observation, reward, done, misc = env.step(action)
            env.render()

            #converting image to correct representation
            _observation = cv2.cvtColor(observation, cv2.COLOR_BGR2RGB)

            #full_img_msg = self.bridge.cv2_to_imgmsg(observation, encoding="rgb8")
            #self.pub_full_img.publish(full_img_msg)
            
            #converting cv2 image to ros msg
            cmprsmsg = self.bridge.cv2_to_compressed_imgmsg(_observation)

            #setting up img msg, then publish it to the relevant nodes
            image_msg = cmprsmsg
            image_msg.format = "jpeg"
            stamp = rospy.Time.now()
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id

            self.pub_img.publish(image_msg)

            #publishing cam info msg 
            cam_info.header.stamp = stamp
            self.pub_camera_info.publish(cam_info)
            
            #create msg with global position of DB, so position on the map, then publish it, for evaluation purposes
            global_pose_msg = Pose2DStamped()
            global_pose_msg.header.stamp = rospy.Time.now()
            global_pose_msg.x = env.cur_pos[0]
            global_pose_msg.y = env.cur_pos[2]
            global_pose_msg.theta = env.cur_angle

            self.pub_global_pose.publish(global_pose_msg)
                       
            lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)          
            pose_msg = LanePose()
            pose_msg.d = lane_pose.dist
            pose_msg.phi = lane_pose.angle_rad
            pose_msg.header.stamp = rospy.Time.now()
            self.pub_lane_pose.publish(pose_msg)
            

            if done:
                env.reset()

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()