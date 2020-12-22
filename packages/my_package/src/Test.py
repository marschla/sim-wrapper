#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, LanePose, Twist2DStamped
import yaml

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, CameraInfo

import gym_duckietown
from gym_duckietown.simulator import Simulator

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.pub_img = rospy.Publisher("fakebot/camera_node/image/compressed",CompressedImage, queue_size=1)

        self.pub_camera_info = rospy.Publisher("fakebot/camera_node/camera_info",CameraInfo,queue_size=1)

        #self.pub_lane_pose = rospy.Publisher("fakebot/lane_filter_node/lane_pose",LanePose,queue_size=1)
        #self.pub_lane_pose = rospy.Publisher("fakebot/sim_node/debug_lanepose",LanePose,queue_size=1)

        #self.sub = rospy.Subscriber("fakebot/sim_node/actuator_cmd", WheelsCmdStamped, self.callback)
        self.sub = rospy.Subscriber("fakebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)
        self.sub = rospy.Subscriber("fakebot/lane_filter_node/lane_pose", LanePose, self.callback_pose)


        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'

        self.v_left = 0
        self.v_right = 0

    def callback(self,msg):
        self.v_left = msg.vel_left
        self.v_right = msg.vel_right

    def callback_pose(self,msg):
        pose = [msg.d, msg.phi]
        rospy.loginfo("pose estimate = %s" % pose)
        

    def run(self):

        env = Simulator(
            seed=123, # random seed
            map_name="small_loop",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4, # start close to straight
            full_transparency=True,
            distortion=False,
        )

        self.bridge = CvBridge()

        image_msg = CompressedImage()

        cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        filename = cali_file_folder + rospy.get_namespace().strip("/") + "default.yaml"

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

        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():
            
            action = [self.v_left,self.v_right]
            #rospy.loginfo(action)
            observation, reward, done, misc = env.step(action)
            env.render()

            _observation = cv2.cvtColor(observation, cv2.COLOR_BGR2RGB)
            
            cmprsmsg = self.bridge.cv2_to_compressed_imgmsg(_observation)

            #print(np.shape(observation))
            
            image_msg = cmprsmsg

            image_msg.format = "jpeg"
            stamp = rospy.Time.now()
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id

            #print(len(image_msg.format))
            #print(len(image_msg.data))
            #print(len(image_msg.header.frame_id))
            #print("--------------------------------------------------")

            self.pub_img.publish(image_msg)

            cam_info.header.stamp = stamp

            self.pub_camera_info.publish(cam_info)
            
            
            lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
            #print(lane_pose)

            pose = [lane_pose.dist,lane_pose.angle_rad]
            rospy.loginfo("real pose = %s" % pose)

            '''
            pose_msg = LanePose()
            pose_msg.d = lane_pose.dist
            pose_msg.phi = lane_pose.angle_rad
            pose_msg.header.stamp = rospy.Time.now()
            self.pub_lane_pose.publish(pose_msg)
            '''

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