#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, LanePose, Twist2DStamped, SegmentList, Segment
import yaml

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, CameraInfo, Image


class MyTestNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyTestNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.pub_segs_img = rospy.Publisher("fakebot/segs_filtered",CompressedImage, queue_size=1,dt_topic_type=TopicType.DEBUG)

        self.sub_segs = rospy.Subscriber("fakebot/ground_projection_node/lineseglist_out", SegmentList, self.cb_segs,queue_size=1)

        self.min_dist = 0.0
        self.max_dist = 0.5

        self.bridge = CvBridge()

        self.debug_img_bg = None

    def cb_segs(self,seglist_msg):

        seg_list = seglist_msg.segments

        seg_list_filtered = []

        seg_list_filtered_msg = SegmentList()
        
        for segment in seg_list:
            ave_x = (segment.points[0].x + segment.points[1].x)/2.0
            ave_y = (segment.points[0].y + segment.points[1].y)/2.0

            dist = np.sqrt(ave_x**2 + ave_y**2)

            if dist > self.min_dist and dist < self.max_dist:

                seg_list_filtered.append(segment)

        seg_list_filtered_msg.segments = seg_list_filtered
        
        if self.pub_segs_img.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.debug_image(seg_list_filtered_msg))
            debug_image_msg.header.stamp = rospy.Time.now()

            self.pub_segs_img.publish(debug_image_msg)
            
        rospy.logwarn(self.pub_segs_img.get_num_connections())


    def run(self):

        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():


            rate.sleep()

    def debug_image(self, seg_list):
        """
        Generates a debug image with all the projected segments plotted with respect to the robot's origin.
        Args:
            seg_list (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane relative to the robot origin
        Returns:
            :obj:`numpy array`: an OpenCV image
        """
        # dimensions of the image are 1m x 1m so, 1px = 2.5mm
        # the origin is at x=200 and y=300

        # if that's the first call, generate the background
        if self.debug_img_bg is None:

            # initialize gray image
            self.debug_img_bg = np.ones((400, 400, 3), np.uint8) * 128

            # draw vertical lines of the grid
            for vline in np.arange(40,361,40):
                cv2.line(self.debug_img_bg,
                         pt1=(vline, 20),
                         pt2=(vline, 300),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "-20cm", (120-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "  0cm", (200-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "+20cm", (280-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw horizontal lines of the grid
            for hline in np.arange(20, 301, 40):
                cv2.line(self.debug_img_bg,
                         pt1=(40, hline),
                         pt2=(360, hline),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "20cm", (2, 220+3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, " 0cm", (2, 300+3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw robot marker at the center
            cv2.line(self.debug_img_bg,
                     pt1=(200 + 0, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 + 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 - 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

        # map segment color variables to BGR colors
        color_map = {Segment.WHITE: (255, 255, 255),
                     Segment.RED: (0, 0, 255),
                     Segment.YELLOW: (0, 255, 255)}

        image = self.debug_img_bg.copy()

        # plot every segment if both ends are in the scope of the image (within 50cm from the origin)
        for segment in seg_list.segments:
            if not np.any(np.abs([segment.points[0].x, segment.points[0].y,
                                  segment.points[1].x, segment.points[1].y]) > 0.50):
                cv2.line(image,
                         pt1=(int(segment.points[0].y * -400) + 200, int(segment.points[0].x * -400) + 300),
                         pt2=(int(segment.points[1].y * -400) + 200, int(segment.points[1].x * -400) + 300),
                         color=color_map.get(segment.color, (0, 0, 0)),
                         thickness=1)

        return image

if __name__ == '__main__':
    # create the node
    node = MyTestNode(node_name='my_test_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()