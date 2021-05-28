#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import math
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge
import statistics as st
import numpy as np


class entropy_calc_topic():

    def __init__(self):
        print("INITIALISING NODE -> /entropy_node")
        rospy.init_node('entropy_node')

        self.img_path = "/home/parischatz/entropy_ws/src/hrt_entropy/behaviour_detection_package/images/"

        self.error_list_ang = []
        self.error_list_lin = []
        self.estimation_errors_ang = []
        self.estimation_errors_lin = []
        self.error_history = []

        self.a_ang = 0.14
        self.a_lin = 0.08

        self.bins_ang = [-5*self.a_ang, -2.5*self.a_ang, -self.a_ang, -0.5 *
                         self.a_ang,  0.5*self.a_ang, self.a_ang, 2.5*self.a_ang, 5*self.a_ang]
        self.bins_lin = [-5*self.a_lin, -2.5*self.a_lin, -self.a_lin, -0.5 *
                         self.a_lin,  0.5*self.a_lin, self.a_lin, 2.5*self.a_lin, 5*self.a_lin]

        self.entropy_thresh = 0.5
        self.entropy_stdev = 0.05

        self.entropy = 0.0
        self.entropy_history = []
        self.entropy_average = 0.0
        self.entropy_sum = 0.0
        self.entropy_average_counter = 0.0

        self.image_pub = rospy.Publisher('img_rviz', Image, queue_size=10)

        self.sub_ang_error = rospy.Subscriber(
            "estimation_error_ang", Float32, self.estimation_error_ang_callback)

        self.sub_lin_error = rospy.Subscriber(
            "estimation_error_lin", Float32, self.estimation_error_lin_callback)

        self.pup_entropy = rospy.Publisher(
            "entropy", Float32, queue_size=1)

        self.pub_alpha_ang = rospy.Publisher(
            "alpha_ang", Float32, queue_size=1)

    def estimation_error_ang_callback(self, data):
        self.error_list_ang.append(data.data)
        # error hist for CDU
        if len(self.estimation_errors_ang) > 100:
            self.estimation_errors_ang.pop(0)
        self.estimation_errors_ang.append(data.data)

    def estimation_error_lin_callback(self, data):
        self.error_list_lin.append(data.data)
        # error hist for CDU
        if len(self.estimation_errors_lin) > 100:
            self.estimation_errors_lin.pop(0)
        self.estimation_errors_lin.append(data.data)

    def calculate_error_frequencies(self, error_list, bins):
        """
        This function calculates the frequencies of errors, by counting
        the errors that fall in the according bin (temp) and summing (souma)
        each bin to get the frequency of errors (frequencies) for each bin

        Args:
            error_list ([float]): list of errors that are used to
            calculate entropy
            bins ([float]): bins of according measurement

        Returns:
            [float],[float],[float]: [proportion of errors]
            ,[sum of errors],[error frequencies]
        """
        p = [0]*9
        for val in error_list:
            # TODO fix this, it's ugly
            if val <= bins[0]:
                p[0] += 1
            elif val > bins[0] and val <= bins[1]:
                p[1] += 1
            elif val > bins[1] and val <= bins[2]:
                p[2] += 1
            elif val > bins[2] and val <= bins[3]:
                p[3] += 1
            elif val > bins[3] and val <= bins[4]:
                p[4] += 1
            elif val > bins[4] and val <= bins[5]:
                p[5] += 1
            elif val > bins[5] and val <= bins[6]:
                p[6] += 1
            elif val > bins[6] and val <= bins[7]:
                p[7] += 1
            elif val >= bins[7]:
                p[8] += 1

        temp = p
        souma = float(sum(temp))
        frequencies = [x / souma for x in temp if souma != 0]
        return p, souma, frequencies

    def calculate_entropy(self, event=None):
        _, _, frequencies_ang = self.calculate_error_frequencies(
            self.error_list_ang, self.bins_ang)
        entropy_ang = sum([-val*math.log(val, 9)
                           for val in frequencies_ang if val != 0])

        _, _, frequencies_lin = self.calculate_error_frequencies(
            self.error_list_lin, self.bins_lin)
        entropy_lin = sum([-val*math.log(val, 9)
                           for val in frequencies_lin if val != 0])

        self.entropy = 1.0*entropy_ang + 0.0*entropy_lin
        self.pup_entropy.publish(self.entropy)

        if len(self.entropy_history) > 100:
            self.entropy_history.pop(0)

        self.entropy_history.append(self.entropy)

        self.error_list_ang = []
        self.error_list_lin = []
        print("Entropy angular:{}, Entropy linear:{}, Total Entropy:{}".format(
            entropy_ang, entropy_lin, self.entropy))

        # stuff for total average entropy
        self.entropy_sum += self.entropy
        self.entropy_average_counter += 1
        self.entropy_average = self.entropy_sum/self.entropy_average_counter

    def dcu_node(self, event=None):
        if len(self.entropy_history) >= 100:
            if st.mean(self.entropy_history[-100:]) > 0 and st.mean(self.entropy_history[-100:]) < self.entropy_thresh:
                if st.stdev(self.entropy_history[-100:]) < self.entropy_stdev:
                    self.error_history = self.error_history + \
                        self.estimation_errors_ang[-100:]
                    self.entropy_stdev = st.stdev(self.entropy_history[-100:])
                    self.entropy_thresh = st.mean(self.entropy_history[-100:])
                    print(np.percentile(self.entropy_history, 90))
                    self.pub_alpha_ang.publish(
                        np.percentile(self.entropy_history, 90))

    def wais(self, event=None):
        """Function that checks entropy and publishes an image for Rviz

        Args:
            event ([type], optional): [description]. Defaults to None.
        """
        if self.entropy > self.entropy_thresh:
            img_path = self.img_path+"warning.png"
        else:
            img_path = self.img_path+"smooth.png"
        if os.path.isfile(img_path):
            img = cv2.imread(img_path)
            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            self.image_pub.publish(img_msg)
        if self.entropy > self.entropy_thresh:
            # wait only if warning
            rospy.sleep(3)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    duration_of_entropy_calc = 2.5
    try:
        node = entropy_calc_topic()
        # The rospy.Timer is used for ansychronous execution
        rospy.Timer(rospy.Duration(1.0 / (1/duration_of_entropy_calc)),
                    node.calculate_entropy)
        # rospy.Timer(rospy.Duration(1.0/1.0), node.wais)
        # rospy.Timer(rospy.Duration(15.0/1.0), node.dcu_node)
        node.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        print("Total average entropy: {}".format(node.entropy_average))
