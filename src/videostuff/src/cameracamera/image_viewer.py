#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import time
from scipy import signal 
from sensor_msgs.msg import Image
from std_msgs.msg import *
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

def listener():
    rospy.init_node("ImageRec")
    rospy.on_shutdown(shutdown)
    rospy.Subscriber("/left/image_raw", Image, frontImageEvent)
    rospy.Subscriber("/down/image_raw", Image, downImageEvent)
    rospy.spin()
    
def shutdown():
    rospy.loginfo("ImageRec stopping.")
    rospy.sleep(1)

def Gaussian(sigma):
    # Define unnormalized Gaussian function
    a = math.ceil(3.5 * sigma)
    num_points = 2*a + 1
    g = signal.gaussian(num_points, sigma)
    # Normalize
    g = g / np.sum(g)
    return g


def dGaussian(sigma):
    # Define unnormalized derivative of Gaussian function
    a = math.ceil(3.5 * sigma)
    num_points = 2*a + 1
    x = np.arange(-a, a + 1)
    d = x * signal.gaussian(num_points, sigma)
    # Normalize
    k = 1 / sum(x * d)
    d = -k * d
    return d


def compute_grad(img, sigma=2.5, mode="valid"):
    start_time = time.time()
    g = Gaussian(sigma)
    d = dGaussian(sigma)
    g_v = np.expand_dims(g, axis=1)
    d_h = np.expand_dims(d, axis=0)
    t = time.time()
    temp_h = signal.convolve2d(img, g_v, mode)
    print(time.time() - t)
    grad_h = signal.convolve2d(temp_h, d_h, mode)
    print(time.time() - t)
    return np.abs(grad_h)


def detect_gate(
    img, sigma=10, count_thresh=0.025, num_bins=35, neighborhood=2,
    vote_thresh=1000, cutoff_left=200, cutoff_right=200, cutoff_top=100,
    cutoff_bot=100, downsample_factor=0.5,
):
    bot_bound = img.shape[0] if cutoff_bot == 0 else -cutoff_bot
    right_bound = img.shape[1] if cutoff_right == 0 else -cutoff_right
    img = img[cutoff_top:bot_bound, cutoff_left:right_bound, :]
    img = cv2.resize(img, None, fx=downsample_factor, fy=downsample_factor)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    mag_v = compute_grad(sat, sigma=sigma)

    num_pixels = mag_v.shape[0] * mag_v.shape[1]
    k = int(num_pixels * count_thresh)
    t = np.partition(mag_v.flatten(), -k)[-k]
    _, thresh = cv2.threshold(mag_v, t, 1, cv2.THRESH_BINARY)
    thresh = thresh.astype("uint8")

    height, width = thresh.shape
    w = float(width) / num_bins
    col_sums = np.sum(thresh, axis=0)
    boundaries = np.array([i * w for i in range(num_bins)])
    centers = np.array([w / 2 + i * w for i in range(num_bins)])

    votes = np.zeros(num_bins)
    for col_idx, num_votes in enumerate(col_sums.tolist()):
        idx = int(col_idx // w)
        votes[idx] += num_votes

    peak_1 = np.argmax(votes)
    center_1 = int(centers[peak_1])
    lower_1 = max(int(center_1 - w * neighborhood), 0)
    upper_1 = min(int(center_1 + w * neighborhood), len(col_sums) - 1)
    range_1 = np.arange(lower_1, upper_1 + 1)
    votes_1 = np.sum(range_1 * col_sums[range_1])

    post_loc_1 = None
    post_loc_2 = None
    post_frac_1 = None
    post_frac_2 = None
    if votes_1 >= vote_thresh:
        com_1 = votes_1 / np.sum(col_sums[range_1])
        offset = (img.shape[1] - thresh.shape[1]) / 2
        post_loc_1 = com_1 + offset
        post_frac_1 = post_loc_1 / img.shape[1]

        # Remove votes of first post
        votes_cpy = votes.copy()
        lower = max(peak_1 - neighborhood, 0)
        upper = min(peak_1 + neighborhood, num_bins - 1)
        votes_cpy[lower:(upper+1)] = 0

        peak_2 = np.argmax(votes_cpy)
        center_2 = int(centers[peak_2])
        lower_2 = max(int(center_2 - w * neighborhood), 0)
        upper_2 = min(int(center_2 + w * neighborhood), len(col_sums) - 1)
        range_2 = np.arange(lower_2, upper_2 + 1)
        votes_2 = np.sum(range_2 * col_sums[range_2])

        if votes_2 >= vote_thresh:
            com_2 = votes_2 / np.sum(col_sums[range_2])
            post_loc_2 = com_2 + offset
            post_frac_2 = post_loc_2 / img.shape[1]

    img_cpy = img.copy()
    color = (0, 0, 255)
    thickness = 3
    if post_loc_1 is not None:
        cv2.line(
            img_cpy,
            pt1=(int(post_loc_1), 0),
            pt2=(int(post_loc_1), img.shape[0]),
            color=color,
            thickness=thickness,
        )
    if post_loc_2 is not None:
        cv2.line(
            img_cpy,
            pt1=(int(post_loc_2), 0),
            pt2=(int(post_loc_2), img.shape[0]),
            color=color,
            thickness=thickness,
        )
    #cv2.imshow(".", img_cpy)
    #cv2.waitKey()
    bridge1 = CvBridge()
    img_msg = bridge1.cv2_to_imgmsg(img_cpy, encoding="bgr8")
    imgpub = rospy.Publisher("debugimg", Image, queue_size = 10)
    imgpub.publish(img_msg)
    return (post_frac_1, post_frac_2)

def frontImageEvent(msg):
    frontImageTime = msg.header.stamp
    mat = np.array(msg.data)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    mat = np.asarray(cv_image)
    pub = rospy.Publisher('frontcv', Point, queue_size=10)
    rospy.loginfo("Front image received")
    ret = Point()
    print(mat.shape)
    gate = detect_gate(mat)
    print(gate)
    if gate==None:
        ret.x = -1
        ret.y = -1
        ret.z = 0
    else:
        ret.x = gate[0]
        ret.y = gate[1]
        ret.z = 0
    if ret.x==None:
        ret.x = -1
    if ret.y==None:
        ret.y = -1
    pub.publish(ret)
    
def downImageEvent(msg):
    downImageEvent = msg.header.stamp
    downImageMatrix = msg.data
    rospy.loginfo("Down image received")

if __name__=="__main__":
    print(cv2.__version__)
    listener()
