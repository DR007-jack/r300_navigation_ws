#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Auto-publish an initial pose for AMCL after startup.

Problem:
- On a cold start, AMCL won't publish map->odom until it has an initial pose.
- In practice this means users must click "2D Pose Estimate" in RViz every time.

Solution:
- Wait for required components (map server, /scan data, AMCL services) to be ready.
- Publish one geometry_msgs/PoseWithCovarianceStamped to /initialpose.

ROS1 + Python 2.7 compatible.
"""

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw):
    # planar yaw quaternion
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rospy.init_node('amcl_auto_initialpose')

    topic = rospy.get_param('~topic', '/initialpose')
    frame_id = rospy.get_param('~frame_id', 'map')

    x = float(rospy.get_param('~x', 0.0))
    y = float(rospy.get_param('~y', 0.0))
    yaw = float(rospy.get_param('~yaw', 0.0))

    wait_scan = rospy.get_param('~wait_for_scan', True)
    scan_topic = rospy.get_param('~scan_topic', '/scan')
    wait_timeout = float(rospy.get_param('~wait_timeout', 30.0))

    # covariance diagonal defaults (x, y, yaw)
    cov_x = float(rospy.get_param('~cov_x', 0.25))
    cov_y = float(rospy.get_param('~cov_y', 0.25))
    cov_yaw = float(rospy.get_param('~cov_yaw', 0.0685))

    pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1, latch=True)

    # Wait for AMCL to be alive enough to accept pose and map.
    t0 = rospy.Time.now()

    # Wait for /set_map service (AMCL provides it) so we know AMCL is up.
    try:
        rospy.loginfo('amcl_auto_initialpose: waiting for /set_map service (timeout=%.1fs)', wait_timeout)
        rospy.wait_for_service('/set_map', timeout=wait_timeout)
    except Exception as e:
        rospy.logwarn('amcl_auto_initialpose: /set_map not ready: %s', e)

    if wait_scan:
        # Wait for first scan message to avoid publishing before TF/scan are ready.
        remaining = max(0.1, wait_timeout - (rospy.Time.now() - t0).to_sec())
        try:
            rospy.loginfo('amcl_auto_initialpose: waiting for first %s (timeout=%.1fs)', scan_topic, remaining)
            rospy.wait_for_message(scan_topic, rospy.AnyMsg, timeout=remaining)
        except Exception as e:
            rospy.logwarn('amcl_auto_initialpose: did not receive %s within timeout: %s', scan_topic, e)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # 6x6 covariance, row-major
    cov = [0.0] * 36
    cov[0] = cov_x
    cov[7] = cov_y
    cov[35] = cov_yaw
    msg.pose.covariance = cov

    rospy.logwarn('amcl_auto_initialpose: publishing /initialpose (frame=%s) x=%.3f y=%.3f yaw=%.3f rad',
                  frame_id, x, y, yaw)

    # Publish a few times to increase chance of delivery.
    for _ in range(3):
        pub.publish(msg)
        rospy.sleep(0.2)

    rospy.loginfo('amcl_auto_initialpose: done')
    rospy.spin()


if __name__ == '__main__':
    main()

