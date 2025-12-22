#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Periodic /request_nomotion_update caller for AMCL.

Why:
- AMCL may publish /amcl_pose and map->odom only when motion/update events happen.
- For debugging and some deployments we want a steady pose update even when the robot is stationary.

This node calls the /request_nomotion_update service at a configurable rate.

ROS1 + Python 2.7 compatible.
"""

import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node('amcl_nomotion_update')

    service_name = rospy.get_param('~service', '/request_nomotion_update')
    rate_hz = float(rospy.get_param('~rate', 1.0))
    wait_timeout = float(rospy.get_param('~wait_timeout', 30.0))

    if rate_hz <= 0.0:
        rospy.logwarn('amcl_nomotion_update: rate<=0 (%.3f), node will idle', rate_hz)
        rospy.spin()
        return

    rospy.loginfo('amcl_nomotion_update: waiting for %s (timeout=%.1fs)', service_name, wait_timeout)
    try:
        rospy.wait_for_service(service_name, timeout=wait_timeout)
    except Exception as e:
        rospy.logerr('amcl_nomotion_update: service %s not available: %s', service_name, e)
        rospy.spin()
        return

    # AMCL provides /request_nomotion_update as std_srvs/Empty
    proxy = rospy.ServiceProxy(service_name, Empty)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo('amcl_nomotion_update: calling %s at %.2f Hz', service_name, rate_hz)

    while not rospy.is_shutdown():
        try:
            proxy()
        except Exception as e:
            rospy.logwarn_throttle(5.0, 'amcl_nomotion_update: call failed: %s', e)
        rate.sleep()


if __name__ == '__main__':
    main()

