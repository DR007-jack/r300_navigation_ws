#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Wait for a LaserScan topic to actually publish messages.

Why:
- Some systems start navigation (AMCL/move_base) before the lidar driver is ready.
- Waiting for a publisher is not enough (a node can advertise but publish later).
- This node blocks until a LaserScan message is received on the given topic.

Contract (private params):
- ~topic (str, default: /scan)
- ~timeout (float seconds, default: 60.0)  # 0 or negative means wait forever

Exit codes:
- 0: got a scan message
- 1: timeout

ROS1 + Python 2.7 compatible.
"""

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('wait_for_scan')

    topic = rospy.get_param('~topic', '/scan')
    timeout = float(rospy.get_param('~timeout', 60.0))

    if timeout <= 0:
        rospy.loginfo('wait_for_scan: waiting for first %s (no timeout)...', topic)
        rospy.wait_for_message(topic, LaserScan)
        rospy.loginfo('wait_for_scan: received first scan on %s', topic)
        rospy.spin()
        return

    rospy.loginfo('wait_for_scan: waiting for first %s (timeout=%.1fs)...', topic, timeout)
    try:
        rospy.wait_for_message(topic, LaserScan, timeout=timeout)
        rospy.loginfo('wait_for_scan: received first scan on %s', topic)
        # Keep node alive so that when launched with required="true",
        # roslaunch won't treat the normal exit as a failure and shutdown everything.
        rospy.spin()
    except rospy.ROSException as e:
        # rospy.wait_for_message raises rospy.ROSException on timeout.
        rospy.logerr('wait_for_scan: timeout waiting for %s: %s', topic, e)
        raise SystemExit(1)
    except rospy.ROSInterruptException:
        raise
    except Exception as e:
        rospy.logerr('wait_for_scan: unexpected error while waiting for %s: %s', topic, e)
        raise SystemExit(2)


if __name__ == '__main__':
    main()

