#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple topic normalizer/bridge for LaserScan topics.

This node subscribes to a list of candidate LaserScan topics and republishes
the messages (unchanged) to a single canonical topic (default: /scan).

Why: this repository contains multiple launch files that expect different
laser topic names (e.g. `/scan`, `unilidar/laserscan`). A small, robust
bridge lets the navigation stack always consume `/scan` without editing
drivers or many launch files.

Compatibility: ROS1 (Melodic), Python 2.7 (use shebang and rospy). Keep code
simple to avoid encoding/format issues.
"""

import rospy
from sensor_msgs.msg import LaserScan


try:
    # Python 2 provides `basestring`; in Python 3 it does not exist.
    # Use eval so static analyzers on Python 3 don't flag NameError.
    _basestring = eval('basestring')
except NameError:
    _basestring = str


class ScanBridge(object):
    def __init__(self):
        # Accept either a YAML list or a space-separated string for backward compatibility
        raw = rospy.get_param('~input_topics', ['/scan', 'unilidar/laserscan', '/unilidar/laserscan'])
        if isinstance(raw, _basestring):
            input_topics = [t for t in raw.split() if t]
        else:
            input_topics = list(raw)

        self.output_topic = rospy.get_param('~output_topic', '/scan')
        queue_size = rospy.get_param('~queue_size', 10)

        # If users include the output topic in inputs (very common when people pass '/scan'),
        # the node will end up subscribing to itself and may never get any real laser data.
        # Filter it out to avoid self-loop.
        input_topics = [t for t in input_topics if t and t != self.output_topic]

        # If none remain, keep at least one non-empty topic to avoid creating a node that does nothing.
        if not input_topics:
            rospy.logwarn('scan_topic_bridge: input_topics only contained output_topic=%s; fallback to unilidar/laserscan', self.output_topic)
            input_topics = ['unilidar/laserscan', '/unilidar/laserscan', 'unitree_lidar/scan']

        rospy.loginfo('scan_topic_bridge: publishing to %s, subscribing to %s', self.output_topic, input_topics)

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=queue_size)

        # create subscribers for all candidate topics; callbacks will publish to output
        self.subs = []
        self.active_src = None
        self.active_src_last_msg = rospy.Time(0)
        self.select_timeout = rospy.Duration(rospy.get_param('~select_timeout', 1.0))
        self.last_debug_log = rospy.Time(0)
        self.debug_log_period = rospy.Duration(rospy.get_param('~debug_log_period', 2.0))
        for t in input_topics:
            try:
                sub = rospy.Subscriber(t, LaserScan, self._cb, callback_args=t, queue_size=queue_size)
                self.subs.append(sub)
            except Exception as e:
                rospy.logwarn('scan_topic_bridge: could not subscribe to %s: %s', t, e)

    def _cb(self, msg, src_topic):
        now = rospy.Time.now()

        # Auto-pick the first topic that actually has messages, then stick to it.
        # If it goes silent for a while, allow switching to another topic.
        if self.active_src is None:
            self.active_src = src_topic
            rospy.loginfo('scan_topic_bridge: selected active input topic: %s', self.active_src)
        else:
            if src_topic != self.active_src:
                # Allow switching only if the current active source looks stale.
                if (now - self.active_src_last_msg) > self.select_timeout:
                    rospy.logwarn('scan_topic_bridge: switching active topic %s -> %s (previous stale for %.3fs)',
                                  self.active_src, src_topic, (now - self.active_src_last_msg).to_sec())
                    self.active_src = src_topic
                else:
                    return

        self.active_src_last_msg = now

        # Forward the message unchanged. We preserve the incoming header/frame_id.
        try:
            self.pub.publish(msg)
        except Exception as e:
            rospy.logerr('scan_topic_bridge: publish failed from %s: %s', src_topic, e)

        # Lightweight periodic debug to confirm data is flowing.
        if (now - self.last_debug_log) >= self.debug_log_period:
            self.last_debug_log = now
            rospy.loginfo('scan_topic_bridge: forwarding from %s, frame_id=%s, stamp=%.3f',
                          self.active_src, msg.header.frame_id, msg.header.stamp.to_sec())


def main():
    rospy.init_node('scan_topic_bridge')
    bridge = ScanBridge()
    rospy.spin()


if __name__ == '__main__':
    main()

