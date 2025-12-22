#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Wait for required nav prerequisites then exec move_base.

Why:
- `move_base` can exit early (sometimes without useful logs) if the ROS graph or TF isn't ready.
- A fixed `sleep` is brittle. This wrapper waits for the things move_base/costmaps really need.

Contract:
- Inputs (ROS params, private):
  - ~scan_topic (str, default: /scan)
  - ~wait_for_scan (bool, default: true)
  - ~wait_for_scan_timeout (float sec, default: 30)
  - ~wait_for_tf (bool, default: true)
  - ~tf_pairs (str, default: "map:odom odom:base_link base_link:unilidar_lidar")
  - ~wait_for_tf_timeout (float sec, default: 30)
  - ~extra_args (str, default: "")  # appended to move_base args
- Behavior:
  - Blocks until prerequisites are satisfied or timeouts reached.
  - Then replaces itself with move_base binary (os.execvp).
"""

from __future__ import print_function

import os
import shlex
import sys
import time

try:
    # Python 2
    from pipes import quote as _shell_quote
except Exception:
    # Python 3
    from shlex import quote as _shell_quote

import rospy


def _parse_tf_pairs(pairs_str):
    pairs = []
    for tok in pairs_str.split():
        if ':' not in tok:
            continue
        parent, child = tok.split(':', 1)
        parent = parent.strip()
        child = child.strip()
        if parent and child:
            pairs.append((parent, child))
    return pairs


def _wait_for_topic(topic, timeout_s):
    start = time.time()
    while not rospy.is_shutdown():
        published = False
        try:
            # any topic with at least one publisher is good enough
            for name, pubs in rospy.get_published_topics():
                if name == topic:
                    published = True
                    break
        except Exception:
            # Master might not be ready yet; keep waiting.
            published = False

        if published:
            return True

        if time.time() - start > timeout_s:
            return False

        rospy.sleep(0.2)

    return False


def _wait_for_tf_pairs(pairs, timeout_s):
    import tf

    listener = tf.TransformListener()
    start = time.time()

    # give TF listener some time to fill its buffer
    rospy.sleep(0.5)

    remaining = set(pairs)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        ok = []
        for parent, child in list(remaining):
            try:
                if listener.canTransform(parent, child, now):
                    ok.append((parent, child))
            except Exception:
                pass
        for item in ok:
            remaining.discard(item)

        if not remaining:
            return True

        if time.time() - start > timeout_s:
            rospy.logwarn("TF wait timeout, still missing: %s", " ".join(["%s->%s" % (p, c) for p, c in sorted(remaining)]))
            return False

        rospy.sleep(0.2)

    return False


def main():
    rospy.init_node('move_base_wait_and_run', anonymous=False)

    scan_topic = rospy.get_param('~scan_topic', '/scan')
    wait_for_scan = rospy.get_param('~wait_for_scan', True)
    wait_for_scan_timeout = float(rospy.get_param('~wait_for_scan_timeout', 30.0))

    wait_for_tf = rospy.get_param('~wait_for_tf', True)
    tf_pairs_str = rospy.get_param('~tf_pairs', 'map:odom odom:base_link base_link:unilidar_lidar')
    tf_pairs = _parse_tf_pairs(tf_pairs_str)
    wait_for_tf_timeout = float(rospy.get_param('~wait_for_tf_timeout', 30.0))

    extra_args = rospy.get_param('~extra_args', '')

    rospy.loginfo("move_base wrapper starting. wait_for_scan=%s (%s), wait_for_tf=%s (%s)",
                  wait_for_scan, scan_topic, wait_for_tf, tf_pairs_str)

    # 1) Ensure ROS master is reachable (get_param will throw if not)
    start = time.time()
    while not rospy.is_shutdown():
        try:
            rospy.get_param('/rosversion')
            break
        except Exception:
            if time.time() - start > 10.0:
                rospy.logwarn("ROS master not reachable yet, still waiting...")
                start = time.time()
            rospy.sleep(0.2)

    # 2) Wait for /scan to exist (publisher present)
    if wait_for_scan:
        rospy.loginfo("Waiting for topic %s (timeout=%.1fs)...", scan_topic, wait_for_scan_timeout)
        ok = _wait_for_topic(scan_topic, wait_for_scan_timeout)
        if not ok:
            rospy.logwarn("Topic %s not ready after %.1fs; will still start move_base.", scan_topic, wait_for_scan_timeout)

    # 3) Wait for TF chain pieces (best-effort)
    if wait_for_tf and tf_pairs:
        rospy.loginfo("Waiting for TF pairs: %s (timeout=%.1fs)...", tf_pairs_str, wait_for_tf_timeout)
        ok = _wait_for_tf_pairs(tf_pairs, wait_for_tf_timeout)
        if not ok:
            rospy.logwarn("TF not fully ready after %.1fs; will still start move_base.", wait_for_tf_timeout)

    # 4) Exec move_base
    move_base_path = rospy.get_param('~move_base_path', '/opt/ros/melodic/lib/move_base/move_base')
    cmd = [move_base_path]

    # Preserve node name given by launch (roslaunch passes __name:=move_base etc.)
    # We also preserve any args passed after a '--' sentinel.
    argv = sys.argv[1:]
    passthrough = []
    if '--' in argv:
        idx = argv.index('--')
        passthrough = argv[idx + 1:]

    if extra_args:
        cmd.extend(shlex.split(extra_args))
    cmd.extend(passthrough)

    rospy.loginfo("Exec: %s", " ".join([_shell_quote(x) for x in cmd]))

    # Replace current process so roslaunch tracks move_base directly.
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()

