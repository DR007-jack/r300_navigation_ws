#!/usr/bin/env python2
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

import math
from collections import deque


def _isfinite(x):
    """Python 2.7 compatible finiteness check.

    - Python3: math.isfinite exists.
    - Python2.7: no math.isfinite; emulate using (not isnan and not isinf).
    """
    try:
        return math.isfinite(x)
    except AttributeError:
        return (not math.isnan(x)) and (not math.isinf(x))


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

        # Optional safety filter: sanitize LaserScan ranges before republishing.
        # This protects consumers like AMCL/costmap from NaN/Inf/0/negative values.
        #
        # NOTE: Default is enabled because it is generally safe and improves robustness.
        self.enable_filter = rospy.get_param('~enable_filter', True)
        self.min_range_floor = float(rospy.get_param('~min_range_floor', 0.05))
        self.drop_invalid_to_inf = rospy.get_param('~drop_invalid_to_inf', True)
        # Some AMCL/costmap stacks (or downstream filters) behave better when "no return"
        # is represented as a large finite value instead of +Inf.
        # If enabled, we will replace any +Inf ranges with (range_max + inf_epsilon).
        self.replace_inf = rospy.get_param('~replace_inf', False)
        self.inf_epsilon = float(rospy.get_param('~inf_epsilon', 0.01))

        # If enabled, overwrite outgoing scan header stamps to rospy.Time.now().
        # This is a pragmatic workaround for setups where sensor timestamps are
        # not synchronized with the machine running navigation/TF consumers.
        # NOTE: It reduces temporal fidelity (motion distortion), so keep it off
        # unless you see TF_OLD_DATA / extrapolation / AMCL crashes.
        self.force_now_stamp = rospy.get_param('~force_now_stamp', False)

        # Additional robustness knobs for flaky/dirty scans:
        # - If too many ranges are invalid, AMCL may segfault intermittently.
        #   We can drop such scans (safer) so consumers wait for a usable one.
        # - Optionally, when dropping a scan, republish the last known-good scan
        #   to keep downstream nodes alive (use with caution).
        self.min_valid_ratio = float(rospy.get_param('~min_valid_ratio', 0.2))
        self.min_valid_beams = int(rospy.get_param('~min_valid_beams', 50))
        # Default to republishing last good scan to avoid downstream nodes timing out
        # when the lidar publishes very dirty scans for a while.
        self.on_bad_scan = rospy.get_param('~on_bad_scan', 'republish_last')  # drop|republish_last

        # Optional lightweight range smoothing to remove isolated spikes.
        # Set to 0 to disable.
        self.median_filter_window = int(rospy.get_param('~median_filter_window', 0))
        if self.median_filter_window < 0:
            self.median_filter_window = 0
        if self.median_filter_window > 0 and (self.median_filter_window % 2) == 0:
            # median needs odd window
            self.median_filter_window += 1

        self._last_good_scan = None

        # Throttling for logs.
        self._last_sanitize_warn = rospy.Time(0)
        self._last_bad_warn = rospy.Time(0)
        self._last_drop_warn = rospy.Time(0)
        self._warn_period = rospy.Duration(rospy.get_param('~filter_warn_period', 5.0))

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

        # Forward the message. By default we sanitize ranges to avoid downstream crashes.
        try:
            out = msg

            if self.enable_filter:
                out, invalid_count, total, is_bad = self._sanitize_scan(msg)

                if is_bad:
                    # Keep /scan alive even when current scan is too dirty.
                    if self.on_bad_scan == 'republish_last' and self._last_good_scan is not None:
                        self.pub.publish(self._last_good_scan)
                        self._warn_throttle('_last_drop_warn', now,
                                           'scan_topic_bridge: BAD scan -> republishing last good scan (valid %d/%d, thresholds ratio>=%.2f beams>=%d)',
                                           max(0, total - invalid_count), total, self.min_valid_ratio, self.min_valid_beams)
                    else:
                        self._warn_throttle('_last_drop_warn', now,
                                           'scan_topic_bridge: BAD scan -> dropped (no last_good or on_bad_scan=%s) (valid %d/%d, thresholds ratio>=%.2f beams>=%d)',
                                           str(self.on_bad_scan), max(0, total - invalid_count), total, self.min_valid_ratio, self.min_valid_beams)
                    return

            if self.force_now_stamp:
                # Never mutate the original message. If sanitize is disabled we may
                # be holding a reference to `msg`.
                if out is msg:
                    tmp = LaserScan()
                    tmp.header = msg.header
                    tmp.angle_min = msg.angle_min
                    tmp.angle_max = msg.angle_max
                    tmp.angle_increment = msg.angle_increment
                    tmp.time_increment = msg.time_increment
                    tmp.scan_time = msg.scan_time
                    tmp.range_min = msg.range_min
                    tmp.range_max = msg.range_max
                    tmp.ranges = list(msg.ranges)
                    tmp.intensities = list(msg.intensities) if msg.intensities else []
                    out = tmp

                out.header.stamp = now

            # We preserve the incoming header/frame_id unless explicitly changed by sanitization.
            self.pub.publish(out)

            # Track last known-good scan.
            if self.enable_filter:
                # Update only when scan passes thresholds.
                if 'is_bad' in locals() and (not is_bad):
                    self._last_good_scan = out
        except Exception as e:
            rospy.logerr('scan_topic_bridge: publish failed from %s: %s', src_topic, e)

        # Lightweight periodic debug to confirm data is flowing.
        if (now - self.last_debug_log) >= self.debug_log_period:
            self.last_debug_log = now
            rospy.loginfo('scan_topic_bridge: forwarding from %s, frame_id=%s, stamp=%.3f',
                          self.active_src, msg.header.frame_id, msg.header.stamp.to_sec())

    def _warn_throttle(self, attr_name, now, fmt, *args):
        """Throttled warnings without relying on rospy's log* throttling impl."""
        try:
            last = getattr(self, attr_name)
        except Exception:
            last = rospy.Time(0)
        if (now - last) >= self._warn_period:
            setattr(self, attr_name, now)
            try:
                rospy.logwarn(fmt, *args)
            except Exception:
                # Fallback: never crash due to logging formatting issues.
                rospy.logwarn('scan_topic_bridge: ' + str(fmt))

    def _sanitize_scan(self, msg):
        """Return a sanitized LaserScan.

        Rules:
        - Ensure range_min is at least min_range_floor (default 0.05).
        - If range_max is invalid, keep it but avoid breaking ranges filtering.
        - For each range:
          - If NaN/Inf (not finite) -> set to +Inf (or 0 if configured otherwise)
          - If <= 0 or < range_min -> set to +Inf
          - If range_max>0 and r > range_max -> set to +Inf
        """

        # Shallow-copy into a new message so we don't mutate input in place.
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time

        # Clamp range_min to a safe floor.
        in_min = msg.range_min
        in_max = msg.range_max
        # Some drivers publish 0 here; AMCL expects a sensible positive value.
        out.range_min = max(float(in_min), self.min_range_floor)
        out.range_max = float(in_max)

        # Sanitize ranges.
        invalid_count = 0
        total = len(msg.ranges)

        # If driver publishes a non-positive range_max, treat as "unknown" and don't upper-bound.
        has_valid_max = (out.range_max > 0.0)

        # If we are asked to replace Inf, we need a finite replacement value.
        # Fall back to a conservative constant if range_max is not usable.
        if self.replace_inf:
            if has_valid_max:
                inf_replacement = float(out.range_max) + float(self.inf_epsilon)
            else:
                inf_replacement = 100.0
        else:
            inf_replacement = float('inf')

        # Precompute what to write when encountering invalid values.
        invalid_replacement = inf_replacement if self.drop_invalid_to_inf else 0.0

        ranges_out = []
        for r in msg.ranges:
            try:
                rf = float(r)
            except Exception:
                invalid_count += 1
                ranges_out.append(invalid_replacement)
                continue

            if (not _isfinite(rf)):
                invalid_count += 1
                ranges_out.append(invalid_replacement)
                continue

            # 0 / negative / too close -> invalid
            if rf <= 0.0 or rf < out.range_min:
                invalid_count += 1
                ranges_out.append(inf_replacement)
                continue

            # too far -> invalid
            if has_valid_max and rf > out.range_max:
                invalid_count += 1
                ranges_out.append(inf_replacement)
                continue

            ranges_out.append(rf)

        # Optional median filter over finite values.
        if self.median_filter_window > 1 and len(ranges_out) >= self.median_filter_window:
            half = self.median_filter_window // 2
            win = deque(maxlen=self.median_filter_window)
            filtered = list(ranges_out)
            for i in range(len(ranges_out)):
                # build window centered at i
                win.clear()
                for j in range(i - half, i + half + 1):
                    if j < 0 or j >= len(ranges_out):
                        continue
                    v = ranges_out[j]
                    if _isfinite(v) and v > 0.0:
                        win.append(v)

                if len(win) >= 3:
                    s = sorted(win)
                    filtered[i] = s[len(s) // 2]
            ranges_out = filtered

        out.ranges = ranges_out
        out.intensities = list(msg.intensities) if msg.intensities else []

        # Throttled warning if we are filtering a lot.
        now = rospy.Time.now()
        if total > 0 and invalid_count > 0:
            ratio = 100.0 * float(invalid_count) / float(total)
            self._warn_throttle('_last_sanitize_warn', now,
                                'scan_topic_bridge: sanitized scan from frame_id=%s: %d/%d (%.1f%%) ranges invalid; range_min %.3f->%.3f',
                                msg.header.frame_id, invalid_count, total, ratio, float(in_min), float(out.range_min))

        # Guardrail: if too little usable data remains, log once (drop decision is in caller).
        valid_count = max(0, total - invalid_count)
        valid_ratio = (float(valid_count) / float(total)) if total > 0 else 0.0
        is_bad = (total <= 0) or (valid_ratio < self.min_valid_ratio) or (valid_count < self.min_valid_beams)
        if is_bad and total > 0:
            self._warn_throttle('_last_bad_warn', now,
                                'scan_topic_bridge: scan deemed BAD (valid %d/%d=%.1f%%) < thresholds (min_valid_ratio=%.2f, min_valid_beams=%d)',
                                valid_count, total, 100.0 * valid_ratio, self.min_valid_ratio, self.min_valid_beams)

        return out, invalid_count, total, is_bad

    def _should_drop_sanitized(self, invalid_count, total):
        if total <= 0:
            return True
        valid_count = max(0, total - invalid_count)
        valid_ratio = float(valid_count) / float(total)
        return (valid_ratio < self.min_valid_ratio) or (valid_count < self.min_valid_beams)


def main():
    rospy.init_node('scan_topic_bridge')
    bridge = ScanBridge()
    rospy.spin()


if __name__ == '__main__':
    main()


