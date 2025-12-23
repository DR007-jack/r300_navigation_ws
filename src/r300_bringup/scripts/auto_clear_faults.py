#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Optionally attempt to clear base faults before starting navigation.

This script is designed to be used as a roslaunch 'gate' (usually with
required="true"). It will:
- wait for /scout_status
- if fault_code == 0: keep running (gate open)
- else: call /scout/clear_faults (std_srvs/Trigger) a few times
- wait for fault_code to become 0

Notes:
- This does NOT bypass the hardware safety chain (E-Stop, motor driver lockout,
  RC requirements). If the underlying condition persists, fault_code will stay
  non-zero and this gate will timeout.

Contract (private params):
- ~status_topic (str, default: /scout_status)
- ~rs_status_topic (str, default: /rs_status)
- ~service (str, default: /scout/clear_faults)
- ~timeout (float seconds, default: 20.0)  # 0 or negative means wait forever
- ~call_retries (int, default: 3)
- ~call_interval (float seconds, default: 1.0)
- ~require_pc_control (bool, default: False)
- ~pc_control_swb_value (int, default: 0)

Exit codes:
- 0: fault_code cleared OR was already zero
- 1: timeout
- 2: service call failure

ROS1 + Python 2.7 compatible.
"""

import rospy
from scout_msgs.msg import ScoutStatus, ScoutRsStatus
from std_srvs.srv import Trigger


def main():
    rospy.init_node('auto_clear_faults')

    status_topic = rospy.get_param('~status_topic', '/scout_status')
    rs_status_topic = rospy.get_param('~rs_status_topic', '/rs_status')
    service_name = rospy.get_param('~service', '/scout/clear_faults')
    timeout = float(rospy.get_param('~timeout', 20.0))
    call_retries = int(rospy.get_param('~call_retries', 3))
    call_interval = float(rospy.get_param('~call_interval', 1.0))

    require_pc_control = bool(rospy.get_param('~require_pc_control', False))
    pc_control_swb_value = int(rospy.get_param('~pc_control_swb_value', 0))

    deadline = None
    if timeout > 0:
        deadline = rospy.Time.now() + rospy.Duration(timeout)

    def time_left_ok():
        if deadline is None:
            return True
        return rospy.Time.now() <= deadline

    rospy.loginfo('auto_clear_faults: waiting for %s (timeout=%.1fs)...', status_topic, timeout)

    last_status = {'msg': None}
    last_rs_status = {'msg': None}

    def cb(msg):
        last_status['msg'] = msg

    def cb_rs(msg):
        last_rs_status['msg'] = msg

    rospy.Subscriber(status_topic, ScoutStatus, cb, queue_size=1)
    if require_pc_control:
        rospy.Subscriber(rs_status_topic, ScoutRsStatus, cb_rs, queue_size=1)

    # Wait for first status
    r = rospy.Rate(20)
    while not rospy.is_shutdown() and last_status['msg'] is None and time_left_ok():
        r.sleep()

    if last_status['msg'] is None:
        rospy.logerr('auto_clear_faults: timeout waiting for first %s', status_topic)
        raise SystemExit(1)

    if require_pc_control:
        rospy.loginfo('auto_clear_faults: require_pc_control=true. Will wait for %s and require swb==%d (PC control).',
                      rs_status_topic, pc_control_swb_value)

        # Wait for first rs_status
        while not rospy.is_shutdown() and last_rs_status['msg'] is None and time_left_ok():
            r.sleep()
        if last_rs_status['msg'] is None:
            rospy.logerr('auto_clear_faults: timeout waiting for first %s (need it to verify SWB)', rs_status_topic)
            raise SystemExit(1)

        # Wait until SWB is in PC-control position
        last_warn_t = rospy.Time(0)
        while not rospy.is_shutdown() and time_left_ok():
            msg_rs = last_rs_status['msg']
            if msg_rs and int(msg_rs.swb) == pc_control_swb_value:
                rospy.loginfo('auto_clear_faults: SWB OK (swb=%d).', int(msg_rs.swb))
                break
            now = rospy.Time.now()
            if (now - last_warn_t).to_sec() > 1.0:
                swb_val = 'None' if msg_rs is None else str(int(msg_rs.swb))
                rospy.logwarn('auto_clear_faults: waiting for PC control (need swb==%d, current swb=%s). Please set SWB to PC/industrial control position.',
                              pc_control_swb_value, swb_val)
                last_warn_t = now
            r.sleep()

        if last_rs_status['msg'] is None or int(last_rs_status['msg'].swb) != pc_control_swb_value:
            rospy.logerr('auto_clear_faults: timed out waiting for SWB to be in PC control position (swb==%d).',
                         pc_control_swb_value)
            raise SystemExit(1)

    if last_status['msg'].fault_code == 0:
        rospy.loginfo('auto_clear_faults: fault_code=0, nothing to do. Gate open.')
        rospy.spin()
        return

    rospy.logwarn('auto_clear_faults: detected fault_code=%d. Will call %s up to %d times.',
                  int(last_status['msg'].fault_code), service_name, call_retries)

    try:
        if timeout > 0:
            rospy.wait_for_service(service_name, timeout=5.0)
        else:
            rospy.wait_for_service(service_name)
    except Exception as e:
        rospy.logerr('auto_clear_faults: service %s not available: %s', service_name, e)
        raise SystemExit(2)

    proxy = rospy.ServiceProxy(service_name, Trigger)

    for i in range(call_retries):
        if rospy.is_shutdown() or (not time_left_ok()):
            break

        try:
            resp = proxy()
            if resp.success:
                rospy.loginfo('auto_clear_faults: clear_faults call %d/%d OK: %s', i + 1, call_retries, resp.message)
            else:
                rospy.logwarn('auto_clear_faults: clear_faults call %d/%d returned success=false: %s',
                              i + 1, call_retries, resp.message)
        except Exception as e:
            rospy.logerr('auto_clear_faults: clear_faults call failed: %s', e)

        # Wait a bit for status to update
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and time_left_ok():
            msg = last_status['msg']
            if msg and msg.fault_code == 0:
                rospy.loginfo('auto_clear_faults: fault cleared (fault_code=0). Gate open.')
                rospy.spin()
                return
            if (rospy.Time.now() - t0).to_sec() >= call_interval:
                break
            r.sleep()

    # Final check
    msg = last_status['msg']
    if msg and msg.fault_code == 0:
        rospy.loginfo('auto_clear_faults: fault cleared (fault_code=0). Gate open.')
        rospy.spin()
        return

    rospy.logerr('auto_clear_faults: fault not cleared. Last fault_code=%s',
                 'None' if msg is None else str(int(msg.fault_code)))
    raise SystemExit(1)


if __name__ == '__main__':
    main()

