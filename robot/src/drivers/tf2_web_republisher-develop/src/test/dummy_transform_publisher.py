#! /usr/bin/env python
import geometry_msgs.msg
import rospy
import tf2_ros

if __name__ == '__main__':
    """
    Broadcast a transform from parent_frame to target_frame forever,
    with changing translation.
    """

    rospy.init_node('dummy_transform_publisher', anonymous=True)
    br = tf2_ros.TransformBroadcaster()
    current_transform = geometry_msgs.msg.TransformStamped()
    current_transform.header.frame_id = rospy.get_param('~parent_frame')
    current_transform.child_frame_id = rospy.get_param('~child_frame')
    # Ensure rotation quaternion is well-formed
    current_transform.transform.rotation.w = 1.0

    osc_rate_hz = rospy.get_param('~osc_rate')
    update_rate_hz = rospy.get_param('~update_rate')
    # we oscillate between 0 and 1, so this is our step size
    # for osc_rate = 1Hz, update_rate = 60Hz -> 1/60
    step_size = (1.0 * osc_rate_hz) / update_rate_hz
    update_rate = rospy.Rate(update_rate_hz)
    counting_up = True

    while not rospy.is_shutdown():
        current_transform.header.stamp = rospy.Time.now()
        if counting_up:
            current_transform.transform.translation.x += step_size
            if current_transform.transform.translation.x >= 1.0:
                counting_up = False
        else:
            current_transform.transform.translation.x -= step_size
            if current_transform.transform.translation.x <= 0.0:
                counting_up = True
        br.sendTransform(current_transform)
        update_rate.sleep()



