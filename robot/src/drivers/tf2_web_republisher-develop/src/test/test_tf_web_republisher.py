#!/usr/bin/env python
PKG = 'test_tf2_web_republisher'

import actionlib
import rospy
from tf2_web_republisher.msg import TFSubscriptionAction, TFSubscriptionGoal, TFArray
from tf2_web_republisher.srv import RepublishTFs, RepublishTFsRequest

import sys
import unittest


class TestTfRepublisher(unittest.TestCase):
    def setUp(self):
        self.msgs_received = 0

    def transform_cb(self, _):
        self.msgs_received += 1

    """
    Test the action interface for tf2_web_republisher.
    """
    def test_action(self):
        client = actionlib.SimpleActionClient('tf2_web_republisher',
                                              TFSubscriptionAction)
        client.wait_for_server(timeout=rospy.Duration(2))
        goal = TFSubscriptionGoal(
            source_frames=['foo', 'bar'],
            target_frame='world',
            angular_thres=0.1,
            trans_thres=0.05,
            rate=2.0)

        client.send_goal(goal, done_cb=None, active_cb=None,
                         feedback_cb=self.transform_cb)
        rospy.sleep(1.3)
        client.cancel_goal()
        # We should have gotten two feedback messages by now
        self.assertEquals(2, self.msgs_received)
        rospy.sleep(1.0)
        # We cancelled, so we expect no further feedback
        self.assertEquals(2, self.msgs_received)

    def test_service(self):
        self.assertEquals(0, self.msgs_received)
        rospy.wait_for_service('republish_tfs', 2.0)
        proxy = rospy.ServiceProxy('republish_tfs', RepublishTFs)
        result = proxy(RepublishTFsRequest(source_frames=['foo', 'bar'],
                                           target_frame='world',
                                           angular_thres=0.1,
                                           trans_thres=0.05,
                                           rate=2,
                                           timeout=rospy.Duration(1.0)))
        sub = rospy.Subscriber(result.topic_name,
                               TFArray,
                               self.transform_cb)
        rospy.sleep(1.3)
        self.assertTrue(any([topic_tuple[0] == result.topic_name
                             for topic_tuple in rospy.get_published_topics()]),
                            msg=str(rospy.get_published_topics()))
        sub.unregister()
        self.assertEquals(2, self.msgs_received)
        rospy.sleep(2.0)
        self.assertFalse(any([topic_tuple[0] == result.topic_name
                              for topic_tuple in rospy.get_published_topics()]))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_tf_web_republisher')
    rostest.rosrun(PKG, 'test_tf2_web_republisher', TestTfRepublisher)

