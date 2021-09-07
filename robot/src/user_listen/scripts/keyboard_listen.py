#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import select
import termios
import tty


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        pub = rospy.Publisher('/fight_with_hair/user_listen/keyboard', String, queue_size=10)
        rospy.init_node('keyboard_listen', anonymous=True)
        while True:
            key = get_key()
            if key == '\x03':
                break
            pub.publish(key)

    except rospy.ROSInternalException:
        pass
