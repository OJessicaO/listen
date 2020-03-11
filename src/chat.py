#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


soundhandle = SoundClient()

def callback(data):
    rospy.sleep(1)
    soundhandle.say("I heard " + data.data)
    rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)


    rospy.Subscriber("/recognizer/output", String, callback)

    #??? rospy.rate(10)
    rospy.spin()
