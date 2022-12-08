#!/usr/bin/env python
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python
# END SHEBANG
 
# BEGIN IMPORT
import rospy
# END IMPORT
 
# BEGIN STD_MSGS
from std_msgs.msg import Int32
# END STD_MSGS
 
 
rospy.init_node('topic_publisher')
 
# BEGIN PUB
pub_area = rospy.Publisher('robodetect_area', Int32)
pub_nurse_x = rospy.Publisher('robodetect_nurse_x', Int32)
pub_nurse_y = rospy.Publisher('robodetect_nurse_y', Int32)
# END PUB
 
# BEGIN LOOP
rate = rospy.Rate(114)
 
count = 0
while not rospy.is_shutdown():
    pub_area.publish(count)
    pub_nurse_x.publish(count*10)
    pub_nurse_y.publish(count*2)
    count += 1
    rate.sleep()
# END LOOP
# END ALL