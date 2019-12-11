import rospy
import tf
import time
from std_msgs.msg import Float64

def talker():
    pub_kata = rospy.Publisher('/RobotArm_NoCollision/kata_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        ang = [-2.0, 2.0]
        pub_kata.publish(ang[i])
        time.sleep(1)

        if i == 0:
            i = 1
        else:
            i = 0
    
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
