import rospy
from std_msgs.msg import String


class StateController():
    def __init__(self):
        self.current_state = None
        self.next_state = "first"
        self.current_state_subscriber = rospy.Subscriber('current_state', String, self.updateState)
        self.in_progress = rospy.Subscriber('in_progress', String, self.updateState)
        self.nextState = rospy.Publisher('next_state', String, queue_size=10)
        self.nav_check = rospy.Publisher('nav', String, queue_size=10)
        self.rate = 10

    def current_state_callback(self, data):
        self.current_state = data.data 



    def main(self):
        rospy.init_node('state_controller')
        r   = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            self.pub.publish()
            r.sleep()